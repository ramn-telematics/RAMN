/* Host tests for ECU A's registration-code (OTP) screen.
 *
 * White-box: this #includes ramn_screen_regcode.c so it can reach the static
 * SCREENREGCODE_ProcessRxCANMessage and the state behind it. Same approach as
 * the two suites next door, and the same reason it needs a binary of its own:
 * two files cannot each #include a .c into one link.
 *
 * The question here is not what the panel looks like -- it is WHICH FRAMES GET
 * TO PUT SOMETHING ON IT. So the assertion surface is the six large characters
 * RAMN_SPI_DrawLargeChar is handed, which is exactly what a person standing in
 * front of the vehicle would read off the screen, and every case says whether
 * a given frame should produce them.
 *
 * Fixtures speak as ECU D: they hold their own copy of the session key,
 * derived independently from the two nonces rather than read out of ECU A, so
 * a MAC that verifies here says the two ends agree rather than that the
 * implementation agrees with itself.
 */
#include <string.h>

#include "harness.h"
#include "fakes_screen.h"
#include "fakes.h"

/* ENABLE_SCREEN comes from the real ramn_config.h for an ECU A build. */
#include "../../Core/Src/ramn_screen_regcode.c"

#ifdef ENABLE_REGCODE_SECOC

#include "ramn_secoc_session.h"
#include "ramn_secoc_keys.h"

/* ------------------------------------------------------------------ */
/* Fixtures                                                            */
/* ------------------------------------------------------------------ */

static uint8_t                test_session_key[RAMN_SECOC_KEY_BYTES];
static RAMN_SecOC_Freshness_t test_fv;

static void feed_frame(uint32_t id, const uint8_t *b, uint32_t dlc, uint32_t tick)
{
    FDCAN_RxHeaderTypeDef h;
    memset(&h, 0, sizeof h);
    h.Identifier  = id;
    h.DataLength  = dlc;
    h.IdType      = FDCAN_STANDARD_ID;
    h.RxFrameType = FDCAN_DATA_FRAME;
    h.FDFormat    = FDCAN_FD_CAN;

    if (id == REGCODE_CAN_ID) SCREENREGCODE_ProcessRxCANMessage(&h, b, tick);
    RAMN_SecOC_LINK_ProcessRxCANMessage(&h, b, tick);
}

static const CapturedFrame_t *last_frame(uint32_t id)
{
    for (int i = fake_can_tx_count - 1; i >= 0; i--)
        if (fake_can_tx[i].header.Identifier == id) return &fake_can_tx[i];
    return NULL;
}

/* Runs the handshake as ECU D would, against ECU A's verifier half. */
static int establish_session(uint32_t tick)
{
    fake_reset();
    /* The rate limit is real and correct on a bus; across cases in one binary
       it would refuse the second case's handshake. */
    RAMN_SecOC_LINK_Init();

    uint8_t req[4] = {0, 0, 0, 0};
    feed_frame(SESSION_CAN_ID_REQ, req, FDCAN_DLC_BYTES_4, tick);

    const CapturedFrame_t *ch = last_frame(SESSION_CAN_ID_CHALLENGE);
    if (ch == NULL) return 0;

    uint8_t nonceA[RAMN_SECOC_NONCE_BYTES];
    memcpy(nonceA, ch->data, RAMN_SECOC_NONCE_BYTES);

    uint8_t nonceD[RAMN_SECOC_NONCE_BYTES];
    for (unsigned i = 0; i < RAMN_SECOC_NONCE_BYTES; i++) nonceD[i] = (uint8_t)(0xD0 + i);

    uint8_t resp[RAMN_SECOC_NONCE_BYTES + RAMN_SECOC_SESSION_MAC_BYTES];
    memcpy(resp, nonceD, RAMN_SECOC_NONCE_BYTES);
    RAMN_SecOC_SESSION_Mac(RAMN_SecOC_KEYS_GetImageKey(), SESSION_CAN_ID_RESPONSE,
                           nonceA, nonceD, &resp[RAMN_SECOC_NONCE_BYTES]);
    feed_frame(SESSION_CAN_ID_RESPONSE, resp, FDCAN_DLC_BYTES_16, tick);

    RAMN_SecOC_Session_t mine;
    RAMN_SecOC_SESSION_Reset(&mine);
    memcpy(mine.nonceA, nonceA, RAMN_SECOC_NONCE_BYTES);
    memcpy(mine.nonceD, nonceD, RAMN_SECOC_NONCE_BYTES);
    RAMN_SecOC_SESSION_Derive(&mine, RAMN_SecOC_KEYS_GetImageKey());
    memcpy(test_session_key, mine.key, RAMN_SECOC_KEY_BYTES);

    RAMN_SecOC_FreshnessInit(&test_fv);
    fake_reset();
    return RAMN_SecOC_LINK_Ready() ? 1 : 0;
}

/* Returns this module to the state it is in when nothing has been shown, so
   one case's ten-second timer and cooldown cannot decide the next one's
   result. Deliberately does NOT touch regFv -- that survives everything short
   of a rekey, and several cases below are about exactly that. */
static void clear_screen_state(void)
{
    screenActive        = False;
    screenActivatedTick = 0;
    lastTimeoutTick     = 0;
    registrationCode    = 0;
    RAMN_SCREENREGCODE_DisplayRequested = False;
    RAMN_SCREENREGCODE_NoSessionDrops   = 0;
    RAMN_SCREENREGCODE_AuthFailures     = 0;
    fake_screen_reset();
}

/* Builds the frame ECU D would send: code, freshness, authenticator. */
static void build_regcode(uint8_t *out, uint32_t code, uint32_t fv, const uint8_t *key)
{
    memset(out, 0, REGCODE_CAN_FRAME_BYTES);
    out[0] = (uint8_t)(code);
    out[1] = (uint8_t)(code >> 8);
    out[2] = (uint8_t)(code >> 16);
    out[3] = (uint8_t)(code >> 24);
    out[REGCODE_SECOC_FV_OFFSET]      = (uint8_t)((fv >> 8) & 0xFF);
    out[REGCODE_SECOC_FV_OFFSET + 1]  = (uint8_t)( fv       & 0xFF);

    RAMN_SecOC_Ctx_t ctx;
    ctx.dataId = (uint16_t)REGCODE_CAN_ID;
    ctx.macLen = REGCODE_SECOC_MAC_BYTES;
    ctx.key    = key;
    ctx.fv     = &test_fv;
    uint8_t authLen = (uint8_t)(REGCODE_CAN_FRAME_BYTES - REGCODE_SECOC_MAC_BYTES);
    RAMN_SecOC_ComputeMac(&ctx, fv, out, authLen, &out[authLen]);
}

/* The next frame ECU D would send, on the next freshness value. */
static uint32_t next_regcode(uint8_t *out, uint32_t code)
{
    uint32_t fv = RAMN_SecOC_TxFreshness(&test_fv);
    build_regcode(out, code, fv, test_session_key);
    return fv;
}

/* What the panel would read. Init is what draws the code, and the screen
   manager calls it when DisplayRequested goes up -- so a case that expects
   digits calls it here, and one that expects nothing asserts the flag never
   went up in the first place. */
static const char *panel_digits(void)
{
    SCREENREGCODE_Init();
    return fake_large_chars;
}

/* ------------------------------------------------------------------ */
/* Cases                                                               */
/* ------------------------------------------------------------------ */

static void case_an_authentic_code_is_displayed(void)
{
    h_case_begin("a code from ECU D reaches the panel");

    if (!CHECK_OK(establish_session(1), "a session is up")) return;
    clear_screen_state();

    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(f, 123456);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested != False, "the screen is requested");
    CHECK(strcmp(panel_digits(), "123456") == 0, "and the six digits are the code ECU D sent");
    CHECK(RAMN_SCREENREGCODE_AuthFailures == 0, "nothing was refused");
}

static void case_an_unauthenticated_code_shows_nothing(void)
{
    h_case_begin("six digits from someone without the key never reach the panel");

    /* The attack the whole feature exists for: anyone can put 0x7A0 on the
       bus, and before SecOC that was enough to show a driver any code at all. */
    if (!CHECK_OK(establish_session(1), "a session is up")) return;
    clear_screen_state();

    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(f, 111111);
    memset(&f[REGCODE_CAN_FRAME_BYTES - REGCODE_SECOC_MAC_BYTES], 0x5A,
           REGCODE_SECOC_MAC_BYTES);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested == False, "the screen is not requested");
    CHECK(RAMN_SCREENREGCODE_AuthFailures == 1, "and the refusal is counted, not silent");
}

static void case_a_code_signed_with_the_wrong_key_shows_nothing(void)
{
    h_case_begin("a well-formed code under the wrong key is refused");

    /* Distinct from a forged MAC: this is an attacker who has the format
       exactly right, which is the realistic case once the wire layout is
       public -- and it is public, this is an open repository. */
    if (!CHECK_OK(establish_session(1), "a session is up")) return;
    clear_screen_state();

    uint8_t wrongKey[RAMN_SECOC_KEY_BYTES];
    memset(wrongKey, 0x42, sizeof wrongKey);

    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    build_regcode(f, 222222, RAMN_SecOC_TxFreshness(&test_fv), wrongKey);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested == False, "nothing is displayed");
    CHECK(RAMN_SCREENREGCODE_AuthFailures == 1, "and it is counted as an auth failure");
}

static void case_the_code_itself_cannot_be_edited_in_flight(void)
{
    h_case_begin("changing one digit of a genuine frame invalidates it");

    /* The authenticator covers the code, so a man in the middle cannot take a
       real frame and change what it says. Without this, an attacker who cannot
       forge a code can still choose one. */
    if (!CHECK_OK(establish_session(1), "a session is up")) return;
    clear_screen_state();

    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(f, 314159);
    f[0] ^= 0x01;
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested == False, "the edited code is refused");
    CHECK(RAMN_SCREENREGCODE_AuthFailures == 1, "and counted");
}

static void case_the_freshness_field_cannot_be_edited_either(void)
{
    h_case_begin("moving the freshness value of a genuine frame invalidates it");

    /* The transmitted freshness sits INSIDE the authenticated region, which is
       what stops a recorded frame being re-dated forward to look current. */
    if (!CHECK_OK(establish_session(1), "a session is up")) return;
    clear_screen_state();

    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(f, 271828);
    f[REGCODE_SECOC_FV_OFFSET + 1] = (uint8_t)(f[REGCODE_SECOC_FV_OFFSET + 1] + 5);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested == False, "the re-dated frame is refused");
    CHECK(RAMN_SCREENREGCODE_AuthFailures == 1, "and counted");
}

static void case_a_recorded_code_cannot_be_shown_twice(void)
{
    h_case_begin("a genuine frame replayed later is refused");

    /* A one-time code that can be shown twice is not one. The freshness
       counter is the whole of this: same bytes, same key, same everything --
       the only thing that changed is that ECU A has already seen it. */
    if (!CHECK_OK(establish_session(1), "a session is up")) return;
    clear_screen_state();

    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(f, 606060);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);
    if (!CHECK_OK(RAMN_SCREENREGCODE_DisplayRequested != False,
                  "the first showing is accepted")) return;

    /* Leaving the screen purges the code, as it should -- and must not purge
       the record of having seen it. */
    SCREENREGCODE_Deinit();
    clear_screen_state();

    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100000);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested == False, "the replay is refused");
    CHECK(RAMN_SCREENREGCODE_AuthFailures == 1, "and counted");
}

static void case_a_code_seen_while_busy_cannot_be_replayed_afterwards(void)
{
    h_case_begin("a code ignored because the screen was busy is still spent");

    /* Verification happens BEFORE the screen-state gates, and this is why. A
       genuine code that arrives while the previous one is still up is dropped
       either way -- but if its freshness were not recorded, the attacker who
       captured it could replay it the moment the screen frees up, and show a
       code the driver was never meant to see now. */
    if (!CHECK_OK(establish_session(1), "a session is up")) return;
    clear_screen_state();

    uint8_t first[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(first, 700007);
    feed_frame(REGCODE_CAN_ID, first, FDCAN_DLC_BYTES_12, 100);
    if (!CHECK_OK(screenActive != False, "the screen is showing the first code")) return;

    /* A second genuine code, sent while the first is still up. */
    uint8_t second[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(second, 800008);
    feed_frame(REGCODE_CAN_ID, second, FDCAN_DLC_BYTES_12, 200);
    CHECK(registrationCode == 700007, "it does not disturb the code on screen");

    /* The screen frees up, and the attacker replays what they captured. */
    clear_screen_state();
    feed_frame(REGCODE_CAN_ID, second, FDCAN_DLC_BYTES_12, 300000);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested == False,
          "the replayed code is refused even though it was never displayed");
}

static void case_with_no_session_nothing_is_displayed(void)
{
    h_case_begin("with no session ECU A shows nothing at all");

    /* Fail closed. Falling back to displaying an unverified code would mean an
       attacker only has to stop the handshake completing -- the easy half --
       to get whatever they like in front of the driver. */
    if (!CHECK_OK(establish_session(1), "a session is up to build a frame under")) return;
    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(f, 999999);

    RAMN_SecOC_LINK_Init();
    clear_screen_state();
    if (!CHECK_OK(RAMN_SecOC_LINK_Ready() == 0, "and now there is no session")) return;

    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested == False, "nothing is displayed");
    CHECK(RAMN_SCREENREGCODE_NoSessionDrops == 1, "and it is counted as a missing session");
    CHECK(RAMN_SCREENREGCODE_AuthFailures == 0,
          "not as an auth failure -- the two need different responses");
}

static void case_a_rekey_does_not_lock_the_code_out(void)
{
    h_case_begin("after a new session the counter starts over rather than rejecting everything");

    /* A session restarts both counters at zero. A high-water mark carried over
       from the previous one would refuse every code of this one -- and the
       carried value is not merely stale, it was recorded under a key that no
       longer exists. */
    if (!CHECK_OK(establish_session(1), "a first session is up")) return;
    clear_screen_state();

    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    for (int i = 0; i < 5; i++) {
        (void)next_regcode(f, 100000 + (uint32_t)i);
        feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);
        clear_screen_state();
    }

    if (!CHECK_OK(establish_session(50000), "a second session replaces it")) return;
    clear_screen_state();

    (void)next_regcode(f, 424242);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 50100);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested != False, "the first code of the new session lands");
    CHECK(strcmp(panel_digits(), "424242") == 0, "with the digits it carried");
}

static void case_a_stale_freshness_value_is_refused_without_hashing(void)
{
    h_case_begin("a value at or below the counter is refused");

    /* Cheap to reject and rejected first, which is what keeps a flood of
       replayed frames from costing a BLAKE2s each on the 1 KB CAN RX task. */
    if (!CHECK_OK(establish_session(1), "a session is up")) return;
    clear_screen_state();

    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(f, 505050);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);
    if (!CHECK_OK(RAMN_SCREENREGCODE_DisplayRequested != False, "a code lands")) return;
    clear_screen_state();

    /* Re-sign the SAME freshness value, so the MAC is genuinely correct and
       only the counter can refuse it. */
    build_regcode(f, 606060, test_fv.txCounter, test_session_key);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 200);

    CHECK(RAMN_SCREENREGCODE_DisplayRequested == False,
          "a correctly signed frame at a spent freshness value is still refused");
}

static void case_a_value_beyond_the_window_is_refused(void)
{
    h_case_begin("a freshness value far ahead of the counter is refused");

    /* Otherwise an attacker who cannot forge a MAC can still strand the link:
       jump the counter to the end of its range once and every genuine code
       after it is stale. They cannot -- the window is what stops it -- and
       this says so. */
    if (!CHECK_OK(establish_session(1), "a session is up")) return;
    clear_screen_state();

    uint8_t f[REGCODE_CAN_FRAME_BYTES];
    (void)next_regcode(f, 121212);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 100);
    if (!CHECK_OK(RAMN_SCREENREGCODE_DisplayRequested != False, "a code lands")) return;
    uint32_t seen = test_fv.txCounter;
    clear_screen_state();

    build_regcode(f, 131313, seen + RAMN_SECOC_FV_WINDOW + 1, test_session_key);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 200);
    CHECK(RAMN_SCREENREGCODE_DisplayRequested == False, "the jump is refused");

    /* And the genuine sender is not locked out by the attempt. */
    clear_screen_state();
    (void)next_regcode(f, 141414);
    feed_frame(REGCODE_CAN_ID, f, FDCAN_DLC_BYTES_12, 300);
    CHECK(RAMN_SCREENREGCODE_DisplayRequested != False,
          "and ECU D's next code still lands");
}

#endif /* ENABLE_REGCODE_SECOC */

int main(void)
{
    printf("ECU A registration-code screen host tests\n");
    printf("(assertion surface: the digits that would be lit on the panel)\n");

#ifdef ENABLE_REGCODE_SECOC
    case_an_authentic_code_is_displayed();
    case_an_unauthenticated_code_shows_nothing();
    case_a_code_signed_with_the_wrong_key_shows_nothing();
    case_the_code_itself_cannot_be_edited_in_flight();
    case_the_freshness_field_cannot_be_edited_either();
    case_a_recorded_code_cannot_be_shown_twice();
    case_a_code_seen_while_busy_cannot_be_replayed_afterwards();
    case_with_no_session_nothing_is_displayed();
    case_a_rekey_does_not_lock_the_code_out();
    case_a_stale_freshness_value_is_refused_without_hashing();
    case_a_value_beyond_the_window_is_refused();
#else
    printf("\nENABLE_REGCODE_SECOC is off -- nothing to verify\n");
#endif

    printf("\n%d checks | %d hard failures | %d known bugs confirmed",
           h_checks, h_failures - h_bugs_fixed, h_bugs_confirmed);
    if (h_bugs_fixed) printf(" | %d markers to remove", h_bugs_fixed);
    printf("\n");
    return h_failures ? 1 : 0;
}
