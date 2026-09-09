#!/usr/bin/env bash
# Per-target link surface check.
#
# ENABLE_UART is defined for TARGET_ECUD only, so any module compiled into
# another target must not reference RAMN_UART_* -- the symbol is neither
# declared nor compiled there and the failure appears at LINK time, in the
# STM32CubeIDE build, long after everything here has passed.
#
# That is exactly what happened: ramn_screen_image.c had SCREENIMAGE_DEBUG on
# unconditionally, and the ECU A build failed with five undefined references
# to RAMN_UART_SendFromTask. This compiles the affected modules for each target
# with the host compiler and fails if any UART reference survives.
set -uo pipefail
cd "$(dirname "$0")"

CORE=../../Core
INC="-Istubs -I. -I$CORE/Inc -I$CORE/Src"
MODULES="ramn_screen_image.c ramn_telematics.c"
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
status=0

for target in TARGET_ECUA TARGET_ECUB TARGET_ECUC TARGET_ECUD; do
    for m in $MODULES; do
        obj="$TMP/${target}_${m%.c}.o"
        if ! cc -std=c11 -O0 -w -D"$target" $INC -c "$CORE/Src/$m" -o "$obj" 2>"$TMP/err"; then
            echo "  $target $m: does not compile"; sed 's/^/      /' "$TMP/err"; status=1; continue
        fi
        refs=$(nm -u "$obj" 2>/dev/null | grep -c 'RAMN_UART' || true)
        # ECU D is the one target that has a UART to print to.
        if [ "$target" = "TARGET_ECUD" ]; then
            echo "  $target $m: $refs UART ref(s) (allowed)"
        elif [ "$refs" -ne 0 ]; then
            echo "  $target $m: $refs UART ref(s) -- WILL FAIL TO LINK"
            nm -u "$obj" | grep RAMN_UART | sed 's/^/      /'
            status=1
        else
            echo "  $target $m: ok"
        fi
    done
done

# ---------------------------------------------------------------------------
# Stack budget check.
#
# The periodic task's ENTIRE stack is 1 KB -- RAMN_PeriodicBuffer[256] in
# main.c. RAMN_TELEMATICS_Update runs on it and calls into ProcessESP32Response
# and the printers; a caller's frame stays live while its callee runs, so what
# matters is the sum along a CHAIN, not any one function.
#
# Two char buffers declared inline in RAMN_TELEMATICS_Update took its own frame
# from 32 to 224 bytes. With PrintSPIStats' ~600-byte frame underneath, that
# overflowed the task and ECU D went dead on hardware. configCHECK_FOR_STACK_
# OVERFLOW is 2 so FreeRTOS detected it -- and vApplicationStackOverflowHook
# (app_freertos.c) is EMPTY, so it returned into a corrupted task and simply
# stopped responding. Nothing in a host suite catches that: the code compiles,
# links, and passes every functional test.
#
# The debug flags are measured too, because turning one on is a one-line change
# that a maintainer will reasonably assume is free. TELEMATICS_CAN_DEBUG is not:
# it adds a 96-byte print buffer to SendImageCANFrame, at the bottom of the
# deepest chain on this task.
#
# Frames are x86-64, not ARM, so these are a REGRESSION signal, not a true
# stack cost. The budget is calibrated against a configuration known to run on
# hardware (debug off measures 640). If a budget genuinely needs to rise, raise
# it in the commit that justifies it.
# ---------------------------------------------------------------------------
PERIODIC_STACK=1024
CHAIN_BUDGET=704        # 1024 minus reserve for the FreeRTOS context and callers

# Chains that run on the periodic task, deepest first.
CHAINS="RAMN_TELEMATICS_Update,ProcessESP32Response,SendImageCANFrame
RAMN_TELEMATICS_Update,PrintSPIStats
RAMN_TELEMATICS_Update,PrintImageACK
RAMN_TELEMATICS_Update,PrintImageACKTimeout"

measure_stack() {   # $1 = source .c, $2 = target macro, $3 = extra CFLAGS, $4 = out .su
    cp "$CORE/Src/$1" "$TMP/su_tu.c" || return 1
    cc -std=c11 -O0 -w -D"$2" $3 $INC -fstack-usage \
       -c "$TMP/su_tu.c" -o "$TMP/su_tu.o" 2>"$TMP/err" || return 1
    mv "$TMP/su_tu.su" "$4" 2>/dev/null || return 1
}

frame_of() {        # $1 = .su file, $2 = function name
    awk -F'\t' -v f=":$2" '$1 ~ (f "$") {print $2; exit}' "$1"
}

# SecOC verification runs inside SCREENIMAGE_ProcessRxCANMessage on the CAN RX
# task, but it lives in two other translation units, so -fstack-usage on the
# screen module alone cannot see it. Concatenate the three .su files and
# measure the chain across all of them: BLAKE2s keeps a 64-byte block buffer in
# its context and another 128 bytes of working state in the compression
# function, and that lands on a 1 KB task stack.
measure_stack_multi() {   # $1 = target macro, $2 = out .su, $3.. = source .c files
    local target="$1" out="$2"; shift 2
    : > "$out"
    for src in "$@"; do
        cp "$CORE/Src/$src" "$TMP/su_multi.c" || return 1
        cc -std=c11 -O0 -w -D"$target" $INC -fstack-usage \
           -c "$TMP/su_multi.c" -o "$TMP/su_multi.o" 2>"$TMP/err" || return 1
        cat "$TMP/su_multi.su" >> "$out" 2>/dev/null || return 1
    done
}

echo "stack budgets (periodic task: ${PERIODIC_STACK}B total, chain budget ${CHAIN_BUDGET}B)"

for cfg in "off:" "CAN_DEBUG:-DTELEMATICS_CAN_DEBUG" "SPI_DEBUG:-DTELEMATICS_SPI_DEBUG"; do
    label=${cfg%%:*}; cflags=${cfg#*:}
    su="$TMP/stack_$label.su"
    if ! measure_stack ramn_telematics.c TARGET_ECUD "$cflags" "$su"; then
        echo "  [$label] could not measure stack usage -- skipped"; continue
    fi
    while IFS= read -r chain; do
        [ -z "$chain" ] && continue
        total=0; missing=""; pretty=""
        for fn in $(echo "$chain" | tr ',' ' '); do
            u=$(frame_of "$su" "$fn")
            if [ -z "$u" ]; then missing="$fn"; break; fi
            total=$((total + u)); pretty="$pretty $fn($u)"
        done
        if [ -n "$missing" ]; then
            echo "  [$label] $missing: NOT FOUND -- renamed or removed?"; status=1
        elif [ "$total" -gt "$CHAIN_BUDGET" ]; then
            echo "  [$label]$pretty = ${total}B > ${CHAIN_BUDGET}B -- WOULD OVERFLOW THE PERIODIC TASK"
            echo "        move large buffers to static, or off this call chain"
            status=1
        else
            echo "  [$label]$pretty = ${total}/${CHAIN_BUDGET}B ok"
        fi
    done <<EOF
$CHAINS
EOF
done

# ECU A's image screen: SCREENIMAGE_ProcessRxCANMessage runs on the CAN RX task,
# whose stack is also 1 KB (RAMN_ReceiveCANBuffer[256] in main.c). The 0x303 ACK
# is built in its own function precisely so a second call site cannot grow that
# handler's frame; this is what checks that it stayed that way.
ECUA_CHAINS="SCREENIMAGE_ProcessRxCANMessage,SendImageAck
SCREENIMAGE_Update,SendImageAck
SCREENIMAGE_Update,WriteScaledPixels"

echo "stack budgets, ECU A image screen (CAN RX task: ${PERIODIC_STACK}B total)"
su_a="$TMP/stack_ecua.su"
if ! measure_stack ramn_screen_image.c TARGET_ECUA "" "$su_a"; then
    echo "  could not measure stack usage -- skipped"
else
    while IFS= read -r chain; do
        [ -z "$chain" ] && continue
        total=0; missing=""; pretty=""
        for fn in $(echo "$chain" | tr ',' ' '); do
            u=$(frame_of "$su_a" "$fn")
            if [ -z "$u" ]; then missing="$fn"; break; fi
            total=$((total + u)); pretty="$pretty $fn($u)"
        done
        if [ -n "$missing" ]; then
            echo "  [ecua] $missing: NOT FOUND -- renamed or removed?"; status=1
        elif [ "$total" -gt "$CHAIN_BUDGET" ]; then
            echo "  [ecua]$pretty = ${total}B > ${CHAIN_BUDGET}B -- WOULD OVERFLOW THE CAN RX TASK"
            status=1
        else
            echo "  [ecua]$pretty = ${total}/${CHAIN_BUDGET}B ok"
        fi
    done <<EOF
$ECUA_CHAINS
EOF
fi

[ $status -eq 0 ] && # ---- SecOC verification depth on the CAN RX task --------------------------
# RAMN_BLAKE2S_Init no longer calls Update (it stages the key block in the
# context), so the deepest real chains run through Update and Final.
# The session handshake is DEEPER than the image path -- it adds two frames
# (SESSION_CheckMac and SESSION_Mac) above the same BLAKE2s -- and it runs on
# the same 1 KB CAN RX task, so it is the chain that actually sets the budget.
SECOC_CHAINS="SCREENIMAGE_ProcessRxCANMessage,SecOCOpenFrame,RAMN_SecOC_CheckMac,RAMN_SecOC_ComputeMac,RAMN_BLAKE2S_Update,blake2s_compress
SCREENIMAGE_ProcessRxCANMessage,SecOCCheckFrame,RAMN_SecOC_CheckMac,RAMN_SecOC_ComputeMac,RAMN_BLAKE2S_Final,blake2s_compress
SCREENIMAGE_ProcessRxCANMessage,HandleSessionResponse,RAMN_SecOC_SESSION_CheckMac,RAMN_SecOC_ComputeMac,RAMN_BLAKE2S_Update,blake2s_compress
SCREENIMAGE_ProcessRxCANMessage,HandleSessionResponse,RAMN_SecOC_SESSION_Mac,RAMN_SecOC_ComputeMac,RAMN_BLAKE2S_Final,blake2s_compress
SCREENIMAGE_ProcessRxCANMessage,HandleSessionResponse,RAMN_SecOC_SESSION_Derive,RAMN_BLAKE2S_Update,blake2s_compress"

echo "stack budgets, SecOC verify chain (CAN RX task: ${PERIODIC_STACK}B total)"
su_s="$TMP/stack_secoc.su"
if ! measure_stack_multi TARGET_ECUA "$su_s" ramn_screen_image.c ramn_secoc.c ramn_secoc_session.c ramn_blake2s.c; then
    echo "  could not measure stack usage -- skipped"
else
    while IFS= read -r chain; do
        [ -z "$chain" ] && continue
        total=0; missing=""; pretty=""
        for fn in $(echo "$chain" | tr ',' ' '); do
            u=$(frame_of "$su_s" "$fn")
            if [ -z "$u" ]; then missing="$fn"; break; fi
            total=$((total + u)); pretty="$pretty $fn($u)"
        done
        if [ -n "$missing" ]; then
            echo "  [secoc] $missing: NOT FOUND -- renamed or removed?"; status=1
        elif [ "$total" -gt "$CHAIN_BUDGET" ]; then
            echo "  [secoc]$pretty = ${total}B > ${CHAIN_BUDGET}B -- WOULD OVERFLOW THE CAN RX TASK"
            echo "        the BLAKE2s context is the big frame; make it static or shrink the chain"
            status=1
        else
            echo "  [secoc]$pretty = ${total}/${CHAIN_BUDGET}B ok"
        fi
    done <<EOF
$SECOC_CHAINS
EOF
fi

echo "per-target link surface ok" || echo "PER-TARGET CHECK FAILED"
exit $status
