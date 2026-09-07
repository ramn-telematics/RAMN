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
# main.c -- and RAMN_TELEMATICS_Update runs on it, with PrintSPIStats' ~600
# byte frame live underneath. Two char buffers declared inline in
# RAMN_TELEMATICS_Update took its own frame from 32 to 224 bytes; that
# overflowed the task and ECU D went dead on hardware. configCHECK_FOR_STACK_
# OVERFLOW is 2 so FreeRTOS detected it, but vApplicationStackOverflowHook is
# empty, so it returned into a corrupted task and simply stopped responding.
#
# Nothing in a host suite catches that: the code compiles, links, and passes
# every functional test. So measure the frames instead. Budgets are per
# function, sized to the deepest call chain that runs on the periodic task:
# a caller's frame stays live while its callee runs, so caller + callee must
# leave room for the FreeRTOS context and everything else on that task.
#
# Frames are x86-64, not ARM, so these numbers are not the real stack cost --
# they are a REGRESSION signal. Adding a buffer shows up here the same way it
# shows up on target. If a budget genuinely needs to rise, raise it in the
# same commit that justifies it.
# ---------------------------------------------------------------------------
echo "stack budgets (periodic task total: 1024 bytes)"

# function:max_frame_bytes -- hot-path functions on the 1 KB periodic task
BUDGETS="RAMN_TELEMATICS_Update:64 PrintImageACK:128 PrintImageACKTimeout:64"

su_out="$TMP/stack.su"
if cp "$CORE/Src/ramn_telematics.c" "$TMP/su_tu.c" && \
   cc -std=c11 -O0 -w -DTARGET_ECUD $INC -fstack-usage \
      -c "$TMP/su_tu.c" -o "$TMP/su_tu.o" 2>"$TMP/err"; then
    mv "$TMP/su_tu.su" "$su_out" 2>/dev/null || true
fi

if [ ! -s "$su_out" ]; then
    echo "  could not measure stack usage (compiler lacks -fstack-usage?) -- skipped"
else
    for b in $BUDGETS; do
        fn=${b%%:*}; budget=${b##*:}
        used=$(awk -F'\t' -v f=":$fn" '$1 ~ (f "$") {print $2}' "$su_out" | head -1)
        if [ -z "$used" ]; then
            echo "  $fn: NOT FOUND in stack usage output -- renamed or removed?"
            status=1
        elif [ "$used" -gt "$budget" ]; then
            echo "  $fn: $used bytes > budget $budget -- THIS IS WHAT KILLED ECU D"
            echo "      move large buffers to static, or into their own function"
            status=1
        else
            echo "  $fn: $used/$budget bytes ok"
        fi
    done
fi

[ $status -eq 0 ] && echo "per-target link surface ok" || echo "PER-TARGET CHECK FAILED"
exit $status
