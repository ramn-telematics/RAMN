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

[ $status -eq 0 ] && echo "per-target link surface ok" || echo "PER-TARGET CHECK FAILED"
exit $status
