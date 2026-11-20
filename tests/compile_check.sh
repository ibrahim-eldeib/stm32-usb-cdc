#!/usr/bin/env bash
# SPDX-License-Identifier: MIT — Copyright (c) 2026 ibrahim-eldeib
# Compile src/usb_cdc.c against several STM32Cube HAL packages to catch
# family-specific API differences. Needs arm-none-eabi-gcc on PATH and the
# STM32Cube firmware packages under $CUBE_REPO (default ~/STM32Cube/Repository).
#
#   CUBE_REPO=~/STM32Cube/Repository tests/compile_check.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CUBE_REPO="${CUBE_REPO:-$HOME/STM32Cube/Repository}"
OUT="$(mktemp -d)"
fail=0

# family-prefix  package-glob             device-define   cpu flags
targets=(
  "h5 STM32Cube_FW_H5_*  STM32H523xx -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard"
  "g0 STM32Cube_FW_G0_*  STM32G0B1xx -mcpu=cortex-m0plus"
  "f4 STM32Cube_FW_F4_*  STM32F407xx -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
  "h7 STM32Cube_FW_H7_*  STM32H743xx -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard"
  "g4 STM32Cube_FW_G4_*  STM32G474xx -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
  "f1 STM32Cube_FW_F1_*  STM32F103xB -mcpu=cortex-m3"
  "l4 STM32Cube_FW_L4_*  STM32L476xx -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
)

for t in "${targets[@]}"; do
  read -r fam glob dev cpu <<<"$t"
  pkg="$(ls -d "$CUBE_REPO"/$glob 2>/dev/null | sort -V | tail -1)"
  if [ -z "$pkg" ]; then echo "[skip] $fam: no $glob in $CUBE_REPO"; continue; fi
  FAM="$(echo "$fam" | tr a-z A-Z)"
  hal="$pkg/Drivers/STM32${FAM}xx_HAL_Driver"
  inc="$OUT/$fam"; mkdir -p "$inc"
  cp "$hal/Inc/stm32${fam}xx_hal_conf_template.h" "$inc/stm32${fam}xx_hal_conf.h"
  printf '#include "stm32%sxx_hal.h"\n' "$fam" > "$inc/main.h"
  for mode in ring callback; do
    extra=""; [ "$mode" = callback ] && extra="-DUSB_CDC_RX_RING_SIZE=0"
    if arm-none-eabi-gcc -c -std=gnu11 -Wall -Wextra -Werror -O2 -mthumb $cpu -D$dev -DUSE_HAL_DRIVER $extra \
        -I"$inc" -I"$ROOT/include" -I"$hal/Inc" \
        -I"$pkg/Drivers/CMSIS/Device/ST/STM32${FAM}xx/Include" -I"$pkg/Drivers/CMSIS/Include" \
        "$ROOT/src/usb_cdc.c" -o "$OUT/$fam-$mode.o" 2>"$OUT/$fam-$mode.log"; then
      echo "[ok]   $fam ($dev, $(basename "$pkg"), rx=$mode)"
    else
      echo "[FAIL] $fam ($dev, rx=$mode)"; sed 's/^/       /' "$OUT/$fam-$mode.log" | head -20; fail=1
    fi
  done
done
rm -rf "$OUT"
exit $fail
