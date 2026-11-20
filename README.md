# stm32-usb-cdc

A small USB virtual COM port (CDC-ACM) for any STM32 with a USB device
peripheral. It is two files (`usb_cdc.c`, `usb_cdc.h`) written directly on the
STM32Cube HAL PCD driver, so you don't need the CubeMX `USB_DEVICE` middleware, USBX,
ThreadX or TinyUSB. Windows 10/11, Linux and macOS use their built-in driver.

- **TX:** `USB_CDC_Write()` never blocks. It queues data in a ring buffer, and data queued before the host opens
  the port is kept, so boot messages aren't lost.
- **RX:** either a ring with `USB_CDC_Read()` and real flow control (the host is
  paused while the ring is full), or a per-byte callback from the interrupt, for an RTOS
  queue or line parser.
- **Thread-safe** writes from any task or the main loop. Works bare-metal and with FreeRTOS.
- Handles the STM32H5/U5 unique-ID + instruction-cache HardFault (see [the guide](docs/GUIDE.md#7-the-problems-we-hit-and-how-the-code-avoids-them)).

| USB core in the part | Families (examples) | Status |
|---|---|---|
| `USB_DRD_FS` (packet memory) | H5, G0, U5 (U535/545), C0 | **Running on hardware** (STM32H523, Windows 11): enumeration, 25 Hz JSON stream, commands. That board runs the project version this library was taken from; the library build itself is compiled and linked into the same firmware but not yet flashed. |
| `USB` (packet memory) | F0, F1, F3, L0, L1, L4x2, G4, WB | Compiles for the same code path; not yet tested on hardware |
| `USB_OTG_FS` / `USB_OTG_HS` (FIFO core) | F2, F4, F7, H7, L4+, U5 (U575) | Compiles with `-Wall -Wextra -Werror` against F4 and H7 HAL; not yet tested on hardware |

When you bring it up on a new family, run `tools/cdc_test.py --echo` and add the
result to this table.

## Quick start

1. **CubeMX:**
   - Enable the USB peripheral in **Device (FS)** mode.
   - Enable its interrupt in NVIC.
   - Give it an exact 48 MHz clock: HSI48 + CRS where the part has it, otherwise a PLL output from a crystal.
   - **Do not** enable the `USB_DEVICE` middleware.
   - On OTG parts, disable VBUS sensing unless PA9 is wired to VBUS.
2. Copy `src/usb_cdc.c` and `include/usb_cdc.h` into the project. If you want to change
   any option, also copy `include/usb_cdc_conf_template.h` as `usb_cdc_conf.h`.
3. Call `USB_CDC_Init()` once, after the generated USB init:

```c
#include "usb_cdc.h"

MX_USB_PCD_Init();                 /* generated: MX_USB_PCD_Init / MX_USB_OTG_FS_PCD_Init ... */
USB_CDC_Init(&hpcd_USB_DRD_FS);    /* the PCD handle CubeMX generated */

USB_CDC_Write("hello\r\n", 7);

uint8_t buf[64];
uint32_t n = USB_CDC_Read(buf, sizeof buf);   /* ring mode (default) */
```

4. Plug the board in. A COM port appears (`COMx` on Windows, `/dev/ttyACM0` on Linux,
   `/dev/cu.usbmodem*` on macOS). Then run `python tools/cdc_test.py`.

With FreeRTOS, see [`examples/freertos_line_commands.c`](examples/freertos_line_commands.c),
and call `USB_CDC_Init()` **before the first FreeRTOS call** (see the guide, §7.2).

## API

| Function | Notes |
|---|---|
| `void USB_CDC_Init(PCD_HandleTypeDef *hpcd)` | Sets up the endpoint memory and connects to the bus. Call once, after `HAL_PCD_Init`. |
| `void USB_CDC_DeInit(void)` | Disconnects from the bus. |
| `uint32_t USB_CDC_Write(const void *p, uint32_t n)` | Never blocks. Returns the number of bytes queued, which is less than `n` only when the TX ring is full. |
| `uint32_t USB_CDC_WriteBlocking(p, n, timeout_ms)` | Waits until all bytes are queued or the timeout passes. Not for ISRs. |
| `uint32_t USB_CDC_TxFree(void)` | Free space in the TX ring, in bytes. |
| `uint32_t USB_CDC_Available(void)` / `USB_CDC_Read(buf, n)` | Ring mode only. |
| `void USB_CDC_RxByteCallback(uint8_t b)` | Weak. Callback mode only (`USB_CDC_RX_RING_SIZE 0`). Runs in the USB interrupt. |
| `void USB_CDC_LineStateCallback(coding, dtr, rts)` | Weak. Called when the host sets the baud rate, DTR or RTS. |
| `void USB_CDC_GetSerialString(char *out, uint32_t size)` | Weak. The default is the chip's unique ID in hex. |
| `bool USB_CDC_IsConfigured(void)` | True once the host has enumerated the device. |
| `bool USB_CDC_IsPortOpen(void)` | True once the device is enumerated and a terminal has set DTR. |
| `USB_CDC_LineCoding_t USB_CDC_GetLineCoding(void)` | Baud rate, parity, stop bits and data bits as last set by the host. |

The options (VID/PID, strings, ring sizes, packet-memory and FIFO layout) are in
[`include/usb_cdc_conf_template.h`](include/usb_cdc_conf_template.h).

## Repository layout

```
include/usb_cdc.h                 public API
include/usb_cdc_conf_template.h   options (copy as usb_cdc_conf.h)
src/usb_cdc.c                     the driver
examples/bare_metal_echo.c        echo + printf retarget
examples/freertos_line_commands.c ISR line assembly -> queue -> task
tools/cdc_test.py                 find the port, monitor it, loopback test
tests/compile_check.sh            compile the driver against several Cube HAL packages
docs/GUIDE.md                     how it works, porting per family, troubleshooting
```

## Detailed guide

[docs/GUIDE.md](docs/GUIDE.md) covers:
- how USB enumeration and this driver work
- the CubeMX settings each family needs
- the bugs we hit while bringing it up on the STM32H5, and how to recognise each failure from the PC side

## License

MIT, see [LICENSE](LICENSE). The STM32Cube HAL that this driver builds on is ST's, under its own license (BSD-3-Clause), and is not included here.
