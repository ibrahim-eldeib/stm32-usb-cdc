# stm32-usb-cdc — detailed guide

This guide explains what the driver does, how to put it on any STM32, and how
to find out why a board doesn't show up as a COM port. Section 7 is the record
of the bugs we hit bringing it up on an STM32H523 camera-gimbal board. Most of
them will hit any project that adds USB to an STM32.

Contents
1. [Why a self-written driver](#1-why-a-self-written-driver)
2. [What happens when you plug the board in](#2-what-happens-when-you-plug-the-board-in)
3. [How the driver is built](#3-how-the-driver-is-built)
4. [Adding it to a project, step by step](#4-adding-it-to-a-project-step-by-step)
5. [Family-specific notes](#5-family-specific-notes)
6. [Using it with FreeRTOS](#6-using-it-with-freertos)
7. [The problems we hit and how the code avoids them](#7-the-problems-we-hit-and-how-the-code-avoids-them)
8. [Troubleshooting from the PC side](#8-troubleshooting-from-the-pc-side)
9. [Limits and possible extensions](#9-limits-and-possible-extensions)

---

## 1. Why a self-written driver

These are the usual ways to get a COM port on an STM32:

| Option | Problem |
|---|---|
| CubeMX `USB_DEVICE` middleware (classic ST USB Device Library) | Not offered for newer families (H5, U5 and others get USBX instead). It generates about 10 files that CubeMX regenerates, and its CDC class needs glue code anyway. |
| USBX (Azure RTOS) | Built for ThreadX. Standalone mode exists but is large. It doesn't fit a FreeRTOS project well. |
| TinyUSB | Good, but it's an external dependency with its own port layer and build integration. |
| **This driver** | About 600 lines on the HAL PCD driver that CubeMX already generates for every family. It drops into any project. |

The HAL **PCD** driver ("Peripheral Controller Driver") is the low layer every
ST USB stack sits on. It already handles the hardware: endpoint registers, packet memory or
FIFOs, and interrupts. It reports events through callbacks:
`HAL_PCD_ResetCallback`, `HAL_PCD_SetupStageCallback`,
`HAL_PCD_DataInStageCallback`, `HAL_PCD_DataOutStageCallback`. A USB device
"stack" is really just the code that answers those four events correctly.
For one fixed device type (CDC-ACM), that code is small.

## 2. What happens when you plug the board in

1. **Attach.** The device turns on its 1.5 kΩ pull-up on D+ (`HAL_PCD_Start` →
   `USB_DevConnect`). Without the pull-up the host sees *nothing*: no device and no error.
2. **Bus reset.** The host resets the bus. The HAL calls `HAL_PCD_ResetCallback`,
   and the driver opens endpoint 0 (control, 64 bytes).
3. **GET_DESCRIPTOR(device).** The host asks for 64 bytes. The device answers with 18 bytes:
   VID/PID, class 0x02 (CDC) and a 64-byte EP0 packet size.
4. **SET_ADDRESS(n).** The device must switch to address *n* only **after** the
   status stage of this request. On the packet-memory cores the HAL does that when the zero-length
   status packet completes. On OTG cores it writes the address immediately, which
   is what OTG needs. The driver just calls `HAL_PCD_SetAddress` and lets each core
   handle it.
5. **GET_DESCRIPTOR(configuration).** The configuration descriptor is 67 bytes, so the reply takes two EP0
   packets (64 + 3). The driver sends EP0 data one packet at a time itself, because
   not every core's HAL continues multi-packet EP0 transfers.
6. **GET_DESCRIPTOR(string)** for language, manufacturer, product and **serial number**.
   Windows uses the serial number to recognise the same board again (same COM number).
7. **SET_CONFIGURATION(1).** The driver opens the data endpoints (EP1 OUT/IN bulk 64 B,
   EP2 IN interrupt 8 B), starts receiving on EP1 OUT and starts sending any queued
   TX data. The device is now *configured*.
8. **CDC class requests,** when a program opens the COM port:
   - `SET_LINE_CODING`: 7 bytes of baud rate, stop bits, parity and data bits.
   - `GET_LINE_CODING`.
   - `SET_CONTROL_LINE_STATE`: DTR/RTS. The driver records DTR, which is `USB_CDC_IsPortOpen()`.

   A real UART bridge would use these settings. A virtual port only stores them and reports them back.

Every control transfer has up to three stages: **setup**, optional **data** (in or out),
and **status** (a zero-length packet in the opposite direction). The driver's EP0 code
is a small state machine over exactly these stages:

```
            SETUP
              │
   ┌──────────┼───────────────────────┐
   │ IN data  │ no data               │ OUT data (SET_LINE_CODING)
   ▼          ▼                       ▼
DATA_IN ──► STATUS_IN (send ZLP)    DATA_OUT ──► STATUS_IN (send ZLP)
   │  (64 B at a time,
   │   ZLP if the reply is shorter than asked
   │   and ends on a 64 B boundary)
   ▼
STATUS_OUT (receive ZLP) ──► IDLE
```

Anything the driver doesn't support (device qualifier, BOS, vendor requests, Microsoft
OS descriptors) is answered with a **STALL**. That is the correct answer for "not supported", and
hosts carry on without it.

## 3. How the driver is built

```
          task / main loop                         USB interrupt
 ┌──────────────────────────────┐      ┌──────────────────────────────────────┐
 │ USB_CDC_Write() ─► TX ring ──┼──────┼─► tx_kick(): ≤64 B into s_tx_pkt ──► EP1 IN
 │   (IRQs locked a few µs)     │      │   IN complete ─► tx_kick() again      │
 │                              │      │                                      │
 │ USB_CDC_Read() ◄─ RX ring ◄──┼──────┼── EP1 OUT complete ◄─ host           │
 │   (re-arms EP1 OUT if it     │      │   ring full? don't re-arm = NAK      │
 │    was paused)               │      │   (callback mode: per-byte callback) │
 └──────────────────────────────┘      │ EP0: state machine above             │
                                       └──────────────────────────────────────┘
```

- **One packet in flight per direction.** That is simple, and it is plenty for logs, telemetry and
  commands. The camera board streams about 5 KB/s of JSON (25 lines/s) without visible loss. Throughput has not been
  benchmarked yet; `tools/cdc_test.py --echo` prints it. Double-buffered endpoints would give more (§9).
- **Locking.** `USB_CDC_Write` and `USB_CDC_Read` save PRIMASK, disable interrupts,
  copy bytes and restore PRIMASK. This works on every Cortex-M, works before and after an RTOS
  scheduler starts, and doesn't care which tasks call it. The critical section is a
  memcpy of at most the caller's length plus one 64-byte packet hand-off.
- **Boot log.** `USB_CDC_Write` accepts data even before the host has configured the
  device. It stays in the TX ring and is sent the moment the host configures the device.
  That way the messages printed during startup are not lost.
- **The zero-length packet (ZLP) rule.** A bulk transfer ends with a packet shorter than 64 B. If the data ends
  exactly on a 64-byte boundary, the driver sends one ZLP so the host's read
  completes instead of waiting for more.
- **Peripheral differences** are confined to `USB_CDC_Init`:
  - Packet-memory cores (`USB`, `USB_DRD_FS`): `HAL_PCDEx_PMAConfig` gives each endpoint a buffer.
    The first 64 bytes of packet memory hold the buffer-descriptor table (8 bytes × 8 endpoints), so
    buffers start at `USB_CDC_PMA_BASE` = 0x40. The five buffers use 320 bytes, which fits even the
    512-byte packet memory of F1/F3.
  - OTG cores: `HAL_PCDEx_SetRxFiFo` and `HAL_PCDEx_SetTxFiFo`, in 32-bit words. The defaults
    (0x80 + 0x20 + 0x40 + 0x10 = 304 words) fit the 320-word OTG_FS of the F4.
    On OTG the core copies TX data into its FIFO later, from its own interrupt. That's
    why `s_tx_pkt` is never touched until the IN-complete callback.
- **Callbacks.** By default the driver defines the strong `HAL_PCD_*Callback`
  functions. If the HAL is configured with `USE_HAL_PCD_REGISTER_CALLBACKS 1`, it
  registers its handlers instead. Either way you don't touch the HAL.

## 4. Adding it to a project, step by step

1. **CubeMX, Connectivity → USB (or USB_OTG_FS / USB_DRD_FS):**
   - Mode = **Device_Only**.
   - VBUS sensing **disabled** unless the VBUS pin is wired (OTG parts).
   - DMA **disabled** (OTG HS/FS); the driver assumes no DMA.
   - Low power, LPM and battery charging: disabled.
2. **CubeMX, Middleware:** leave **USB_DEVICE / USBX unset**. If it is already in the project,
   remove it; its `usbd_conf.c` defines the same HAL callbacks, so the link fails with
   *multiple definition of HAL_PCD_SetupStageCallback*.
3. **NVIC:** enable the USB interrupt. Check that `stm32xxxx_it.c` has the handler calling
   `HAL_PCD_IRQHandler(&hpcd_...)`.
4. **Clock:** the USB clock must be **48 MHz within ±0.25 %**:
   - Parts with HSI48: select HSI48 for USB and enable **CRS** synchronised to USB SOF.
     CubeMX: RCC → CRS SYNC = USB. This needs no crystal.
   - Others: use a crystal and a PLL output of exactly 48 MHz (F1: 72 MHz ÷ 1.5; F4: PLLQ).
   - A wrong USB clock looks like "Device Descriptor Request Failed" or random disconnects.
5. **Supply:** on parts with a separate VDDUSB pin (L4, L5, U5, H5, G0B/C, H7), it must be
   powered, and `HAL_PWREx_EnableVddUSB()` called. CubeMX puts that in `HAL_PCD_MspInit`.
   H7 uses `HAL_PWREx_EnableUSBVoltageDetector()` instead.
6. **Files:** add `usb_cdc.c` to the build and `usb_cdc.h` (+ optional
   `usb_cdc_conf.h`) to an include folder.
   - STM32CubeIDE picks up new files in `Core/Src` by itself.
   - With a hand-kept makefile, add the file to `C_SRCS`/`OBJS` (and `objects.list` for CubeIDE-generated makefiles).
7. **Code:** call `USB_CDC_Init(&hpcd_xxx)` once, after `MX_..._PCD_Init()`.
   Call it before any long startup delays, so a hang later in startup can't stop the board from enumerating.
8. **Check on the PC:** `python tools/cdc_test.py`. For a new family, flash
   `examples/bare_metal_echo.c` and run `python tools/cdc_test.py --echo`.

## 5. Family-specific notes

| Family | Notes |
|---|---|
| **H5, U5, L5** | ICACHE + unique ID → HardFault (§7.3); the driver handles it. VDDUSB must be enabled. H5 has HSI48 + CRS. |
| **G0** (G0B1/G0C1) | `USB_DRD_FS` core. The IRQ is shared (`USB_UCPD1_2_IRQn`). HSI48 + CRS. VDDUSB on some packages. |
| **F0** (F04x/F07x) | On small packages PA11/PA12 are remapped: `__HAL_REMAP_PIN_ENABLE(HAL_REMAP_PA11_PA12)`. HSI48 + CRS. |
| **F1** (F102/F103) | **No internal D+ pull-up.** The board needs a 1.5 kΩ resistor from D+ to 3.3 V, fixed or switched by a GPIO. Some cheap boards fit 10 kΩ and are unreliable. For a GPIO-switched pull-up, implement `HAL_PCDEx_SetConnectionState()`. The USB clock comes from the PLL (72 MHz ÷ 1.5). The IRQ is `USB_LP_CAN1_RX0_IRQn`, shared with CAN. |
| **F3, G4** | `USB` core, `USB_LP_IRQn`. G4 has HSI48 + CRS; F3 needs a crystal. |
| **L0, L4x2** | `USB` core, HSI48 + CRS. On L4, set VDDUSB. |
| **F2, F4, F7** | OTG_FS (and OTG_HS with its internal FS PHY). Disable VBUS sensing if PA9 isn't wired: `Init.vbus_sensing_enable = DISABLE`. Some old F4 HAL versions also need `USB_OTG_FS->GCCFG \|= USB_OTG_GCCFG_NOVBUSSENS`. 48 MHz from PLLQ. |
| **H7** | OTG_HS or OTG_FS in FS mode with the embedded PHY. Use `HAL_PWREx_EnableUSBVoltageDetector()`. No DMA (the driver's buffers may be in DTCM). HSI48 + CRS available. |

## 6. Using it with FreeRTOS

- **Interrupt priority:**
  - Callback mode calls your code from the USB interrupt. If that code uses
    `xQueueSendFromISR` or similar, the USB IRQ priority must be numerically ≥
    `configMAX_SYSCALL_INTERRUPT_PRIORITY`. Priority 5 with the default CubeMX FreeRTOS config is fine.
  - Ring mode doesn't call any RTOS function in the interrupt.
- **When to call `USB_CDC_Init`:** see §7.2. In short, call it before the first FreeRTOS call,
  or immediately before `vTaskStartScheduler()`.
- **Writing from tasks:** `USB_CDC_Write` is safe from any task and never blocks, so
  it is safe in control loops. If the ring is full, the extra bytes are dropped and
  the return value says how many were taken.
- **Commands:** the `freertos_line_commands.c` example assembles lines in the
  interrupt and sends complete lines through a queue, the same way as a UART
  RX interrupt. On the camera board, USB lines go into the same queue as the UART
  commands, so every command works over both.

## 7. The problems we hit and how the code avoids them

These are from adding USB to the STM32H523 board. Each one shows up on the
PC in a recognisable way (§8).

### 7.1 "USB cable connected, nothing appears on the PC"

**Cause:** the firmware initialised the USB peripheral (`MX_USB_PCD_Init` was
generated by CubeMX) but had no USB device code and never called `HAL_PCD_Start`.
With no D+ pull-up, Windows saw nothing at all, not even an unknown device. Only
DFU mode worked, because that is the ROM bootloader's own USB code.

**Fix:** a USB device implementation (this driver), with `HAL_PCD_Start` called
from `USB_CDC_Init`.

**Lesson:** CubeMX generating `MX_USB_PCD_Init()` doesn't mean the board does
anything on USB. Without a class driver and `HAL_PCD_Start`, it is electrically
invisible.

### 7.2 FreeRTOS masks the USB interrupt until the scheduler starts

FreeRTOS creation calls (`xQueueCreate`, `xSemaphoreCreateMutex`, `xTaskCreate`) use
critical sections. Before the scheduler runs, the critical-nesting counter starts at
a non-zero sentinel value. So the **exit** of those critical sections doesn't lower
BASEPRI again. From the first such call until `vTaskStartScheduler()`, every interrupt at
or below `configMAX_SYSCALL_INTERRUPT_PRIORITY` is masked, including USB (and the UARTs).

If USB is connected and the code then spends seconds there (sensor init with `HAL_Delay`),
the host gets no answers to its enumeration requests. It reports *Device Descriptor
Request Failed* or gives up.

**Fix:** call `USB_CDC_Init()` either before the first FreeRTOS call, where interrupts
still run during `HAL_Delay` startup code, or right before `vTaskStartScheduler()`. The
camera board does the first, right after the peripheral inits.

### 7.3 HardFault on STM32H5 when the PC asks for the serial number

**Symptom:** the board appeared on the PC for less than a second and vanished.
`LastArrivalDate` and `LastRemovalDate` in Windows were the same second, and the event log
said *"requires further installation"*. The board's LED never showed the startup pattern.
It looked like a hang during startup.

**Cause:** the serial-number string was built from the chip's unique ID (`HAL_GetUIDw0..2`,
at `UID_BASE` = 0x08FFF800 on H5). That address is in the **read-only system-flash
area**. On STM32H5 (and U5/L5-type parts with an ICACHE), reading that area while the ICACHE
is enabled causes a **bus fault**, which ends in HardFault. ST's own H5 examples set an MPU
region to non-cacheable before reading data in that area. The read happened inside the USB
interrupt at the exact moment Windows requested string descriptor 3. So the device got
through the device descriptor and died one request later.

**Fix in the driver:** `USB_CDC_GetSerialString()` runs **once in `USB_CDC_Init`**, with
`ICACHE->CR.EN` cleared around the three reads and restored afterwards. The USB interrupt
only ever copies the stored string. It is a weak function, so a project can supply a fixed
serial number instead.

**Lesson:** on any part with ICACHE, the UID, flash-size and package registers, OTP, and
factory calibration values must be read with the cache off or through a non-cacheable MPU region.
This applies to all your code, not just USB.

### 7.4 Messages printed during startup were lost

Before the host configures the device, there is nowhere to send data. The first version
dropped writes until then, so the boot log (sensor init results) never reached the PC.
**Fix:** writes are always queued, and the queue is sent on `SET_CONFIGURATION`. The ring
(4 KB on the camera board) holds the whole boot log.

### 7.5 "Start after flashing" is not a reset

`STM32_Programmer_CLI … -s 0x08000000` (or "Run after programming" in DFU mode) jumps from
the ROM bootloader into the application **without a reset**. Clocks, the PLL, interrupt
state and the USB peripheral are left as the bootloader set them. Some applications then fail in
`SystemClock_Config` or behave differently.

**Lesson:** judge a USB firmware only after a real reset, with BOOT0 released and the reset
pressed or power cycled.

### 7.6 Opening the port right after closing it can fail once (Windows)

`PermissionError(13, 'A device attached to the system is not functioning')` came up once, when a
script opened the port immediately after another one closed it. A retry a second later
worked. `tools/cdc_test.py` retries automatically. Keep one program on the port at a time.

## 8. Troubleshooting from the PC side

What the PC shows tells you how far the device got:

| What you see | How far it got | Look at |
|---|---|---|
| **Nothing at all** (no device, no error) | Never turned on the D+ pull-up | Firmware doesn't reach `USB_CDC_Init` (stuck or HardFault earlier, watchdog loop). F1 has no pull-up resistor. VDDUSB not enabled. USB pins not connected. |
| **"Unknown USB Device (Device Descriptor Request Failed)"** | Pull-up on, but no answer to the first request | USB IRQ not enabled, handler missing, or masked (§7.2). USB clock not 48 MHz. Callbacks not linked (check that `HAL_PCD_SetupStageCallback` is a `T` symbol, not `W`, in `arm-none-eabi-nm`). |
| **Appears for a moment, then disappears** | Crashed in the middle of enumeration | A fault inside a USB callback (§7.3), a watchdog reset, or a brown-out when USB power is drawn. |
| **"Device Descriptor Request Failed" after a while / "Port reset failed"** | Unstable | USB clock tolerance (no CRS, wrong PLL), cable or hub, a missing 1.5 kΩ resistor (F1). |
| **COM port present, no data** | Configured, nothing sent | Nothing calls `USB_CDC_Write`; the program on the PC doesn't set DTR (if you gate output on `USB_CDC_IsPortOpen`); another program holds the port. |
| **Data arrives, but in bursts or late** | Working | The output ended on a 64-byte boundary without a ZLP (the driver handles this), or the PC program reads in large blocks. |

**Windows (PowerShell):**

```powershell
# Everything Windows has seen with ST's VID, or unknown devices; also devices no longer present
Get-PnpDevice | Where-Object { $_.InstanceId -match 'VID_0483|VID_0000' } |
  ForEach-Object {
    $a = (Get-PnpDeviceProperty -InstanceId $_.InstanceId -KeyName DEVPKEY_Device_LastArrivalDate).Data
    $r = (Get-PnpDeviceProperty -InstanceId $_.InstanceId -KeyName DEVPKEY_Device_LastRemovalDate -ErrorAction SilentlyContinue).Data
    "{0,-8} {1}  arrived={2} removed={3}" -f $_.Status, $_.InstanceId, $a, $r }

# Recent plug-and-play events for the device
Get-WinEvent -LogName 'Microsoft-Windows-Kernel-PnP/Configuration' -MaxEvents 30 |
  Where-Object Message -match '0483' | Format-List TimeCreated, Message
```

- **Arrival and removal in the same second** means the firmware crashed during enumeration (§7.3).
- **An instance ID that ends in a location** (`…\5&2F137D3B&0&4`) instead of the serial number means
  Windows never got the serial string.
- `python -m serial.tools.list_ports -v` lists COM ports with VID:PID and serial number.

**Linux:** `dmesg -w` while plugging in, `lsusb -v -d 0483:5740`, and the port is `/dev/ttyACM*`.
**ModemManager** may grab new ACM ports for a few seconds and send AT commands. Stop it, or add a
udev rule with `ENV{ID_MM_DEVICE_IGNORE}="1"` for your VID:PID.

**macOS:** `ls /dev/cu.usbmodem*`; `ioreg -p IOUSB -l` shows the descriptors.

**On the device:** if you have UART or SWD, set breakpoints or print from `on_reset`,
`on_setup` (the 8 setup bytes) and `on_data_in`. A healthy enumeration is:
reset → GET_DESCRIPTOR(dev) → reset → SET_ADDRESS → GET_DESCRIPTOR(dev, cfg, strings) →
SET_CONFIGURATION → SET_LINE_CODING / SET_CONTROL_LINE_STATE when a terminal opens the port.

## 9. Limits and possible extensions

- **Full speed only** (12 Mbit/s), 64-byte packets, one packet in flight per direction. That is
  well below the bus limit (about 1 MB/s for full-speed bulk). Not yet benchmarked. If you need more,
  use double-buffered bulk endpoints (packet-memory cores) or larger transfers per IN (OTG cores).
- **One CDC function.** A composite device (two COM ports, CDC + HID) needs an Interface
  Association Descriptor and more endpoints.
- **No suspend or remote wakeup** handling. Bus-powered devices that must meet the 2.5 mA suspend
  current need `HAL_PCD_SuspendCallback`/`ResumeCallback` handling.
- **VID/PID 0x0483:0x5740** belongs to ST. Fine for development and in-house tools. A product
  you sell needs its own VID/PID.
- **No DMA** on OTG cores.
