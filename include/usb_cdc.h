/* SPDX-License-Identifier: MIT — Copyright (c) 2026 ibrahim-eldeib */
/**
 * @file    usb_cdc.h
 * @brief   stm32-usb-cdc — a small USB CDC-ACM (virtual COM port) device for
 *          any STM32 with a USB device peripheral, written directly on the
 *          STM32Cube HAL PCD driver. No USB_DEVICE middleware, USBX, ThreadX
 *          or TinyUSB needed.
 *
 * Works with the three STM32 USB device cores:
 *   - USB        (FS device, packet memory)   F0, F1, F3, L0, L1, L4x2, G4, WB ...
 *   - USB_DRD_FS (FS dual-role, packet memory) H5, G0, U5 (U535/545), C0 ...
 *   - USB_OTG_FS / USB_OTG_HS (FIFO core)     F2, F4, F7, H7, L4+, U5 (U575) ...
 *
 * Windows 10/11, Linux and macOS load their built-in CDC-ACM driver — no
 * .inf file, no ST VCP driver.
 */
#ifndef USB_CDC_H
#define USB_CDC_H

#if defined(__has_include)
#  if __has_include("usb_cdc_conf.h")
#    include "usb_cdc_conf.h"
#  endif
#else
#  include "usb_cdc_conf.h"
#endif

#ifndef USB_CDC_HAL_HEADER
#define USB_CDC_HAL_HEADER "main.h"
#endif
#include USB_CDC_HAL_HEADER

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** CDC line coding as set by the host (baud rate etc.). Informational for a
 *  virtual port; useful if you build a USB-to-UART bridge. */
typedef struct {
    uint32_t baud;
    uint8_t  stop_bits;   /* 0 = 1, 1 = 1.5, 2 = 2 */
    uint8_t  parity;      /* 0 none, 1 odd, 2 even, 3 mark, 4 space */
    uint8_t  data_bits;   /* 5, 6, 7, 8 or 16 */
} USB_CDC_LineCoding_t;

/**
 * Configure endpoint memory and connect to the bus (enables the D+ pull-up).
 * Call once, after the CubeMX-generated HAL_PCD_Init() (MX_USB_PCD_Init /
 * MX_USB_OTG_FS_PCD_Init ...). Safe to call before an RTOS scheduler starts.
 */
void USB_CDC_Init(PCD_HandleTypeDef *hpcd);

/** Disconnect from the bus (D+ pull-up off) and stop the peripheral. */
void USB_CDC_DeInit(void);

/**
 * Queue bytes for transmission. Never blocks; returns how many bytes were
 * accepted (less than len only when the TX ring is full). Data written before
 * the host opens the port stays queued, so early boot messages are not lost.
 * Safe from any task or main loop; do not call from inside a USB callback.
 */
uint32_t USB_CDC_Write(const void *data, uint32_t len);

/** Like USB_CDC_Write but waits (polling HAL_GetTick) until all bytes are
 *  queued or timeout_ms passes. Returns bytes queued. Not for ISRs. */
uint32_t USB_CDC_WriteBlocking(const void *data, uint32_t len, uint32_t timeout_ms);

/** Free space in the TX ring, in bytes. */
uint32_t USB_CDC_TxFree(void);

#if !defined(USB_CDC_RX_RING_SIZE) || (USB_CDC_RX_RING_SIZE > 0)
/** Bytes waiting in the RX ring. */
uint32_t USB_CDC_Available(void);
/** Copy up to len received bytes into buf; returns the number copied. */
uint32_t USB_CDC_Read(void *buf, uint32_t len);
#endif

/**
 * RX-callback mode only (USB_CDC_RX_RING_SIZE == 0): called for every
 * received byte from the USB interrupt. Weak, default does nothing.
 * Keep it short; with FreeRTOS use only ...FromISR() calls, and give the USB
 * IRQ a priority numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY.
 */
void USB_CDC_RxByteCallback(uint8_t byte);

/** Weak hook: called (from the USB interrupt) when the host changes the line
 *  coding or DTR/RTS. Default does nothing. */
void USB_CDC_LineStateCallback(const USB_CDC_LineCoding_t *coding, bool dtr, bool rts);

/** Weak hook: fill the USB serial-number string (max 32 chars, NUL-terminated).
 *  Default = the 96-bit unique device ID as 24 hex digits, read once in
 *  USB_CDC_Init (see README "unique ID and the instruction cache"). */
void USB_CDC_GetSerialString(char *out, uint32_t out_size);

/** true once the host has enumerated and configured the device. */
bool USB_CDC_IsConfigured(void);

/** true while a terminal program has the port open (host set DTR). Most
 *  terminals and pyserial set DTR on open; some programs don't. */
bool USB_CDC_IsPortOpen(void);

/** Current line coding as last set by the host. */
USB_CDC_LineCoding_t USB_CDC_GetLineCoding(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_H */
