/* SPDX-License-Identifier: MIT — Copyright (c) 2026 ibrahim-eldeib */
/**
 * @file    usb_cdc_conf_template.h
 * @brief   Configuration for stm32-usb-cdc.
 *
 * Copy this file into your project as "usb_cdc_conf.h" (somewhere on the
 * include path) and edit it. Every option has a default in usb_cdc.c, so
 * an empty usb_cdc_conf.h is valid too.
 */
#ifndef USB_CDC_CONF_H
#define USB_CDC_CONF_H

/* Header that pulls in your family's HAL (stm32f4xx_hal.h, stm32h5xx_hal.h, ...).
 * Every STM32CubeMX project's main.h already does this. */
#define USB_CDC_HAL_HEADER          "main.h"

/* ---- USB identity ------------------------------------------------------- */
/* 0x0483:0x5740 is ST's "Virtual COM Port" ID. Fine for development and in-house
 * tools; a product you sell needs its own VID/PID. */
#define USB_CDC_VID                 0x0483U
#define USB_CDC_PID                 0x5740U
#define USB_CDC_BCD_DEVICE          0x0200U
#define USB_CDC_MANUFACTURER_STR    "STMicroelectronics"
#define USB_CDC_PRODUCT_STR         "STM32 Virtual COM Port"
#define USB_CDC_MAX_POWER_MA        100U        /* reported to the host, 2..500 */
#define USB_CDC_SELF_POWERED        0           /* 1 if the board has its own supply */

/* ---- Buffers ------------------------------------------------------------ */
#define USB_CDC_TX_RING_SIZE        2048U       /* bytes, power of two */

/* RX mode:
 *   > 0  : received bytes go into a ring of this size (power of two), read with
 *          USB_CDC_Read(). When the ring is full the device NAKs the host
 *          (real flow control, nothing is lost).
 *   0    : no ring; every byte is handed to USB_CDC_RxByteCallback() from the
 *          USB interrupt. Use this to feed an RTOS queue / line parser. */
#define USB_CDC_RX_RING_SIZE        512U

/* ---- Peripheral selection ------------------------------------------------
 * Auto-detected from the device header. Force one only if your part has both
 * kinds of USB peripheral and you use the other one:
 *   #define USB_CDC_FORCE_OTG       1   (USB_OTG_FS / USB_OTG_HS core)
 *   #define USB_CDC_FORCE_FSDEV     1   (USB / USB_DRD_FS "packet memory" core) */

/* ---- Packet-memory (PMA) layout, USB / USB_DRD_FS parts only --------------
 * Byte offset of the first endpoint buffer. The buffer-descriptor table sits
 * at the start of the PMA (8 bytes per endpoint), so 0x40 is always safe. */
#define USB_CDC_PMA_BASE            0x40U

/* ---- FIFO sizes in 32-bit words, USB_OTG_FS / USB_OTG_HS parts only -------
 * The total must fit the core's FIFO RAM (OTG_FS on F4 = 320 words). */
#define USB_CDC_OTG_RX_FIFO_WORDS   0x80U
#define USB_CDC_OTG_TX0_FIFO_WORDS  0x20U
#define USB_CDC_OTG_TX1_FIFO_WORDS  0x40U
#define USB_CDC_OTG_TX2_FIFO_WORDS  0x10U

#endif /* USB_CDC_CONF_H */
