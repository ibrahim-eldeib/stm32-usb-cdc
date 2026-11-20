/* SPDX-License-Identifier: MIT — Copyright (c) 2026 ibrahim-eldeib */
/**
 * @file    usb_cdc.c
 * @brief   stm32-usb-cdc — USB CDC-ACM device on the STM32Cube HAL PCD driver.
 *
 * Endpoints
 *   EP0        control, 64 B
 *   EP1 OUT    bulk, host -> device, 64 B
 *   EP1 IN     bulk, device -> host, 64 B
 *   EP2 IN     interrupt, CDC notifications, 8 B (opened, never sent on)
 *
 * TX : USB_CDC_Write() -> ring -> 64-byte packets on EP1 IN (refilled from the
 *      IN-complete interrupt; a zero-length packet ends a run that stops on a
 *      64-byte boundary).
 * RX : EP1 OUT packets -> RX ring (USB_CDC_Read, NAK flow control when full)
 *      or, with USB_CDC_RX_RING_SIZE == 0, -> USB_CDC_RxByteCallback().
 *
 * See README.md for the porting guide and the problems this code avoids.
 */

#include "usb_cdc.h"
#include <string.h>

/* ========================================================================== */
/*  Defaults for options not set in usb_cdc_conf.h                            */
/* ========================================================================== */
#ifndef USB_CDC_VID
#define USB_CDC_VID                 0x0483U
#endif
#ifndef USB_CDC_PID
#define USB_CDC_PID                 0x5740U
#endif
#ifndef USB_CDC_BCD_DEVICE
#define USB_CDC_BCD_DEVICE          0x0200U
#endif
#ifndef USB_CDC_MANUFACTURER_STR
#define USB_CDC_MANUFACTURER_STR    "STMicroelectronics"
#endif
#ifndef USB_CDC_PRODUCT_STR
#define USB_CDC_PRODUCT_STR         "STM32 Virtual COM Port"
#endif
#ifndef USB_CDC_MAX_POWER_MA
#define USB_CDC_MAX_POWER_MA        100U
#endif
#ifndef USB_CDC_SELF_POWERED
#define USB_CDC_SELF_POWERED        0
#endif
#ifndef USB_CDC_TX_RING_SIZE
#define USB_CDC_TX_RING_SIZE        2048U
#endif
#ifndef USB_CDC_RX_RING_SIZE
#define USB_CDC_RX_RING_SIZE        512U
#endif
#ifndef USB_CDC_PMA_BASE
#define USB_CDC_PMA_BASE            0x40U
#endif
#ifndef USB_CDC_OTG_RX_FIFO_WORDS
#define USB_CDC_OTG_RX_FIFO_WORDS   0x80U
#endif
#ifndef USB_CDC_OTG_TX0_FIFO_WORDS
#define USB_CDC_OTG_TX0_FIFO_WORDS  0x20U
#endif
#ifndef USB_CDC_OTG_TX1_FIFO_WORDS
#define USB_CDC_OTG_TX1_FIFO_WORDS  0x40U
#endif
#ifndef USB_CDC_OTG_TX2_FIFO_WORDS
#define USB_CDC_OTG_TX2_FIFO_WORDS  0x10U
#endif

#if (USB_CDC_TX_RING_SIZE & (USB_CDC_TX_RING_SIZE - 1U)) != 0U
#error "USB_CDC_TX_RING_SIZE must be a power of two"
#endif
#if (USB_CDC_RX_RING_SIZE > 0U) && ((USB_CDC_RX_RING_SIZE & (USB_CDC_RX_RING_SIZE - 1U)) != 0U)
#error "USB_CDC_RX_RING_SIZE must be 0 or a power of two"
#endif
#if (USB_CDC_RX_RING_SIZE > 0U) && (USB_CDC_RX_RING_SIZE < 128U)
#error "USB_CDC_RX_RING_SIZE must be >= 128 (two packets) or 0"
#endif

/* ---- Which USB core? ------------------------------------------------------ */
#if defined(USB_CDC_FORCE_OTG) && USB_CDC_FORCE_OTG
#  define USB_CDC_CORE_OTG   1
#elif defined(USB_CDC_FORCE_FSDEV) && USB_CDC_FORCE_FSDEV
#  define USB_CDC_CORE_OTG   0
#elif defined(USB_OTG_FS) || defined(USB_OTG_HS)
#  define USB_CDC_CORE_OTG   1
#elif defined(USB_DRD_FS) || defined(USB)
#  define USB_CDC_CORE_OTG   0
#else
#  error "stm32-usb-cdc: this device has no USB device peripheral (or the HAL header was not included)"
#endif

#if !defined(HAL_PCD_MODULE_ENABLED)
#error "stm32-usb-cdc: enable the PCD HAL module (CubeMX: USB peripheral in Device mode)"
#endif

/* ========================================================================== */
/*  Descriptors                                                               */
/* ========================================================================== */
#define EP0_MPS        64U
#define DATA_MPS       64U
#define CMD_MPS        8U
#define EP_DATA_OUT    0x01U
#define EP_DATA_IN     0x81U
#define EP_CMD_IN      0x82U

static const uint8_t dev_desc[18] = {
    18, 0x01,                                   /* DEVICE                      */
    0x00, 0x02,                                 /* USB 2.00                    */
    0x02, 0x00, 0x00,                           /* class CDC                   */
    EP0_MPS,
    (uint8_t)(USB_CDC_VID & 0xFFU), (uint8_t)(USB_CDC_VID >> 8),
    (uint8_t)(USB_CDC_PID & 0xFFU), (uint8_t)(USB_CDC_PID >> 8),
    (uint8_t)(USB_CDC_BCD_DEVICE & 0xFFU), (uint8_t)(USB_CDC_BCD_DEVICE >> 8),
    1, 2, 3,                                    /* manufacturer, product, serial */
    1                                           /* configurations              */
};

#define CFG_LEN 67U
static const uint8_t cfg_desc[CFG_LEN] = {
    9, 0x02, CFG_LEN, 0x00, 2, 1, 0,
    (uint8_t)(0x80U | (USB_CDC_SELF_POWERED ? 0x40U : 0x00U)),
    (uint8_t)(USB_CDC_MAX_POWER_MA / 2U),
    /* Interface 0 — CDC communication (ACM) */
    9, 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    5, 0x24, 0x00, 0x10, 0x01,                  /* header, CDC 1.10            */
    5, 0x24, 0x01, 0x00, 1,                     /* call management             */
    4, 0x24, 0x02, 0x02,                        /* ACM: line coding + state    */
    5, 0x24, 0x06, 0, 1,                        /* union: 0 controls 1         */
    7, 0x05, EP_CMD_IN, 0x03, CMD_MPS, 0x00, 16,
    /* Interface 1 — CDC data */
    9, 0x04, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    7, 0x05, EP_DATA_OUT, 0x02, DATA_MPS, 0x00, 0,
    7, 0x05, EP_DATA_IN,  0x02, DATA_MPS, 0x00, 0,
};

/* ========================================================================== */
/*  State                                                                     */
/* ========================================================================== */
typedef enum { EP0_IDLE, EP0_DATA_IN, EP0_STATUS_IN, EP0_DATA_OUT, EP0_STATUS_OUT } ep0_state_t;

static PCD_HandleTypeDef *s_pcd;
static volatile bool      s_configured;
static uint8_t            s_config_value;
static volatile bool      s_dtr, s_rts;
static char               s_serial[33];

static ep0_state_t        s_ep0_state;
static uint8_t            s_ep0_buf[EP0_MPS];
static uint8_t            s_str_buf[2 + 2 * 32];
static const uint8_t     *s_ep0_tx_ptr;
static uint16_t           s_ep0_tx_rem;
static bool               s_ep0_tx_zlp;
static uint8_t            s_ep0_out_req;

/* 115200 8N1 until the host says otherwise */
static uint8_t s_line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };

static uint8_t            s_tx_pkt[DATA_MPS];
static uint8_t            s_tx_ring[USB_CDC_TX_RING_SIZE];
static volatile uint32_t  s_tx_head, s_tx_tail;
static volatile bool      s_tx_busy;
static bool               s_tx_zlp;

static uint8_t            s_rx_pkt[DATA_MPS];
#if USB_CDC_RX_RING_SIZE > 0U
static uint8_t            s_rx_ring[USB_CDC_RX_RING_SIZE];
static volatile uint32_t  s_rx_head, s_rx_tail;
static volatile bool      s_rx_armed;
#endif

#define TX_MASK (USB_CDC_TX_RING_SIZE - 1U)
#define RX_MASK (USB_CDC_RX_RING_SIZE - 1U)

/* ========================================================================== */
/*  Small helpers                                                             */
/* ========================================================================== */
static inline uint32_t irq_lock(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    return pm;
}
static inline void irq_unlock(uint32_t pm) { __set_PRIMASK(pm); }

__weak void USB_CDC_RxByteCallback(uint8_t byte) { (void)byte; }
__weak void USB_CDC_LineStateCallback(const USB_CDC_LineCoding_t *c, bool dtr, bool rts)
{
    (void)c; (void)dtr; (void)rts;
}

/*
 * Default serial number = unique device ID (96 bit) as 24 hex digits.
 *
 * IMPORTANT (STM32H5 / U5 / L5 / ...): the unique ID sits in a read-only
 * system flash area that must not be read through the enabled ICACHE — the
 * access bus-faults and the MCU ends in HardFault. So the cache is switched
 * off around the read, and the read happens once here at init, never inside
 * the USB interrupt (where the host's GET_DESCRIPTOR(serial) would trigger it).
 */
__weak void USB_CDC_GetSerialString(char *out, uint32_t out_size)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t uid[3];

#if defined(ICACHE) && defined(ICACHE_CR_EN)
    uint32_t icache_on = ICACHE->CR & ICACHE_CR_EN;
    if (icache_on)
    {
        ICACHE->CR &= ~ICACHE_CR_EN;
        while ((ICACHE->CR & ICACHE_CR_EN) != 0U) { }
    }
#endif
    uid[0] = *(const volatile uint32_t *)(UID_BASE + 0U);
    uid[1] = *(const volatile uint32_t *)(UID_BASE + 4U);
    uid[2] = *(const volatile uint32_t *)(UID_BASE + 8U);
#if defined(ICACHE) && defined(ICACHE_CR_EN)
    if (icache_on) ICACHE->CR |= ICACHE_CR_EN;
#endif

    uint32_t n = 0;
    for (int w = 0; w < 3; w++)
        for (int b = 0; b < 8 && n + 1U < out_size; b++)
            out[n++] = hex[(uid[w] >> (28 - 4 * b)) & 0xFU];
    if (out_size) out[n] = '\0';
}

static USB_CDC_LineCoding_t decode_line_coding(void)
{
    USB_CDC_LineCoding_t c;
    c.baud      = (uint32_t)s_line_coding[0] | ((uint32_t)s_line_coding[1] << 8) |
                  ((uint32_t)s_line_coding[2] << 16) | ((uint32_t)s_line_coding[3] << 24);
    c.stop_bits = s_line_coding[4];
    c.parity    = s_line_coding[5];
    c.data_bits = s_line_coding[6];
    return c;
}

/* ========================================================================== */
/*  EP0 (control) state machine                                               */
/* ========================================================================== */
static void ep0_stall(void)
{
    HAL_PCD_EP_SetStall(s_pcd, 0x80);
    HAL_PCD_EP_SetStall(s_pcd, 0x00);
    s_ep0_state = EP0_IDLE;
}

static void ep0_status_in(void)
{
    s_ep0_state = EP0_STATUS_IN;
    HAL_PCD_EP_Transmit(s_pcd, 0x80, NULL, 0);
}

/* EP0 IN data is always sent one packet at a time from here: the HAL does not
 * continue multi-packet EP0 transfers on every core. */
static void ep0_in_next(void)
{
    if (s_ep0_tx_rem > 0U)
    {
        uint16_t n = (s_ep0_tx_rem > EP0_MPS) ? (uint16_t)EP0_MPS : s_ep0_tx_rem;
        const uint8_t *p = s_ep0_tx_ptr;
        s_ep0_tx_ptr += n;
        s_ep0_tx_rem -= n;
        HAL_PCD_EP_Transmit(s_pcd, 0x80, (uint8_t *)p, n);
    }
    else if (s_ep0_tx_zlp)
    {
        s_ep0_tx_zlp = false;
        HAL_PCD_EP_Transmit(s_pcd, 0x80, NULL, 0);
    }
    else
    {
        s_ep0_state = EP0_STATUS_OUT;
        HAL_PCD_EP_Receive(s_pcd, 0x00, NULL, 0);
    }
}

static void ep0_send(const uint8_t *data, uint16_t len, uint16_t wLength)
{
    if (len > wLength) len = wLength;
    s_ep0_tx_ptr = data;
    s_ep0_tx_rem = len;
    /* short reply that ends exactly on a packet boundary needs a ZLP */
    s_ep0_tx_zlp = (len < wLength) && ((len % EP0_MPS) == 0U);
    s_ep0_state  = EP0_DATA_IN;
    ep0_in_next();
}

static uint16_t string_desc(uint8_t idx)
{
    const char *s;
    switch (idx)
    {
    case 0:  s_str_buf[0] = 4; s_str_buf[1] = 0x03; s_str_buf[2] = 0x09; s_str_buf[3] = 0x04; return 4;
    case 1:  s = USB_CDC_MANUFACTURER_STR; break;
    case 2:  s = USB_CDC_PRODUCT_STR;      break;
    case 3:  s = s_serial;                 break;   /* captured at init, never read UID here */
    default: return 0;
    }
    uint16_t n = 0;
    while (s[n] != '\0' && n < 32U)
    {
        s_str_buf[2 + 2 * n]     = (uint8_t)s[n];
        s_str_buf[2 + 2 * n + 1] = 0;
        n++;
    }
    s_str_buf[0] = (uint8_t)(2U + 2U * n);
    s_str_buf[1] = 0x03;
    return (uint16_t)(2U + 2U * n);
}

/* ========================================================================== */
/*  Data endpoints                                                            */
/* ========================================================================== */
/* Start the next IN packet if idle. Call with interrupts locked or from the
 * USB interrupt. */
static void tx_kick(void)
{
    if (!s_configured || s_tx_busy) return;

    uint32_t avail = (s_tx_head - s_tx_tail) & TX_MASK;
    if (avail == 0U)
    {
        if (s_tx_zlp)
        {
            s_tx_zlp  = false;
            s_tx_busy = true;
            HAL_PCD_EP_Transmit(s_pcd, EP_DATA_IN, NULL, 0);
        }
        return;
    }
    uint32_t n = (avail > DATA_MPS) ? DATA_MPS : avail;
    for (uint32_t i = 0; i < n; i++)
        s_tx_pkt[i] = s_tx_ring[(s_tx_tail + i) & TX_MASK];
    s_tx_tail = (s_tx_tail + n) & TX_MASK;
    s_tx_zlp  = (n == DATA_MPS);
    s_tx_busy = true;
    /* s_tx_pkt stays untouched until the IN-complete callback, which matters on
     * the OTG core: it copies into the FIFO later, from its own interrupt. */
    HAL_PCD_EP_Transmit(s_pcd, EP_DATA_IN, s_tx_pkt, n);
}

static void rx_arm(void)
{
    HAL_PCD_EP_Receive(s_pcd, EP_DATA_OUT, s_rx_pkt, DATA_MPS);
}

static void open_data_eps(void)
{
    HAL_PCD_EP_Open(s_pcd, EP_DATA_IN,  DATA_MPS, EP_TYPE_BULK);
    HAL_PCD_EP_Open(s_pcd, EP_DATA_OUT, DATA_MPS, EP_TYPE_BULK);
    HAL_PCD_EP_Open(s_pcd, EP_CMD_IN,   CMD_MPS,  EP_TYPE_INTR);
}

static void close_data_eps(void)
{
    HAL_PCD_EP_Close(s_pcd, EP_DATA_IN);
    HAL_PCD_EP_Close(s_pcd, EP_DATA_OUT);
    HAL_PCD_EP_Close(s_pcd, EP_CMD_IN);
}

/* ========================================================================== */
/*  PCD event handlers                                                        */
/* ========================================================================== */
static void on_reset(PCD_HandleTypeDef *hpcd)
{
    if (s_configured) close_data_eps();
    s_configured   = false;
    s_config_value = 0;
    s_dtr = s_rts  = false;
    s_tx_busy      = false;
    s_tx_zlp       = false;
    s_ep0_state    = EP0_IDLE;
#if USB_CDC_RX_RING_SIZE > 0U
    s_rx_armed     = false;
#endif
    HAL_PCD_EP_Open(hpcd, 0x00, EP0_MPS, EP_TYPE_CTRL);
    HAL_PCD_EP_Open(hpcd, 0x80, EP0_MPS, EP_TYPE_CTRL);
}

static void on_setup(PCD_HandleTypeDef *hpcd)
{
    const uint8_t *s = (const uint8_t *)hpcd->Setup;
    uint8_t  bmRequestType = s[0];
    uint8_t  bRequest      = s[1];
    uint16_t wValue        = (uint16_t)(s[2] | (s[3] << 8));
    uint16_t wIndex        = (uint16_t)(s[4] | (s[5] << 8));
    uint16_t wLength       = (uint16_t)(s[6] | (s[7] << 8));
    uint8_t  type          = (uint8_t)((bmRequestType >> 5) & 0x03U);

    s_ep0_state = EP0_IDLE;

    if (type == 0U)                                  /* standard requests */
    {
        switch (bRequest)
        {
        case 0x00: /* GET_STATUS */
            s_ep0_buf[0] = ((bmRequestType & 0x1FU) == 0U && USB_CDC_SELF_POWERED) ? 1U : 0U;
            s_ep0_buf[1] = 0;
            ep0_send(s_ep0_buf, 2, wLength);
            return;
        case 0x01: /* CLEAR_FEATURE */
            if ((bmRequestType & 0x1FU) == 0x02U && wValue == 0U)       /* ENDPOINT_HALT */
                HAL_PCD_EP_ClrStall(hpcd, (uint8_t)(wIndex & 0xFFU));
            ep0_status_in();
            return;
        case 0x03: /* SET_FEATURE */
            if ((bmRequestType & 0x1FU) == 0x02U && wValue == 0U && (wIndex & 0x7FU) != 0U)
                HAL_PCD_EP_SetStall(hpcd, (uint8_t)(wIndex & 0xFFU));
            ep0_status_in();
            return;
        case 0x05: /* SET_ADDRESS — the HAL applies it at the right moment for each core */
            HAL_PCD_SetAddress(hpcd, (uint8_t)(wValue & 0x7FU));
            ep0_status_in();
            return;
        case 0x06: /* GET_DESCRIPTOR */
        {
            uint8_t dtype = (uint8_t)(wValue >> 8);
            if (dtype == 0x01U) { ep0_send(dev_desc, sizeof(dev_desc), wLength); return; }
            if (dtype == 0x02U) { ep0_send(cfg_desc, sizeof(cfg_desc), wLength); return; }
            if (dtype == 0x03U)
            {
                uint16_t n = string_desc((uint8_t)(wValue & 0xFFU));
                if (n) { ep0_send(s_str_buf, n, wLength); return; }
            }
            ep0_stall();   /* device qualifier, BOS, other strings: not supported */
            return;
        }
        case 0x08: /* GET_CONFIGURATION */
            s_ep0_buf[0] = s_config_value;
            ep0_send(s_ep0_buf, 1, wLength);
            return;
        case 0x09: /* SET_CONFIGURATION */
            if ((wValue & 0xFFU) == 1U)
            {
                if (!s_configured)
                {
                    open_data_eps();
#if USB_CDC_RX_RING_SIZE > 0U
                    s_rx_armed = true;
#endif
                    rx_arm();
                }
                s_config_value = 1;
                s_tx_busy      = false;
                s_tx_zlp       = false;
                s_configured   = true;
                tx_kick();                       /* send what was queued before */
            }
            else if ((wValue & 0xFFU) == 0U)
            {
                if (s_configured) close_data_eps();
                s_configured   = false;
                s_config_value = 0;
            }
            else { ep0_stall(); return; }
            ep0_status_in();
            return;
        case 0x0A: /* GET_INTERFACE */
            s_ep0_buf[0] = 0;
            ep0_send(s_ep0_buf, 1, wLength);
            return;
        case 0x0B: /* SET_INTERFACE */
            ep0_status_in();
            return;
        default:
            ep0_stall();
            return;
        }
    }

    if (type == 1U)                                  /* CDC class requests */
    {
        switch (bRequest)
        {
        case 0x20: /* SET_LINE_CODING (data stage follows) */
            if (wLength == 0U || wLength > sizeof(s_ep0_buf)) { ep0_stall(); return; }
            s_ep0_out_req = bRequest;
            s_ep0_state   = EP0_DATA_OUT;
            HAL_PCD_EP_Receive(hpcd, 0x00, s_ep0_buf, wLength);
            return;
        case 0x21: /* GET_LINE_CODING */
            ep0_send(s_line_coding, sizeof(s_line_coding), wLength);
            return;
        case 0x22: /* SET_CONTROL_LINE_STATE */
        {
            s_dtr = (wValue & 0x01U) != 0U;
            s_rts = (wValue & 0x02U) != 0U;
            USB_CDC_LineCoding_t c = decode_line_coding();
            USB_CDC_LineStateCallback(&c, s_dtr, s_rts);
            ep0_status_in();
            return;
        }
        case 0x23: /* SEND_BREAK */
            ep0_status_in();
            return;
        default:
            ep0_stall();
            return;
        }
    }

    ep0_stall();                                     /* vendor / reserved */
}

static void on_data_in(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    (void)hpcd;
    epnum &= 0x7FU;
    if (epnum == 0U)
    {
        if (s_ep0_state == EP0_DATA_IN)        ep0_in_next();
        else if (s_ep0_state == EP0_STATUS_IN) s_ep0_state = EP0_IDLE;
    }
    else if (epnum == (EP_DATA_IN & 0x7FU))
    {
        s_tx_busy = false;
        tx_kick();
    }
}

static void on_data_out(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    epnum &= 0x7FU;
    if (epnum == 0U)
    {
        if (s_ep0_state == EP0_DATA_OUT)
        {
            if (s_ep0_out_req == 0x20U)
            {
                memcpy(s_line_coding, s_ep0_buf, sizeof(s_line_coding));
                USB_CDC_LineCoding_t c = decode_line_coding();
                USB_CDC_LineStateCallback(&c, s_dtr, s_rts);
            }
            ep0_status_in();
        }
        else if (s_ep0_state == EP0_STATUS_OUT)
        {
            s_ep0_state = EP0_IDLE;
        }
        return;
    }

    if (epnum != EP_DATA_OUT) return;

    uint32_t n = HAL_PCD_EP_GetRxCount(hpcd, EP_DATA_OUT);
    if (n > DATA_MPS) n = DATA_MPS;
#if USB_CDC_RX_RING_SIZE > 0U
    for (uint32_t i = 0; i < n; i++)
        s_rx_ring[(s_rx_head + i) & RX_MASK] = s_rx_pkt[i];
    s_rx_head = (s_rx_head + n) & RX_MASK;
    uint32_t free_space = (USB_CDC_RX_RING_SIZE - 1U) - ((s_rx_head - s_rx_tail) & RX_MASK);
    if (free_space >= DATA_MPS) rx_arm();
    else                        s_rx_armed = false;   /* host is NAKed until Read() frees room */
#else
    for (uint32_t i = 0; i < n; i++)
        USB_CDC_RxByteCallback(s_rx_pkt[i]);
    rx_arm();
#endif
}

/* ---- Hook the handlers into the HAL --------------------------------------- */
#if defined(USE_HAL_PCD_REGISTER_CALLBACKS) && (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void cb_reset(PCD_HandleTypeDef *h)                { on_reset(h); }
static void cb_setup(PCD_HandleTypeDef *h)                { on_setup(h); }
static void cb_in(PCD_HandleTypeDef *h, uint8_t ep)       { on_data_in(h, ep); }
static void cb_out(PCD_HandleTypeDef *h, uint8_t ep)      { on_data_out(h, ep); }
static void register_callbacks(PCD_HandleTypeDef *h)
{
    HAL_PCD_RegisterCallback(h, HAL_PCD_RESET_CB_ID, cb_reset);
    HAL_PCD_RegisterCallback(h, HAL_PCD_SETUPSTAGE_CB_ID, cb_setup);
    HAL_PCD_RegisterDataInStageCallback(h, cb_in);
    HAL_PCD_RegisterDataOutStageCallback(h, cb_out);
}
#else
/* Strong definitions replace the HAL's weak ones. If the CubeMX USB_DEVICE
 * middleware is also in the project (usbd_conf.c defines these too) the link
 * fails with "multiple definition" — remove the middleware. */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *h)                     { on_reset(h); }
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *h)                { on_setup(h); }
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *h, uint8_t ep)   { on_data_in(h, ep); }
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *h, uint8_t ep)  { on_data_out(h, ep); }
static void register_callbacks(PCD_HandleTypeDef *h) { (void)h; }
#endif

/* ========================================================================== */
/*  Public API                                                                */
/* ========================================================================== */
void USB_CDC_Init(PCD_HandleTypeDef *hpcd)
{
    s_pcd        = hpcd;
    s_configured = false;
    s_tx_head    = s_tx_tail = 0;
    s_tx_busy    = false;
#if USB_CDC_RX_RING_SIZE > 0U
    s_rx_head    = s_rx_tail = 0;
    s_rx_armed   = false;
#endif

    USB_CDC_GetSerialString(s_serial, sizeof(s_serial));
    register_callbacks(hpcd);

#if USB_CDC_CORE_OTG
    HAL_PCDEx_SetRxFiFo(hpcd, USB_CDC_OTG_RX_FIFO_WORDS);
    HAL_PCDEx_SetTxFiFo(hpcd, 0, USB_CDC_OTG_TX0_FIFO_WORDS);
    HAL_PCDEx_SetTxFiFo(hpcd, 1, USB_CDC_OTG_TX1_FIFO_WORDS);
    HAL_PCDEx_SetTxFiFo(hpcd, 2, USB_CDC_OTG_TX2_FIFO_WORDS);
#else
    HAL_PCDEx_PMAConfig(hpcd, 0x00,        PCD_SNG_BUF, USB_CDC_PMA_BASE + 0x000U);
    HAL_PCDEx_PMAConfig(hpcd, 0x80,        PCD_SNG_BUF, USB_CDC_PMA_BASE + 0x040U);
    HAL_PCDEx_PMAConfig(hpcd, EP_DATA_IN,  PCD_SNG_BUF, USB_CDC_PMA_BASE + 0x080U);
    HAL_PCDEx_PMAConfig(hpcd, EP_DATA_OUT, PCD_SNG_BUF, USB_CDC_PMA_BASE + 0x0C0U);
    HAL_PCDEx_PMAConfig(hpcd, EP_CMD_IN,   PCD_SNG_BUF, USB_CDC_PMA_BASE + 0x100U);
#endif

    HAL_PCD_Start(hpcd);   /* D+ pull-up on -> the host starts enumerating */
}

void USB_CDC_DeInit(void)
{
    if (s_pcd == NULL) return;
    HAL_PCD_Stop(s_pcd);
    s_configured = false;
}

bool USB_CDC_IsConfigured(void) { return s_configured; }
bool USB_CDC_IsPortOpen(void)   { return s_configured && s_dtr; }
USB_CDC_LineCoding_t USB_CDC_GetLineCoding(void) { return decode_line_coding(); }

uint32_t USB_CDC_TxFree(void)
{
    return (USB_CDC_TX_RING_SIZE - 1U) - ((s_tx_head - s_tx_tail) & TX_MASK);
}

uint32_t USB_CDC_Write(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    if (s_pcd == NULL || p == NULL || len == 0U) return 0;

    uint32_t pm = irq_lock();
    uint32_t free_space = USB_CDC_TxFree();
    if (len > free_space) len = free_space;
    for (uint32_t i = 0; i < len; i++)
        s_tx_ring[(s_tx_head + i) & TX_MASK] = p[i];
    s_tx_head = (s_tx_head + len) & TX_MASK;
    tx_kick();
    irq_unlock(pm);
    return len;
}

uint32_t USB_CDC_WriteBlocking(const void *data, uint32_t len, uint32_t timeout_ms)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t done = 0, t0 = HAL_GetTick();
    while (done < len)
    {
        done += USB_CDC_Write(p + done, len - done);
        if (done < len && (HAL_GetTick() - t0) >= timeout_ms) break;
    }
    return done;
}

#if USB_CDC_RX_RING_SIZE > 0U
uint32_t USB_CDC_Available(void)
{
    return (s_rx_head - s_rx_tail) & RX_MASK;
}

uint32_t USB_CDC_Read(void *buf, uint32_t len)
{
    uint8_t *p = (uint8_t *)buf;
    if (p == NULL || len == 0U) return 0;

    uint32_t pm = irq_lock();
    uint32_t avail = (s_rx_head - s_rx_tail) & RX_MASK;
    if (len > avail) len = avail;
    for (uint32_t i = 0; i < len; i++)
        p[i] = s_rx_ring[(s_rx_tail + i) & RX_MASK];
    s_rx_tail = (s_rx_tail + len) & RX_MASK;

    uint32_t free_space = (USB_CDC_RX_RING_SIZE - 1U) - ((s_rx_head - s_rx_tail) & RX_MASK);
    if (s_configured && !s_rx_armed && free_space >= DATA_MPS)
    {
        s_rx_armed = true;
        rx_arm();                                   /* stop NAKing the host */
    }
    irq_unlock(pm);
    return len;
}
#endif
