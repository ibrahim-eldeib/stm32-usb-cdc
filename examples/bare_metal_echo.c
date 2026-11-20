/*
 * Bare-metal example: echo everything back, and print a counter every second.
 * Paste the marked parts into a CubeMX-generated main.c.
 *
 * Config (usb_cdc_conf.h): defaults are fine (RX ring mode).
 * Test from the PC:  python tools/cdc_test.py      (loopback + throughput)
 */
#include "main.h"
#include "usb_cdc.h"
#include <stdio.h>

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;   /* name depends on the part:
                                               hpcd_USB_FS, hpcd_USB_DRD_FS,
                                               hpcd_USB_OTG_FS, hpcd_USB_OTG_HS */

/* Optional: printf() -> USB. With newlib-nano, printf calls _write(). */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    return (int)USB_CDC_WriteBlocking(ptr, (uint32_t)len, 20);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();          /* must give the USB peripheral 48 MHz */
    MX_GPIO_Init();
    MX_USB_OTG_FS_PCD_Init();      /* CubeMX: USB peripheral, Device-only, NO USB_DEVICE middleware */

    USB_CDC_Init(&hpcd_USB_OTG_FS);

    uint32_t last = HAL_GetTick(), count = 0;
    uint8_t buf[64];

    for (;;)
    {
        /* Echo. Read only what the TX ring can take, so nothing is dropped:
         * when TX is full, RX stays in its ring and the host is NAKed. */
        uint32_t room = USB_CDC_TxFree();
        uint32_t n = USB_CDC_Read(buf, room < sizeof(buf) ? room : sizeof(buf));
        if (n) USB_CDC_Write(buf, n);

        if (HAL_GetTick() - last >= 1000U)
        {
            last += 1000U;
            if (USB_CDC_IsPortOpen())
                printf("alive %lu\r\n", (unsigned long)count++);
        }
    }
}
