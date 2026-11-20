/*
 * FreeRTOS example: assemble received bytes into lines in the USB interrupt
 * and hand complete commands to a task through a queue. This is how the
 * library is used on the STM32H5 camera-gimbal board.
 *
 * Config (usb_cdc_conf.h):
 *     #define USB_CDC_RX_RING_SIZE  0U      // callback mode
 *
 * NVIC: the USB IRQ priority must be numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY
 * (e.g. 5 when configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY is 5), otherwise the
 * ...FromISR() call below is illegal.
 */
#include "main.h"
#include "usb_cdc.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define CMD_LEN 128
static QueueHandle_t q_cmd;
static char          line[CMD_LEN];
static uint16_t      line_len;

/* Runs in the USB interrupt, once per received byte. */
void USB_CDC_RxByteCallback(uint8_t c)
{
    BaseType_t woken = pdFALSE;
    if (c == '\r' || c == '\n')
    {
        if (line_len && q_cmd)
        {
            line[line_len] = '\0';
            xQueueSendFromISR(q_cmd, line, &woken);
        }
        line_len = 0;
    }
    else if (line_len < CMD_LEN - 1)
        line[line_len++] = (char)c;
    else
        line_len = 0;                       /* overflow: drop the line */
    portYIELD_FROM_ISR(woken);
}

/* printf-style helper; never blocks, so it is safe in control loops. */
void usb_printf(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) USB_CDC_Write(buf, (uint32_t)(n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1));
}

static void Cmd_Task(void *arg)
{
    (void)arg;
    char cmd[CMD_LEN];
    for (;;)
    {
        if (xQueueReceive(q_cmd, cmd, portMAX_DELAY) == pdTRUE)
        {
            if (strcmp(cmd, "PING") == 0) usb_printf("PONG\r\n");
            else                          usb_printf("ERR unknown: %s\r\n", cmd);
        }
    }
}

/* In main(), after the CubeMX inits: */
void app_start(PCD_HandleTypeDef *hpcd)
{
    /* Connect USB BEFORE the first FreeRTOS call (or right before
     * vTaskStartScheduler): FreeRTOS critical sections executed before the
     * scheduler runs leave BASEPRI raised, which masks the USB IRQ until the
     * scheduler starts. Anything printed before the host opens the port is
     * kept in the TX ring (boot log). */
    USB_CDC_Init(hpcd);

    q_cmd = xQueueCreate(8, CMD_LEN);
    xTaskCreate(Cmd_Task, "cmd", 512, NULL, 2, NULL);
    vTaskStartScheduler();
}
