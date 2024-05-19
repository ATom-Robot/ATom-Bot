#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "app_uart.h"

static const char *TAG = "uart_events";

#define BYTE0(dwTemp) (*((char *)(&dwTemp) + 0))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

uint8_t Data_Buff[32] = {0XAA, 0XFF, 0XF1};
uint8_t Data_Sent[32] = {0XAA, 0XFF, 0XE2};
uint8_t Data_Check[12] = {0XAA, 0XFF, 0X00};

#define BUF_SIZE (128)
#define RD_BUF_SIZE (BUF_SIZE)
#define PATTERN_CHR_NUM (3)

#define UART_TX_PIN GPIO_NUM_14
#define UART_RX_PIN GPIO_NUM_21

Chassis_data chassis;

static QueueHandle_t uart1_queue;

static void get_ChassisData(uint8_t data);

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart1_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, RD_BUF_SIZE);
            switch (event.type)
            {
            // Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
            {
                // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                int recv_len;
                recv_len = uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
                if (recv_len > 0)
                {
                    for (int i = 0; i < recv_len; i++)
                    {
                        get_ChassisData(dtmp[i]);
                    }
                }
                break;
            }
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_1);
                xQueueReset(uart1_queue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider encreasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_1);
                xQueueReset(uart1_queue);
                break;
            // Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            // Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            // Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGE(TAG, "uart frame error");
                break;
            // UART_PATTERN_DET
            case UART_PATTERN_DET:
                ESP_LOGI(TAG, "uart pattern det");
                break;
            // Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

esp_err_t APPUart_Init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config =
        {
            .baud_rate = 500000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };
    // Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart1_queue, 0);
    uart_param_config(UART_NUM_1, &uart_config);

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM_1, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(UART_NUM_1, '+', PATTERN_CHR_NUM, 9, 0, 0);
    // Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(UART_NUM_1, 20);

    // Create a task to handler UART event from ISR
    BaseType_t result = xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 4096, NULL, 5, NULL, 1);
    assert("Failed to create uart task" && result == (BaseType_t)1);

    return ESP_OK;
}

void uart_data_Anl(uint8_t *Data_Pack)
{
    uint8_t sc = 0, ac = 0;

    for (uint8_t i = 0; i < Data_Pack[3] + 4; i++)
    {
        sc += Data_Pack[i];
        ac += sc;
    }
    if (sc != Data_Pack[Data_Pack[3] + 4] || ac != Data_Pack[Data_Pack[3] + 5])
    {
        return;
    }

    /* 参数ID=0xF5 */
    if (Data_Pack[2] == 0xF5)
    {
        chassis.angle_r = *((int16_t *)(Data_Pack + 4));
        chassis.angle_l = *((int16_t *)(Data_Pack + 6));
        chassis.pitch = *((int16_t *)(Data_Pack + 8));
        chassis.roll = *((int16_t *)(Data_Pack + 10));
        chassis.yaw = *((int16_t *)(Data_Pack + 12));
        chassis.voltage = *((int16_t *)(Data_Pack + 14)) * 0.01f;

        // printf("angle_r:%d angle_l:%d pitch:%d roll:%d yaw:%d vol:%.2f\n",
        //        chassis.angle_r,
        //        chassis.angle_l,
        //        chassis.pitch,
        //        chassis.roll,
        //        chassis.yaw,
        //        chassis.voltage);
    }
}

// 获取底盘串口数据
static void get_ChassisData(uint8_t data)
{
    static uint8_t sta = 0, datalen = 0, datacnt = 0;
    static uint8_t recv_data[64];

    if (sta == 0)
    {
        datalen = 0;
        datacnt = 0;

        recv_data[0] = data;
        if (data == 0xAA)
            sta = 1;
    }
    else if (sta == 1 && data == 0xFF)
    {
        recv_data[1] = data;
        sta = 2;
    }
    else if (sta == 2) /* ID */
    {
        recv_data[2] = data;
        sta = 3;
    }
    else if (sta == 3)
    {
        recv_data[3] = data;
        datalen = data;
        sta = 4;
    }
    else if (sta == 4)
    {
        recv_data[4 + datacnt++] = data;
        if (datacnt >= datalen)
        {
            sta = 5;
        }
    }
    else if (sta == 5)
    {
        recv_data[4 + datacnt++] = data;
        sta = 6;
    }
    else if (sta == 6)
    {
        sta = 0;
        recv_data[4 + datacnt++] = data;

        uart_data_Anl(recv_data);
        return;
    }
    else
        sta = 0;
}

void data_sendto_ChassisData(int16_t _a, int16_t _b, int16_t _c, int16_t _d)
{
    uint8_t i, cnt = 4;
    uint8_t sc = 0, ac = 0;

    Data_Buff[cnt++] = BYTE0(_a);
    Data_Buff[cnt++] = BYTE1(_a);

    Data_Buff[cnt++] = BYTE0(_b);
    Data_Buff[cnt++] = BYTE1(_b);

    Data_Buff[cnt++] = BYTE0(_c);
    Data_Buff[cnt++] = BYTE1(_c);

    Data_Buff[cnt++] = BYTE0(_d);
    Data_Buff[cnt++] = BYTE1(_d);

    Data_Buff[3] = cnt - 4;

    for (i = 0; i < cnt; i++)
    {
        sc += Data_Buff[i];
        ac += sc;
    }

    Data_Buff[cnt++] = sc;
    Data_Buff[cnt++] = ac;

    for (i = 0; i < cnt; i++)
    {
        uart_write_bytes(UART_NUM_1, &Data_Buff[i], 1);
    }
}
