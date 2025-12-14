/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

#include "odrive_ascii.h"

static const char *TAG = "main";

static const int RX_BUF_SIZE = 1024;
odrv_ascii_t odrv;
#define TXD_PIN (GPIO_NUM_14)
#define RXD_PIN (GPIO_NUM_12)

static size_t odrive_uart_write_bytes(const uint8_t *buf, size_t len)
{
    return uart_write_bytes(UART_NUM_1, buf, len);
}

static size_t odrive_uart_read_bytes(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    return uart_read_bytes(UART_NUM_1, buf, len, pdMS_TO_TICKS(timeout_ms));
}

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

    odrv_ascii_init(&odrv, odrive_uart_write_bytes, odrive_uart_read_bytes);
}

int sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void get_ff_task(void *arg)
{
    while (1)
    {
        double pos_out = -1, vel_out = -1;
        char out_buff[100] = "";
        odrv_get_position_and_velocity(&odrv, 0, &pos_out, &vel_out);
        ESP_LOGI(TAG, "pos_out: %5.2f vel_out: %5.2f", pos_out, vel_out);
        vTaskDelay(pdMS_TO_TICKS(200));
        odrv_read_property(&odrv, out_buff, sizeof(out_buff), "vbus_voltage");
        ESP_LOGI(TAG, "vbus_voltage: %s", out_buff);
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

void app_main(void)
{
    init();
    xTaskCreate(get_ff_task, "get_ff_task", 1024 * 16, NULL, configMAX_PRIORITIES - 1,
                NULL);
}
