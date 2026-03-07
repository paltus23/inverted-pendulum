/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

#include "as5048a.h"
#include "odrive_ascii.h"

static const char *TAG = "main";

/// @brief uart
static const int RX_BUF_SIZE = 1024;
odrv_ascii_t odrv;
#define TXD_PIN (GPIO_NUM_14)
#define RXD_PIN (GPIO_NUM_12)

/// @brief SPI
#define AS5048A_SPI VSPI_HOST

#define PIN_NUM_MOSI 23 // yellow
#define PIN_NUM_MISO 19 // orange
#define PIN_NUM_CLK 18  // green
#define PIN_NUM_CS 22   // blue

spi_device_handle_t spi;

as5048a_handle_t as5048a_handle;
void spi_send_func(uint16_t data);
uint16_t spi_read_func(void);
void spi_select_func(void);
void spi_deselect_func(void);

void spi_send_func(uint16_t data)
{
    spi_transaction_t trans_desc = {
        .length = 16,       // length in BITS!
        .tx_buffer = &data, // pointer to data
        .rx_buffer = NULL   // no receive
    };

    esp_err_t err = spi_device_polling_transmit(spi, &trans_desc);
    ESP_LOGI(TAG, "%s %d", __func__, err);
}
uint16_t spi_read_func(void)
{
    uint8_t data[2] = {0, 0};
    uint8_t tx_data[2] = {0, 0};
    spi_transaction_t trans_desc = {
        .length = 16,      // length in BITS!
        .tx_buffer = NULL, // no transmit
        .rx_buffer = data  //  pointer to data
    };

    esp_err_t err = spi_device_polling_transmit(spi, &trans_desc);
    ESP_LOGI(TAG, "%s %d", __func__, err);
    return (data[1] << 8) + data[0];
}
void spi_select_func(void)
{
    gpio_set_level(PIN_NUM_CS, 0);
}
void spi_deselect_func(void)
{
    gpio_set_level(PIN_NUM_CS, 1);
}

void spi_delay_dummy(void)
{
    vTaskDelay(1);
}

static size_t odrive_uart_write_bytes(const uint8_t *buf, size_t len)
{
    return uart_write_bytes(UART_NUM_1, buf, len);
}

static size_t odrive_uart_read_bytes(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    return uart_read_bytes(UART_NUM_1, buf, len, pdMS_TO_TICKS(timeout_ms));
}

void init_odrive(void)
{
    ESP_LOGI(TAG, "%s", __func__);
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

void init_as5048a(void)
{

    // Set CS
    ESP_LOGI(TAG, "%s", __func__);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_CS);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(PIN_NUM_CS, 1);

    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .data0_io_num = -1,
        .data1_io_num = -1,
        .data2_io_num = -1,
        .data3_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .flags = 0,
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(AS5048A_SPI, &buscfg, SPI_DMA_DISABLED);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 200 * 1000, // Clock out at 10 MHz
        .mode = 0,                    // SPI mode 0
        .spics_io_num = -1,           // CS pin
        .flags = 0,
        .address_bits = 0,
        .command_bits = 0,
        .dummy_bits = 0,
        .queue_size = 3,
        .pre_cb = NULL,
        .post_cb = NULL,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
    };
    ret = spi_bus_add_device(AS5048A_SPI, &devcfg, &spi);

    as5048a_make_handle(spi_send_func,
                        spi_read_func,
                        spi_select_func,
                        spi_deselect_func,
                        spi_delay_dummy,
                        &as5048a_handle);

    uint16_t position = 0;
    uint16_t diag = 0;
    uint16_t data = 0xBFFF;
    const uint8_t buf[2] = {0xff, 0xbf};
    uint8_t rx_buf[4] = {
        0,
    };
    const char *str_data = "Hello world from ESP using SPI";

    gpio_dump_io_configuration(stdout, (1ULL << PIN_NUM_MISO) | (1ULL << PIN_NUM_MOSI) | (1ULL << PIN_NUM_CLK));

    while (1)
    {
        spi_transaction_t trans_desc;
        memset(&trans_desc, 0, sizeof(spi_transaction_t));

        trans_desc.length = 32;          // length in BITS!
        trans_desc.tx_buffer = str_data; // pointer to data
        trans_desc.rx_buffer = rx_buf;   // no receive

        spi_select_func();
        vTaskDelay(1);
        esp_err_t err = spi_device_polling_transmit(spi, &trans_desc);
        ESP_LOGI(TAG, "%s data:%02x%02x%02x%02x %d", __func__, rx_buf[3], rx_buf[2], rx_buf[1], rx_buf[0], err);
        spi_deselect_func();

        // uint8_t err = as5048a_get_position(&as5048a_handle, &position);
        // ESP_LOGI(TAG, "%s position: %d err: %d", __func__, position, err);

        // err = as5048a_get_diag(&as5048a_handle, &diag);
        // ESP_LOGI(TAG, "%s diag: %x err: %d", __func__, diag, err);

        // uint16_t error_status = as5048a_get_error_status(&as5048a_handle);
        // ESP_LOGI(TAG, "%s error_status: %x", __func__, error_status);
        vTaskDelay(50);
    }
}

void init(void)
{
    init_odrive();
    init_as5048a();
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
