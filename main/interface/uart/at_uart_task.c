/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2017 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP32 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
// #include "iostream"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_at.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

#include "ff.h"

#define PEDRO_SIZE 12000

// #include "esp_log.h"
// #include "esp_http_client.h"

// // #include <string.h>
// // #include <stdlib.h>
// // #include "freertos/FreeRTOS.h"
// // #include "freertos/task.h"
// // #include "esp_log.h"
// #include "esp_system.h"
// // #include "nvs_flash.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "protocol_examples_common.h"
#include "esp_http_client_example.h"
// // #include "esp-idf/examples/common_components/protocol_examples_common/include/protocol_examples_common.h"
// #include "esp_tls.h"

#ifdef CONFIG_AT_BASE_ON_UART
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "at_interface.h"

#ifdef CONFIG_IDF_TARGET_ESP8266
#include "esp8266/uart_register.h"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/uart.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/uart.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/uart.h"
#endif

typedef struct
{
    int32_t baudrate;
    int8_t data_bits;
    int8_t stop_bits;
    int8_t parity;
    int8_t flow_control;
} at_nvm_uart_config_struct;

typedef struct
{
    int32_t tx;
    int32_t rx;
    int32_t cts;
    int32_t rts;
} at_uart_pins_t;

static bool at_default_flag = false;
static at_uart_pins_t s_at_uart_port_pin;
static QueueHandle_t esp_at_uart_queue = NULL;
static const uint8_t esp_at_uart_parity_table[] = {UART_PARITY_DISABLE, UART_PARITY_ODD, UART_PARITY_EVEN};

#if defined(CONFIG_IDF_TARGET_ESP32)
#define CONFIG_AT_UART_PORT_TX_PIN_DEFAULT 17
#define CONFIG_AT_UART_PORT_RX_PIN_DEFAULT 16
#define CONFIG_AT_UART_PORT_CTS_PIN_DEFAULT 15
#define CONFIG_AT_UART_PORT_RTS_PIN_DEFAULT 14
#ifndef CONFIG_AT_UART_PORT
#define CONFIG_AT_UART_PORT UART_NUM_1
#endif
#define AT_UART_BAUD_RATE_MAX 5000000
#define AT_UART_BAUD_RATE_MIN 80
#elif defined(CONFIG_IDF_TARGET_ESP8266)
#define CONFIG_AT_UART_PORT_TX_PIN_DEFAULT 15
#define CONFIG_AT_UART_PORT_RX_PIN_DEFAULT 13
#define CONFIG_AT_UART_PORT_CTS_PIN_DEFAULT 3
#define CONFIG_AT_UART_PORT_RTS_PIN_DEFAULT 1
#ifndef CONFIG_AT_UART_PORT
#define CONFIG_AT_UART_PORT UART_NUM_0
#endif
#define AT_UART_BAUD_RATE_MAX 4500000
#define AT_UART_BAUD_RATE_MIN 80
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#define CONFIG_AT_UART_PORT_TX_PIN_DEFAULT 17
#define CONFIG_AT_UART_PORT_RX_PIN_DEFAULT 18
#define CONFIG_AT_UART_PORT_CTS_PIN_DEFAULT 20
#define CONFIG_AT_UART_PORT_RTS_PIN_DEFAULT 19
#ifndef CONFIG_AT_UART_PORT
#define CONFIG_AT_UART_PORT UART_NUM_1
#endif
#define AT_UART_BAUD_RATE_MAX 5000000
#define AT_UART_BAUD_RATE_MIN 80
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#define CONFIG_AT_UART_PORT_TX_PIN_DEFAULT 7
#define CONFIG_AT_UART_PORT_RX_PIN_DEFAULT 6
#define CONFIG_AT_UART_PORT_CTS_PIN_DEFAULT 5
#define CONFIG_AT_UART_PORT_RTS_PIN_DEFAULT 4
#ifndef CONFIG_AT_UART_PORT
#define CONFIG_AT_UART_PORT UART_NUM_1
#endif
#define AT_UART_BAUD_RATE_MAX 5000000
#define AT_UART_BAUD_RATE_MIN 80
#endif

#define AT_UART_PATTERN_TIMEOUT_MS 20

static uart_port_t esp_at_uart_port = CONFIG_AT_UART_PORT;

static bool at_nvm_uart_config_set(at_nvm_uart_config_struct *uart_config);
static bool at_nvm_uart_config_get(at_nvm_uart_config_struct *uart_config);

static int32_t at_port_write_data(uint8_t *data, int32_t len)
{
    uint32_t length = 0;

    length = uart_write_bytes(esp_at_uart_port, (char *)data, len);
    return length;
}

static int32_t at_port_read_data(uint8_t *buf, int32_t len)
{
    TickType_t ticks_to_wait = portTICK_RATE_MS;
    uint8_t *data = NULL;
    size_t size = 0;

    if (len == 0)
    {
        return 0;
    }

    if (buf == NULL)
    {
        if (len == -1)
        {
            if (ESP_OK != uart_get_buffered_data_len(esp_at_uart_port, &size))
            {
                return -1;
            }
            len = size;
        }

        if (len == 0)
        {
            return 0;
        }

        data = (uint8_t *)malloc(len);
        if (data)
        {
            len = uart_read_bytes(esp_at_uart_port, data, len, ticks_to_wait);
            free(data);
            return len;
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return uart_read_bytes(esp_at_uart_port, buf, len, ticks_to_wait);
    }
}

static int32_t at_port_get_data_length(void)
{
    size_t size = 0;
#ifndef CONFIG_IDF_TARGET_ESP8266
    int pattern_pos = 0;
#endif

    if (ESP_OK == uart_get_buffered_data_len(esp_at_uart_port, &size))
    {
#ifndef CONFIG_IDF_TARGET_ESP8266
        pattern_pos = uart_pattern_get_pos(esp_at_uart_port);
        if (pattern_pos >= 0)
        {
            size = pattern_pos;
        }
#endif
        return size;
    }
    else
    {
        return 0;
    }
}

static bool at_port_wait_write_complete(int32_t timeout_msec)
{
    if (ESP_OK == uart_wait_tx_done(esp_at_uart_port, timeout_msec / portTICK_PERIOD_MS))
    {
        return true;
    }

    return false;
}

static void uart_task(void *pvParameters)
{
    uart_event_t event;
    uint32_t data_len = 0;
    BaseType_t retry_flag = pdFALSE;
#ifndef CONFIG_IDF_TARGET_ESP8266
    int pattern_pos = -1;
    uint8_t *data = NULL;
#endif

    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(esp_at_uart_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
        retry:
            switch (event.type)
            {
            // Event of UART receving data
            case UART_DATA:
            case UART_BUFFER_FULL:
                data_len += event.size;
                // we can put all data together to process
                retry_flag = pdFALSE;
                while (xQueueReceive(esp_at_uart_queue, (void *)&event, (portTickType)0) == pdTRUE)
                {
                    if ((event.type == UART_DATA) || (event.type == UART_BUFFER_FULL))
                    {
                        data_len += event.size;
                    }
                    else
                    {
                        retry_flag = pdTRUE;
                        break;
                    }
                }
                esp_at_port_recv_data_notify(data_len, portMAX_DELAY);
                data_len = 0;

                if (retry_flag == pdTRUE)
                {
                    goto retry;
                }
                break;
#ifndef CONFIG_IDF_TARGET_ESP8266
            case UART_PATTERN_DET:
                pattern_pos = uart_pattern_pop_pos(esp_at_uart_port);
                if (pattern_pos >= 0)
                {
                    data = (uint8_t *)malloc(pattern_pos + 3);
                    uart_read_bytes(esp_at_uart_port, data, pattern_pos + 3, 0);
                    free(data);
                    data = NULL;
                }
                else
                {
                    uart_flush_input(esp_at_uart_port);
                    xQueueReset(esp_at_uart_queue);
                }
                esp_at_transmit_terminal();
                break;
#endif
            case UART_FIFO_OVF:
                retry_flag = pdFALSE;
                while (xQueueReceive(esp_at_uart_queue, (void *)&event, (portTickType)0) == pdTRUE)
                {
                    if ((event.type == UART_DATA) || (event.type == UART_BUFFER_FULL) || (event.type == UART_FIFO_OVF))
                    {
                        // Put all data together to process
                    }
                    else
                    {
                        retry_flag = pdTRUE;
                        break;
                    }
                }
                esp_at_port_recv_data_notify(at_port_get_data_length(), portMAX_DELAY);
                data_len = 0;
                if (retry_flag == pdTRUE)
                {
                    goto retry;
                }
                break;

            // Others
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}
static void at_uart_init(void)
{
    at_nvm_uart_config_struct uart_nvm_config;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = CONFIG_AT_UART_DEFAULT_DATABITS - 5,
        .parity = esp_at_uart_parity_table[CONFIG_AT_UART_DEFAULT_PARITY_BITS],
        .stop_bits = CONFIG_AT_UART_DEFAULT_STOPBITS,
        .flow_ctrl = CONFIG_AT_UART_DEFAULT_FLOW_CONTROL,
        .rx_flow_ctrl_thresh = 122,
    };

    uart_intr_config_t intr_config = {
        .intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M,
        .rxfifo_full_thresh = 100,
        .rx_timeout_thresh = 10,
        .txfifo_empty_intr_thresh = 10};

    int32_t tx_pin = CONFIG_AT_UART_PORT_TX_PIN_DEFAULT;
    int32_t rx_pin = CONFIG_AT_UART_PORT_RX_PIN_DEFAULT;
    int32_t cts_pin = CONFIG_AT_UART_PORT_CTS_PIN_DEFAULT;
    int32_t rts_pin = CONFIG_AT_UART_PORT_RTS_PIN_DEFAULT;

    char *data = NULL;
    const esp_partition_t *partition = esp_at_custom_partition_find(0x40, 0xff, "factory_param");

    memset(&uart_nvm_config, 0x0, sizeof(uart_nvm_config));

    if (partition)
    {
        data = (char *)malloc(ESP_AT_FACTORY_PARAMETER_SIZE); // Notes
        assert(data != NULL);
        if (esp_partition_read(partition, 0, data, ESP_AT_FACTORY_PARAMETER_SIZE) != ESP_OK)
        {
            free(data);
            data = NULL;
        }
    }

    if (at_nvm_uart_config_get(&uart_nvm_config))
    {
        if ((uart_nvm_config.baudrate >= AT_UART_BAUD_RATE_MIN) && (uart_nvm_config.baudrate <= AT_UART_BAUD_RATE_MAX))
        {
            uart_config.baud_rate = uart_nvm_config.baudrate;
        }

        if ((uart_nvm_config.data_bits >= UART_DATA_5_BITS) && (uart_nvm_config.data_bits <= UART_DATA_8_BITS))
        {
            uart_config.data_bits = uart_nvm_config.data_bits;
        }

        if ((uart_nvm_config.stop_bits >= UART_STOP_BITS_1) && (uart_nvm_config.stop_bits <= UART_STOP_BITS_2))
        {
            uart_config.stop_bits = uart_nvm_config.stop_bits;
        }

        if ((uart_nvm_config.parity == UART_PARITY_DISABLE) || (uart_nvm_config.parity == UART_PARITY_ODD) || (uart_nvm_config.parity == UART_PARITY_EVEN))
        {
            uart_config.parity = uart_nvm_config.parity;
        }

        if ((uart_nvm_config.flow_control >= UART_HW_FLOWCTRL_DISABLE) && (uart_nvm_config.flow_control <= UART_HW_FLOWCTRL_CTS_RTS))
        {
            uart_config.flow_ctrl = uart_nvm_config.flow_control;
        }
    }
    else
    {
        if (data)
        {
            if ((data[0] == 0xFC) && (data[1] == 0xFC))
            { // check magic flag, should be 0xfc 0xfc
                if ((data[12] != 0xFF) || (data[13] != 0xFF) || (data[14] != 0xFF) || (data[15] != 0xFF))
                {
                    uart_config.baud_rate = *(int32_t *)&data[12];
                }
            }
        }
        uart_nvm_config.baudrate = uart_config.baud_rate;
        uart_nvm_config.data_bits = uart_config.data_bits;
        uart_nvm_config.flow_control = uart_config.flow_ctrl;
        uart_nvm_config.parity = uart_config.parity;
        uart_nvm_config.stop_bits = uart_config.stop_bits;
        at_nvm_uart_config_set(&uart_nvm_config);
    }

    if (data)
    {
        if ((data[0] == 0xFC) && (data[1] == 0xFC))
        { // check magic flag, should be 0xfc 0xfc
            if (data[5] != 0xFF)
            {
#if defined(CONFIG_IDF_TARGET_ESP32)
                assert((data[5] == 0) || (data[5] == 1) || (data[5] == 2));
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
                assert((data[5] == 0) || (data[5] == 1));
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
                assert((data[5] == 0) || (data[5] == 1));
#elif defined(CONFIG_IDF_TARGET_ESP8266)
                assert(data[5] == 0);
#endif
                esp_at_uart_port = data[5];
            }
            if ((data[16] != 0xFF) && (data[17] != 0xFF))
            {
                tx_pin = data[16];
                rx_pin = data[17];
            }

            if (data[18] != 0xFF)
            {
                cts_pin = data[18];
            }
            else
            {
                cts_pin = -1;
            }

            if (data[19] != 0xFF)
            {
                rts_pin = data[19];
            }
            else
            {
                rts_pin = -1;
            }

            if (data[20] != 0xFF)
            {
                gpio_set_direction(data[20], GPIO_MODE_OUTPUT);
                gpio_set_level(data[20], 1);
            }

            if (data[21] != 0xFF)
            {
                gpio_set_direction(data[21], GPIO_MODE_OUTPUT);
                gpio_set_level(data[21], 1);
            }
        }
        free(data);
        data = NULL;
    }
    // Set UART parameters
    uart_param_config(esp_at_uart_port, &uart_config);
#ifndef CONFIG_IDF_TARGET_ESP8266
    // Set UART pins,(-1: default pin, no change.)
    uart_set_pin(esp_at_uart_port, tx_pin, rx_pin, rts_pin, cts_pin);
    // Install UART driver, and get the queue.
    uart_driver_install(esp_at_uart_port, 2048, 8192, 30, &esp_at_uart_queue, 0);
#else
    // Install UART driver, and get the queue.
    uart_driver_install(esp_at_uart_port, 1024, 2048, 10, &esp_at_uart_queue, 0);
    if ((tx_pin == 15) && (rx_pin == 13))
    { // swap pin
        uart_enable_swap();
        assert((cts_pin == -1) || (cts_pin == 3));
        assert((rts_pin == -1) || (rts_pin == 1));
    }
    else
    {
        assert((tx_pin == 1) && (rx_pin == 3));
    }
#endif
    uart_intr_config(esp_at_uart_port, &intr_config);

    // set actual uart pins
    s_at_uart_port_pin.tx = tx_pin;
    s_at_uart_port_pin.rx = rx_pin;
    s_at_uart_port_pin.cts = cts_pin;
    s_at_uart_port_pin.rts = rts_pin;

    xTaskCreate(uart_task, "uTask", 1024, (void *)esp_at_uart_port, 1, NULL);
}

static bool at_nvm_uart_config_set(at_nvm_uart_config_struct *uart_config)
{
    nvs_handle handle;
    if (uart_config == NULL)
    {
        return false;
    }

    if (nvs_open("UART", NVS_READWRITE, &handle) == ESP_OK)
    {
        if (nvs_set_i32(handle, "rate", uart_config->baudrate) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
        if (nvs_set_i8(handle, "databits", uart_config->data_bits) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
        if (nvs_set_i8(handle, "stopbits", uart_config->stop_bits) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
        if (nvs_set_i8(handle, "parity", uart_config->parity) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
        if (nvs_set_i8(handle, "flow_ctrl", uart_config->flow_control) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
    }
    else
    {
        return false;
    }
    nvs_close(handle);
    return true;
}

static bool at_nvm_uart_config_get(at_nvm_uart_config_struct *uart_config)
{
    nvs_handle handle;
    if (uart_config == NULL)
    {
        return false;
    }

    if (nvs_open("UART", NVS_READONLY, &handle) == ESP_OK)
    {
        if (nvs_get_i32(handle, "rate", &uart_config->baudrate) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
        if (nvs_get_i8(handle, "databits", &uart_config->data_bits) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
        if (nvs_get_i8(handle, "stopbits", &uart_config->stop_bits) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
        if (nvs_get_i8(handle, "parity", &uart_config->parity) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
        if (nvs_get_i8(handle, "flow_ctrl", &uart_config->flow_control) != ESP_OK)
        {
            nvs_close(handle);
            return false;
        }
    }
    else
    {
        return false;
    }
    nvs_close(handle);

    return true;
}

static uint8_t at_setupCmdUart(uint8_t para_num)
{
    int32_t value = 0;
    int32_t cnt = 0;

    at_nvm_uart_config_struct uart_config;

    memset(&uart_config, 0x0, sizeof(uart_config));
    if (para_num != 5)
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK)
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if ((value < AT_UART_BAUD_RATE_MIN) || (value > AT_UART_BAUD_RATE_MAX))
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    uart_config.baudrate = value;

    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK)
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if ((value < 5) || (value > 8))
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    uart_config.data_bits = value - 5;

    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK)
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if ((value < 1) || (value > 3))
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    uart_config.stop_bits = value;

    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK)
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if ((value >= 0) && (value <= 2))
    {
        uart_config.parity = esp_at_uart_parity_table[value];
    }
    else
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    if (esp_at_get_para_as_digit(cnt++, &value) != ESP_AT_PARA_PARSE_RESULT_OK)
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    if ((value < 0) || (value > 3))
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }
    uart_config.flow_control = value;

    if (at_default_flag)
    {
        if (at_nvm_uart_config_set(&uart_config) == false)
        {
            return ESP_AT_RESULT_CODE_ERROR;
        }
    }
    esp_at_response_result(ESP_AT_RESULT_CODE_OK);

    uart_wait_tx_done(esp_at_uart_port, portMAX_DELAY);
    uart_set_baudrate(esp_at_uart_port, uart_config.baudrate);
    uart_set_word_length(esp_at_uart_port, uart_config.data_bits);
    uart_set_stop_bits(esp_at_uart_port, uart_config.stop_bits);
    uart_set_parity(esp_at_uart_port, uart_config.parity);
    uart_set_hw_flow_ctrl(esp_at_uart_port, uart_config.flow_control, 120);
    return ESP_AT_RESULT_CODE_PROCESS_DONE;
}

static uint8_t at_setupCmdUartDef(uint8_t para_num)
{
    uint8_t ret = ESP_AT_RESULT_CODE_ERROR;
    at_default_flag = true;
    ret = at_setupCmdUart(para_num);
    at_default_flag = false;

    return ret;
}

static uint8_t at_queryCmdUart(uint8_t *cmd_name)
{
    uint32_t baudrate = 0;
    uart_word_length_t data_bits = UART_DATA_8_BITS;
    uart_stop_bits_t stop_bits = UART_STOP_BITS_1;
    uart_parity_t parity = UART_PARITY_DISABLE;
    uart_hw_flowcontrol_t flow_control = UART_HW_FLOWCTRL_DISABLE;

    uint8_t buffer[64];

    uart_get_baudrate(esp_at_uart_port, &baudrate);
    uart_get_word_length(esp_at_uart_port, &data_bits);
    uart_get_stop_bits(esp_at_uart_port, &stop_bits);
    uart_get_parity(esp_at_uart_port, &parity);
    uart_get_hw_flow_ctrl(esp_at_uart_port, &flow_control);

    data_bits += 5;
    if (UART_PARITY_DISABLE == parity)
    {
        parity = 0;
    }
    else if (UART_PARITY_ODD == parity)
    {
        parity = 1;
    }
    else if (UART_PARITY_EVEN == parity)
    {
        parity = 2;
    }
    else
    {
        parity = 0xff;
    }

    snprintf((char *)buffer, sizeof(buffer) - 1, "%s:%d,%d,%d,%d,%d\r\n", cmd_name, baudrate, data_bits, stop_bits, parity, flow_control);

    esp_at_port_write_data(buffer, strlen((char *)buffer));

    return ESP_AT_RESULT_CODE_OK;
}

static uint8_t at_queryCmdUartDef(uint8_t *cmd_name)
{
    at_nvm_uart_config_struct uart_nvm_config;
    uint8_t buffer[64];
    memset(&uart_nvm_config, 0x0, sizeof(uart_nvm_config));
    at_nvm_uart_config_get(&uart_nvm_config);

    uart_nvm_config.data_bits += 5;
    if (UART_PARITY_DISABLE == uart_nvm_config.parity)
    {
        uart_nvm_config.parity = 0;
    }
    else if (UART_PARITY_ODD == uart_nvm_config.parity)
    {
        uart_nvm_config.parity = 1;
    }
    else if (UART_PARITY_EVEN == uart_nvm_config.parity)
    {
        uart_nvm_config.parity = 2;
    }
    else
    {
        uart_nvm_config.parity = 0xff;
    }
    snprintf((char *)buffer, sizeof(buffer) - 1, "%s:%d,%d,%d,%d,%d\r\n", cmd_name, uart_nvm_config.baudrate,
             uart_nvm_config.data_bits, uart_nvm_config.stop_bits, uart_nvm_config.parity, uart_nvm_config.flow_control);

    esp_at_port_write_data(buffer, strlen((char *)buffer));
    return ESP_AT_RESULT_CODE_OK;
}

uint8_t at_test_cmd_test(uint8_t *cmd_name)
{
    uint8_t buffer[64] = {0};

    snprintf((char *)buffer, 64, "this cmd is test cmd: %s\r\n", cmd_name);

    esp_at_port_write_data(buffer, strlen((char *)buffer));

    return ESP_AT_RESULT_CODE_OK;
}

uint8_t at_query_cmd_test(uint8_t *cmd_name)
{
    uint8_t buffer[64] = {0};

    snprintf((char *)buffer, 64, "this cmd is query cmd: %s\r\n", cmd_name);

    esp_at_port_write_data(buffer, strlen((char *)buffer));

    return ESP_AT_RESULT_CODE_OK;
}

// uint8_t at_setup_cmd_test(uint8_t para_num)
// {
//     int32_t para_int_1 = 0;
//     uint8_t *para_str_2 = NULL;
//     uint8_t num_index = 0;
//     uint8_t buffer[64] = {0};

//     if (esp_at_get_para_as_digit(num_index++, &para_int_1) != ESP_AT_PARA_PARSE_RESULT_OK) {
//         return ESP_AT_RESULT_CODE_ERROR;
//     }

//     if (esp_at_get_para_as_str(num_index++, &para_str_2) != ESP_AT_PARA_PARSE_RESULT_OK) {
//         return ESP_AT_RESULT_CODE_ERROR;
//     }

//     snprintf((char *)buffer, 64, "this cmd is setup cmd and cmd num is: %u\r\n", para_num);
//     esp_at_port_write_data(buffer, strlen((char *)buffer));

//     memset(buffer, 0, 64);
//     snprintf((char *)buffer, 64, "first parameter is: %d\r\n", para_int_1);
//     esp_at_port_write_data(buffer, strlen((char *)buffer));

//     memset(buffer, 0, 64);
//     snprintf((char *)buffer, 64, "second parameter is: %s\r\n", para_str_2);
//     esp_at_port_write_data(buffer, strlen((char *)buffer));

//     return ESP_AT_RESULT_CODE_OK;
// }

static xSemaphoreHandle at_sync_sema = NULL;

void wait_data_callback(void)
{
    xSemaphoreGive(at_sync_sema);
}

uint8_t at_setup_cmd_test(uint8_t para_num)
{
    int32_t specified_len = 0;
    int32_t received_len = 0;
    int32_t remain_len = 0;
    uint8_t *buf = NULL;
    uint8_t buffer[64] = {0};

    if (esp_at_get_para_as_digit(0, &specified_len) != ESP_AT_PARA_PARSE_RESULT_OK)
    {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    buf = (uint8_t *)malloc(specified_len);
    if (buf == NULL)
    {
        memset(buffer, 0, 64);
        snprintf((char *)buffer, 64, "malloc failed\r\n");
        esp_at_port_write_data(buffer, strlen((char *)buffer));
    }

    // sample code
    // users don't have to create semaphores here
    if (!at_sync_sema)
    {
        at_sync_sema = xSemaphoreCreateBinary();
        assert(at_sync_sema != NULL);
    }

    // output input prompt ">"
    esp_at_port_write_data((uint8_t *)">", strlen(">"));

    // set the callback function which will be called by AT port after receiving the input data
    esp_at_port_enter_specific(wait_data_callback);

    // receie input data
    while (xSemaphoreTake(at_sync_sema, portMAX_DELAY))
    {
        received_len += esp_at_port_read_data(buf + received_len, specified_len - received_len);

        if (specified_len == received_len)
        {
            esp_at_port_exit_specific();

            // get the length of the remaining input data
            remain_len = esp_at_port_get_data_length();
            if (remain_len > 0)
            {
                // sample code
                // if the remaining data length > 0, the actual input data length is greater than the specified received data length
                // users can customize the operation to process the remaining data
                // here is just a simple print out of the remaining data
                esp_at_port_recv_data_notify(remain_len, portMAX_DELAY);
            }

            // sample code
            // output received data
            memset(buffer, 0, 64);
            snprintf((char *)buffer, 64, "\r\nreceived data is: ");
            esp_at_port_write_data(buffer, strlen((char *)buffer));

            esp_at_port_write_data(buf, specified_len);

            break;
        }
    }

    free(buf);

    return ESP_AT_RESULT_CODE_OK;
}

// uint8_t at_setup_cmd_test(uint8_t para_num)
// {
//     int32_t para_int_1 = 0;
//     int32_t para_int_2 = 0;
//     uint8_t *para_str_3 = NULL;
//     uint8_t *para_str_4 = NULL;
//     uint8_t num_index = 0;
//     uint8_t buffer[64] = {0};
//     esp_at_para_parse_result_type parse_result = ESP_AT_PARA_PARSE_RESULT_OK;

//     snprintf((char *)buffer, 64, "this cmd is setup cmd and cmd num is: %u\r\n", para_num);
//     esp_at_port_write_data(buffer, strlen((char *)buffer));

//     parse_result = esp_at_get_para_as_digit(num_index++, &para_int_1);
//     if (parse_result != ESP_AT_PARA_PARSE_RESULT_OK) {
//         return ESP_AT_RESULT_CODE_ERROR;
//     } else {
//         memset(buffer, 0, 64);
//         snprintf((char *)buffer, 64, "first parameter is: %d\r\n", para_int_1);
//         esp_at_port_write_data(buffer, strlen((char *)buffer));
//     }

//     parse_result = esp_at_get_para_as_digit(num_index++, &para_int_2);
//     if (parse_result != ESP_AT_PARA_PARSE_RESULT_OMITTED) {
//         if (parse_result != ESP_AT_PARA_PARSE_RESULT_OK) {
//             return ESP_AT_RESULT_CODE_ERROR;
//         } else {
//             // sample code
//             // user needs to customize the operation
//             memset(buffer, 0, 64);
//             snprintf((char *)buffer, 64, "second parameter is: %d\r\n", para_int_2);
//             esp_at_port_write_data(buffer, strlen((char *)buffer));
//         }
//     } else {
//         // sample code
//         // the second parameter is omitted
//         // user needs to customize the operation
//         memset(buffer, 0, 64);
//         snprintf((char *)buffer, 64, "second parameter is omitted\r\n");
//         esp_at_port_write_data(buffer, strlen((char *)buffer));
//     }

//     parse_result = esp_at_get_para_as_str(num_index++, &para_str_3);
//     if (parse_result != ESP_AT_PARA_PARSE_RESULT_OMITTED) {
//         if (parse_result != ESP_AT_PARA_PARSE_RESULT_OK) {
//             return ESP_AT_RESULT_CODE_ERROR;
//         } else {
//             // sample code
//             // user needs to customize the operation
//             memset(buffer, 0, 64);
//             snprintf((char *)buffer, 64, "third parameter is: %s\r\n", para_str_3);
//             esp_at_port_write_data(buffer, strlen((char *)buffer));
//         }
//     } else {
//         // sample code
//         // the third parameter is omitted
//         // user needs to customize the operation
//         memset(buffer, 0, 64);
//         snprintf((char *)buffer, 64, "third parameter is omitted\r\n");
//         esp_at_port_write_data(buffer, strlen((char *)buffer));
//     }

//     parse_result = esp_at_get_para_as_str(num_index++, &para_str_4);
//     if (parse_result != ESP_AT_PARA_PARSE_RESULT_OK) {
//         return ESP_AT_RESULT_CODE_ERROR;
//     } else {
//         memset(buffer, 0, 64);
//         snprintf((char *)buffer, 64, "fourth parameter is: %s\r\n", para_str_4);
//         esp_at_port_write_data(buffer, strlen((char *)buffer));
//     }

//     return ESP_AT_RESULT_CODE_OK;
// }

// xSemaphoreHandle at_operation_sema = NULL;

void pedro_function()
{
}

// #define MAX_HTTP_RECV_BUFFER 512
// #define MAX_HTTP_OUTPUT_BUFFER 2048
// static const char *TAG = "HTTP_CLIENT";

// esp_err_t _http_event_handler(esp_http_client_event_t *evt)
// {
//     static char *output_buffer; // Buffer to store response of http request from event handler
//     static int output_len;      // Stores number of bytes read
//     switch (evt->event_id)
//     {
//     case HTTP_EVENT_ERROR:
//         ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
//         break;
//     case HTTP_EVENT_ON_CONNECTED:
//         ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
//         break;
//     case HTTP_EVENT_HEADER_SENT:
//         ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
//         break;
//     case HTTP_EVENT_ON_HEADER:
//         ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
//         break;
//     case HTTP_EVENT_ON_DATA:
//         ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
//         /*
//          *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
//          *  However, event handler can also be used in case chunked encoding is used.
//          */
//         if (!esp_http_client_is_chunked_response(evt->client))
//         {
//             // If user_data buffer is configured, copy the response into the buffer
//             if (evt->user_data)
//             {
//                 memcpy(evt->user_data + output_len, evt->data, evt->data_len);
//             }
//             else
//             {
//                 if (output_buffer == NULL)
//                 {
//                     output_buffer = (char *)malloc(esp_http_client_get_content_length(evt->client));
//                     output_len = 0;
//                     if (output_buffer == NULL)
//                     {
//                         ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
//                         return ESP_FAIL;
//                     }
//                 }
//                 memcpy(output_buffer + output_len, evt->data, evt->data_len);
//             }
//             output_len += evt->data_len;
//         }

//         break;
//     case HTTP_EVENT_ON_FINISH:
//         ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
//         if (output_buffer != NULL)
//         {
//             // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
//             // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
//             free(output_buffer);
//             output_buffer = NULL;
//             output_len = 0;
//         }
//         break;
//     case HTTP_EVENT_DISCONNECTED:
//         ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
//         int mbedtls_err = 0;
//         esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
//         if (err != 0)
//         {
//             if (output_buffer != NULL)
//             {
//                 free(output_buffer);
//                 output_buffer = NULL;
//                 output_len = 0;
//             }
//             ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
//             ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
//         }
//         break;
//     }
//     return ESP_OK;
// }

static void http_download_chunk_2(void)
{
    uint8_t data_2[15] = "pedro_func\n";
    uint8_t data_3[15] = "pedro_no_func\n";
    esp_http_client_config_t config = {
        .url = "http://httpbin.org/encoding/utf8",
        .event_handler = _http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    uint8_t buffer[1024] = {0};

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP chunk encoding Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));

        esp_at_port_write_data(data_2, strlen((char *)data_2));

        // memcpy(buffer, esp_http_client_get_content_length(client), 20);
        snprintf((char *)buffer, 1024, "HTTP chunk encoding Status = %d, content_length = %d\n",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        esp_at_port_write_data(data_3, strlen((char *)data_3));
        snprintf((char *)buffer, strlen((char *)buffer), "Error perform http request %s", esp_err_to_name(err));
    }
    esp_at_port_write_data(buffer, strlen((char *)buffer));
    // uint8_t data_buffer[15000] = {0};
    // esp_http_client_read(client, (char *)data_buffer, strlen((char *)data_buffer));
    // esp_at_port_write_data(data_buffer, strlen((char *)data_buffer));
    esp_http_client_cleanup(client);
}

static void http_native_request_2(char *output_buffer)
{
    // char output_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};   // Buffer to store response of http request
    int content_length = 0;
    // uint8_t buffer[200] = {0};
    // uint8_t buffer_1[200] = {0};
    uint8_t buffer_2[1024] = {0};
    // uint8_t buffer_3[4000] = {0};
    // uint8_t buffer_4[200] = {0};

    esp_http_client_config_t config = {
        .url = "http://httpbin.org/range/2000",
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET Request
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));

        // snprintf((char *)buffer, 200, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    }
    else
    {
        content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0)
        {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");

            // snprintf((char *)buffer_1, 200, "HTTP client fetch headers failed");
        }
        else
        {
            int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
            if (data_read >= 0)
            {
                ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d\n",
                         esp_http_client_get_status_code(client),
                         esp_http_client_get_content_length(client));

                snprintf((char *)buffer_2, 1024, "HTTP GET Status = %d, content_length = %d\n",
                         esp_http_client_get_status_code(client),
                         esp_http_client_get_content_length(client));
                esp_at_port_write_data(buffer_2, strlen((char *)buffer_2));
                // delete(buffer_2);

                ESP_LOG_BUFFER_HEX(TAG, output_buffer, strlen(output_buffer));

                // snprintf((char *)buffer_3, strlen(output_buffer), output_buffer);
                esp_at_port_write_data((uint8_t *)output_buffer, strlen((char *)output_buffer));
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read response");

                // snprintf((char *)buffer_4, 200, "Failed to read response");
            }
        }
    }
    // esp_http_client_close(client);

    // POST Request
    // const char *post_data = "{\"field1\":\"value1\"}";
    // esp_http_client_set_url(client, "http://httpbin.org/post");
    // esp_http_client_set_method(client, HTTP_METHOD_POST);
    // esp_http_client_set_header(client, "Content-Type", "application/json");
    // err = esp_http_client_open(client, strlen(post_data));
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    // } else {
    //     int wlen = esp_http_client_write(client, post_data, strlen(post_data));
    //     if (wlen < 0) {
    //         ESP_LOGE(TAG, "Write failed");
    //     }
    //     int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
    //     if (data_read >= 0) {
    //         ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
    //         esp_http_client_get_status_code(client),
    //         esp_http_client_get_content_length(client));
    //         ESP_LOG_BUFFER_HEX(TAG, output_buffer, strlen(output_buffer));
    //     } else {
    //         ESP_LOGE(TAG, "Failed to read response");
    //     }
    // }
    esp_http_client_cleanup(client);
}

// static const char *TAG = "example";
static void spiffs(char *output_buffer)
{

    ESP_LOGI(TAG, "Initializing SPIFFS");
    uint8_t arr[200] = {0};
    snprintf((char *)arr, 200, "Initializing SPIFFS\n");
    esp_at_port_write_data(arr, strlen((char *)arr));
    // arr = {0};

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
            // uint8_t *arr = new uint8_t(200);
            snprintf((char *)arr, 200, "Failed to mount or format filesystem\n");
            esp_at_port_write_data(arr, strlen((char *)arr));
            // arr = {0};
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
            // uint8_t *arr = new uint8_t(200);
            snprintf((char *)arr, 200, "Failed to find SPIFFS partition\n");
            esp_at_port_write_data(arr, strlen((char *)arr));
            // arr = {0};
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
            // uint8_t *arr = new uint8_t(200);
            snprintf((char *)arr, 200, "Failed to initialize SPIFFS (%s)\n", esp_err_to_name(ret));
            esp_at_port_write_data(arr, strlen((char *)arr));
            // arr = {0};
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        // uint8_t *arr = new uint8_t(200);
        snprintf((char *)arr, 200, "Failed to get SPIFFS partition information (%s)\n", esp_err_to_name(ret));
        esp_at_port_write_data(arr, strlen((char *)arr));
        // arr = {0};
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
        // uint8_t *arr = new uint8_t(200);
        snprintf((char *)arr, 200, "Partition size: total: %d, used: %d\n", total, used);
        esp_at_port_write_data(arr, strlen((char *)arr));
        // arr = {0};
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    // uint8_t *arr = new uint8_t(200);
    snprintf((char *)arr, 200, "Opening file\n");
    esp_at_port_write_data(arr, strlen((char *)arr));
    // arr = {0};
    FILE *f = fopen("/spiffs/hello.txt", "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        // uint8_t *arr = new uint8_t(200);
        snprintf((char *)arr, 200, "Failed to open file for writing\n");
        esp_at_port_write_data(arr, strlen((char *)arr));
        // arr = {0};
        return;
    }
    // fprintf(f, "Hola pedro");
    // for(int loop=0;loop < MAX_HTTP_OUTPUT_BUFFER;++loop)
    // {
    fwrite(output_buffer, sizeof(char), PEDRO_SIZE, f);
    // }
    // snprintf(f, MAX_HTTP_OUTPUT_BUFFER, output_buffer);
    fclose(f);
    ESP_LOGI(TAG, "File written");
    // uint8_t *arr = new uint8_t(200);
    snprintf((char *)arr, 200, "File written\n");
    esp_at_port_write_data(arr, strlen((char *)arr));
    // arr = {0};

    // // Check if destination file exists before renaming
    // struct stat st;
    // if (stat("/spiffs/foo.txt", &st) == 0)
    // {
    //     // Delete it if it exists
    //     unlink("/spiffs/foo.txt");
    // }

    // // Rename original file
    // ESP_LOGI(TAG, "Renaming file");
    // // uint8_t *arr = new uint8_t(200);
    // snprintf((char *)arr, 200, "Renaming file\n");
    // esp_at_port_write_data(arr, strlen((char *)arr));
    // // arr = {0};

    // if (rename("/spiffs/hello.txt", "/spiffs/foo.txt") != 0)
    // {
    //     ESP_LOGE(TAG, "Rename failed");
    //     // uint8_t *arr = new uint8_t(200);
    //     snprintf((char *)arr, 200, "Rename failed\n");
    //     esp_at_port_write_data(arr, strlen((char *)arr));
    //     // arr = {0};

    //     return;
    // }

    // Open renamed file for reading
    ESP_LOGI(TAG, "Reading file");
    f = fopen("/spiffs/hello.txt", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        // uint8_t *arr = new uint8_t(200);
        snprintf((char *)arr, 200, "Failed to open file for reading\n");
        esp_at_port_write_data(arr, strlen((char *)arr));
        // arr = {0};

        return;
    }
    char line[128];
    // uint8_t array[MAX_HTTP_OUTPUT_BUFFER] = {0};
    // fgets(line, sizeof(line), f);
    while (fgets(line, sizeof(line), f))
    {
        // cout << line << '\n';
        // snprintf((char *)array, 2048, line);
        esp_at_port_write_data((uint8_t *)line, strlen((char *)line));
    }
    // f.close();
    fclose(f);
    // strip newline
    char *pos = strchr(line, '\n');
    if (pos)
    {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);
    // uint8_t *arr = new uint8_t(200);
    // snprintf((char *)arr, 200, "Read from file: '%s'\n", line);
    // esp_at_port_write_data(arr, strlen((char *)arr));
    // arr = {0};

    // esp_at_port_write_data(array, strlen((char *)array));

    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(conf.partition_label);
    ESP_LOGI(TAG, "SPIFFS unmounted");
    // uint8_t *arr = new uint8_t(200);
    snprintf((char *)arr, 200, "SPIFFS unmounted\n");
    esp_at_port_write_data(arr, strlen((char *)arr));
    // arr = {0};
}

static void http_perform_as_stream_reader_pedro(char *buffer)
{
    // char *buffer = malloc(MAX_HTTP_RECV_BUFFER + 1);
    // char *buffer = malloc(10000);
    uint8_t arr[200] = {0};

    // uint8_t array[MAX_HTTP_OUTPUT_BUFFER] = {0};
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "Cannot malloc http receive buffer");
        snprintf((char *)arr, 200, "Cannot malloc http receive buffer\n");
        esp_at_port_write_data(arr, strlen((char *)arr));
        return;
    }
    esp_http_client_config_t config = {
        // .url = "http://httpbin.org/get",
        .url = "http://httpbin.org/range/12000",
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err;
    if ((err = esp_http_client_open(client, 0)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        snprintf((char *)arr, 200, "Failed to open HTTP connection: %s\n", esp_err_to_name(err));
        esp_at_port_write_data(arr, strlen((char *)arr));
        free(buffer);
        return;
    }
    int content_length = esp_http_client_fetch_headers(client);
    int total_read_len = 0, read_len;
    if (total_read_len < content_length && content_length <= PEDRO_SIZE)
    {
        read_len = esp_http_client_read(client, buffer, content_length);
        if (read_len <= 0)
        {
            ESP_LOGE(TAG, "Error read data");
            snprintf((char *)arr, 200, "Error read data\n");
            esp_at_port_write_data(arr, strlen((char *)arr));
        }
        buffer[read_len] = 0;
        ESP_LOGD(TAG, "read_len = %d", read_len);
        // snprintf((char *)buffer, 513, buffer);
        esp_at_port_write_data((uint8_t *)buffer, strlen((char *)buffer));
    }
    ESP_LOGI(TAG, "\nHTTP Stream reader Status = %d, content_length = %d",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));

    snprintf((char *)arr, 200, "\nHTTP Stream reader Status = %d, content_length = %d\n",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));

    esp_at_port_write_data(arr, strlen((char *)arr));

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(buffer);
}

uint8_t at_exe_cmd_test(uint8_t *cmd_name)
{
    uint8_t buffer_2[64] = {0};

    snprintf((char *)buffer_2, 64, "this cmd is execute cmd: %s\r\n", cmd_name);

    esp_at_port_write_data(buffer_2, strlen((char *)buffer_2));

    // user-defined operation of sending data to server or MCU
    uint8_t data[15] = "pedro\n";

    // send_data_to_server();
    // pedro_function();
    esp_at_port_write_data(data, strlen((char *)data));
    // pedro_function();
    // http_download_chunk_2();
    // char output_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};
    char *buffer = malloc(PEDRO_SIZE + 1);

    // http_native_request_2(output_buffer);

    http_perform_as_stream_reader_pedro(buffer);
    spiffs(buffer);
    // save_fat_fs();
    // fat_fs_save(output_buffer);

    // output SEND OK
    esp_at_response_result(ESP_AT_RESULT_CODE_SEND_OK);

    return ESP_AT_RESULT_CODE_OK;
}

uint8_t at_exe_cmd_send_test(uint8_t *cmd_name)
{
    uint8_t buffer[64] = {0};

    snprintf((char *)buffer, 64, "this cmd is execute cmd: %s\r\n", cmd_name);

    esp_at_port_write_data(buffer, strlen((char *)buffer));

    // user-defined operation of sending data to server or MCU
    uint8_t data[15] = "pedro\n";

    // send_data_to_server();
    // pedro_function();
    esp_at_port_write_data(data, strlen((char *)data));

    // send_file_to_flash();

    // output SEND OK
    esp_at_response_result(ESP_AT_RESULT_CODE_SEND_OK);

    return ESP_AT_RESULT_CODE_OK;
}

// #define BUFFER_LEN (2048)
// static xSemaphoreHandle at_sync_sema_ = NULL;

// void wait_data_callback(void)
// {
// xSemaphoreGive(at_sync_sema);
// }

// uint8_t at_exe_cmd_test(uint8_t *cmd_name)
// {
//     int32_t received_len = 0;
//     int32_t remain_len = 0;
//     uint8_t *buf = NULL;
//     uint8_t buffer[64] = {0};

//     buf = (uint8_t *)malloc(BUFFER_LEN);
//     if (buf == NULL) {
//         memset(buffer, 0, 64);
//         snprintf((char *)buffer, 64, "malloc failed\r\n");
//         esp_at_port_write_data(buffer, strlen((char *)buffer));
//     }

//     // sample code
//     // users don't have to create semaphores here
//     if (!at_sync_sema) {
//         at_sync_sema = xSemaphoreCreateBinary();
//         assert(at_sync_sema != NULL);
//     }

//     // output input prompt ">"
//     esp_at_port_write_data((uint8_t *)">", strlen(">"));

//     // set the callback function which will be called by AT port after receiving the input data
//     esp_at_port_enter_specific(wait_data_callback);

//     // receie input data
//     while(xSemaphoreTake(at_sync_sema, portMAX_DELAY)) {
//         memset(buf, 0, BUFFER_LEN);

//         received_len = esp_at_port_read_data(buf, BUFFER_LEN);
//         // check whether to exit the mode
//         // the exit condition is the “+++” string received
//         if ((received_len == 3) && (strncmp((const char *)buf, "+++", 3)) == 0) {
//             esp_at_port_exit_specific();

//             // sample code
//             // if the remaining data length > 0, it means that there is still data left in the buffer to be processed
//             // users can customize the operation to process the remaining data
//             // here is just a simple print out of the remaining data
//             remain_len = esp_at_port_get_data_length();
//             if (remain_len > 0) {
//                 esp_at_port_recv_data_notify(remain_len, portMAX_DELAY);
//             }

//             break;
//         } else if (received_len > 0) {
//             // sample code
//             // users can customize the operation to process the received data
//             // here is just a simple print received data
//             memset(buffer, 0, 64);
//             snprintf((char *)buffer, 64, "\r\nreceived data is: ");
//             esp_at_port_write_data(buffer, strlen((char *)buffer));

//             esp_at_port_write_data(buf, strlen((char *)buf));
//         }
//     }

//     free(buf);

//     return ESP_AT_RESULT_CODE_OK;
// }

static esp_at_cmd_struct at_custom_cmd[] = {
    {"+UART", NULL, at_queryCmdUart, at_setupCmdUartDef, NULL},
    {"+UART_CUR", NULL, at_queryCmdUart, at_setupCmdUart, NULL},
    {"+UART_DEF", NULL, at_queryCmdUartDef, at_setupCmdUartDef, NULL},
    {"+TEST", at_test_cmd_test, at_query_cmd_test, at_setup_cmd_test, at_exe_cmd_test},
    // {"+SENDTEST", at_test_cmd_send_test, at_query_cmd_send_test, at_setup_cmd_send_test, at_exe_cmd_send_test},
    {"+SENDTEST", NULL, NULL, NULL, at_exe_cmd_send_test},
};

void at_status_callback(esp_at_status_type status)
{
    /**
     * ESP8266 CAN NOT provide uart_enable_pattern_det_baud_intr() feature due to hardware reason
     */
#ifndef CONFIG_IDF_TARGET_ESP8266
    switch (status)
    {
    case ESP_AT_STATUS_NORMAL:
        uart_disable_pattern_det_intr(esp_at_uart_port);
        break;

    case ESP_AT_STATUS_TRANSMIT:
    {
        /**
         * As the implement of API uart_enable_pattern_det_baud_intr() in esp-idf,
         * the last three timeout parameters is different on ESP32 and non ESP32 platform.
         *
         * That is, on ESP32 platform, it uses the APB clocks as the unit;
         * on non ESP32 platform (ESP32-S2, ESP32-C3, ..), it uses the UART baud rate clocks as the unit.
         *
         * Notes:
         * on non ESP32 platform, due to the value of input parameters have a limit of 0xFFFF (see as macro: UART_RX_GAP_TOUT_V..),
         * so the maximum uart baud rate is recommended to be less than (0xFFFF * 1000 / AT_UART_PATTERN_TIMEOUT_MS) = 3276750 ~= 3.2Mbps
         * otherwise, this uart_enable_pattern_det_baud_intr() will not work.
         */
#ifdef CONFIG_IDF_TARGET_ESP32
        int apb_clocks = (uint32_t)APB_CLK_FREQ * AT_UART_PATTERN_TIMEOUT_MS / 1000;
        uart_enable_pattern_det_baud_intr(esp_at_uart_port, '+', 3, apb_clocks, apb_clocks, apb_clocks);
#else
        uint32_t uart_baud = 0;
        uart_get_baudrate(esp_at_uart_port, &uart_baud);
        int uart_clocks = (uint32_t)uart_baud * AT_UART_PATTERN_TIMEOUT_MS / 1000;
        uart_enable_pattern_det_baud_intr(esp_at_uart_port, '+', 3, uart_clocks, uart_clocks, uart_clocks);
#endif
    }
    break;
    }
#endif
}

void at_pre_deepsleep_callback(void)
{
    /* Do something before deep sleep
     * Set uart pin for power saving, in case of leakage current
     */
    if (s_at_uart_port_pin.tx >= 0)
    {
        gpio_set_direction(s_at_uart_port_pin.tx, GPIO_MODE_DISABLE);
    }
    if (s_at_uart_port_pin.rx >= 0)
    {
        gpio_set_direction(s_at_uart_port_pin.rx, GPIO_MODE_DISABLE);
    }
    if (s_at_uart_port_pin.cts >= 0)
    {
        gpio_set_direction(s_at_uart_port_pin.cts, GPIO_MODE_DISABLE);
    }
    if (s_at_uart_port_pin.rts >= 0)
    {
        gpio_set_direction(s_at_uart_port_pin.rts, GPIO_MODE_DISABLE);
    }
}

void at_pre_restart_callback(void)
{
    /* Do something before restart
     */
    uart_disable_rx_intr(esp_at_uart_port);
    esp_at_port_wait_write_complete(ESP_AT_PORT_TX_WAIT_MS_MAX);
}

void at_interface_init(void)
{
    esp_at_device_ops_struct esp_at_device_ops = {
        .read_data = at_port_read_data,
        .write_data = at_port_write_data,
        .get_data_length = at_port_get_data_length,
        .wait_write_complete = at_port_wait_write_complete,
    };

    esp_at_custom_ops_struct esp_at_custom_ops = {
        .status_callback = at_status_callback,
        .pre_deepsleep_callback = at_pre_deepsleep_callback,
        .pre_restart_callback = at_pre_restart_callback,
    };

    at_uart_init();

    esp_at_device_ops_regist(&esp_at_device_ops);
    esp_at_custom_ops_regist(&esp_at_custom_ops);
}

void at_custom_init(void)
{
    esp_at_custom_cmd_array_regist(at_custom_cmd, sizeof(at_custom_cmd) / sizeof(at_custom_cmd[0]));

    esp_at_port_write_data((uint8_t *)"\r\nready\r\n", strlen("\r\nready\r\n"));
}
#endif