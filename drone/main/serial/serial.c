#include "serial.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>
#include <stddef.h>



#define UART_PORT UART_NUM_1
#define UART_TX_PIN GPIO_NUM_17
#define UART_BAUD_RATE 19200

static const char *TAG = "serial";

void serial_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialisé à %d bauds sur TX GPIO %d", UART_BAUD_RATE, UART_TX_PIN);
}

void serial_send(const char *data)
{
    uart_write_bytes(UART_PORT, data, (size_t)1);
}

