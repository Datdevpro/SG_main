#include "data_sender.h"
#include "driver/uart.h"
#include <string.h>
#include <stdbool.h>
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
//#include "esp_log.h"
//#include "esp_sntp.h"
//#include "esp_wifi.h"
//#include "nvs_flash.h"
//#include "esp_event.h"
//#include <time.h>

#define UART_PORT UART_NUM_2
#define TXD_PIN  43
#define RXD_PIN   44

void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT  // ✅ VALID src_clk for ESP32-S3
    };
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uart_send_json(int id, bool snore_detected, float ahi) {
    char json_buffer[200];
    // Format JSON string
    snprintf(json_buffer, sizeof(json_buffer),
    "{\"id\": %d, \"snore_detected\": %s,\"AHI_level\": %.1f}\n",
    id,
    snore_detected ? "true" : "false",
    ahi
    );  // <- \n is end marker

    // Send over UART1
    uart_write_bytes(UART_PORT, json_buffer, strlen(json_buffer));

}

void send_flag_start_time(bool flag_snore) // send the 2 boolean values for count time start snore and pumping
{
    //bool flag[2] = {flag_snore, flag_pump};
    uint8_t data = flag_snore ? 1 : 0;
    uart_write_bytes(UART_PORT, (const char*)&data, 1);

}


void uart_receive_response() {
    uint8_t rx_buffer[200];
    int length = 0;

    // Kiểm tra xem có dữ liệu trong buffer không
    if (uart_get_buffered_data_len(UART_PORT, (size_t*)&length) == ESP_OK && length > 0) {
        int read = uart_read_bytes(UART_PORT, rx_buffer, length, 100 / portTICK_PERIOD_MS);
        if (read > 0) {
            rx_buffer[read] = '\0'; // null-terminate để in được chuỗi
            printf("Received from ESP32: %s\n", (char*)rx_buffer);
        }
    }
}