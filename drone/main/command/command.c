#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "serial/serial.h"


#define PORT 7099
#define TAG "COMMAND"

static bool client_ready = false;
static struct sockaddr_in client_addr;

static void print_hex(const uint8_t *data, int len) {
    printf("Received %d bytes: ", len);
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

void udp_server_task(void *pvParameters) {
    char rx_buffer[128];
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in server_addr;

    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    ESP_LOGI(TAG, "UDP server listening on port %d", PORT);

    while (1) {
        socklen_t socklen = sizeof(client_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
                           (struct sockaddr *)&client_addr, &socklen);
        if (len > 1) {
            print_hex((uint8_t *)rx_buffer, len);

            printf("Send: ");
            for (size_t i = 1; i < len; i++) {
               printf("%02X ", rx_buffer[i]);
               serial_send(&rx_buffer[i]);
            }
            printf("\n");

            if (!client_ready) {
                client_ready = true;
                ESP_LOGI(TAG, "Client registered: %s:%d",
                         inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    close(sock);
    vTaskDelete(NULL);
}

void keepalive_task(void *pvParameters) {
    const uint8_t ping_msg[] = {0x48, 0x01, 0x00, 0x00, 0x00};
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    while (1) {
        if (client_ready) {
            sendto(sock, ping_msg, sizeof(ping_msg), 0,
                   (struct sockaddr *)&client_addr, sizeof(client_addr));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    close(sock);
    vTaskDelete(NULL);
}

void flying_command() {

    ESP_LOGI(TAG, "Init Serial");
    serial_init();

    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    xTaskCreate(keepalive_task, "keepalive", 2048, NULL, 4, NULL);
}

