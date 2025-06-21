#include "camera.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "lwip/sockets.h"
#include "string.h"

#define TAG "CAMERA"

// === CONFIG CAMERA ===
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13


#define LED_GPIO_NUM      21

#define CAM_XCLK_FREQ   20000000

#define UDP_TARGET_IP   "192.168.100.2"
#define UDP_TARGET_PORT 5000
#define MAX_UDP_PACKET_SIZE 1400


#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif


static int udp_socket;
static struct sockaddr_in dest_addr;

esp_err_t camera_init()
{

    camera_config_t config = {



        .pin_pwdn     = PWDN_GPIO_NUM,
        .pin_reset    = RESET_GPIO_NUM,
        .pin_xclk     = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,

        .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,

        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href  = HREF_GPIO_NUM,
        .pin_pclk  = PCLK_GPIO_NUM,

        .xclk_freq_hz = 20 * 1000 * 1000,       // 20 MHz OK
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_QQVGA,        // 160×120
        .jpeg_quality = 15,

        .fb_count   = 2,                        // ➜ double buffer
        .fb_location= CAMERA_FB_IN_PSRAM,       // ➜ 8 Mo dispo
        .grab_mode  = CAMERA_GRAB_LATEST        // évite empilement


    };




    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erreur init camera: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Camera initialisée");
    return ESP_OK;
}

static esp_err_t udp_init()
{
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Erreur création socket UDP");
        return ESP_FAIL;
    }

    dest_addr.sin_addr.s_addr = inet_addr(UDP_TARGET_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_TARGET_PORT);

    ESP_LOGI(TAG, "Socket UDP prêt vers %s:%d", UDP_TARGET_IP, UDP_TARGET_PORT);
    return ESP_OK;
}

void camera_stream_task(void *pvParameters)
{
    if (udp_init() != ESP_OK) {
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Capture frame échouée");
            continue;
        }

        // Fragmentation si image > MAX_UDP_PACKET_SIZE
        size_t bytes_sent = 0;
        while (bytes_sent < fb->len) {
            size_t chunk_size = MIN(MAX_UDP_PACKET_SIZE, fb->len - bytes_sent);
            sendto(udp_socket, fb->buf + bytes_sent, chunk_size, 0,
                   (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            bytes_sent += chunk_size;
        }

        esp_camera_fb_return(fb);
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100 FPS max (à ajuster)
    }
}


