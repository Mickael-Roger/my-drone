#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "wifi/wifi.h"
#include "command/command.h"
#include "icm/icm.h"
#include "camera.h"


static const char *TAG = "main";



void app_main(void)
{

    float heading, roll, pitch;

    ESP_LOGI(TAG, "Initialisation du module Wi-Fi...");
    wifi_init_softap();

    //get_icm(&heading, &roll, &pitch);

    ESP_LOGI(TAG, "En attente de connexion client...");

    // Boucle d'attente pour une connexion (exemple simple, à adapter ensuite)
    while (!wifi_is_client_connected()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Client connecté au point d'accès !");

    flying_command();


    if (camera_init() == ESP_OK) {
        xTaskCreatePinnedToCore(camera_stream_task, "cam_stream", 4096, NULL, 5, NULL, 1);
    }

    //get_icm(&heading, &roll, &pitch);

    while(1){
        vTaskDelay(pdMS_TO_TICKS(10));
        //ESP_LOGI(TAG, "Heading: %.2f°, Roll: %.2f°, Pitch: %.2f°\n", heading, roll, pitch);
        //printf("Heading: %.2f°, Roll: %.2f°, Pitch: %.2f°\n", heading, roll, pitch);
    }

}

