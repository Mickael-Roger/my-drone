#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "wifi/wifi.h"
#include "command/command.h"
//#include "icm/icm.h"

static const char *TAG = "main";

void app_main(void)
{

    float heading, roll, pitch;

    ESP_LOGI(TAG, "Initialisation du module Wi-Fi...");
    wifi_init_softap();


    ESP_LOGI(TAG, "En attente de connexion client...");

    // Boucle d'attente pour une connexion (exemple simple, à adapter ensuite)
    while (!wifi_is_client_connected()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Client connecté au point d'accès !");

    flying_command();

    //get_icm(&heading, &roll, &pitch);

    //while(1){
    //    ESP_LOGI(TAG, "Heading: %.2f°, Roll: %.2f°, Pitch: %.2f°\n", heading, roll, pitch);
    //    printf("Heading: %.2f°, Roll: %.2f°, Pitch: %.2f°\n", heading, roll, pitch);
    //}

}

