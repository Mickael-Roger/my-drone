#pragma once

#include "esp_err.h"

esp_err_t camera_init(void);
void camera_stream_task(void *pvParameters);

