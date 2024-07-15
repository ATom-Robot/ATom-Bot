#pragma once

#include "freertos/queue.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t App_Stream_run(const QueueHandle_t frame_i, const bool return_fb);

#ifdef __cplusplus
}
#endif
