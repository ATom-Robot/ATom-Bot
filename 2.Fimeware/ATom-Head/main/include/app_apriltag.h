#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

void register_apriltag_detection(const QueueHandle_t frame_i,
                                 const QueueHandle_t frame_o,
                                 const bool camera_fb_return);
