#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int thro;
    int yaw;
    int angle;
} Chassis_data;

esp_err_t APPUart_Init(void);
void data_sendto_ChassisData(int32_t _a, int32_t _b, int32_t _c, int32_t _d);

#ifdef __cplusplus
}
#endif
