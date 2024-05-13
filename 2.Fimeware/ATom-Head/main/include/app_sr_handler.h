/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

    void en_sr_detect_task(void);

    bool sr_echo_is_playing(void);

    void sr_handler_task(void *pvParam);

#ifdef __cplusplus
}
#endif
