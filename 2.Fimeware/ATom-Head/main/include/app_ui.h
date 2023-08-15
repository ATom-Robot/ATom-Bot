#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ui_wakeup_emoji_start(void);
void ui_wakeup_emoji_over(void);

void ui_acquire(void);
void ui_release(void);

#ifdef __cplusplus
}
#endif
