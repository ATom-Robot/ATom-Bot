// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp32-hal-timer.h"
#include "driver/timer.h"
#include "soc/soc_caps.h"
#include "esp_log.h"

typedef union
{
    struct
    {
        uint32_t reserved0:   10;
        uint32_t alarm_en:     1;             /*When set  alarm is enabled*/
        uint32_t level_int_en: 1;             /*When set  level type interrupt will be generated during alarm*/
        uint32_t edge_int_en:  1;             /*When set  edge type interrupt will be generated during alarm*/
        uint32_t divider:     16;             /*Timer clock (T0/1_clk) pre-scale value.*/
        uint32_t autoreload:   1;             /*When set  timer 0/1 auto-reload at alarming is enabled*/
        uint32_t increase:     1;             /*When set  timer 0/1 time-base counter increment. When cleared timer 0 time-base counter decrement.*/
        uint32_t enable:       1;             /*When set timer 0/1 time-base counter is enabled*/
    };
    uint32_t val;
} timer_cfg_t;

#define NUM_OF_TIMERS   SOC_TIMER_GROUP_TOTAL_TIMERS
static const char *TAG = "hal-timer";

typedef struct hw_timer_s
{
    uint8_t group;
    uint8_t num;
} hw_timer_t;

// Works for all chips
static hw_timer_t timer_dev[4] =
{
    {0, 0}, {1, 0},  {0, 1},  {1, 1}
};

// NOTE: (in IDF 5.0 there wont be need to know groups/numbers
// timer_init() will list thru all timers and return free timer handle)


inline uint64_t timerRead(hw_timer_t *timer)
{

    uint64_t value;
    timer_get_counter_value(timer->group, timer->num, &value);
    return value;
}

uint64_t timerAlarmRead(hw_timer_t *timer)
{
    uint64_t value;
    timer_get_alarm_value(timer->group, timer->num, &value);
    return value;
}

void timerWrite(hw_timer_t *timer, uint64_t val)
{
    timer_set_counter_value(timer->group, timer->num, val);
}

void timerAlarmWrite(hw_timer_t *timer, uint64_t alarm_value, bool autoreload)
{
    timer_set_alarm_value(timer->group, timer->num, alarm_value);
    timerSetAutoReload(timer, autoreload);
}

void timerSetConfig(hw_timer_t *timer, uint32_t config)
{
    timer_cfg_t cfg;
    cfg.val = config;
    timer_set_alarm(timer->group, timer->num, cfg.alarm_en);
    timerSetDivider(timer, cfg.divider);
    timerSetAutoReload(timer, cfg.autoreload);
    timerSetCountUp(timer, cfg.increase);

    if (cfg.enable)
    {
        timerStart(timer);
    }
    else
    {
        timerStop(timer);
    }
    return;
}

uint32_t timerGetConfig(hw_timer_t *timer)
{
    timer_config_t timer_cfg;
    timer_get_config(timer->group, timer->num, &timer_cfg);

    //Translate to default uint32_t
    timer_cfg_t cfg;
    cfg.alarm_en = timer_cfg.alarm_en;
    cfg.autoreload = timer_cfg.auto_reload;
    cfg.divider = timer_cfg.divider;
    cfg.edge_int_en = timer_cfg.intr_type;
    cfg.level_int_en = !timer_cfg.intr_type;
    cfg.enable = timer_cfg.counter_en;
    cfg.increase = timer_cfg.counter_dir;

    return cfg.val;
}

void timerSetCountUp(hw_timer_t *timer, bool countUp)
{
    timer_set_counter_mode(timer->group, timer->num, countUp);
}

bool timerGetCountUp(hw_timer_t *timer)
{
    timer_cfg_t config;
    config.val = timerGetConfig(timer);
    return config.increase;
}

void timerSetAutoReload(hw_timer_t *timer, bool autoreload)
{
    timer_set_auto_reload(timer->group, timer->num, autoreload);
}

bool timerGetAutoReload(hw_timer_t *timer)
{
    timer_cfg_t config;
    config.val = timerGetConfig(timer);
    return config.autoreload;
}

// Set divider from 2 to 65535
void timerSetDivider(hw_timer_t *timer, uint16_t divider)
{
    if (divider < 2)
    {
        ESP_LOGE(TAG, "Timer divider must be set in range of 2 to 65535");
        return;
    }
    timer_set_divider(timer->group, timer->num, divider);
}

uint16_t timerGetDivider(hw_timer_t *timer)
{
    timer_cfg_t config;
    config.val = timerGetConfig(timer);
    return config.divider;
}

void timerStart(hw_timer_t *timer)
{
    timer_start(timer->group, timer->num);
}

void timerStop(hw_timer_t *timer)
{
    timer_pause(timer->group, timer->num);
}

void timerRestart(hw_timer_t *timer)
{
    timerWrite(timer, 0);
}

bool timerStarted(hw_timer_t *timer)
{
    timer_cfg_t config;
    config.val = timerGetConfig(timer);
    return config.enable;
}

void timerAlarmEnable(hw_timer_t *timer)
{
    timer_set_alarm(timer->group, timer->num, true);
}

void timerAlarmDisable(hw_timer_t *timer)
{
    timer_set_alarm(timer->group, timer->num, false);
}

bool timerAlarmEnabled(hw_timer_t *timer)
{
    timer_cfg_t config;
    config.val = timerGetConfig(timer);
    return config.alarm_en;
}

static void _on_apb_change(void *arg, apb_change_ev_t ev_type, uint32_t old_apb, uint32_t new_apb)
{
    hw_timer_t *timer = (hw_timer_t *)arg;
    if (ev_type == APB_BEFORE_CHANGE)
    {
        timerStop(timer);
    }
    else
    {
        old_apb /= 1000000;
        new_apb /= 1000000;
        uint16_t divider = (new_apb * timerGetDivider(timer)) / old_apb;
        timerSetDivider(timer, divider);
        timerStart(timer);
    }
}

bool IRAM_ATTR timerFnWrapper(void *arg)
{
    void (*fn)(void) = arg;
    fn();

    // some additional logic or handling may be required here to approriately yield or not
    return false;
}

void timerAttachInterruptFlag(hw_timer_t *timer, void (*fn)(void), bool edge, int intr_alloc_flags)
{
    if (edge)
    {
        ESP_LOGW(TAG, "EDGE timer interrupt is not supported! Setting to LEVEL...");
    }
    timer_isr_callback_add(timer->group, timer->num, timerFnWrapper, fn, intr_alloc_flags);
}

void timerAttachInterrupt(hw_timer_t *timer, void (*fn)(void), bool edge)
{
    timerAttachInterruptFlag(timer, fn, edge, 0);
}

void timerDetachInterrupt(hw_timer_t *timer)
{
    timer_isr_callback_remove(timer->group, timer->num);
}
