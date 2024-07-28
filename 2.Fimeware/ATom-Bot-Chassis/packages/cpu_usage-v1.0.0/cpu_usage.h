/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-04     tyx          first implementation
 */

#ifndef __CPU_USAGE_H__
#define __CPU_USAGE_H__

#include "rtthread.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CPU_USAGE_STATE_DEACTIVATED  (0x0)
#define CPU_USAGE_STATE_ACTIVATED    (0x1)
#define CPU_USAGE_STATE_SUSPEND      (0x2)

struct cpu_usage
{
    struct rt_timer time;           /* Period timer */
    rt_uint8_t state;               /* Running state */
    rt_uint8_t cpus;                /* Number of CPUs */
    rt_uint16_t reserved;
    rt_tick_t suspend_tick;         /* CPU suspend timestamp */
    rt_tick_t period;               /* Stat Period */
    struct _idle_stat
    {
        rt_tick_t idle_tick;
        rt_tick_t last_tick;
        rt_tick_t load;
        rt_thread_t tid;
    }
    idle_stat[1];
};
typedef struct cpu_usage cpu_usage_t;

int cpu_usage_init(void);
void cpu_usage_deinit(void);
cpu_usage_t *cpu_usage_obj(void);
void cpu_usage_suspend(void);
void cpu_usage_resume(void);
float cpu_load_average(void);

#ifdef __cplusplus
}
#endif

#endif
