/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-04     tyx          first implementation
 */

#include "cpu_usage.h"

#ifndef CPU_USAGE_PERIOD_TICK
#define CPU_USAGE_PERIOD_TICK RT_TICK_PER_SECOND
#endif

cpu_usage_t *cpu_usage_obj(void)
{
    static struct cpu_usage _usage;
    return &_usage;
}

static void timeout(void *param)
{
    cpu_usage_t *obj = param;

    if (obj->state == CPU_USAGE_STATE_ACTIVATED)
    {
        if (++obj->period == CPU_USAGE_PERIOD_TICK)
        {
            obj->period = 0;
            obj->idle_stat[0].load = CPU_USAGE_PERIOD_TICK - 
                (obj->idle_stat[0].idle_tick - obj->idle_stat[0].last_tick);
            obj->idle_stat[0].last_tick = obj->idle_stat[0].idle_tick;
        }

        if (rt_thread_self() == obj->idle_stat[0].tid)
        {
            obj->idle_stat[0].idle_tick++;
        }
    }
}

int cpu_usage_init(void)
{
    cpu_usage_t *obj = cpu_usage_obj();
    rt_timer_t t = &obj->time;
    char idle_name[RT_NAME_MAX];
    int i;

    if (rt_object_get_type(&t->parent) != RT_Object_Class_Timer)
    {
        /* init time */
        rt_timer_init(t, "usage", timeout, obj, 1,
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
        /* set cpus */
        obj->cpus = sizeof(obj->idle_stat) / sizeof(obj->idle_stat[0]);
        /* get idle thread handle */
        for (i = 0; i < obj->cpus; i++)
        {
            rt_snprintf(idle_name, sizeof(idle_name), "tidle%d", i);
            obj->idle_stat[i].tid = rt_thread_find(idle_name);
        }
        /* set flags */
        obj->state = CPU_USAGE_STATE_ACTIVATED;
        /* start */
        rt_timer_start(t);
    }
    return 0;
}

void cpu_usage_deinit(void)
{
    cpu_usage_t *obj = cpu_usage_obj();
    rt_timer_t t = &obj->time;

    if (rt_object_get_type(&t->parent) == RT_Object_Class_Timer)
    {
        rt_timer_stop(t);
        rt_timer_detach(t);
        rt_memset(obj, 0, sizeof(*obj));
    }
}

void cpu_usage_suspend(void)
{
    cpu_usage_t *obj = cpu_usage_obj();
    rt_timer_t t = &obj->time;

    if (obj->state == CPU_USAGE_STATE_ACTIVATED)
    {
        rt_timer_stop(t);
        obj->state = CPU_USAGE_STATE_SUSPEND;
        obj->suspend_tick = rt_tick_get();
    }
}

void cpu_usage_resume(void)
{
    cpu_usage_t *obj = cpu_usage_obj();
    rt_timer_t t = &obj->time;
    rt_tick_t tick;
    int i;

    if (obj->state == CPU_USAGE_STATE_SUSPEND)
    {
        tick = rt_tick_get() - obj->suspend_tick;
        for (i = 0; i < obj->cpus; i++)
        {
            obj->idle_stat[i].idle_tick += tick;
        }
        obj->state = CPU_USAGE_STATE_ACTIVATED;
        rt_timer_start(t);
    }
}

float cpu_load_average(void)
{
    cpu_usage_t *obj = cpu_usage_obj();
    rt_tick_t usage = 0;
    float load = 0.0;
    int i;

    if (obj->state == CPU_USAGE_STATE_ACTIVATED)
    {
        for (i = 0; i < obj->cpus; i++)
        {
            usage += obj->idle_stat[i].load;
        }
        load = 100.0 * usage / (CPU_USAGE_PERIOD_TICK * obj->cpus);
    }
    return load;
}

#ifdef RT_USING_COMPONENTS_INIT
INIT_APP_EXPORT(cpu_usage_init);
#endif
