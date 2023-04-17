# API

`int cpu_usage_init(void)`

初始化 CPU 使用率统计功能。必须初始化之后才能获取正确的使用率

| 参数 | 描述 |
| ---- | ----|
| 无 | |
| **返回** | **描述** | 
| 0 | 总是返回 0 |

`void cpu_usage_deinit(void)`

反初始化 CPU 使用率统计功能。当统计功能不需要时，调用此函数释放系统资源

| 参数 | 描述 |
| ---- | ----|
| 无 | |
| **返回** | **描述** | 
| 无 | 无 |

`cpu_usage_t *cpu_usage_obj(void)`

获取统计对象，可以从统计对象中获得更详细的数据

| 参数 | 描述 |
| ---- | ----|
| 无 | |
| **返回** | **描述** | 
| cpu_usage_t | CPU 统计对象 |

统计对象成员介绍

| 成员 | 描述 |
| ---- | ----|
| time | 与系统心跳同频定时器 |
| state | 运行状态 |
| cpus | CPU 数量 |
| suspend_tick | 上次挂起时的 tick 值 |
| period | 统计周期 |
| idle_stat.idle_tick | idle 运行总时长（tick） |
| idle_stat.last_tick | 上个周期开始时间戳（tick）|
| idle_stat.load      | 上个周期其他线程执行时长（tick）|
| idle_stat.tid       | idle 线程对象 |

统计状态介绍

| 状态 | 描述 |
| ---- | ----|
| CPU_USAGE_STATE_DEACTIVATED | 未初始化，停止状态 |
| CPU_USAGE_STATE_ACTIVATED | 正在运行 |
| CPU_USAGE_STATE_SUSPEND | 被挂起，等待恢复 |

`void cpu_usage_suspend(void)`

挂起 CPU 使用率统计功能。系统低功耗期间使用。

| 参数 | 描述 |
| ---- | ----|
| 无 | |
| **返回** | **描述** | 
| 无 | 无 |

`void cpu_usage_resume(void)`

恢复 CPU 使用率统计功能。低功耗恢复时使用。

| 参数 | 描述 |
| ---- | ----|
| 无 | |
| **返回** | **描述** | 
| 无 | 无 |

`float cpu_load_average(void)`

获取 CPU 平均使用率

| 参数 | 描述 |
| ---- | ----|
| 无 | |
| **返回** | **描述** | 
| float | CPU 平均使用率 |
