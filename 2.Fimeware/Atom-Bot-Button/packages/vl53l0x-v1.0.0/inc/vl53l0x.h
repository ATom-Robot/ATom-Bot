
#ifndef _VL53L0X_H_
#define _VL53L0X_H_

#include <rtthread.h>
#include "vl53l0x_platform.h"

extern VL53L0X_Dev_t vl53l0x_dev;
extern int rt_hw_vl53l0x_init(const char *name, struct rt_sensor_config *cfg, rt_base_t xsht_pin);

#endif
