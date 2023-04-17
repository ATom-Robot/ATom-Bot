# mpu6xxx

[中文页](README_ZH.md) | English

## Introduction

This software package is a universal sensor driver package for InvenSense's six-axis series sensors, compatible with mpu6000, mpu6050, mpu6500, mpu9250, icm20608 and other sensors. And the new version of this software package has been connected to the Sensor framework, through the Sensor framework, developers can quickly drive this sensor. To view the README of the **old version of the package**, please click [here](README_OLD.md).

## Support

| Contains equipment          | Accelerometer | Gyroscope | Magnetometer |
| --------------------------- | ------------- | --------- | ------------ |
| **Communication Interface** |               |           |              |
| IIC                         | √             | √         | √            |
| SPI                         | √             | √         | √            |
| **Work Mode**               |               |           |              |
| Polling                     | √             | √         | √            |
| Interruption                |               |           |              |
| FIFO                        |               |           |              |
| **Power Mode**              |               |           |              |
| Power down                  | √             | √         | √            |
| Low power consumption       |               |           |              |
| Normal                      | √             | √         | √            |
| High power consumption      |               |           |              |
| **Data output rate**        |               |           |              |
| **Measuring Range**         | √             | √         | √            |
| **Self-check**              |               |           |              |
| **Multi-instance**          |               |           |              |

## Instructions for use

### Dependence

- RT-Thread 4.0.0+
- Sensor component
- IIC/SPI driver: mpu6xxx devices use IIC/SPI for data communication, and need system IIC/SPI driver support;

### Get the package

To use the MPU6xxx software package, you need to select it in the RT-Thread package management. The specific path is as follows:

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
      mpu6xxx: Universal 6-axis sensor driver package,support: accelerometer, gyroscope.
              Version (latest)  --->
        [*]   Enable mpu6xxx acce
        [*]   Enable mpu6xxx gyro
        [*]   Enable mpu6xxx mag
```

**Enable MPU6xxx acce**: Configure to enable the accelerometer function

**Enable MPU6xxx gyro**: Configure to turn on the gyroscope function

**Enable MPU6xxx mag**: Configure to turn on the Magnetometer function

**Version**: software package version selection

### Using packages

The initialization function of MPU6xxx software package is as follows:

```
int rt_hw_mpu6xxx_init(const char *name, struct rt_sensor_config *cfg);
```

This function needs to be called by the user. The main functions of the function are:

- Device configuration and initialization (configure interface devices and interrupt pins according to the incoming configuration information);
- Register the corresponding sensor device and complete the registration of the MPU6xxx device;

#### Initialization example

```
#include "sensor_inven_mpu6xxx.h"

int rt_hw_mpu6xxx_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)MPU6XXX_ADDR_DEFAULT;
    cfg.irq_pin.pin = RT_PIN_NONE;

    rt_hw_mpu6xxx_init("mpu", &cfg);
    return 0;
}
INIT_APP_EXPORT(rt_hw_mpu6xxx_port);
```

## Precautions

No

## contact information

Maintenance man:

- [guozhanxin](https://github.com/Guozhanxin)

- Homepage: <https://github.com/RT-Thread-packages/mpu-6xxx>
