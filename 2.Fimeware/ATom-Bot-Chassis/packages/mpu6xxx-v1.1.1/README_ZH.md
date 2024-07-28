# mpu6xxx

中文页 | [English](README.md)

## 简介

本软件包是为 InvenSense 公司的六轴系列传感器提供的通用传感器驱动包，兼容 mpu6000、mpu6050、mpu6500、mpu9250、icm20608 等传感器。并且本软件包新的版本已经对接到了 Sensor 框架，通过 Sensor 框架，开发者可以快速的将此传感器驱动起来。若想查看**旧版软件包**的 README 请点击[这里](README_OLD.md)。

## 支持情况

| 包含设备         | 加速度计 | 陀螺仪 | 磁力计 |
| ---------------- | -------- | ------ | ------ |
| **通讯接口**     |          |        |        |
| IIC              | √        | √      | √      |
| SPI              | √        | √      | √      |
| **工作模式**     |          |        |        |
| 轮询             | √        | √      | √      |
| 中断             |          |        |        |
| FIFO             |          |        |        |
| **电源模式**     |          |        |        |
| 掉电             | √        | √      | √      |
| 低功耗           |          |        |        |
| 普通             | √        | √      | √      |
| 高功耗           |          |        |        |
| **数据输出速率** |          |        |        |
| **测量范围**     | √        | √      | √      |
| **自检**         |          |        |        |
| **多实例**       |          |        |        |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC/SPI 驱动：mpu6xxx 设备使用 IIC/SPI 进行数据通讯，需要系统 IIC/SPI 驱动支持；

### 获取软件包

使用 MPU6xxx 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

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

**Enable MPU6xxx acce**： 配置开启加速度计功能

**Enable MPU6xxx gyro**： 配置开启陀螺仪功能

**Enable MPU6xxx mag**： 配置开启磁力计功能

**Version**：软件包版本选择

### 使用软件包

MPU6xxx 软件包初始化函数如下所示：

```
int rt_hw_mpu6xxx_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息，配置接口设备和中断引脚）；
- 注册相应的传感器设备，完成 MPU6xxx 设备的注册；

#### 初始化示例

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

## 注意事项

暂无

## 联系人信息

维护人:

- [guozhanxin](https://github.com/Guozhanxin) 

- 主页：<https://github.com/RT-Thread-packages/mpu-6xxx>