# mpu6xxx 软件包

## 介绍

`mpu6xxx` 软件包是 RT-Thread 针对 InvenSense 公司的六轴系列传感器推出的一个软件包，兼容 mpu6000、mpu6050、mpu6500、mpu9250、icm20608 等传感器。使用这个软件包，可以在 RT-Thread 上非常方便的使用该系列传感器，实现参数的配置和数据的读取——包括读取三轴加速度（3-axis accelerometer）、三轴陀螺仪（3-axis gyroscope）和温度值。

软件包具有以下优点：

- 同时支持 I2C、SPI 协议 
- 方便的参数配置(包括：采样率、量程、低通滤波器等)
- 返回的数据单位统一（accel:mg、gyro:degC/10s、temp:degC）

本文主要介绍该软件包的使用方式、API，以及 `MSH` 测试命令。

### 目录结构

```
mpu6xxx
│   README.md                       // 软件包说明
│   SConscript                      // RT-Thread 默认的构建脚本
│   LICENSE                         // 许可证文件
├───docs 
│   |   api.md                      // API 使用说明
├───samples                         // 示例代码
│   |   mpu6xxx_sample.c            // 软件包使用示例代码
└───src                             // 源文件
└───inc                             // 头文件
```

### 许可证

mpu6xxx 遵循 Apache-2.0 许可，详见 `LICENSE` 文件。

### 依赖

- RT_Thread 3.0+
- i2c/spi 设备驱动

## 获取方式

使用 `mpu6xxx package` 需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
      mpu6xxx: Universal 6-axis sensor driver package,support: accelerometer, gyroscope.
```

进入 mpu6xxx 软件包的配置菜单按自己的需求进行具体的配置

```
    --- mpu6xxx: Universal 6-axis sensor driver library       
              Version (v0.0.1)  --->
        [*]   Enable mpu6xxx sample
```

**Enable mpu6xxx sample** ：开启 mpu6xxx  使用示例

配置完成后让 RT-Thread 的包管理器自动更新，或者使用 pkgs --update 命令更新包到 BSP 中。

## 使用方法

mpu6xxx 软件包的使用流程一般如下：

1. 初始化 mpu6xxx 设备 `mpu6xxx_init`
2. 配置相关参数 `mpu6xxx_set_param` (也可以不调用，使用默认参数)
3. 读取传感器数据 `mpu6xxx_get_gyro/accel/temp`

详细的使用方法可以参考[mpu6xxx 示例程序](samples/mpu6xxx_sample.c) 和 [API 说明文档](doc/api.md)。

## MSH 测试命令

mpu6xxx软件包提供了丰富的测试命令，项目只要在 RT-Thread 上开启 MSH 功能即可。在做一些基于 `mpu6xxx` 的应用开发、调试时，这些命令会非常实用，它可以探测传感器设备、读取三轴加速度、陀螺仪与温度值。具体功能可以输入 `mpu6xxx` ，查看完整的命令列表

```
msh >mpu6xxx

mpu6xxx [OPTION] [PARAM]
         probe <dev_name>      Probe mpu6xxx by given name
         sr <var>              Set sample rate to var
                               var = [1000 -  4] when dlpf is enable
                               var = [8000 - 32] when dlpf is disable
         gr <var>              Set gyro range to var
                               var = [0 - 3] means [250 - 2000DPS]
         ar <var>              Set accel range to var
                               var = [0 - 3] means [2 - 16G]
         sleep <var>           Set sleep status
                               var = 0 means disable, = 1 means enable
         read [num]            read [num] times mpu6xxx
                               num default 5
```

### 在指定的设备上探测传感器

当第一次使用 `mpu6xxx` 命令时，直接输入 `mpu6xxx probe <dev_name>` ，其中 `<dev_name>` 为指定的 `i2c/spi` 设备的名称，例如：`i2c1` 或 `spi10`。如果有传感器连在总线设备下，就不会提示错误；如果总线上没有这个传感器，将会显示提示找不到相关设备，日志如下：

```
msh >mpu6xxx probe i2c2
[I/MPU6XXX] Find device: mpu6050!
[I/MPU6XXX] mpu6xxx init succeed
msh >mpu6xxx probe spi10
[I/MPU6XXX] Find device: mpu6500!
[I/MPU6XXX] mpu6xxx init succeed
msh >mpu6xxx probe spi11
[E/MPU6XXX] Can't find device:'spi11'
msh >
```

### 读取数据

输入 `mpu6xxx read [num]` 命令，即可按照读取次数（可选参数 num ，默认值为 5 ）读取三轴加速度、三轴陀螺仪与温度检测值，日志如下：

```
msh >mpu6xxx read 2
accel.x =  12, accel.y =  -2, accel.z = 977 gyro.x =  -9 gyro.y =  13, gyro.z = -11, temp = 28.82
accel.x =  12, accel.y =   5, accel.z = 971 gyro.x =  -3 gyro.y =  15, gyro.z =  -7, temp = 28.78
msh >
```

## 注意事项

暂无。

## 联系方式

- 维护：[guozhanxin](https://github.com/Guozhanxin)
- 主页：<https://github.com/RT-Thread-packages/mpu-6xxx>