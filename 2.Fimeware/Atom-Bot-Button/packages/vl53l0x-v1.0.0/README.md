# VL53L0X TOF传感器驱动软件包



## 1 简介

vl53l0x软件包是基于RT-Thread sensor框架实现的一个驱动包。基于该软件包，RT-Thread应用程序可以使用标准的sensor接口访问vl53l0x，获取传感器数据。



### 1.1 目录结构

| 名称       | 说明                           |
| ---------- | ------------------------------ |
| docs       | 文档目录                       |
| vl53l0x    | 官方库函数以及i2c platform对接 |
| examples   | 例子目录                       |
| inc        | 头文件目录                     |
| src        | 源代码目录                     |
| LICENSE    | 许可证文件                     |
| SConscript | RT-Thread默认构建脚本          |



### 1.2 许可证

vl53l0x软件包遵循 Apache license v2.0 许可，详见 `LICENSE` 文件。

<br>

## 2 传感器介绍

vl53l0x是 STMicroelectronics（意法半导体）公司推出的新一代单点TOF（Time of Flight）传感器，具备体积小、测量精度高、测量距离远、无需增加外部光学器件、使用简单等优点；此外，vl53l0x采用940nm肉眼不可见光源，集成物理红外过滤器，提高对环境光的抗干扰特性，具备良好的鲁棒性和防光学串扰特性。

测量参数：

| 功能 | 量程     | 分辨率 | 精度 |
| ---- | -------- | ------ | ---- |
| 距离 | 0—2000mm | 1mm    | —    |

应用场合：

* 相机对焦
* 1D手势识别
* 白色家电检测（额温枪、水龙头）
* 机器人避障

<br>

## 3 支持情况



| 包含设备     | TOF  |
| ------------ | ---- |
| **通信接口** |      |
| IIC          | √    |
| SPI          |      |
| **工作模式** |      |
| 轮询         | √    |
| 中断         |      |
| FIFO         |      |
| **电源模式** |      |
| 掉电         | √    |
| 低功耗       |      |
| 普通         | √    |

>注：
>
>目前暂时只支持单次测量模式；后续增加：
>
>* 连续测量模式
>* 定时测量模式
>* 测距校准功能

<br>



## 4 使用说明

### 4.1 依赖

- RT-Thread 4.0.0+

- sensor 框架组件

- I2C 驱动，vl53l0x使用 I2C 进行数据通讯，需要系统 I2C 驱动框架支持

- PIN驱动，用于vl53l0x开关机（复位）引脚控制

  

### 4.2 获取软件包

使用 vl53l0xpackage 需要在 RT-Thread 的包管理器中选择它，具体路径如下。然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。如需使用示例程序，则使能<code>Enable vl53l0x sample</code>。

```
RT-Thread online packages --->
    peripheral libraries and drivers --->
        sensors drivers --->
            [*] VL53L0X Time of flight(TOF) sensor.
            		[*] Enable vl53l0x sample
                    Version (latest)  --->
```

>  **Version**：软件包版本选择，默认选择最新版本。 



### 4.3 初始化

vl53l0x软件包初始化函数如下所示：

```c
int rt_hw_vl53l0x_init(const char *name, struct rt_sensor_config *cfg，rt_base_t xsht_pin);
```

| 参数      | 描述           |
| --------- | -------------- |
| name      | 传感器名称     |
| cfg       | sensor配置信息 |
| xsht      | 电源控制GPIO   |
| **返回**  | —              |
| RT_EOK    | 初始化成功     |
| -RT_ERROR | 初始化失败     |



该函数需要由用户调用，函数主要完成的功能有:

- 根据配置信息配置i2c名称、i2c地址等（可增加其他配置信息），然后初始化设备

- 选择电源控制GPIO

- 注册相应的传感器设备，完成 vl53l0x设备的注册

  

**参考示例：**

```c
#include "vl53l0x.h"

static int rt_hw_vl53l0x_port(void)
{
    struct rt_sensor_config cfg;
    	
	cfg.intf.dev_name = "i2c1"; 		/* i2c bus */
    cfg.intf.user_data = (void *)0x29;	/* i2c slave addr */
    rt_hw_vl53l0x_init("vl53l0x", &cfg, 57);/* xshutdown ctrl pin */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_vl53l0x_port);
```



### 4.4 读取数据

vl53l0x软件包基于sensor框架，sensor框架继承于RT-Thread标准设备框架，可以使用RT-Thread标准设备接口"open/read"读取传感器数据。



**参考伪代码:**

```
temp_dev = rt_device_find("tof_vl53l0x");
rt_device_open(temp_dev, RT_DEVICE_FLAG_RDONLY)；
rt_device_read(temp_dev, 0, &sensor_data, 1);
```



### 4.5 msh/finsh测试

#### 查看设备注册

```
msh >list_device
device           type         ref count
-------- -------------------- ----------
tof_vl53 Sensor Device        0       
i2c1     I2C Bus              0       
pin      Miscellaneous Device 0             
uart2    Character Device     0       
uart1    Character Device     2       
```

>注：
>
>完整设备名称为“tof_vl53l0x”，终端显示有长度限制



#### 运行例程周期打印数据

```b
 \ | /
- RT -     Thread Operating System
 / | \     4.0.1 build Dec 17 2020
 2006 - 2019 Copyright by rt-thread team
[I/sensor] rt_sensor init success
[I/vl53l0x] vl53l0x info:
      Name[VL53L0X ES1 or later]
      Type[VL53L0X]
      ProductId[VL53L0CBV0DH/1$1]

msh >distance[153mm],timestamp[87]
distance[161mm],timestamp[192]
distance[154mm],timestamp[297]
distance[152mm],timestamp[402]
```



#### 使用RTT自带的测试命令

* 探测设备

```b
msh >sensor probe tof_vl53l0x
[I/sensor.cmd] device id: 0xee!
```



* 获取传感器信息

```
msh >sensor info             
vendor    :STMicroelectronics
model     :vl53l0x
unit      :mm
range_max :2000
range_min :0
period_min:100ms
fifo_max  :0
```

> 注：
>
> sensor框架暂未提供tof传感器相关描述信息，已在RT-Thread源码上PR。

* 读取测距数据

```b
msh >sensor read 3
[I/sensor.cmd] num:  0, distance:  212, timestamp:131863
[I/sensor.cmd] num:  1, distance:  213, timestamp:131878
[I/sensor.cmd] num:  2, distance:  213, timestamp:131893
```

<br>

## 5 注意事项

暂无

<br>

## 6 联系方式

- 维护：[Acuity](https://github.com/Prry)
- 主页：<https://github.com/Prry/rtt-vl53l0x>    