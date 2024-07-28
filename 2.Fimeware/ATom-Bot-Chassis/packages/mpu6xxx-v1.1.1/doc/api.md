# API 说明

在 RT-Thread 上编程，需要用到 mpu6000/6050/6500/9250/icm20608 等 6 轴传感器时，使用 mpu6xxx 软件包就可以轻松完成传感器的配置以及传感器数据的读取，本章介绍 mpu6xxx 软件包提供的常用 API。

### 初始化函数

```{.c}
struct mpu6xxx_device *mpu6xxx_init(const char *dev_name, rt_uint8_t param);
```

使用指定的通信设备（I2C/SPI）初始化mpu6xxx ，并返回控制句柄。

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev_name               | 用于同 mpu6xxx 通信的设备名（支持 I2C 总线设备和 SPI 设备） |
|param | I2C 通信，根据此处传入的 I2C 地址寻找设备（例如：0x68） |
| **返回**          | **描述**                                |
|struct mpu6xxx_device *                  | mpu6xxx_device 结构体的指针，它在调用 mpu6xxx 库的其他函数时使用 |
|NULL                 | 失败                                |

### 反初始化函数

```{.c}
void mpu6xxx_deinit(struct mpu6xxx_device *dev);
```

释放 mpu6xxx 设备占据的内存空间

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | mpu6xxx_device 结构体的指针 |
| **返回** | **描述**                    |
| 无返回值 |                             |

### 设定参数

```{.c}
rt_err_t mpu6xxx_set_param(struct mpu6xxx_device *dev, enum mpu6xxx_cmd cmd, rt_uint16_t param);
```

为挂载上的 mpu6xxx 设备设定参数

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev               | mpu6xxx_device 结构体的指针 |
|cmd | 支持的配置选项，详见下面的介绍 |
|param | 设定的具体的参数值 |
| **返回**          | **描述**                                |
|RT_EOK                  | 成功 |
|< 0                 | 失败                                |

cmd 参数指要配置的选项，param 参数表示设定的参数的具体值。详情如下:

**支持的配置选项** 

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|MPU6XXX_GYRO_RANGE               | 配置陀螺仪的测量范围 |
|MPU6XXX_ACCEL_RANGE | 配置加速度计的测量范围 |
|MPU6XXX_DLPF_CONFIG | 配置低通滤波器 |
| MPU6XXX_SAMPLE_RATE | 配置采样率 |
|MPU6XXX_SLEEP                  | 配置休眠模式 |

**MPU6XXX_GYRO_RANGE 参数的值** 

```
MPU6XXX_GYRO_RANGE_250DPS  // ±250°/s
MPU6XXX_GYRO_RANGE_500DPS  // ±500°/s
MPU6XXX_GYRO_RANGE_1000DPS // ±1000°/s
MPU6XXX_GYRO_RANGE_2000DPS // ±2000°/s
```

**MPU6XXX_ACCEL_RANGE 参数的值** 

```
MPU6XXX_ACCEL_RANGE_2G   // ±2G
MPU6XXX_ACCEL_RANGE_4G   // ±4G
MPU6XXX_ACCEL_RANGE_8G   // ±8G
MPU6XXX_ACCEL_RANGE_16G  // ±16G
```

**MPU6XXX_DLPF_CONFIG 参数的值** 

```
MPU6XXX_DLPF_DISABLE  //256HZ
MPU6XXX_DLPF_188HZ    //188HZ
MPU6XXX_DLPF_98HZ
MPU6XXX_DLPF_42HZ
MPU6XXX_DLPF_20HZ
MPU6XXX_DLPF_10HZ
MPU6XXX_DLPF_5HZ
```

**MPU6XXX_SAMPLE_RATE 参数的值** 

```
传入采样率的值（16-bit unsigned value.）
如果使能了低通滤波器（dlpf）的话，范围为 1000HZ - 4HZ。否则，范围为 8000HZ - 32HZ
```

**MPU6XXX_SLEEP  参数的值** 

```
MPU6XXX_SLEEP_DISABLE //退出休眠模式
MPU6XXX_SLEEP_ENABLE  //进入休眠模式
```


### 读取陀螺仪数据   

```{.c}
rt_err_t mpu6xxx_get_gyro(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *gyro);
```

读取陀螺仪数据 （单位： deg/10s）

| 参数     | 描述                                    |
| :------- | :-------------------------------------- |
| dev      | mpu6xxx_device 结构体的指针             |
| gyro     | 存储 mpu6xxx 3轴陀螺仪数据 结构体的指针 |
| **返回** | **描述**                                |
| RT_EOK   | 成功                                    |
| < 0      | 失败                                    |

3 轴陀螺仪数据的结构体定义如下

```{.c}
struct mpu6xxx_3axes
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};
```

### 读取加速度计数据

```{.c}
rt_err_t mpu6xxx_get_accel(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *accel);
```

读取加速度计数据 （单位： mg）

| 参数     | 描述                                    |
| :------- | :-------------------------------------- |
| dev      | mpu6xxx_device 结构体的指针             |
| accel    | 存储 mpu6xxx 3轴加速度数据 结构体的指针 |
| **返回** | **描述**                                |
| RT_EOK   | 成功                                    |
| < 0      | 失败                                    |

## 校准传感器

### 校准陀螺仪

```c
rt_err_t mpu6xxx_set_gyro_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset);
```

 校准陀螺仪。注意offset的单位和读取数据的单位不同。

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | mpu6xxx_device 结构体的指针 |
| offset   | 存储 校准量 结构体的指针    |
| **返回** | **描述**                    |
| RT_EOK   | 成功                        |
| 其他     | 失败                        |

### 校准加速度传感器

```c
rt_err_t mpu6xxx_set_gyro_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset);
```

校准加速度传感器。注意offset的单位和读取数据的单位不同。

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | mpu6xxx_device 结构体的指针 |
| offset   | 存储 校准量 结构体的指针    |
| **返回** | **描述**                    |
| RT_EOK   | 成功                        |
| 其他     | 失败                        |

### 读取温度计数据

```{.c}
rt_err_t mpu6xxx_get_temp(struct mpu6xxx_device *dev, float *temp);
```

读取温度计数据 （单位：摄氏度）

| 参数     | 描述                            |
| :------- | :------------------------------ |
| dev      | mpu6xxx_device 结构体的指针     |
| temp     | 存储 mpu6xxx 温度数据地址的指针 |
| **返回** | **描述**                        |
| RT_EOK   | 成功                            |
| < 0      | 失败                            |
