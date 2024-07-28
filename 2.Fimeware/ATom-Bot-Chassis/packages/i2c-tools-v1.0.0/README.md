# i2c-tools

## 1、介绍

i2c-tools 包含了一些很方便对 i2c 设备进行调试的小工具。

硬件 I2C (可以 list_device 查看 i2c 设备)：

	---------------
	i2c tools help:
	---------------
	i2c scan  bus_name
	i2c read  bus_name address  register  [len=1]
	i2c write bus_name address [register] data_0 data_1 ...


![](docs/i2c.gif)

软件 I2C (可以自定义两个引脚作为 SCL 和 SDA)：

---------------
	i2c tools help:
	---------------
	i2c scan  pin_sda pin_scl
	i2c read  pin_sda pin_scl address  register  [len=1]
	i2c write pin_sda pin_scl address [register] data_0 data_1 ...
	

![](docs/sw_i2c.gif)

## 2、联系方式

* 维护：Wu Han
* 主页：http://wuhanstudio.cc
* 邮箱：wuhanstudio@hust.edu.cn
