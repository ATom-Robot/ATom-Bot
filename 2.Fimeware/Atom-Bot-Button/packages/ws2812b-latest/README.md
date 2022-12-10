# rt_ws2812b

#### 1.简介

用于 RT-Thread 的 ws2812b 软件驱动包，使用 SPI + DMA 方式驱动。

##### 1.1 目录结构

|  名称   |       说明       |
| :-----: | :--------------: |
| example |   示例文件目录   |
| 根目录  | 源码及头文件目录 |

##### 1.2 许可证

rt_ws2812b package 遵循 Apachev2.0许可，详见 `LICENSE` 文件。

##### 1.3 依赖

-  RT-Thread V4.0.3

------

#### 2. 获取方式

###### 通过 git 克隆

```shell
git clone https://github.com/maplerian/rt_ws2812b.git
```

###### 在 ENV 中的开启

```
RT-Thread online packages  --->
      peripheral libraries and drivers --->
        -*- ws2812b: Ws2812b software driver package using SPI+DMA
```

#### 3. 注意事项

> SPI的通讯频率为 13.333Mhz， 因此， 每 2byte(MCU) == 1bit(ws2812b)。
>
> 1个ws2812b节点需要 2 x 8 x 3 = 48字节 因此，太多节点的人，请谨慎使用。
>
> SPI的通讯频率我测试过 2.5Mhz、3.333Mhz、6.666Mhz、13.333Mhz,
>
> 最终选择13.333Mhz的原因:这个相对来说比较稳定，但也是有一定的问题
>
> 比如：
>
> ​	当颜色值（大于0的值）小于10时，第一个节点的颜色会出现异常，其后的其它节点颜色正常

#### 4. 联系方式

- 维护：maplerian
- 主页：<https://github.com/maplerian/rt_ws2812b>

