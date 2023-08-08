# ATom-Bot-Head Fimeware

## 工程使用说明

本工程为 ATom-Bot 机器人的头部固件源码。

## 硬件结构说明

主控采用 esp32-s3，PCB 采用底板+屏幕拓展板结构，底板与屏幕拓展板采用 fpc 排线相连接。

PCB 主要搭载：摄像头x1、麦克风x1、LCDx1、I2S 音频功放芯片x1。

## 硬件连接示意图

TODO

## 使用注意事项

TODO

## 项目开发进度

[ATom-Bot软硬件待办列表](https://docs.qq.com/doc/DZmRyVVZGSG9iQnpq)

## 开发中常用指令

```
idf.py set-target esp32s3

idf.py menuconfig	(配置工程)
idf.py -p COMX erase_flash	(擦除整个flash)
idf.py -p COMX flash	(下载，包括全部资源)
idf.py -p COMX -b 921600 flash monitor	(使用指定波特率下载固件并打开串口监视器)
```

