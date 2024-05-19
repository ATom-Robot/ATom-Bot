/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-02-26     Rbb66         First version
 */
#include "ano.h"
#include "bsp_motor.h"
#include "bsp_pid.h"

#define DBG_SECTION_NAME  "ano"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

#define BYTE0(dwTemp) (*((uint8_t *)(&dwTemp)))
#define BYTE1(dwTemp) (*((uint8_t *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((uint8_t *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((uint8_t *)(&dwTemp) + 3))

#define PID_PARAM_FACTOR       1000.0f

// Thread
#define THREAD_STACK_SIZE      512
#define THREAD_PRIORITY        15
#define THREAD_TICK            10

static rt_thread_t tid_ano = RT_NULL;
static rt_device_t dev_ano = RT_NULL;
static rt_sem_t rx_sem = RT_NULL;

int rec_target_yaw = 0;
int rec_target_rpm = 0;
float rec_target_motor_num = 0;
/* angel pid will be open or close*/
rt_bool_t angel_control = RT_FALSE;

static rt_err_t ano_sender_send(rt_uint16_t cmd, void *param, rt_uint16_t size)
{
    switch (cmd)
    {
    case COMMAND_SEND_SENSOR:
        if (size == sizeof(struct cmd_sensor))
        {
        }
        else
        {
            return RT_ERROR;
        }

        break;
    default:
        return RT_ERROR;
    }

    return RT_EOK;
}

static struct command_sender ano_sender =
{
    .name = "ano",
    .send = ano_sender_send
};

static int _send_data(uint8_t *buffer, uint8_t length)
{
    if (dev_ano != RT_NULL)
    {
        return rt_device_write(dev_ano, 0, buffer, length);
    }

    return RT_ERROR;
}

#define _GET_PID_PARAM(buffer, offset)  (float)((1/PID_PARAM_FACTOR) * ((int16_t)(*(buffer + offset) << 8) | *(buffer + (offset + 1))));

static void _get_pid_param(uint8_t *buffer, float *kpid)
{
    for (int i = 0; i < 9; i++)
    {
        kpid[i] = _GET_PID_PARAM(buffer, (i + 2) * 2);
    }
}

static void ano_send_check(uint8_t id, uint8_t check_sum, uint8_t ac)
{
    uint8_t data_to_send[12];
    uint8_t _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0x00;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = id;
    data_to_send[_cnt++] = check_sum;
    data_to_send[_cnt++] = ac;

    data_to_send[3] = _cnt - 4;

    uint8_t sum1 = 0, sum2 = 0;
    for (uint8_t i = 0; i < _cnt; i++)
    {
        sum1 += data_to_send[i];
        sum2 += sum1;
    }

    data_to_send[_cnt++] = sum1;
    data_to_send[_cnt++] = sum2;

    _send_data(data_to_send, _cnt);
}

static void ano_sentPar(uint16_t id, int32_t data)
{
    uint8_t data_to_send[35];
    uint8_t _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0XE2;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = BYTE0(id);
    data_to_send[_cnt++] = BYTE1(id);

    data_to_send[_cnt++] = BYTE0(data);
    data_to_send[_cnt++] = BYTE1(data);
    data_to_send[_cnt++] = BYTE2(data);
    data_to_send[_cnt++] = BYTE3(data);

    data_to_send[3] = _cnt - 4;

    uint8_t sum1 = 0, sum2 = 0;
    for (uint8_t i = 0; i < _cnt; i++)
    {
        sum1 += data_to_send[i];
        sum2 += sum1;
    }

    data_to_send[_cnt++] = sum1;
    data_to_send[_cnt++] = sum2;

    _send_data(data_to_send, _cnt);
}

static void ano_parse_frame(uint8_t *buffer, uint8_t length)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < (length - 1); i++)
        sum += *(buffer + i);
    if (!(sum == *(buffer + length - 1)))
        return;
    if (!(*(buffer) == 0xAA && *(buffer + 1) == 0xFF))
        return;

    /* 参数读取ID 0XE1 */
    if (buffer[2] == 0xE1)
    {
        uint16_t id = buffer[4] + ((uint16_t)buffer[5] << 8);
        switch (id)
        {
        case 1:
            ano_sentPar(id, rec_target_rpm); /* 速度 */
            break;

        case 2:
            ano_sentPar(id, rec_target_rpm * 10); /* 圈数放大10倍发送 */
            break;

        default:
            ano_sentPar(id, 0);
            break;
        }
    }
    /* 参数写入ID 0XE2，接收到的是整数*/
    else if (buffer[2] == 0xE2)
    {
        uint16_t id = buffer[4] + ((uint16_t)buffer[5] << 8);
        switch (id)
        {
        /* 转速 */
        case 1:
        {
            static int Rpm;
            Rpm = *(int *)(&buffer[6]);
            rec_target_rpm = limit_amplitude(Rpm, RPM_MAX);
            LOG_D("recv rpm speed target:%d", rec_target_rpm);
            break;
        }
        /* 位置 */
        case 2:
        {
            rec_target_motor_num = (float) * (int *)(&buffer[6]) / 10;
            LOG_D("recv motor pos target num:%f", rec_target_motor_num);
            break;
        }
        /* 位置 */
        case 3:
        {
			angel_control = RT_TRUE;
            rec_target_yaw = *(int *)(&buffer[6]);
            LOG_D("recv yaw angle target:%d", rec_target_yaw);
            break;
        }
        }
    }
    ano_send_check(buffer[2], buffer[buffer[3] + 4], buffer[buffer[3] + 5]);
}

static int ano_receive_byte(uint8_t data)
{
    static uint8_t RxBuffer[50];
    static uint8_t _data_len = 0, _data_cnt = 0;
    static uint8_t state = 0;

    if (state == 0 && data == 0xAA)
    {
        _data_cnt = 0;
        _data_len = 0;

        state = 1;
        RxBuffer[0] = data;
    }
    else if (state == 1 && data == 0xFF)
    {
        state = 2;
        RxBuffer[1] = data;
    }
    /* ID */
    else if (state == 2 && data < 0XF1)
    {
        state = 3;
        RxBuffer[2] = data;
    }
    /* 数据长度 */
    else if (state == 3 && data < 50)
    {
        state = 4;
        RxBuffer[3] = data;
        _data_len = data;
    }
    else if (state == 4)
    {
        RxBuffer[4 + _data_cnt++] = data;
        if (_data_cnt >= _data_len)
            state = 5;
    }
    else if (state == 5)
    {
        RxBuffer[4 + _data_cnt++] = data;
        state = 6;
    }
    else if (state == 6)
    {
        state = 0;
        RxBuffer[4 + _data_cnt] = data;

        ano_parse_frame(RxBuffer, _data_cnt + 4);
        return 1;
    }
    else
        state = 0;

    return 0;
}

int ano_send_motorpwm(uint16_t m_1, uint16_t m_2, uint16_t m_3, uint16_t m_4, uint16_t m_5, uint16_t m_6, uint16_t m_7, uint16_t m_8)
{
    uint8_t data_to_send[21];
    uint8_t _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0x06;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = BYTE1(m_1);
    data_to_send[_cnt++] = BYTE0(m_1);
    data_to_send[_cnt++] = BYTE1(m_2);
    data_to_send[_cnt++] = BYTE0(m_2);
    data_to_send[_cnt++] = BYTE1(m_3);
    data_to_send[_cnt++] = BYTE0(m_3);
    data_to_send[_cnt++] = BYTE1(m_4);
    data_to_send[_cnt++] = BYTE0(m_4);
    data_to_send[_cnt++] = BYTE1(m_5);
    data_to_send[_cnt++] = BYTE0(m_5);
    data_to_send[_cnt++] = BYTE1(m_6);
    data_to_send[_cnt++] = BYTE0(m_6);
    data_to_send[_cnt++] = BYTE1(m_7);
    data_to_send[_cnt++] = BYTE0(m_7);
    data_to_send[_cnt++] = BYTE1(m_8);
    data_to_send[_cnt++] = BYTE0(m_8);

    data_to_send[3] = _cnt - 4;

    uint8_t sum = 0;
    for (uint8_t i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    return _send_data(data_to_send, _cnt);
}

int ano_send_user_data(uint8_t number, int16_t d0, int16_t d1, int16_t d2, int16_t d3, int16_t d4, int16_t d5)
{
    uint8_t data_to_send[35];
    uint8_t _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0xF0 + number;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = BYTE0(d0);
    data_to_send[_cnt++] = BYTE1(d0);

    data_to_send[_cnt++] = BYTE0(d1);
    data_to_send[_cnt++] = BYTE1(d1);

    data_to_send[_cnt++] = BYTE0(d2);
    data_to_send[_cnt++] = BYTE1(d2);

    data_to_send[_cnt++] = BYTE0(d3);
    data_to_send[_cnt++] = BYTE1(d3);

    data_to_send[_cnt++] = BYTE0(d4);
    data_to_send[_cnt++] = BYTE1(d4);

    data_to_send[_cnt++] = BYTE0(d5);
    data_to_send[_cnt++] = BYTE1(d5);

    data_to_send[3] = _cnt - 4;

    uint8_t sum1 = 0, sum2 = 0;
    for (uint8_t i = 0; i < _cnt; i++)
    {
        sum1 += data_to_send[i];
        sum2 += sum1;
    }

    data_to_send[_cnt++] = sum1;
    data_to_send[_cnt++] = sum2;

    return _send_data(data_to_send, _cnt);
}

static uint8_t ano_getbyte(void)
{
    uint8_t tmp;

    while (rt_device_read(dev_ano, -1, &tmp, 1) != 1)
        rt_sem_take(rx_sem, RT_WAITING_FOREVER);

    return tmp;
}

static rt_err_t ano_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(rx_sem);

    return RT_EOK;
}

struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

int ano_set_device(const char *device_name)
{
    rt_device_t dev = RT_NULL;

    dev = rt_device_find(device_name);
    if (dev == RT_NULL)
    {
        LOG_E("Can not find device: %s\n", device_name);
        return RT_ERROR;
    }

    config.baud_rate = 500000;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);

    /* check whether it's a same device */
    if (dev == dev_ano) return RT_ERROR;
    /* open this device and set the new device in finsh shell */
    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX) == RT_EOK)
    {
        if (dev_ano != RT_NULL)
        {
            /* close old finsh device */
            rt_device_close(dev_ano);
            rt_device_set_rx_indicate(dev_ano, RT_NULL);
        }

        dev_ano = dev;
        rt_device_set_rx_indicate(dev_ano, ano_rx_ind);
    }

    return RT_EOK;
}

command_sender_t ano_get_sender(void)
{
    return &ano_sender;
}

static void ano_thread_entry(void *param)
{
    while (1)
    {
        ano_receive_byte(ano_getbyte());
    }
}

int ano_init(void *param)
{
    if (ano_set_device((char *)param) != RT_EOK)
    {
        LOG_E("Failed to find device");
        return RT_ERROR;
    }

    rx_sem = rt_sem_create("anoRx", 0, RT_IPC_FLAG_FIFO);
    if (rx_sem == RT_NULL)
    {
        LOG_E("Failed to create sem\n");
        return RT_ERROR;
    }

    tid_ano = rt_thread_create("ano", ano_thread_entry, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TICK);
    if (tid_ano == RT_NULL)
    {
        LOG_E("Failed to create thread\n");
        return RT_ERROR;
    }

    rt_thread_startup(tid_ano);

    LOG_D("ano thread start");

    return RT_EOK;
}
