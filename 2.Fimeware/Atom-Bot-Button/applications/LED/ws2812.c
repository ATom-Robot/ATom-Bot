/*
 * @Author: maplerian
 * @Date: 2020-07-11 16:02:23
 * @LastEditors: maplerian
 * @LastEditTime: 2020-07-11 22:08:53
 * @Description: file content
 */

#include "ws2812.h"

uint16_t _rand16seed;

ws2812_t ws2812_create(char *spi_name, uint16_t led_node_length)
{
    if (!spi_name || !led_node_length)
        return RT_NULL;
    struct rt_spi_device *spi = (struct rt_spi_device *)rt_device_find(spi_name);
    if (spi == RT_NULL)
        return RT_NULL;
    ws2812_t ws2812 = (ws2812_t)rt_malloc(sizeof(struct ws2812));
    ws2812->buff = (uint8_t *)rt_malloc(OneNodeBuffLength * led_node_length);
    if (!ws2812->buff)
    {
        rt_kprintf("There is not enough memory to create a buffer.\r\n");
        rt_free(ws2812);
        return RT_NULL;
    }
    //  配置 spi
    struct rt_spi_configuration ws2812_spi_config;
    ws2812_spi_config.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB; //  高位在前
    ws2812_spi_config.max_hz = 8 * 1000 * 1000;            //  200ns / bit => 8bit 1.6us => 1.25us < 1.6us < 1.85us 可行     6M 3b = 500ns 5b = 833ns
    ws2812_spi_config.data_width = 8;
    rt_spi_configure((struct rt_spi_device *)spi, &ws2812_spi_config);
    //  赋值
    ws2812->spi = spi;
    ws2812->node_len = led_node_length;
    return ws2812;
}

void ws2812_clear_buff(ws2812_t ws2812)
{
    if (!ws2812 || !ws2812->buff || !ws2812->node_len)
        return;
    rt_memset(ws2812->buff, 0x00, OneNodeBuffLength * ws2812->node_len);
}

//  写颜色节点颜色到缓冲区
static void ws2812_write_buff(ws2812_t pWs2812, uint8_t index, uint8_t R, uint8_t G, uint8_t B)
{
    // 先组合成8*3个字节的数据
    uint8_t temp[24] = {0};
    for (uint8_t i = 0; i < 8; i++)
    {
        temp[i] = (G & 0x01) ? 0xfc : 0xc0;
        G = G >> 1;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        temp[i + 8] = (R & 0x01) ? 0xfc : 0xc0;
        R = R >> 1;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        temp[i + 16] = (B & 0x01) ? 0xfc : 0xc0;
        B = B >> 1;
    }
    // 拷贝到对应的Buff中
    rt_memcpy(&pWs2812->buff[index * 24], temp, 24);
}

void ws2812_write_rgb_to_node(ws2812_t pWs2812, uint8_t index, uint8_t R, uint8_t G, uint8_t B)
{
    ws2812_write_buff(pWs2812, index, R, G, B);
}

void ws2812_write_rgb_to_all(ws2812_t ws2812, uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t i;
    for (i = 0; i < ws2812->node_len; i++)
        ws2812_write_buff(ws2812, i, r, g, b);
}

void ws2812_send(ws2812_t ws2812)
{
    if (!ws2812 || !ws2812->spi || !ws2812->buff)
    {
        rt_kprintf("Parame is null\r\n");
        return;
    }
    rt_spi_send(ws2812->spi, ws2812->buff, OneNodeBuffLength * ws2812->node_len);
}

#define PIXEL_NUM 2

uint8_t pixelBuffer[PIXEL_NUM][24];

void ws281x_show(ws2812_t ws2812)
{
    rt_spi_send(ws2812->spi, pixelBuffer, OneNodeBuffLength * ws2812->node_len);
}

uint32_t set_ws2812_color(uint8_t red, uint8_t green, uint8_t blue)
{
    return green << 16 | red << 8 | blue;
}

void ws281x_setPixelColor(uint16_t n, uint32_t GRBcolor)
{
    uint8_t i;
    if (n < PIXEL_NUM)
    {
        for (i = 0; i < 24; ++i)
        {
            pixelBuffer[n][i] = (((GRBcolor << i) & 0X800000) ? 0xfc : 0xc0);
        }
    }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t ws281x_wheel(uint8_t wheelPos)
{
    wheelPos = 255 - wheelPos;
    if (wheelPos < 85)
    {
        return set_ws2812_color(255 - wheelPos * 3, 0, wheelPos * 3);
    }
    if (wheelPos < 170)
    {
        wheelPos -= 85;
        return set_ws2812_color(0, wheelPos * 3, 255 - wheelPos * 3);
    }
    wheelPos -= 170;
    return set_ws2812_color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

// Fill the dots one after the other with a color
void ws281x_colorWipe(ws2812_t ws2812, uint32_t c, uint8_t wait)
{
    for (uint16_t i = 0; i < PIXEL_NUM; i++)
    {
        ws281x_setPixelColor(i, c);
        ws281x_show(ws2812);
        rt_thread_mdelay(wait);
    }
}

void ws281x_rainbow(ws2812_t ws2812, uint8_t wait)
{
    uint16_t i, j;

    for (j = 0; j < 256; j++)
    {
        for (i = 0; i < PIXEL_NUM; i++)
        {
            ws281x_setPixelColor(i, ws281x_wheel((i + j) & 255));
        }
        ws281x_show(ws2812);
        rt_thread_mdelay(wait);
    }
}

// Slightly different, this makes the rainbow equally distributed throughout
void ws281x_rainbowCycle(ws2812_t ws2812, uint8_t wait)
{
    uint16_t i, j;

    // 5 cycles of all colors on wheel
    for (j = 0; j < 256 * 5; j++)
    {
        for (i = 0; i < PIXEL_NUM; i++)
        {
            ws281x_setPixelColor(i, ws281x_wheel(((i * 256 / PIXEL_NUM) + j) & 255));
        }
        ws281x_show(ws2812);
        rt_thread_mdelay(wait);
    }
}

//Theatre-style crawling lights.
void ws281x_theaterChase(ws2812_t ws2812, uint32_t c, uint8_t wait)
{
    //do 10 cycles of chasing
    for (int j = 0; j < 10; j++)
    {
        for (int q = 0; q < 3; q++)
        {
            for (uint16_t i = 0; i < PIXEL_NUM; i = i + 3)
            {
                //turn every third pixel on
                ws281x_setPixelColor(i + q, c);
            }
            ws281x_show(ws2812);

            rt_thread_mdelay(wait);

            for (uint16_t i = 0; i < PIXEL_NUM; i = i + 3)
            {
                //turn every third pixel off
                ws281x_setPixelColor(i + q, 0);
            }
        }
    }
}

//Theatre-style crawling lights with rainbow effect
void ws281x_theaterChaseRainbow(ws2812_t ws2812, uint8_t wait)
{
    // cycle all 256 colors in the wheel
    for (int j = 0; j < 256; j++)
    {
        for (int q = 0; q < 3; q++)
        {
            for (uint16_t i = 0; i < PIXEL_NUM; i = i + 3)
            {
                //turn every third pixel on
                ws281x_setPixelColor(i + q, ws281x_wheel((i + j) % 255));
            }
            ws281x_show(ws2812);

            rt_thread_mdelay(wait);

            for (uint16_t i = 0; i < PIXEL_NUM; i = i + 3)
            {
                //turn every third pixel off
                ws281x_setPixelColor(i + q, 0);
            }
        }
    }
}