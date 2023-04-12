/*
 * @Author: maplerian
 * @Date: 2020-07-11 16:03:03
 * @LastEditors: maplerian
 * @LastEditTime: 2020-07-11 16:53:52
 * @Description: file content
 */ 

#ifndef __WS2812__H__
#define __WS2812__H__

#include <rtthread.h>
#include <rtdevice.h>

//  一位占用多少个字节
#define OneBitOccupyByte    2
#define OneNodeBuffLength   8 * 3 * OneBitOccupyByte

struct ws2812
{
    struct rt_spi_device *spi;
    uint8_t *buff;
    uint16_t node_len;
};
typedef struct ws2812 * ws2812_t;

ws2812_t ws2812_create(char *spi_name, uint16_t led_node_length);
void ws2812_clear_buff(ws2812_t ws2812);
void ws2812_write_rgb_to_node(ws2812_t pWs2812, uint8_t index, uint8_t R, uint8_t G, uint8_t B);
void ws2812_write_rgb_to_all(ws2812_t ws2812, uint8_t r, uint8_t g, uint8_t b);
void ws2812_send(ws2812_t ws2812);

void ws281x_colorWipe(ws2812_t ws2812, uint32_t c, uint8_t wait);
void ws281x_rainbow(ws2812_t ws2812, uint8_t wait);
void ws281x_rainbowCycle(ws2812_t ws2812, uint8_t wait);
void ws281x_theaterChase(ws2812_t ws2812, uint32_t c, uint8_t wait);
void ws281x_theaterChaseRainbow(ws2812_t ws2812, uint8_t wait);

#endif  //!__WS2812__H__

