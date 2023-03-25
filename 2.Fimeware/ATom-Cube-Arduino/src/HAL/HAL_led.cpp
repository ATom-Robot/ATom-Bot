#include "HAL/HAL.h"
#include <FastLED.h>

CRGB color_buffers[CONFIG_RGB_LED_NUM];

void HAL::Led_Init()
{
    FastLED.addLeds<SK6812, CONFIG_RGB_LED_PIN, GRB>(color_buffers, CONFIG_RGB_LED_NUM);
    FastLED.setBrightness(200);
}

void HAL::setRGB(int id, int r, int g, int b)
{
    color_buffers[id] = CRGB(r, g, b);
    FastLED.show();
}

void HAL::setBrightness(float duty)
{
    duty = constrain(duty, 0, 1);
    FastLED.setBrightness((uint8_t)(255 * duty));
    FastLED.show();
}

void HAL::Led_firt_light(void)
{
    for (int whiteLed = 0; whiteLed < CONFIG_RGB_LED_NUM; whiteLed++)
    {
        // Turn our current led on to white, then show the leds
        color_buffers[whiteLed] = CRGB::LawnGreen;
        FastLED.show();
        delay(200);
        // Turn our current led back to black for the next loop around
        color_buffers[whiteLed] = CRGB::Black;
        FastLED.show();
    }
}

void HAL::led_rainbow(void)
{
    static uint8_t gHue = 0;
    fill_rainbow(color_buffers, CONFIG_RGB_LED_NUM, gHue, 7);
    gHue <= UINT8_MAX ? gHue ++ : gHue = 0;
    FastLED.show();
}
