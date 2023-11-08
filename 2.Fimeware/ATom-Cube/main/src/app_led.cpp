#include <FastLED.h>
#include "app_led.h"
#include "freertos/task.h"

/* led */
#define CONFIG_RGB_LED_NUM          4
#define CONFIG_RGB_LED_PIN          1
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

CRGB color_buffers[CONFIG_RGB_LED_NUM];

void Led_Init(void)
{
    FastLED.addLeds<SK6812, CONFIG_RGB_LED_PIN, GRB>(color_buffers, CONFIG_RGB_LED_NUM);
    FastLED.setBrightness(150);
}

void setRGB(int id, int r, int g, int b)
{
    color_buffers[id] = CRGB(r, g, b);
    FastLED.show();
}

void setBrightness(float duty)
{
    duty = constrain(duty, 0, 1);
    FastLED.setBrightness((uint8_t)(255 * duty));
    FastLED.show();
}

void Led_firt_light(void)
{
    for (int whiteLed = 0; whiteLed < CONFIG_RGB_LED_NUM; whiteLed++)
    {
        // Turn our current led on to white, then show the leds
        color_buffers[whiteLed] = CRGB::LawnGreen;
        FastLED.show();
        vTaskDelay(200);
        // Turn our current led back to black for the next loop around
        color_buffers[whiteLed] = CRGB::Black;
        FastLED.show();
    }
}

void led_rainbow(void)
{
    static uint8_t gHue = 0;
    fill_rainbow(color_buffers, CONFIG_RGB_LED_NUM, gHue, 7);
    gHue <= UINT8_MAX ? gHue ++ : gHue = 0;
    FastLED.show();
}
