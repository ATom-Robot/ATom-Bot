#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "app_lcd.h"
#include "app_led.h"
#include "app_shell.h"
#include "freertos/task.h"

static const char *TAG = "main";

extern "C" void app_main()
{
    Led_Init();
    setBrightness(0.25);
    Led_firt_light();

    AppLCD_Init();

    AppLVGL_run();

    while (1)
    {
        led_rainbow();
        vTaskDelay(30);
    }

    APP_Shell_loop();
}
