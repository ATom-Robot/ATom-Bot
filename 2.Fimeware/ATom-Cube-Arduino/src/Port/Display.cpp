#include "Display.h"
#include "HAL/HAL.h"

TaskHandle_t handleTaskLvgl;
SCREEN_CLASS screen;

void TaskLvglUpdate(void *parameter)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for (;;)
    {
        lv_task_handler();
        delay(30);
    }
}

void Port_Init()
{
    screen.init();
    screen.setRotation(0);
    screen.fillScreen(TFT_BLACK);

    lv_init();
    lv_port_disp_init(&screen);

    // Update display in parallel thread.
    BaseType_t result;
    result = xTaskCreatePinnedToCore(
                 TaskLvglUpdate,
                 "LvglThread",
                 5 * 1024,
                 NULL,
                 configMAX_PRIORITIES - 20,
                 &handleTaskLvgl,
                 1);
    assert("Failed to create task" && result == (BaseType_t) 1);

    HAL::Backlight_SetGradual(50, 100);
    Serial.println("Turn backlight!!");
}
