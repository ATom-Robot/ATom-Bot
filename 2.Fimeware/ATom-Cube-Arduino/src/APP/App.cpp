#include "App.h"
#include "HAL/HAL.h"
#include "benchmark/lv_demo_benchmark.h"

void App_Init()
{
    lv_demo_benchmark();
    INIT_DONE();
}

void App_UnInit()
{

}
