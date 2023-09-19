#pragma once

#ifndef Arduino_h
#define Arduino_h

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/gpio_reg.h"

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define digitalPinToTimer(pin) (0)
#define analogInPinToBit(P) (P)
#if SOC_GPIO_PIN_COUNT <= 32
#define digitalPinToPort(pin) (0)
#define digitalPinToBitMask(pin) (1UL << (pin))
#define portOutputRegister(port) ((volatile uint32_t *)GPIO_OUT_REG)
#define portInputRegister(port) ((volatile uint32_t *)GPIO_IN_REG)
#define portModeRegister(port) ((volatile uint32_t *)GPIO_ENABLE_REG)
#elif SOC_GPIO_PIN_COUNT <= 64
#define digitalPinToPort(pin) (((pin) > 31) ? 1 : 0)
#define digitalPinToBitMask(pin) (1UL << (((pin) > 31) ? ((pin)-32) : (pin)))
#define portOutputRegister(port) ((volatile uint32_t *)((port) ? GPIO_OUT1_REG : GPIO_OUT_REG))
#define portInputRegister(port) ((volatile uint32_t *)((port) ? GPIO_IN1_REG : GPIO_IN_REG))
#define portModeRegister(port) ((volatile uint32_t *)((port) ? GPIO_ENABLE1_REG : GPIO_ENABLE_REG))
#else
#error SOC_GPIO_PIN_COUNT > 64 not implemented
#endif


typedef bool boolean;
typedef uint8_t byte;
typedef unsigned int word;

//GPIO FUNCTIONS
#define INPUT 0x01
#define OUTPUT 0x02
#define PULLUP 0x04
#define INPUT_PULLUP 0x05
#define PULLDOWN 0x08
#define INPUT_PULLDOWN 0x09
#define OPEN_DRAIN 0x10
#define OUTPUT_OPEN_DRAIN 0x12

void pinMode(uint8_t pin, uint8_t mode);

#endif /* _ESP32_CORE_ARDUINO_H_ */
