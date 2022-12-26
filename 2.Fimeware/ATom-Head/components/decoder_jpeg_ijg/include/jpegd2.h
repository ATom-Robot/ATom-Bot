#ifndef __JPEGD2_H
#define __JPEGD2_H

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include "cdjpeg.h"
#include <setjmp.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*lcd_write_cb)(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *data);
void mjpegdraw(uint8_t *mjpegbuffer, uint32_t size, uint8_t *outbuffer);

#ifdef __cplusplus
}
#endif

#endif
