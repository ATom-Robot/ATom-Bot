/**
 * @file lv_port_disp_templ.c
 *
 */
/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include "HAL/HAL.h"
#include "Port/Display.h"

/**********************
 *  STATIC VARIABLES
 **********************/
static LGFX_Emma *_lgfxEmma = nullptr;
static lv_disp_draw_buf_t disp_buf;
/**********************
 *   GLOBAL FUNCTIONS
 **********************/
lv_color_t *lv_disp_buf_p = nullptr;

volatile bool disp_flush_enabled = true;

static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    if (disp_flush_enabled)
    {
        /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/
        uint32_t w = (area->x2 - area->x1 + 1);
        uint32_t h = (area->y2 - area->y1 + 1);
        _lgfxEmma->startWrite();
        _lgfxEmma->setAddrWindow(area->x1, area->y1, w, h);
        _lgfxEmma->pushColors((uint16_t *)&color_p->full, w * h, true);
        _lgfxEmma->endWrite();
    }

    lv_disp_flush_ready(disp_drv);
}

void lv_port_disp_init(LGFX_Emma *pLgfxEmma)
{
    _lgfxEmma = pLgfxEmma;

    lv_disp_draw_buf_init(&disp_buf, lv_disp_buf_p, nullptr, DISP_BUF_SIZE);   /*Initialize the display buffer*/

    static lv_disp_drv_t disp_drv;  /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);    /*Basic initialization*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &disp_buf;

    /*Required for Example 3)*/
    disp_drv.full_refresh = 1;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}
