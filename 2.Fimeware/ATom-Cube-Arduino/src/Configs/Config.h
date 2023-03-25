#ifndef __CONFIG_H
#define __CONFIG_H

/*=========================
   Application configuration
 *=========================*/
#define CONFIG_SYSTEM_GMT_OFFSET_DEFAULT      8 // GMT+ 8

#define CONFIG_LIVE_MAP_LEVEL_DEFAULT         16
#define CONFIG_LIVE_MAP_VIEW_WIDTH            LV_HOR_RES
#define CONFIG_LIVE_MAP_VIEW_HEIGHT           LV_VER_RES

/*=========================
   Hardware Configuration
 *=========================*/
/* Screen */
#define CONFIG_SCREEN_CS_PIN        12
#define CONFIG_SCREEN_DC_PIN        13
#define CONFIG_SCREEN_RST_PIN       15
#define CONFIG_SCREEN_SCK_PIN       17
#define CONFIG_SCREEN_MOSI_PIN      16
#define CONFIG_SCREEN_BLK_PIN       2

#define CONFIG_SCREEN_HOR_RES       240
#define CONFIG_SCREEN_VER_RES       240
#define CONFIG_SCREEN_BUFFER_SIZE   (CONFIG_SCREEN_HOR_RES * CONFIG_SCREEN_VER_RES)

#define DISP_HOR_RES                CONFIG_SCREEN_HOR_RES
#define DISP_VER_RES                CONFIG_SCREEN_VER_RES
#define DISP_BUF_SIZE               CONFIG_SCREEN_BUFFER_SIZE

/* Buzzer */
#define CONFIG_BUZZ_PIN             46
#define CONFIG_BUZZ_CHANNEL         1
#define CONFIG_SOUND_ENABLE_DEFAULT false

/* I2C */
#define CONFIG_MCU_SDA_PIN          4
#define CONFIG_MCU_SCL_PIN          5

/* Debug USART */
#define CONFIG_DEBUG_SERIAL         Serial

#endif
