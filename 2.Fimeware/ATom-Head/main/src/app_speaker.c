#include <string.h>

#include "app_speaker.h"
#include "esp_log.h"

static const char *TAG = "App/Speaker";

bool speaker_initOutput(i2s_bits_per_sample_t BPS,
                        int bckPin,
                        int wsPin,
                        int dataOutPin)
{
    i2s_config_t i2s_config =
    {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = ADUIO_SAMPLE_RATE,
        .bits_per_sample = BPS,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 128,
        .use_apll = false
    };

    i2s_pin_config_t pin_config;
    memset(&pin_config, 0, sizeof(i2s_pin_config_t));
    pin_config.bck_io_num = bckPin;
    pin_config.ws_io_num = wsPin;
    pin_config.data_out_num = dataOutPin;
    pin_config.data_in_num = I2S_PIN_NO_CHANGE;

    if (ESP_OK != i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL))
    {
        ESP_LOGI(TAG, "install speaker driver failed");
        return false;
    }
    if (ESP_OK != i2s_set_pin(I2S_NUM, &pin_config))
    {
        ESP_LOGI(TAG, "speaker set pin failed");
        return false;
    }

    i2s_set_clk(I2S_NUM, ADUIO_SAMPLE_RATE, ADUIO_SAMPLE_BITS, I2S_CHANNEL_STEREO);

    return true;
}

size_t speaker_write(char *data, int numData)
{
    size_t sendSize;
    i2s_write(I2S_NUM, (void *)data, numData, &sendSize, portMAX_DELAY);
    return sendSize;
}

void AppSpeaker_Init(void)
{
    speaker_initOutput(I2S_BITS_PER_SAMPLE_16BIT, I2S_BCLK, I2S_LRC, I2S_DOUT);
}

void speaker_uninstall(void)
{
    i2s_driver_uninstall(I2S_NUM);
}

esp_err_t audio_i2s_reconfig_clk(uint32_t rate, uint32_t bits_cfg, i2s_channel_t ch)
{
    i2s_set_clk(I2S_NUM, rate, bits_cfg, ch);
    return ESP_OK;
}

esp_err_t audio_i2s_write(void * audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = i2s_write(I2S_NUM, (char *)audio_buffer, len, bytes_written, timeout_ms);
    return ret;
}
