#include <stdio.h>
#include <string.h>

#include "model_path.h"

#include <sys/dirent.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_spiffs.h"

static char *TAG = "MODEL_LOADER";
static char *SRMODE_BASE_PATH = "/srmodel";

char *get_model_base_path(void)
{
#if defined CONFIG_MODEL_IN_SDCARD
    return "sdcard";
#elif defined CONFIG_MODEL_IN_SPIFFS
    return "srmodel";
#else
    return NULL;
#endif
}

srmodel_list_t *read_models_form_spiffs(esp_vfs_spiffs_conf_t *conf)
{
    struct dirent *ret;
    DIR *dir = NULL;
    dir = opendir(conf->base_path);
    srmodel_list_t *models = NULL;
    int model_num = 0;
    int idx = 0;

    if (dir != NULL)
    {
        // get the number of models
        while ((ret = readdir(dir)) != NULL)
        {
            // NULL if reach the end of directory

            if (ret->d_type == DT_DIR) // continue if d_type is not file
                continue;

            int len = strlen(ret->d_name);
            char *suffix = ret->d_name + len - 12;

            if (strcmp(suffix, "_MODEL_INFO_") == 0)
                model_num ++;
        }

        // allocate model names
        if (model_num == 0)
        {
            return models;
        }
        else
        {
            models = malloc(sizeof(srmodel_list_t));
            models->num = model_num;
            models->partition_label = (char *)conf->partition_label;
            models->model_name = malloc(models->num * sizeof(char *));
            for (int i = 0; i < models->num; i++)
                models->model_name[i] = (char *) calloc(MODEL_NAME_MAX_LENGTH, sizeof(char));
        }

        // read & save model names
        closedir(dir);
        dir = opendir(conf->base_path);
        while ((ret = readdir(dir)) != NULL)
        {
            // NULL if reach the end of directory

            if (ret->d_type == DT_DIR) // continue if d_type is not file
                continue;

            int len = strlen(ret->d_name);
            char *suffix = ret->d_name + len - 12;

            if (strcmp(suffix, "_MODEL_INFO_") == 0)
            {
                memcpy(models->model_name[idx], ret->d_name, (len - 13)*sizeof(char));
                // models->model_name[idx][len-13] = '\0';
                idx ++;
            }
        }
        closedir(dir);
        dir = NULL;

    }
    return models;
}


srmodel_list_t *srmodel_spiffs_init(const char *partition_label)
{
    ESP_LOGI(TAG, "\nInitializing models from SPIFFS, partition label: %s\n", partition_label);

    esp_vfs_spiffs_conf_t conf =
    {
        .base_path = SRMODE_BASE_PATH,
        .partition_label = partition_label,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem\n");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition\n");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)\n", esp_err_to_name(ret));
        }
        return NULL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)\n", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d\n", total, used);
    }

    // Read all model from path
    return read_models_form_spiffs(&conf);
}

srmodel_list_t *esp_srmodel_init(const char *base_path)
{
    return srmodel_spiffs_init(base_path);
}

char *_esp_strstr_(const char *haystack, const char *needle)
{
    if (needle == NULL) return (char *)haystack;
    else return (char *)strstr(haystack, needle);
}

char *esp_srmodel_filter(srmodel_list_t *models, const char *keyword1, const char *keyword2)
{
    if (models == NULL) return NULL;

    // return the first model name including specific keyword
    for (int i = 0; i < models->num; i++)
    {

        if (_esp_strstr_(models->model_name[i], keyword1) != NULL)
        {
            if (_esp_strstr_(models->model_name[i], keyword2) != NULL)
                return models->model_name[i];
        }
    }

    return NULL;
}
