#include <stdio.h>
#include "esp_log.h"

//esp32-camera
#include "esp_camera.h"
// apriltag
#include "apriltag.h"
#include "tag16h5.h"
#include "common/image_u8.h"
#include "common/zarray.h"

#include "app_apriltag.h"

static const char *TAG = "apriltag_dection";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;

static bool gReturnFB = true;

static void task_process_handler(void *arg)
{
    camera_fb_t *frame = NULL;

    apriltag_family_t *tf = tag16h5_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_sigma = 0.0;
    td->quad_decimate = 1.0;
    td->refine_edges = 0;
    td->nthreads = 1;
    td->debug = 0;

    while (true)
    {
        if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
        {
            image_u8_t im =
            {
                .width = (int)frame->width,
                .height = (int)frame->height,
                .stride = (int)frame->width,
                .buf = frame->buf
            };

            zarray_t *detections = apriltag_detector_detect(td, &im);

            for (int i = 0; i < zarray_size(detections); i++)
            {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                printf("id-->%d, ", det->id);
            }
            printf("\n");

            apriltag_detections_destroy(detections);

            double t =  timeprofile_total_utime(td->tp) / 1.0E3;
            printf("%12.3f \n", t);

            if (gReturnFB)
            {
                // return the frame buffer back to the driver for reuse
                esp_camera_fb_return(frame);
            }
        }
    }
}

void register_apriltag_detection(const QueueHandle_t frame_i,
                                 const QueueHandle_t frame_o,
                                 const bool camera_fb_return)
{
    TaskHandle_t xHandle = NULL;

    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    gReturnFB = camera_fb_return;

    xTaskCreate(task_process_handler, "apriltag", 30 * 1024, NULL, tskIDLE_PRIORITY, &xHandle);
    // xTaskCreatePinnedToCore(task_process_handler, TAG, 10 * 1024, NULL, 5, NULL, 0);
}
