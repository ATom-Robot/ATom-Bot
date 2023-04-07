#include "esp_timer.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_system.h"

#include "app_rtsp.h"

//common camera settings, see below for other camera options
#define CAM_QUALITY 10 //0 to 63, with higher being lower-quality (and less bandwidth), 12 seems reasonable
#define CONFIG_CAM_FRAMERATE 100

OV2640 cam;

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

//lifted from Arduino framework
unsigned long millis()
{
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void client_worker(void *client)
{
    OV2640Streamer *streamer = new OV2640Streamer((SOCKET)client, cam);
    CRtspSession *session = new CRtspSession((SOCKET)client, streamer);

    unsigned long lastFrameTime = 0;
    const unsigned long msecPerFrame = (1000 / CONFIG_CAM_FRAMERATE);

    while (session->m_stopped == false)
    {
        session->handleRequests(0);

        unsigned long now = millis();
        if ((now > (lastFrameTime + msecPerFrame)) || (now < lastFrameTime))
        {
            session->broadcastCurrentFrame(now);
            lastFrameTime = now;
        }
        else
        {
            //let the system do something else for a bit
            delay(1);
        }
    }

    //shut ourselves down
    delete streamer;
    delete session;
    vTaskDelete(NULL);
}

void rtsp_server(void)
{
    SOCKET server_socket;
    SOCKET client_socket;
    sockaddr_in server_addr;
    sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    TaskHandle_t xHandle = NULL;

    //note that I am adding this task watchdog, but I never feed it
    //if a client doesn't connect in 60 seconds, we reset
    //I assume by that time that something is wrong with the wifi and reset will clear it up
    //if a client connects, we'll delete this watchdog
    camera_config_t config = esp32cam_config;
    config.frame_size = FRAMESIZE_240X240;
    config.jpeg_quality = CAM_QUALITY;
    cam.init(config);

    sensor_t *s = esp_camera_sensor_get();
#ifdef CONFIG_CAM_HORIZONTAL_MIRROR
    s->set_hmirror(s, 1);
#endif
#ifdef CONFIG_CAM_VERTICAL_FLIP
    s->set_vflip(s, 1);
#endif

    server_addr.sin_family      = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port        = htons(8554); // listen on RTSP port 8554
    server_socket               = socket(AF_INET, SOCK_STREAM, 0);

    int enable = 1;
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    {
        printf("setsockopt(SO_REUSEADDR) failed! errno=%d\n", errno);
    }

    // bind our server socket to the RTSP port and listen for a client connection
    if (bind(server_socket, (sockaddr *)&server_addr, sizeof(server_addr)) != 0)
    {
        printf("bind failed! errno=%d\n", errno);
    }

    if (listen(server_socket, 5) != 0)
    {
        printf("listen failed! errno=%d\n", errno);
    }

    printf("\n\nrtsp://%s:8554/mjpeg/1\n\n", CONFIG_LWIP_LOCAL_HOSTNAME);

    // loop forever to accept client connections
    while (true)
    {
        printf("Minimum free heap size: %u bytes\n", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
        client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &client_addr_len);

        printf("Client connected from: %s\n", inet_ntoa(client_addr.sin_addr));
        xTaskCreate(client_worker, "client_worker", 3584, (void *)client_socket, tskIDLE_PRIORITY, &xHandle);
    }

    close(server_socket);
}
