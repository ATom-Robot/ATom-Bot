#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sys/socket.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "app_wifi.h"
#include "app_uart.h"

static const char *TAG = "tcp-server";

#define TCP_SERVER_BIND_PORT    "1234"

/**
 * @brief Indicates that the file descriptor represents an invalid (uninitialized or closed) socket
 *
 * Used in the TCP server structure `sock[]` which holds list of active clients we serve.
 */
#define INVALID_SOCK (-1)

/**
 * @brief Time in ms to yield to all tasks when a non-blocking socket would block
 *
 * Non-blocking socket operations are typically executed in a separate task validating
 * the socket status. Whenever the socket returns `EAGAIN` (idle status, i.e. would block)
 * we have to yield to all tasks to prevent lower priority tasks from starving.
 */
#define YIELD_TO_ALL_MS 50

static void pack_data_analysis(int len, const char *rx_buffer);

/**
 * @brief Utility to log socket errors
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket number
 * @param[in] err Socket errno
 * @param[in] message Message to print
 */
static void log_socket_error(const char *tag, const int sock, const int err, const char *message)
{
    ESP_LOGE(tag, "[sock=%d]: %s\n"
             "error=%d: %s", sock, message, err, strerror(err));
}

/**
 * @brief Tries to receive data from specified sockets in a non-blocking way,
 *        i.e. returns immediately if no data.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket for reception
 * @param[out] data Data pointer to write the received data
 * @param[in] max_len Maximum size of the allocated space for receiving data
 * @return
 *          >0 : Size of received data
 *          =0 : No data available
 *          -1 : Error occurred during socket read operation
 *          -2 : Socket is not connected, to distinguish between an actual socket error and active disconnection
 */
static int try_receive(const char *tag, const int sock, char *data, size_t max_len)
{
    int len = recv(sock, data, max_len, 0);
    if (len < 0)
    {
        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0;   // Not an error
        }
        if (errno == ENOTCONN)
        {
            ESP_LOGW(tag, "[sock=%d]: Connection closed", sock);
            return -2;  // Socket has been disconnected
        }
        log_socket_error(tag, sock, errno, "Error occurred during receiving");
        return -1;
    }

    return len;
}

/**
 * @brief Sends the specified data to the socket. This function blocks until all bytes got sent.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket to write data
 * @param[in] data Data to be written
 * @param[in] len Length of the data
 * @return
 *          >0 : Size the written data
 *          -1 : Error occurred during socket write operation
 */
static int socket_send(const char *tag, const int sock, const char *data, const size_t len)
{
    int to_write = len;
    while (to_write > 0)
    {
        int written = send(sock, data + (len - to_write), to_write, 0);
        if (written < 0 && errno != EINPROGRESS && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            log_socket_error(tag, sock, errno, "Error occurred during sending");
            return -1;
        }
        to_write -= written;
    }
    return len;
}

/**
 * @brief Returns the string representation of client's address (accepted on this server)
 */
static inline char *get_clients_address(struct sockaddr_storage *source_addr)
{
    static char address_str[128];
    char *res = NULL;
    // Convert ip address to string
    if (source_addr->ss_family == PF_INET)
    {
        res = inet_ntoa_r(((struct sockaddr_in *)source_addr)->sin_addr, address_str, sizeof(address_str) - 1);
    }
#ifdef CONFIG_LWIP_IPV6
    else if (source_addr->ss_family == PF_INET6)
    {
        res = inet6_ntoa_r(((struct sockaddr_in6 *)source_addr)->sin6_addr, address_str, sizeof(address_str) - 1);
    }
#endif
    if (!res)
    {
        address_str[0] = '\0'; // Returns empty string if conversion didn't succeed
    }
    return address_str;
}

static void tcp_server_task(void *pvParameters)
{
    static char rx_buffer[64];
    xSemaphoreHandle *server_ready = pvParameters;
    struct addrinfo hints = { .ai_socktype = SOCK_STREAM };
    struct addrinfo *address_info;
    int listen_sock = INVALID_SOCK;
    const size_t max_socks = CONFIG_LWIP_MAX_SOCKETS - 1;
    static int sock[CONFIG_LWIP_MAX_SOCKETS - 1];

    for (int i = 0; i < max_socks; ++i)
    {
        sock[i] = INVALID_SOCK;
    }

    if (wifi_ip_address[0] == '\0')
    {
        ESP_LOGE(TAG, "wifi was not connect,exit tcpserver!");
        return;
    }

    // Translating the hostname or a string representation of an IP to address_info
    int res = getaddrinfo(wifi_ip_address, TCP_SERVER_BIND_PORT, &hints, &address_info);
    if (res != 0 || address_info == NULL)
    {
        ESP_LOGE(TAG, "couldn't get hostname for `%s` "
                 "getaddrinfo() returns %d, addrinfo=%p", wifi_ip_address, res, address_info);
        goto error;
    }

    // Creating a listener socket
    listen_sock = socket(address_info->ai_family, address_info->ai_socktype, address_info->ai_protocol);

    if (listen_sock < 0)
    {
        log_socket_error(TAG, listen_sock, errno, "Unable to create socket");
        goto error;
    }
    ESP_LOGI(TAG, "Listener socket created");

    // Marking the socket as non-blocking
    int flags = fcntl(listen_sock, F_GETFL);
    if (fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        log_socket_error(TAG, listen_sock, errno, "Unable to set socket non blocking");
        goto error;
    }
    ESP_LOGI(TAG, "Socket marked as non blocking");

    // Binding socket to the given address
    int err = bind(listen_sock, address_info->ai_addr, address_info->ai_addrlen);
    if (err != 0)
    {
        log_socket_error(TAG, listen_sock, errno, "Socket unable to bind");
        goto error;
    }
    ESP_LOGI(TAG, "Socket bound on %s:%s", wifi_ip_address, TCP_SERVER_BIND_PORT);

    // Set queue (backlog) of pending connections to one (can be more)
    err = listen(listen_sock, 1);
    if (err != 0)
    {
        log_socket_error(TAG, listen_sock, errno, "Error occurred during listen");
        goto error;
    }
    ESP_LOGI(TAG, "Socket listening");
    xSemaphoreGive(*server_ready);

    // Main loop for accepting new connections and serving all connected clients
    while (1)
    {
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);

        // Find a free socket
        int new_sock_index = 0;
        for (new_sock_index = 0; new_sock_index < max_socks; ++new_sock_index)
        {
            if (sock[new_sock_index] == INVALID_SOCK)
            {
                break;
            }
        }

        // We accept a new connection only if we have a free socket
        if (new_sock_index < max_socks)
        {
            // Try to accept a new connections
            sock[new_sock_index] = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

            if (sock[new_sock_index] < 0)
            {
                if (errno == EWOULDBLOCK)   // The listener socket did not accepts any connection
                {
                    // continue to serve open connections and try to accept again upon the next iteration
                    ESP_LOGV(TAG, "No pending connections...");
                }
                else
                {
                    log_socket_error(TAG, listen_sock, errno, "Error when accepting connection");
                    goto error;
                }
            }
            else
            {
                // We have a new client connected -> print it's address
                ESP_LOGI(TAG, "[sock=%d]: Connection accepted from IP:%s", sock[new_sock_index], get_clients_address(&source_addr));

                // ...and set the client's socket non-blocking
                flags = fcntl(sock[new_sock_index], F_GETFL);
                if (fcntl(sock[new_sock_index], F_SETFL, flags | O_NONBLOCK) == -1)
                {
                    log_socket_error(TAG, sock[new_sock_index], errno, "Unable to set socket non blocking");
                    goto error;
                }
                ESP_LOGI(TAG, "[sock=%d]: Socket marked as non blocking", sock[new_sock_index]);
            }
        }

        // We serve all the connected clients in this loop
        for (int i = 0; i < max_socks; ++i)
        {
            if (sock[i] != INVALID_SOCK)
            {
                // This is an open socket -> try to serve it
                int len = try_receive(TAG, sock[i], rx_buffer, sizeof(rx_buffer));
                if (len < 0)
                {
                    // Error occurred within this client's socket -> close and mark invalid
                    ESP_LOGI(TAG, "[sock=%d]: try_receive() returned %d -> closing the socket", sock[i], len);
                    close(sock[i]);
                    sock[i] = INVALID_SOCK;
                }
                else if (len > 0)
                {
                    // Received some data -> echo back
                    // ESP_LOGI(TAG, "[sock=%d]: Received %.*s", sock[i], len, rx_buffer);
                    pack_data_analysis(len, rx_buffer);

                    len = socket_send(TAG, sock[i], "server", 7);
                    if (len < 0)
                    {
                        // Error occurred on write to this socket -> close it and mark invalid
                        ESP_LOGI(TAG, "[sock=%d]: socket_send() returned %d -> closing the socket", sock[i], len);
                        close(sock[i]);
                        sock[i] = INVALID_SOCK;
                    }
                }

            } // one client's socket
        } // for all sockets

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(YIELD_TO_ALL_MS));
    }

error:
    if (listen_sock != INVALID_SOCK)
    {
        close(listen_sock);
    }

    for (int i = 0; i < max_socks; ++i)
    {
        if (sock[i] != INVALID_SOCK)
        {
            close(sock[i]);
        }
    }

    free(address_info);
    vTaskDelete(NULL);
}

Chassis_data chassis;

/* 数据解析部分 */
static void pack_data_analysis(int len, const char *rx_buffer)
{
    // 油门、方向
    if (rx_buffer[0] == 0xAA && rx_buffer[1] == 0xBB)
    {
        int thro = *((uint8_t *)(rx_buffer + 3));
        int yaw = *((uint8_t *)(rx_buffer + 5));

        chassis.thro = thro >= 100 ? abs(50 - (thro / 2)) : -abs(50 - (thro / 2));
        chassis.yaw = yaw >= 100 ? abs(50 - (yaw / 2)) : -abs(50 - (yaw / 2));

        ESP_LOGI(TAG, "[thro=%d][yaw=%d]", chassis.thro, chassis.yaw);
    }
    // 舵机角度
    else if (rx_buffer[0] == 0xAA && rx_buffer[1] == 0xBC)
    {
        chassis.angle = *((uint8_t *)(rx_buffer + 3));

        ESP_LOGI(TAG, "[angle=%d]", chassis.angle);
    }

    // 串口发送数据
    data_sendto_ChassisData(chassis.thro, chassis.yaw, chassis.angle, 0);
}

esp_err_t APPTcpServer_run(void)
{
    xSemaphoreHandle server_ready = xSemaphoreCreateBinary();
    assert(server_ready);
    BaseType_t result  = xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 4 * 1024, &server_ready, 5, NULL, 0);
    if (result != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to create tcp server task");
        return ESP_FAIL;
    }
    xSemaphoreTake(server_ready, portMAX_DELAY);
    vSemaphoreDelete(server_ready);

    return ESP_OK;
}
