#include <ws2812.h>
#include <drv_spi.h>

#ifndef WS2812B_EXAMPLE_SPI_NAME
    #define WS2812B_EXAMPLE_SPI_NAME    "spi10"
#endif  //!WS2812B_EXAMPLE_SPI_NAME

#ifndef WS2812B_EXAMPLE_NODE_LENGTH
    #define WS2812B_EXAMPLE_NODE_LENGTH 2
#endif // !WS2812B_EXAMPLE_NODE_LENGTH

int ws2812b_test(int argc, char *argv[])
{
    rt_err_t result;
    struct rt_spi_device *spi_device;
    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_device != RT_NULL);
    result = rt_spi_bus_attach_device(spi_device, WS2812B_EXAMPLE_SPI_NAME, "spi1", (void *)RT_NULL);

    if (result != RT_EOK)
    {
        rt_kprintf("%s attach to %s faild, %d\n", "spi1", WS2812B_EXAMPLE_SPI_NAME, result);
    }

    rt_device_t ws2812b_spi = rt_device_find(WS2812B_EXAMPLE_SPI_NAME);
    if (ws2812b_spi == RT_NULL)
    {
        rt_kprintf("Not Find Spi Device: %s.\r\n", WS2812B_EXAMPLE_SPI_NAME);
        return RT_ERROR;
    }
    ws2812_t ws2812 = ws2812_create(WS2812B_EXAMPLE_SPI_NAME, WS2812B_EXAMPLE_NODE_LENGTH);
    if (!ws2812)
    {
        rt_kprintf("create ws2812 object faild.\r\n");
        return RT_ERROR;
    }

    ws281x_rainbow(ws2812, 50);
    ws281x_colorWipe(ws2812, 0x0000, 0);

    rt_free((void *)ws2812);
    return 1;
}
MSH_CMD_EXPORT(ws2812b_test, test ws2812b function);
