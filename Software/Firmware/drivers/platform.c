#include "stm32f4xx.h"
#include "drv_spi.h"

#define USING_SPI1

/*** SPI1 BUS and device
SPI1_MOSI: PA7
SPI1_MISO: PA6
SPI1_SCK : PA5
SPI1_NSS : PA4
*/
void rt_hw_spi1_init(void)
{
    /* register SPI bus */
    static struct stm32_spi_bus stm32_spi;

    /* SPI1 configure */
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        /* Enable SPI1 Periph clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,
                               ENABLE);

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

        /* Configure SPI1 pins: PA5-SCK, PA6-MISO and PA7-MOSI */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    } /* SPI1 configuration */

    /* register SPI1 */
    stm32_spi_register(SPI1, &stm32_spi, "spi1");

    /* attach spi10 */
    {
        static struct rt_spi_device rt_spi_device_10;
        static struct stm32_spi_cs  stm32_spi_cs_10;
        GPIO_InitTypeDef GPIO_InitStructure;

        stm32_spi_cs_10.GPIOx = GPIOA;
        stm32_spi_cs_10.GPIO_Pin = GPIO_Pin_4;

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        GPIO_SetBits(GPIOA, GPIO_Pin_4);

        rt_spi_bus_attach_device(&rt_spi_device_10, "spi10", "spi1", (void*)&stm32_spi_cs_10);
    } /* attach spi10 */
}


void rt_platform_init(void)
{
    rt_hw_spi1_init();

////#ifdef RT_USING_USB_HOST
////    /* register stm32 usb host controller driver */
//    rt_hw_susb_init();
////#endif

#ifdef RT_USING_DFS
    /* initilize sd card */
#ifdef RT_USING_SDIO
    rt_mmcsd_core_init();
    rt_mmcsd_blk_init();
    stm32f4xx_sdio_init();
    rt_thread_delay(RT_TICK_PER_SECOND);
#else
    rt_hw_sdcard_init();
#endif
#endif /* RT_USING_DFS */
}
