#include "../../../Core/Inc/bsp.h"
#ifdef HAL_SPI_MODULE_ENABLED

/* 包含头文件 */
#include "../user_spi.h"

/* 函数体 */

void SPI_Init(SPI_DRIVES* user_spi, SPI_HandleTypeDef* hspi,
              GPIO_TypeDef* cs_port, uint16_t cs_pin) {
    user_spi->hspi = hspi;
    user_spi->cs_port = cs_port;
    user_spi->cs_pin = cs_pin;
    user_spi->timeout = 10;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);   /* 片选空闲拉高 */
}

/* 一次完整收发，内部管片选。返回 1 成功 */
uint8_t SPI_Transfer(SPI_DRIVES* user_spi, const uint8_t* tx, uint8_t* rx, const uint16_t len) {
    HAL_GPIO_WritePin(user_spi->cs_port, user_spi->cs_pin, GPIO_PIN_RESET);

    const uint8_t result = (HAL_SPI_TransmitReceive(user_spi->hspi,
                            (uint8_t*)tx, rx, len, user_spi->timeout) == HAL_OK);

    HAL_GPIO_WritePin(user_spi->cs_port, user_spi->cs_pin, GPIO_PIN_SET);
    return result;
}

#endif /* HAL_SPI_MODULE_ENABLED */
