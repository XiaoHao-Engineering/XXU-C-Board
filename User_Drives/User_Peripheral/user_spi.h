#ifndef USER_SPI_H
#define USER_SPI_H
#include "main.h"
#ifdef HAL_SPI_MODULE_ENABLED

/* 包含头文件 */
#include "../../Core/Inc/bsp_config.h"

/* 类型定义 */

typedef struct {
    SPI_HandleTypeDef* hspi;    /* SPI 硬件句柄 */
    GPIO_TypeDef*      cs_port; /* 片选端口 */
    uint16_t           cs_pin;  /* 片选引脚 */
    uint32_t           timeout; /* 超时 ms */
} SPI_DRIVES;

/* 函数声明 */
void    SPI_Init(SPI_DRIVES* user_spi, SPI_HandleTypeDef* hspi,
                 GPIO_TypeDef* cs_port, uint16_t cs_pin);
uint8_t SPI_Transfer(SPI_DRIVES* user_spi, const uint8_t* tx, uint8_t* rx, uint16_t len);

#endif /* HAL_SPI_MODULE_ENABLED */
#endif // USER_SPI_H
