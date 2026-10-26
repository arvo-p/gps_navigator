/**
  ******************************************************************************
  * @file    Core/Inc/sd_spi.h
  * @brief   SPI5 microSD card driver for STM32F429I-Discovery (FatFs diskio)
  ******************************************************************************
  */

#ifndef SD_SPI_H
#define SD_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "diskio.h"

/*----------------------------------------------------------------------------
 * Pin Configuration:
 *   SPI5 Bus : PF7 (SCK), PF8 (MISO), PF9 (MOSI)
 *   SD CS    : PC2 (shared with LCD_NCS; LCD_WRX/PD13 is held HIGH so the
 *              ILI9341 ignores SPI traffic while LTDC drives the display)
 *----------------------------------------------------------------------------*/
#define SD_CS_GPIO_PORT            GPIOC
#define SD_CS_PIN                  GPIO_PIN_2
#define SD_CS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()

/* Card type flags */
#define CT_MMC      0x01
#define CT_SD1      0x02
#define CT_SD2      0x04
#define CT_SDC      (CT_SD1 | CT_SD2)
#define CT_BLOCK    0x08

uint8_t SD_SPI_GetCardType(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_SPI_H */
