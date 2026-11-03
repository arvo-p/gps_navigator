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

#include "main.h"
#include "diskio.h"

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
