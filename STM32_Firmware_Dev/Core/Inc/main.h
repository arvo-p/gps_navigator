/**
  ******************************************************************************
  * @file    Core/Inc/main.h
  * @brief   Header for main.c module
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"

/*============================================================================
 * Debug Configuration
 *============================================================================*/
/* Set to 1 to show the debug HUD bar (UART status, UTC time, Fix/Satellites, Zoom),
 * or 0 to hide all debug UI overlays. */
#define APP_DEBUG_ENABLE                1

/*============================================================================
 * Hardware Port & Pin Mapping (Change here when migrating to another MCU)
 *============================================================================*/

/* --- 1. GPS UART + DMA Configuration (ATGM336H-5N) --- */
#define GPS_UART_INSTANCE               USART1
#define GPS_UART_BAUDRATE               9600U
#define GPS_UART_CLK_ENABLE()           __HAL_RCC_USART1_CLK_ENABLE()
#define GPS_UART_GET_PCLK_FREQ()        HAL_RCC_GetPCLK2Freq()
#define GPS_UART_AF                     GPIO_AF7_USART1

/* PA9 = MCU USART1_TX (connected to GPS RX) */
#define GPS_UART_TX_GPIO_PORT           GPIOA
#define GPS_UART_TX_PIN                 GPIO_PIN_9
#define GPS_UART_TX_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()

/* PA10 = MCU USART1_RX (connected to GPS TX) */
#define GPS_UART_RX_GPIO_PORT           GPIOA
#define GPS_UART_RX_PIN                 GPIO_PIN_10
#define GPS_UART_RX_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()

/* GPS UART Interrupt & Circular RX DMA Stream */
#define GPS_UART_IRQn                   USART1_IRQn
#define GPS_UART_IRQHandler             USART1_IRQHandler

#define GPS_DMA_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()
#define GPS_DMA_RX_STREAM               DMA2_Stream2
#define GPS_DMA_RX_CHANNEL              4UL
#define GPS_DMA_RX_IRQn                 DMA2_Stream2_IRQn
#define GPS_DMA_RX_IRQHandler           DMA2_Stream2_IRQHandler
#define GPS_DMA_RX_ISR_REG              (DMA2->LISR)
#define GPS_DMA_RX_IFCR_REG             (DMA2->LIFCR)
#define GPS_DMA_RX_ISR_FLAGS            (DMA_LISR_HTIF2 | DMA_LISR_TCIF2 | DMA_LISR_TEIF2 | DMA_LISR_DMEIF2 | DMA_LISR_FEIF2)
#define GPS_DMA_RX_CLEAR_FLAGS          (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2)

/* --- 2. microSD SPI Configuration (FatFs DiskIO) --- */
#define SD_SPI_INSTANCE                 SPI5
#define SD_SPI_CLK_ENABLE()             __HAL_RCC_SPI5_CLK_ENABLE()
#define SD_SPI_GPIO_PORT                GPIOF
#define SD_SPI_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
#define SD_SPI_SCK_PIN                  GPIO_PIN_7
#define SD_SPI_MISO_PIN                 GPIO_PIN_8
#define SD_SPI_MOSI_PIN                 GPIO_PIN_9
#define SD_SPI_AF                       GPIO_AF5_SPI5
#define SD_SPI_SLOW_PRESCALER           SPI_BAUDRATEPRESCALER_256
#define SD_SPI_FAST_PRESCALER           SPI_BAUDRATEPRESCALER_8

/* SD Card Chip Select (PC2) */
#define SD_CS_GPIO_PORT                 GPIOC
#define SD_CS_PIN                       GPIO_PIN_2
#define SD_CS_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()

/* Shared SPI5 Bus Companion Pins on STM32F429I-Discovery:
 *   - Onboard L3GD20 Gyro CS (PC1) held HIGH so it never drives MISO
 *   - Onboard ILI9341 WRX/DCX (PD13) held HIGH so LCD ignores SD SPI traffic */
#define SD_GYRO_CS_GPIO_PORT            GYRO_CS_GPIO_PORT
#define SD_GYRO_CS_PIN                  GYRO_CS_PIN
#define SD_GYRO_CS_GPIO_CLK_ENABLE()    GYRO_CS_GPIO_CLK_ENABLE()

#define SD_LCD_DCX_GPIO_PORT            LCD_WRX_GPIO_PORT
#define SD_LCD_DCX_PIN                  LCD_WRX_PIN
#define SD_LCD_DCX_GPIO_CLK_ENABLE()    LCD_WRX_GPIO_CLK_ENABLE()

/* --- 3. Display, DMA2D & Board I/O Configuration --- */
#define DISP_HOR_RES                    240
#define DISP_VER_RES                    320
#define DISP_FRAME_BUFFER_ADDR          LCD_FRAME_BUFFER
#define DISP_DMA2D_INSTANCE             DMA2D

#define BOARD_LED_GREEN                 LED3
#define BOARD_LED_RED                   LED4
#define BOARD_USER_BUTTON_PIN           KEY_BUTTON_PIN

/* ATGM336H-5N (CASIC) Constellation Modes ($PCAS04) */
#define ATGM336H_MODE_GPS_ONLY          1U  /* $PCAS04,1*18 */
#define ATGM336H_MODE_BDS_ONLY          2U  /* $PCAS04,2*1B */
#define ATGM336H_MODE_GPS_BDS           3U  /* $PCAS04,3*1A */
#define ATGM336H_MODE_GLONASS_ONLY      4U  /* $PCAS04,4*1D */
#define ATGM336H_MODE_GPS_GLONASS       5U  /* $PCAS04,5*1C */
#define ATGM336H_MODE_GPS_BDS_GLONASS   7U  /* $PCAS04,7*1E */

void Toggle_Leds(void);
void App_Run(void);
void LVGL_Example_Run(void);

/* ATGM336H-5N Control API */
void GPS_ATGM336H_SetConstellation(uint8_t mode);
void GPS_ATGM336H_EnterSleep(uint16_t standby_sec);
void GPS_ATGM336H_Wakeup(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
