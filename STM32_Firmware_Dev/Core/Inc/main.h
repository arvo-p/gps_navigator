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
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"

void Toggle_Leds(void);
void LVGL_Example_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
