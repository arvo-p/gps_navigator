/**
  ******************************************************************************
  * @file    Core/Src/sensor_input_task.c
  * @brief   TASK 1: Sensor_Input_Task (Priority: AboveNormal)
  *          Polls Touchscreen (and future Accelerometer) at ~66 Hz and sends
  *          drag deltas to UI_LVGL_Task.
  ******************************************************************************
  */

#include "app.h"
#include "stm32f429i_discovery_ts.h"

void Sensor_Input_Task(void *argument)
{
  (void)argument;
  bool    dragging     = false;
  int32_t last_touch_x = 0;
  int32_t last_touch_y = 0;

  for (;;)
  {
    TS_StateTypeDef ts;
    BSP_TS_GetState(&ts);

    /* Future Accelerometer Hook:
     * Read I2C/SPI IMU here -> populate accel_mg[0..2] & stationary flag.
     */
    if (ts.TouchDetected)
    {
      int32_t cur_x = (int32_t)ts.X;
      int32_t cur_y = (MY_DISP_VER_RES - 1) - (int32_t)ts.Y;

      if (!dragging)
      {
        dragging     = true;
        last_touch_x = cur_x;
        last_touch_y = cur_y;
      }
      else
      {
        int32_t dx = cur_x - last_touch_x;
        int32_t dy = cur_y - last_touch_y;

        if (dx != 0 || dy != 0)
        {
          last_touch_x = cur_x;
          last_touch_y = cur_y;

          InputEvent_t evt = {
            .dx         = (int16_t)dx,
            .dy         = (int16_t)dy,
            .touch_down = true,
            .accel_mg   = {0, 0, 1000},
            .stationary = false,
          };
          xQueueSend(g_q_input_evt, &evt, 0);
        }
      }
    }
    else
    {
      dragging = false;
    }

    vTaskDelay(pdMS_TO_TICKS(15));
  }
}
