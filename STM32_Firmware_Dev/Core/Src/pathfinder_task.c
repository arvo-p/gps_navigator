/**
  ******************************************************************************
  * @file    Core/Src/pathfinder_task.c
  * @brief   TASK 4: Pathfinder_Task (Priority: Low)
  *          Runs Hybrid A* route computation in the background on request.
  ******************************************************************************
  */

#include "app.h"

void Pathfinder_Task(void *argument)
{
  (void)argument;
  RouteRequest_t req;

  for (;;)
  {
    if (xQueueReceive(g_q_route_req, &req, portMAX_DELAY) == pdPASS)
    {
      xSemaphoreTake(g_mutex_sd, portMAX_DELAY);
      pathfinder_find_route(&g_pathfinder, req.start, req.goal, &g_current_route);
      xSemaphoreGive(g_mutex_sd);

      xTaskNotify(g_ui_task_hdl, NOTIFY_ROUTE_READY, eSetBits);
    }
  }
}
