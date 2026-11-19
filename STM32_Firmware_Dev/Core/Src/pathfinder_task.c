/**
  ******************************************************************************
  * @file    Core/Src/pathfinder_task.c
  * @brief   TASK 4: Pathfinder_Task (Priority: Low)
  *          Runs Hybrid A* route computation and local off-track rerouting
  *          in the background on request.
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
      bool route_success = false;
      xSemaphoreTake(g_mutex_sd, portMAX_DELAY);

      switch (req.cmd)
      {
        case ROUTE_CMD_NEW:
          route_success = pathfinder_find_route(&g_pathfinder,
                                                req.start,
                                                req.goal,
                                                &g_current_route);
          if (route_success)
          {
            nav_init(&g_nav_state);
            nav_update(&g_nav_state, &g_current_route, &g_geocoder, req.start);
          }
          break;

        case ROUTE_CMD_REROUTE:
          if (g_current_route.found && g_current_route.segment_count > 0)
          {
            route_success = pathfinder_reroute_to_existing(&g_pathfinder,
                                                           req.start,
                                                           g_nav_state.current_seg_idx,
                                                           &g_current_route);
            if (route_success)
            {
              nav_init(&g_nav_state);
              nav_update(&g_nav_state, &g_current_route, &g_geocoder, req.start);
            }
          }
          break;

        default:
          break;
      }

      xSemaphoreGive(g_mutex_sd);
      (void)route_success;
      xTaskNotify(g_ui_task_hdl, NOTIFY_ROUTE_READY, eSetBits);
    }
  }
}
