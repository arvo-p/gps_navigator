/**
  ******************************************************************************
  * @file    Core/Src/app.c
  * @brief   FreeRTOS 5-Task GPS Map & Navigation Application Orchestrator:
  *            1. Sensor_Input_Task (Core/Src/sensor_input_task.c, Prio 3)
  *            2. GPS_UART_Task     (Core/Src/gps_uart_task.c,     Prio 3)
  *            3. UI_LVGL_Task      (Core/Src/ui_lvgl_task.c,      Prio 2)
  *            4. Tile_Loader_Task  (Core/Src/tile_loader_task.c,  Prio 1)
  *            5. Pathfinder_Task   (Core/Src/pathfinder_task.c,   Prio 0)
  ******************************************************************************
  */

#include <string.h>
#include "app.h"
#include "stm32f429i_discovery_ts.h"

/*----------------------------------------------------------------------------
 * FreeRTOS Heap in 64 KB Zero-Wait-State CCMRAM (0x10000000)
 *----------------------------------------------------------------------------*/
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".ccmram"), aligned(8)));

/*----------------------------------------------------------------------------
 * Inter-Task Communication (Queues, Mutexes, Handles)
 *----------------------------------------------------------------------------*/
QueueHandle_t     g_q_input_evt   = NULL;
QueueHandle_t     g_q_gps_fix     = NULL;
QueueHandle_t     g_q_tile_req    = NULL;
QueueHandle_t     g_q_route_req   = NULL;
SemaphoreHandle_t g_mutex_sd      = NULL;
TaskHandle_t      g_ui_task_hdl   = NULL;
TaskHandle_t      g_gps_task_hdl  = NULL;

/*----------------------------------------------------------------------------
 * Shared Buffers & Map State (Internal SRAM + External SDRAM)
 *----------------------------------------------------------------------------*/
static uint8_t    s_draw_buf_1[MY_DISP_HOR_RES * 32 * 2] __attribute__((aligned(4)));

GeoCoord          g_gps_pos;
GpsFix_t          g_gps_fix;
MapCamera         g_camera;
VisibleTileList   g_visible_tiles;
TileGraphCache    g_graph_cache;
Pathfinder        g_pathfinder;
RouteResult       g_current_route __attribute__((section(".sdram")));

uint16_t          g_palette_rgb565[256];
volatile bool     g_pak_ready       = false;
TileCacheSlot     g_tile_cache[TILE_CACHE_SLOTS] __attribute__((section(".sdram"), aligned(4)));
uint32_t          g_cache_clock     = 0;
volatile uint32_t g_sd_tiles_loaded = 0;

/*----------------------------------------------------------------------------
 * 1. Display Flush Callback (DMA2D RGB565 -> ARGB8888 in SDRAM)
 *----------------------------------------------------------------------------*/
static void my_disp_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
  int32_t width  = lv_area_get_width(area);
  int32_t height = lv_area_get_height(area);

  uint32_t dst_addr = DISP_FRAME_BUFFER_ADDR + 4U * (area->y1 * MY_DISP_HOR_RES + area->x1);

  DISP_DMA2D_INSTANCE->CR      = 0x00010000UL; /* Memory-to-Memory with PFC */
  DISP_DMA2D_INSTANCE->FGMAR   = (uint32_t)px_map;
  DISP_DMA2D_INSTANCE->OMAR    = dst_addr;
  DISP_DMA2D_INSTANCE->FGOR    = 0;
  DISP_DMA2D_INSTANCE->OOR     = MY_DISP_HOR_RES - width;
  DISP_DMA2D_INSTANCE->FGPFCCR = LTDC_PIXEL_FORMAT_RGB565;
  DISP_DMA2D_INSTANCE->OPFCCR  = LTDC_PIXEL_FORMAT_ARGB8888;
  DISP_DMA2D_INSTANCE->NLR     = (uint32_t)((width << 16) | (uint16_t)height);
  DISP_DMA2D_INSTANCE->CR     |= DMA2D_CR_START;

  while (DISP_DMA2D_INSTANCE->CR & DMA2D_CR_START) {}

  lv_display_flush_ready(disp);
}

/*----------------------------------------------------------------------------
 * 2. Initialize BSP LCD + Touchscreen + LVGL Display Driver
 *----------------------------------------------------------------------------*/
static void LVGL_BSP_Init(void)
{
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, DISP_FRAME_BUFFER_ADDR);
  BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  BSP_TS_Init(MY_DISP_HOR_RES, MY_DISP_VER_RES);

  lv_init();
  lv_tick_set_cb(HAL_GetTick);

  lv_display_t * disp = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);
  lv_display_set_flush_cb(disp, my_disp_flush_cb);
  lv_display_set_buffers(disp, s_draw_buf_1, NULL, sizeof(s_draw_buf_1), LV_DISPLAY_RENDER_MODE_PARTIAL);
}

/*----------------------------------------------------------------------------
 * 3. Initialize GPS Library & Tile Cache
 *----------------------------------------------------------------------------*/
void Tile_Cache_Reset(void)
{
  g_cache_clock     = 0;
  g_sd_tiles_loaded = 0;
  for (int i = 0; i < TILE_CACHE_SLOTS; i++)
  {
    g_tile_cache[i].state    = TILE_SLOT_EMPTY;
    g_tile_cache[i].has_data = false;
    g_tile_cache[i].lru_tick = 0;
  }
}

static void GPS_Library_Init(void)
{
  /* Step A: Starting Coordinate (Liège center: 50.6242000° N, 5.5666000° E) */
  g_gps_pos            = geo_coord_make(506242000L, 55666000L);
  int32_t initial_zoom = 14;

  /* Step B: Initialize MapCamera & compute initial VisibleTileList */
  map_camera_init(&g_camera, g_gps_pos, initial_zoom, MY_DISP_HOR_RES, MY_DISP_VER_RES);
  map_camera_get_visible_tiles(&g_camera, &g_visible_tiles);

  /* Step C: Initialize Road Graph Cache & Hybrid A* Pathfinder */
  tile_graph_cache_init(&g_graph_cache, "binary_map_data", TILE_GRAPH_CACHE_MAX_LOCAL);
  pathfinder_init(&g_pathfinder, &g_graph_cache);
  memset(&g_current_route, 0, sizeof(RouteResult));

  Tile_Cache_Reset();
}

/*============================================================================
 * Application Entry Point Called from main()
 *============================================================================*/
void App_Run(void)
{
  LVGL_BSP_Init();
  GPS_Library_Init();

  /* Create RTOS Queues & SD SPI Mutex */
  g_q_input_evt = xQueueCreate(16, sizeof(InputEvent_t));
  g_q_gps_fix   = xQueueCreate(1,  sizeof(GpsFix_t));
  g_q_tile_req  = xQueueCreate(TILE_CACHE_SLOTS, sizeof(TileRequest_t));
  g_q_route_req = xQueueCreate(1,  sizeof(RouteRequest_t));
  g_mutex_sd    = xSemaphoreCreateMutex();

  /* Create the 5 RTOS Tasks */
  xTaskCreate(UI_LVGL_Task,      "UI_LVGL",     1536, NULL, PRIO_UI_LVGL,     &g_ui_task_hdl);
  xTaskCreate(Sensor_Input_Task, "SensorInput",  512, NULL, PRIO_SENSOR_GPS,  NULL);
  xTaskCreate(GPS_UART_Task,     "GPS_UART",     512, NULL, PRIO_SENSOR_GPS,  &g_gps_task_hdl);
  xTaskCreate(Tile_Loader_Task,  "TileLoader",  1024, NULL, PRIO_TILE_LOADER, NULL);
  xTaskCreate(Pathfinder_Task,   "Pathfinder",  1024, NULL, PRIO_PATHFINDER,  NULL);

  /* Start FreeRTOS Scheduler */
  vTaskStartScheduler();

  while (1) {}
}

void LVGL_Example_Run(void)
{
  App_Run();
}
