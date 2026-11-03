/**
  ******************************************************************************
  * @file    Core/Inc/app.h
  * @brief   Shared RTOS objects, data structures, and task prototypes for the
  *          5-Task GPS Map & Navigation application.
  ******************************************************************************
  */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lvgl.h"
#include "ff.h"
#include "sd_spi.h"
#include "geo_math.h"
#include "map_camera.h"
#include "tile_graph_cache.h"
#include "pathfinder.h"

/*----------------------------------------------------------------------------
 * Constants & Configuration
 *----------------------------------------------------------------------------*/
#define MY_DISP_HOR_RES       DISP_HOR_RES
#define MY_DISP_VER_RES       DISP_VER_RES
#define TILE_PX_SIZE          256
#define TILE_BYTES            (TILE_PX_SIZE * TILE_PX_SIZE) /* 65,536 bytes */
#define TILE_CACHE_SLOTS      9                             /* 3x3 LRU cache (576 KB in SDRAM) */

#define PATH_TILES_Z12        "tiles/tiles_z12.pak"
#define PATH_TILES_Z14        "tiles/tiles_z14.pak"
#define PATH_TILES_Z16        "tiles/tiles_z16.pak"

/* 8-bit Indexed Tile Palette Indices (matching PALETTE_16_DARK) */
#define PAL_IDX_BACKGROUND      0U
#define PAL_IDX_WATER           1U
#define PAL_IDX_MINOR_ROAD      2U
#define PAL_IDX_SECONDARY_ROAD  3U
#define PAL_IDX_PRIMARY_ROAD    4U
#define PAL_IDX_MOTORWAY        5U
#define PAL_IDX_FOREST          10U
#define PAL_IDX_GREENWAY_TRACK  11U
#define PAL_IDX_FOOTWAY         12U

/* Task Priorities */
#define PRIO_PATHFINDER       (tskIDLE_PRIORITY + 0)
#define PRIO_TILE_LOADER      (tskIDLE_PRIORITY + 1)
#define PRIO_UI_LVGL          (tskIDLE_PRIORITY + 2)
#define PRIO_SENSOR_GPS       (tskIDLE_PRIORITY + 3)

/* Task Notification Bits -> UI_LVGL_Task */
#define NOTIFY_SD_MOUNTED     (1UL << 0)
#define NOTIFY_SD_ERROR       (1UL << 1)
#define NOTIFY_TILE_LOADED    (1UL << 2)
#define NOTIFY_ROUTE_READY    (1UL << 3)

/*----------------------------------------------------------------------------
 * Inter-Task Message & Cache Types
 *----------------------------------------------------------------------------*/
typedef struct {
  int16_t dx;
  int16_t dy;
  bool    touch_down;
  int16_t accel_mg[3]; /* Future accelerometer X/Y/Z in milli-g */
  bool    stationary;  /* Motion state from IMU */
} InputEvent_t;

typedef struct {
  GeoCoord pos;
  uint16_t speed_kmh;
  uint16_t heading_deg;
  uint8_t  utc_hour;
  uint8_t  utc_min;
  uint8_t  utc_sec;
  uint8_t  utc_day;
  uint8_t  utc_month;
  uint16_t utc_year;
  uint8_t  satellites;      /* Satellites used in fix ($xxGGA) */
  uint8_t  sats_in_view;    /* Satellites visible ($GPGSV / $GLGSV) */
  bool     uart_ok;
  bool     time_valid;
  bool     valid;
} GpsFix_t;

typedef struct {
  uint8_t slot_idx;
  int32_t tx;
  int32_t ty;
} TileRequest_t;

typedef struct {
  GeoCoord start;
  GeoCoord goal;
} RouteRequest_t;

typedef enum {
  TILE_SLOT_EMPTY = 0,
  TILE_SLOT_LOADING,
  TILE_SLOT_READY
} TileSlotState_t;

typedef struct {
  int32_t                  tx;
  int32_t                  ty;
  uint32_t                 lru_tick;
  volatile TileSlotState_t state;
  volatile bool            has_data;
  uint8_t                  pixels[TILE_BYTES];
} TileCacheSlot;

/*----------------------------------------------------------------------------
 * Shared RTOS Handles & Global Application State (Defined in app.c)
 *----------------------------------------------------------------------------*/
extern QueueHandle_t     g_q_input_evt;
extern QueueHandle_t     g_q_gps_fix;
extern QueueHandle_t     g_q_tile_req;
extern QueueHandle_t     g_q_route_req;
extern SemaphoreHandle_t g_mutex_sd;
extern TaskHandle_t      g_ui_task_hdl;
extern TaskHandle_t      g_gps_task_hdl;

extern GeoCoord          g_gps_pos;
extern GpsFix_t          g_gps_fix;
extern MapCamera         g_camera;
extern VisibleTileList   g_visible_tiles;
extern TileGraphCache    g_graph_cache;
extern Pathfinder        g_pathfinder;
extern RouteResult       g_current_route;

extern uint16_t          g_palette_rgb565[256];
extern volatile bool     g_pak_ready;
extern TileCacheSlot     g_tile_cache[TILE_CACHE_SLOTS];
extern uint32_t          g_cache_clock;
extern volatile uint32_t g_sd_tiles_loaded;

/*----------------------------------------------------------------------------
 * Task Entry Points (Each in its own dedicated task .c file in Core/Src/)
 *----------------------------------------------------------------------------*/
void Sensor_Input_Task(void *argument);
void GPS_UART_Task(void *argument);
void Tile_Loader_Task(void *argument);
void Pathfinder_Task(void *argument);
void UI_LVGL_Task(void *argument);

void Tile_Cache_Reset(void);
void Tile_Palette_Hide_Pedestrian_Paths(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
