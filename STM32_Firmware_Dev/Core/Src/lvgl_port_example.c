/**
  ******************************************************************************
  * @file    Core/Src/lvgl_port_example.c
  * @brief   FreeRTOS 5-Task GPS Map & Navigation Port for STM32F429I-Discovery:
  *            1. Sensor_Input_Task (Touchscreen + Accelerometer hook, Prio 3)
  *            2. GPS_UART_Task     (NMEA/UBX fix publisher, Prio 3)
  *            3. UI_LVGL_Task      (Camera, non-blocking SDRAM tile blit, LVGL, Prio 2)
  *            4. Tile_Loader_Task  (Async microSD .pak tile streamer, Prio 1)
  *            5. Pathfinder_Task   (Background Hybrid A* routing, Prio 0)
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lvgl.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"
#include "ff.h"
#include "sd_spi.h"
#include "geo_math.h"
#include "map_camera.h"
#include "tile_graph_cache.h"
#include "pathfinder.h"

/*----------------------------------------------------------------------------
 * Constants & Configuration
 *----------------------------------------------------------------------------*/
#define MY_DISP_HOR_RES       240
#define MY_DISP_VER_RES       320
#define TILE_PX_SIZE          256
#define TILE_BYTES            (TILE_PX_SIZE * TILE_PX_SIZE) /* 65,536 bytes */
#define TILE_CACHE_SLOTS      9                             /* 3x3 LRU cache (576 KB in SDRAM) */

#define PATH_TILES_Z12        "tiles/tiles_z12.pak"
#define PATH_TILES_Z14        "tiles/tiles_z14.pak"
#define PATH_TILES_Z16        "tiles/tiles_z16.pak"

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
 * FreeRTOS Heap in 64 KB Zero-Wait-State CCMRAM (0x10000000)
 *----------------------------------------------------------------------------*/
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".ccmram"), aligned(8)));

/*----------------------------------------------------------------------------
 * Inter-Task Communication (Queues, Mutexes, Handles)
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

static QueueHandle_t     s_q_input_evt   = NULL;
static QueueHandle_t     s_q_gps_fix     = NULL;
static QueueHandle_t     s_q_tile_req    = NULL;
static QueueHandle_t     s_q_route_req   = NULL;
static SemaphoreHandle_t s_mutex_sd      = NULL;
static TaskHandle_t      s_ui_task_hdl   = NULL;

/*----------------------------------------------------------------------------
 * Static Buffers (Internal SRAM + External SDRAM)
 *----------------------------------------------------------------------------*/
static uint8_t draw_buf_1[MY_DISP_HOR_RES * 32 * 2] __attribute__((aligned(4)));
static FATFS   s_fatfs;

/* GPS Map & Routing State */
static GeoCoord        s_gps_pos;
static MapCamera       s_camera;
static VisibleTileList s_visible_tiles;
static TileGraphCache  s_graph_cache;
static Pathfinder      s_pathfinder;
static RouteResult     s_current_route __attribute__((section(".sdram")));

/* Tile Archive (.pak) State */
static FIL             s_pak_z14;
static DWORD           s_clmt_z14[64];
static TilePakHeader   s_hdr_z14;
static uint16_t        s_palette_rgb565[256];
static volatile bool   s_pak_ready = false;

/* Non-Blocking 9-Slot (3x3) LRU Tile Cache in External SDRAM (.sdram) */
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

static TileCacheSlot     s_tile_cache[TILE_CACHE_SLOTS] __attribute__((section(".sdram"), aligned(4)));
static uint32_t          s_cache_clock     = 0;
static volatile uint32_t s_sd_tiles_loaded = 0;

/* Full-screen RGB565 Map Framebuffer in External SDRAM (.sdram) */
static uint16_t          s_map_fb[MY_DISP_HOR_RES * MY_DISP_VER_RES] __attribute__((section(".sdram"), aligned(4)));

static const lv_image_dsc_t s_map_img_dsc = {
  .header = {
    .magic  = LV_IMAGE_HEADER_MAGIC,
    .cf     = LV_COLOR_FORMAT_RGB565,
    .flags  = 0,
    .w      = MY_DISP_HOR_RES,
    .h      = MY_DISP_VER_RES,
    .stride = MY_DISP_HOR_RES * sizeof(uint16_t),
  },
  .data_size = sizeof(s_map_fb),
  .data      = (const uint8_t *)s_map_fb,
};

/*----------------------------------------------------------------------------
 * 1. Display Flush Callback (DMA2D RGB565 -> ARGB8888 in SDRAM)
 *----------------------------------------------------------------------------*/
static void my_disp_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
  int32_t width  = lv_area_get_width(area);
  int32_t height = lv_area_get_height(area);

  uint32_t dst_addr = LCD_FRAME_BUFFER + 4U * (area->y1 * MY_DISP_HOR_RES + area->x1);

  DMA2D->CR      = 0x00010000UL;                         /* Memory-to-Memory with PFC */
  DMA2D->FGMAR   = (uint32_t)px_map;
  DMA2D->OMAR    = dst_addr;
  DMA2D->FGOR    = 0;
  DMA2D->OOR     = MY_DISP_HOR_RES - width;
  DMA2D->FGPFCCR = LTDC_PIXEL_FORMAT_RGB565;
  DMA2D->OPFCCR  = LTDC_PIXEL_FORMAT_ARGB8888;
  DMA2D->NLR     = (uint32_t)((width << 16) | (uint16_t)height);
  DMA2D->CR     |= DMA2D_CR_START;

  while (DMA2D->CR & DMA2D_CR_START) {}

  lv_display_flush_ready(disp);
}

/*----------------------------------------------------------------------------
 * 2. Initialize BSP LCD + Touchscreen + LVGL Display Driver
 *----------------------------------------------------------------------------*/
void LVGL_BSP_Init(void)
{
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  BSP_TS_Init(MY_DISP_HOR_RES, MY_DISP_VER_RES);

  lv_init();
  lv_tick_set_cb(HAL_GetTick);

  lv_display_t * disp = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);
  lv_display_set_flush_cb(disp, my_disp_flush_cb);
  lv_display_set_buffers(disp, draw_buf_1, NULL, sizeof(draw_buf_1), LV_DISPLAY_RENDER_MODE_PARTIAL);
}

/*----------------------------------------------------------------------------
 * 3. Initialize GPS Library & Tile Cache
 *----------------------------------------------------------------------------*/
static void Tile_Cache_Reset(void)
{
  s_cache_clock     = 0;
  s_sd_tiles_loaded = 0;
  for (int i = 0; i < TILE_CACHE_SLOTS; i++)
  {
    s_tile_cache[i].state    = TILE_SLOT_EMPTY;
    s_tile_cache[i].has_data = false;
    s_tile_cache[i].lru_tick = 0;
  }
}

static void GPS_Library_Init(void)
{
  /* Step A: Starting Coordinate (Liège center: 50.6242000° N, 5.5666000° E) */
  s_gps_pos            = geo_coord_make(506242000L, 55666000L);
  int32_t initial_zoom = 14;

  /* Step B: Initialize MapCamera & compute initial VisibleTileList */
  map_camera_init(&s_camera, s_gps_pos, initial_zoom, MY_DISP_HOR_RES, MY_DISP_VER_RES);
  map_camera_get_visible_tiles(&s_camera, &s_visible_tiles);

  /* Step C: Initialize Road Graph Cache & Hybrid A* Pathfinder */
  tile_graph_cache_init(&s_graph_cache, "binary_map_data", TILE_GRAPH_CACHE_MAX_LOCAL);
  pathfinder_init(&s_pathfinder, &s_graph_cache);
  memset(&s_current_route, 0, sizeof(RouteResult));

  Tile_Cache_Reset();
}

/*----------------------------------------------------------------------------
 * 4. SD Card & .pak Helpers (Called Exclusively Under s_mutex_sd)
 *----------------------------------------------------------------------------*/
static bool Init_Tile_Pak_Locked(const char *path, FIL *fil, TilePakHeader *hdr,
                                 uint16_t *palette_out, DWORD *clmt_buf, UINT clmt_len)
{
  UINT br = 0;

  if (f_open(fil, path, FA_READ) != FR_OK)
  {
    return false;
  }

  fil->cltbl  = clmt_buf;
  clmt_buf[0] = clmt_len;
  f_lseek(fil, CREATE_LINKMAP);

  if (f_lseek(fil, 0) != FR_OK ||
      f_read(fil, hdr, sizeof(TilePakHeader), &br) != FR_OK ||
      br != sizeof(TilePakHeader) ||
      hdr->magic != TILE_PAK_MAGIC)
  {
    f_close(fil);
    return false;
  }

  uint32_t num_x          = hdr->x_max - hdr->x_min + 1U;
  uint32_t num_y          = hdr->y_max - hdr->y_min + 1U;
  uint32_t table_end      = sizeof(TilePakHeader) + (num_x * num_y * 4U);
  uint32_t palette_sector = (table_end + 511U) / 512U;

  if (f_lseek(fil, (FSIZE_t)palette_sector * 512U) != FR_OK ||
      f_read(fil, palette_out, 512U, &br) != FR_OK ||
      br != 512U)
  {
    f_close(fil);
    return false;
  }

  return true;
}

static bool Load_Tile_From_Pak_Locked(FIL *pak_file, const TilePakHeader *hdr,
                                      int32_t tx, int32_t ty, uint8_t *tile_buf)
{
  if (tx < (int32_t)hdr->x_min || tx > (int32_t)hdr->x_max ||
      ty < (int32_t)hdr->y_min || ty > (int32_t)hdr->y_max)
  {
    memset(tile_buf, 0, TILE_BYTES);
    return false;
  }

  uint32_t num_y    = hdr->y_max - hdr->y_min + 1U;
  uint32_t col      = (uint32_t)tx - hdr->x_min;
  uint32_t row      = (uint32_t)ty - hdr->y_min;
  uint32_t grid_idx = col * num_y + row;

  uint32_t sector_idx = 0;
  UINT br = 0;
  FSIZE_t idx_offset = (FSIZE_t)sizeof(TilePakHeader) + (FSIZE_t)grid_idx * sizeof(uint32_t);

  if (f_lseek(pak_file, idx_offset) != FR_OK ||
      f_read(pak_file, &sector_idx, sizeof(uint32_t), &br) != FR_OK ||
      br != sizeof(uint32_t) ||
      sector_idx == 0U)
  {
    memset(tile_buf, 0, TILE_BYTES);
    return false;
  }

  FSIZE_t tile_offset = (FSIZE_t)sector_idx * 512U;
  if (f_lseek(pak_file, tile_offset) != FR_OK ||
      f_read(pak_file, tile_buf, TILE_BYTES, &br) != FR_OK ||
      br != TILE_BYTES)
  {
    memset(tile_buf, 0, TILE_BYTES);
    return false;
  }

  s_sd_tiles_loaded++;
  return true;
}

/*----------------------------------------------------------------------------
 * 5. Non-Blocking Tile Cache Lookup (Queues Async Request on Miss)
 *----------------------------------------------------------------------------*/
static const TileCacheSlot *Request_Or_Get_Tile_Async(int32_t tx, int32_t ty)
{
  if (!s_pak_ready)
  {
    return NULL;
  }

  s_cache_clock++;

  /* 1. Check if tile (tx, ty) is already in the cache (READY or LOADING) */
  for (int i = 0; i < TILE_CACHE_SLOTS; i++)
  {
    if (s_tile_cache[i].state != TILE_SLOT_EMPTY &&
        s_tile_cache[i].tx == tx &&
        s_tile_cache[i].ty == ty)
    {
      s_tile_cache[i].lru_tick = s_cache_clock;
      return &s_tile_cache[i];
    }
  }

  /* 2. Cache miss: pick an EMPTY slot or the oldest READY slot (never evict LOADING) */
  int victim = -1;
  uint32_t min_tick = 0xFFFFFFFFUL;

  for (int i = 0; i < TILE_CACHE_SLOTS; i++)
  {
    if (s_tile_cache[i].state == TILE_SLOT_EMPTY)
    {
      victim = i;
      break;
    }
    if (s_tile_cache[i].state == TILE_SLOT_READY && s_tile_cache[i].lru_tick < min_tick)
    {
      min_tick = s_tile_cache[i].lru_tick;
      victim   = i;
    }
  }

  if (victim < 0)
  {
    return NULL; /* All 9 slots are currently in-flight */
  }

  /* 3. Reserve slot as LOADING and enqueue async read request for Tile_Loader_Task */
  TileCacheSlot *slot = &s_tile_cache[victim];
  slot->tx       = tx;
  slot->ty       = ty;
  slot->lru_tick = s_cache_clock;
  slot->has_data = false;
  slot->state    = TILE_SLOT_LOADING;

  TileRequest_t req = {
    .slot_idx = (uint8_t)victim,
    .tx       = tx,
    .ty       = ty,
  };

  if (xQueueSend(s_q_tile_req, &req, 0) != pdPASS)
  {
    slot->state = TILE_SLOT_EMPTY;
    return NULL;
  }

  return slot;
}

/*----------------------------------------------------------------------------
 * 6. Render Visible Tiles (Non-Blocking) + GPS Dot into s_map_fb
 *----------------------------------------------------------------------------*/
static void Draw_GPS_Marker(int32_t cx, int32_t cy)
{
  const int32_t r_outer    = 7;
  const int32_t r_inner    = 5;
  const uint16_t col_white = 0xFFFFU;
  const uint16_t col_red   = 0xF800U;

  for (int32_t dy = -r_outer; dy <= r_outer; dy++)
  {
    int32_t py = cy + dy;
    if (py < 0 || py >= MY_DISP_VER_RES) continue;

    for (int32_t dx = -r_outer; dx <= r_outer; dx++)
    {
      int32_t px = cx + dx;
      if (px < 0 || px >= MY_DISP_HOR_RES) continue;

      int32_t d2 = dx * dx + dy * dy;
      if (d2 <= r_inner * r_inner)
      {
        s_map_fb[py * MY_DISP_HOR_RES + px] = col_red;
      }
      else if (d2 <= r_outer * r_outer)
      {
        s_map_fb[py * MY_DISP_HOR_RES + px] = col_white;
      }
    }
  }
}

static uint32_t Render_Visible_Tiles_To_MapFB(void)
{
  memset(s_map_fb, 0xDF, sizeof(s_map_fb));

  uint32_t ready_count = 0;

  for (int32_t i = 0; i < s_visible_tiles.count; i++)
  {
    const VisibleTile *vt     = &s_visible_tiles.tiles[i];
    const TileCacheSlot *slot = Request_Or_Get_Tile_Async(vt->tile.x, vt->tile.y);

    int32_t dst_x0 = vt->screen_x;
    int32_t dst_y0 = vt->screen_y;
    int32_t dst_x1 = dst_x0 + TILE_PX_SIZE;
    int32_t dst_y1 = dst_y0 + TILE_PX_SIZE;

    int32_t clip_x0 = (dst_x0 < 0) ? 0 : dst_x0;
    int32_t clip_y0 = (dst_y0 < 0) ? 0 : dst_y0;
    int32_t clip_x1 = (dst_x1 > MY_DISP_HOR_RES) ? MY_DISP_HOR_RES : dst_x1;
    int32_t clip_y1 = (dst_y1 > MY_DISP_VER_RES) ? MY_DISP_VER_RES : dst_y1;

    if (slot != NULL && slot->state == TILE_SLOT_READY)
    {
      if (slot->has_data)
      {
        ready_count++;
      }

      for (int32_t sy = clip_y0; sy < clip_y1; sy++)
      {
        int32_t ty             = sy - dst_y0;
        const uint8_t *src_row = &slot->pixels[ty * TILE_PX_SIZE];
        uint16_t *dst_row      = &s_map_fb[sy * MY_DISP_HOR_RES];

        for (int32_t sx = clip_x0; sx < clip_x1; sx++)
        {
          int32_t tx = sx - dst_x0;
          dst_row[sx] = s_palette_rgb565[src_row[tx]];
        }
      }
    }
    else
    {
      /* Tile is streaming in the background -> draw subtle grid border */
      for (int32_t sy = clip_y0; sy < clip_y1; sy++)
      {
        int32_t ty        = sy - dst_y0;
        uint16_t *dst_row = &s_map_fb[sy * MY_DISP_HOR_RES];
        for (int32_t sx = clip_x0; sx < clip_x1; sx++)
        {
          int32_t tx = sx - dst_x0;
          if ((tx & 31) == 0 || (ty & 31) == 0)
          {
            dst_row[sx] = 0xCE59U;
          }
        }
      }
    }
  }

  /* Project GPS position onto the camera viewport */
  int32_t gps_sx = 0, gps_sy = 0;
  map_camera_latlon_to_screen(&s_camera, s_gps_pos, &gps_sx, &gps_sy);
  Draw_GPS_Marker(gps_sx, gps_sy);

  return ready_count;
}

/*============================================================================
 * TASK 1: Sensor_Input_Task (Priority: AboveNormal)
 * Polls Touchscreen (and future Accelerometer) at ~66 Hz and sends deltas.
 *============================================================================*/
static void Sensor_Input_Task(void *argument)
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
          xQueueSend(s_q_input_evt, &evt, 0);
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

/*============================================================================
 * TASK 2: GPS_UART_Task (Priority: AboveNormal)
 * Receives UART NMEA/UBX frames and publishes the latest GpsFix_t.
 *============================================================================*/
static void GPS_UART_Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    /* Wait for UART DMA / Idle interrupt notification when GPS hardware is connected */
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/*============================================================================
 * TASK 3: Tile_Loader_Task (Priority: BelowNormal)
 * Owns microSD .pak streaming under s_mutex_sd so UI never blocks on SD I/O.
 *============================================================================*/
static void Tile_Loader_Task(void *argument)
{
  (void)argument;

  /* 1. Mount SD & open tiles_z14.pak once (retrying every 1s if card not inserted) */
  while (!s_pak_ready)
  {
    xSemaphoreTake(s_mutex_sd, portMAX_DELAY);
    FRESULT res = f_mount(&s_fatfs, "", 1);
    if (res == FR_OK)
    {
      if (Init_Tile_Pak_Locked(PATH_TILES_Z14, &s_pak_z14, &s_hdr_z14,
                               s_palette_rgb565, s_clmt_z14, 64U))
      {
        s_pak_ready = true;
      }
    }
    xSemaphoreGive(s_mutex_sd);

    if (s_pak_ready)
    {
      xTaskNotify(s_ui_task_hdl, NOTIFY_SD_MOUNTED, eSetBits);
    }
    else
    {
      xTaskNotify(s_ui_task_hdl, NOTIFY_SD_ERROR, eSetBits);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  /* 2. Stream requested 64 KB tiles on demand from s_q_tile_req */
  TileRequest_t req;
  for (;;)
  {
    if (xQueueReceive(s_q_tile_req, &req, portMAX_DELAY) == pdPASS)
    {
      if (req.slot_idx < TILE_CACHE_SLOTS)
      {
        TileCacheSlot *slot = &s_tile_cache[req.slot_idx];

        xSemaphoreTake(s_mutex_sd, portMAX_DELAY);
        bool ok = Load_Tile_From_Pak_Locked(&s_pak_z14, &s_hdr_z14,
                                            req.tx, req.ty, slot->pixels);
        xSemaphoreGive(s_mutex_sd);

        slot->has_data = ok;
        slot->state    = TILE_SLOT_READY;

        /* Notify UI_LVGL_Task that a new tile is ready in SDRAM */
        xTaskNotify(s_ui_task_hdl, NOTIFY_TILE_LOADED, eSetBits);
      }
    }
  }
}

/*============================================================================
 * TASK 4: Pathfinder_Task (Priority: Low)
 * Runs Hybrid A* route computation in the background on request.
 *============================================================================*/
static void Pathfinder_Task(void *argument)
{
  (void)argument;
  RouteRequest_t req;

  for (;;)
  {
    if (xQueueReceive(s_q_route_req, &req, portMAX_DELAY) == pdPASS)
    {
      xSemaphoreTake(s_mutex_sd, portMAX_DELAY);
      pathfinder_find_route(&s_pathfinder, req.start, req.goal, &s_current_route);
      if (s_current_route.found)
      {
        pathfinder_load_geometry(&s_pathfinder, &s_current_route);
      }
      xSemaphoreGive(s_mutex_sd);

      xTaskNotify(s_ui_task_hdl, NOTIFY_ROUTE_READY, eSetBits);
    }
  }
}

/*============================================================================
 * TASK 5: UI_LVGL_Task (Priority: Normal)
 * Sole owner of LVGL, s_camera, and Display Flush. Never blocks on SD card!
 *============================================================================*/
static void UI_LVGL_Task(void *argument)
{
  (void)argument;

  memset(s_map_fb, 0x21, sizeof(s_map_fb));

  lv_obj_t * scr = lv_screen_active();

  /* Full-screen LVGL image widget displaying s_map_fb */
  lv_obj_t * map_img = lv_image_create(scr);
  lv_image_set_src(map_img, &s_map_img_dsc);
  lv_obj_align(map_img, LV_ALIGN_TOP_LEFT, 0, 0);

  /* Compact top HUD overlay bar */
  lv_obj_t * hud_bar = lv_obj_create(scr);
  lv_obj_set_size(hud_bar, 232, 28);
  lv_obj_align(hud_bar, LV_ALIGN_TOP_MID, 0, 4);
  lv_obj_set_style_pad_all(hud_bar, 2, 0);
  lv_obj_set_style_bg_opa(hud_bar, LV_OPA_80, 0);

  lv_obj_t * status_lbl = lv_label_create(hud_bar);
  lv_label_set_text(status_lbl, "RTOS: Mounting SD...");
  lv_obj_center(status_lbl);

  for (;;)
  {
    bool need_redraw = false;

    /* 1. Drain all pending touch/sensor drag events and accumulate delta */
    InputEvent_t evt;
    int32_t sum_dx = 0;
    int32_t sum_dy = 0;
    while (xQueueReceive(s_q_input_evt, &evt, 0) == pdPASS)
    {
      sum_dx += evt.dx;
      sum_dy += evt.dy;
    }

    if (sum_dx != 0 || sum_dy != 0)
    {
      map_camera_pan_pixels(&s_camera, -sum_dx, -sum_dy);
      map_camera_get_visible_tiles(&s_camera, &s_visible_tiles);
      need_redraw = true;
    }

    /* 2. Check for new GPS fix from GPS_UART_Task */
    GpsFix_t fix;
    if (xQueueReceive(s_q_gps_fix, &fix, 0) == pdPASS && fix.valid)
    {
      s_gps_pos = fix.pos;
      need_redraw = true;
    }

    /* 3. Check async notifications from Tile_Loader_Task & Pathfinder_Task */
    uint32_t notify_bits = 0;
    if (xTaskNotifyWait(0, 0xFFFFFFFFUL, &notify_bits, 0) == pdPASS)
    {
      if (notify_bits & NOTIFY_SD_ERROR)
      {
        lv_label_set_text(status_lbl, "SD Err (Retrying...)");
      }
      if (notify_bits & (NOTIFY_SD_MOUNTED | NOTIFY_TILE_LOADED | NOTIFY_ROUTE_READY))
      {
        need_redraw = true;
      }
    }

    /* 4. Re-blit visible tiles from SDRAM cache if camera moved or tile arrived */
    if (need_redraw && s_pak_ready)
    {
      uint32_t n_ready = Render_Visible_Tiles_To_MapFB();
      lv_obj_invalidate(map_img);
      lv_label_set_text_fmt(status_lbl, "z%ld | %lu/%ld rdy | SD:%lu",
                            (long)s_camera.zoom,
                            (unsigned long)n_ready,
                            (long)s_visible_tiles.count,
                            (unsigned long)s_sd_tiles_loaded);
    }

    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/*============================================================================
 * Entry Point Called from main(): Init Hardware, Create RTOS Objects & Start
 *============================================================================*/
void LVGL_Example_Run(void)
{
  LVGL_BSP_Init();
  GPS_Library_Init();

  /* Create RTOS Queues & SD SPI Mutex */
  s_q_input_evt = xQueueCreate(16, sizeof(InputEvent_t));
  s_q_gps_fix   = xQueueCreate(1,  sizeof(GpsFix_t));
  s_q_tile_req  = xQueueCreate(TILE_CACHE_SLOTS, sizeof(TileRequest_t));
  s_q_route_req = xQueueCreate(1,  sizeof(RouteRequest_t));
  s_mutex_sd    = xSemaphoreCreateMutex();

  /* Create the 5 RTOS Tasks */
  xTaskCreate(UI_LVGL_Task,      "UI_LVGL",     1536, NULL, PRIO_UI_LVGL,     &s_ui_task_hdl);
  xTaskCreate(Sensor_Input_Task, "SensorInput",  512, NULL, PRIO_SENSOR_GPS,  NULL);
  xTaskCreate(GPS_UART_Task,     "GPS_UART",     512, NULL, PRIO_SENSOR_GPS,  NULL);
  xTaskCreate(Tile_Loader_Task,  "TileLoader",  1024, NULL, PRIO_TILE_LOADER, NULL);
  xTaskCreate(Pathfinder_Task,   "Pathfinder",  1024, NULL, PRIO_PATHFINDER,  NULL);

  /* Start FreeRTOS Scheduler */
  vTaskStartScheduler();

  while (1) {}
}
