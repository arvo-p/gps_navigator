/**
  ******************************************************************************
  * @file    Core/Src/ui_lvgl_task.c
  * @brief   TASK 5: UI_LVGL_Task (Priority: Normal)
  *          Sole owner of LVGL, g_camera, non-blocking SDRAM tile cache lookup,
  *          map framebuffer rendering, and optional debug HUD overlay
  *          (controlled by APP_DEBUG_ENABLE in Core/Inc/main.h).
  ******************************************************************************
  */

#include <string.h>
#include "app.h"

/* Full-screen RGB565 Map Framebuffer in External SDRAM (.sdram) */
static uint16_t s_map_fb[MY_DISP_HOR_RES * MY_DISP_VER_RES] __attribute__((section(".sdram"), aligned(4)));

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
 * 1. Non-Blocking Tile Cache Lookup (Queues Async Request on Miss)
 *----------------------------------------------------------------------------*/
static const TileCacheSlot *Request_Or_Get_Tile_Async(int32_t tx, int32_t ty)
{
  if (!g_pak_ready)
  {
    return NULL;
  }

  g_cache_clock++;

  /* 1. Check if tile (tx, ty) is already in the cache (READY or LOADING) */
  for (int i = 0; i < TILE_CACHE_SLOTS; i++)
  {
    if (g_tile_cache[i].state != TILE_SLOT_EMPTY &&
        g_tile_cache[i].tx == tx &&
        g_tile_cache[i].ty == ty)
    {
      g_tile_cache[i].lru_tick = g_cache_clock;
      return &g_tile_cache[i];
    }
  }

  /* 2. Cache miss: pick an EMPTY slot or the oldest READY slot (never evict LOADING) */
  int victim = -1;
  uint32_t min_tick = 0xFFFFFFFFUL;

  for (int i = 0; i < TILE_CACHE_SLOTS; i++)
  {
    if (g_tile_cache[i].state == TILE_SLOT_EMPTY)
    {
      victim = i;
      break;
    }
    if (g_tile_cache[i].state == TILE_SLOT_READY && g_tile_cache[i].lru_tick < min_tick)
    {
      min_tick = g_tile_cache[i].lru_tick;
      victim   = i;
    }
  }

  if (victim < 0)
  {
    return NULL; /* All 9 slots are currently in-flight */
  }

  /* 3. Reserve slot as LOADING and enqueue async read request for Tile_Loader_Task */
  TileCacheSlot *slot = &g_tile_cache[victim];
  slot->tx       = tx;
  slot->ty       = ty;
  slot->lru_tick = g_cache_clock;
  slot->has_data = false;
  slot->state    = TILE_SLOT_LOADING;

  TileRequest_t req = {
    .slot_idx = (uint8_t)victim,
    .tx       = tx,
    .ty       = ty,
  };

  if (xQueueSend(g_q_tile_req, &req, 0) != pdPASS)
  {
    slot->state = TILE_SLOT_EMPTY;
    return NULL;
  }

  return slot;
}

/*----------------------------------------------------------------------------
 * 2. Render Visible Tiles (Non-Blocking) + GPS Dot into s_map_fb
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

  for (int32_t i = 0; i < g_visible_tiles.count; i++)
  {
    const VisibleTile *vt     = &g_visible_tiles.tiles[i];
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
          dst_row[sx] = g_palette_rgb565[src_row[tx]];
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
  map_camera_latlon_to_screen(&g_camera, g_gps_pos, &gps_sx, &gps_sy);
  Draw_GPS_Marker(gps_sx, gps_sy);

  return ready_count;
}

/*============================================================================
 * 3. UI_LVGL_Task Entry Point
 *============================================================================*/
void UI_LVGL_Task(void *argument)
{
  (void)argument;

  memset(s_map_fb, 0x21, sizeof(s_map_fb));

  lv_obj_t * scr = lv_screen_active();

  /* Full-screen LVGL image widget displaying s_map_fb */
  lv_obj_t * map_img = lv_image_create(scr);
  lv_image_set_src(map_img, &s_map_img_dsc);
  lv_obj_align(map_img, LV_ALIGN_TOP_LEFT, 0, 0);

#if APP_DEBUG_ENABLE
  /* Compact top debug HUD overlay bar (only created when APP_DEBUG_ENABLE == 1) */
  lv_obj_t * hud_bar = lv_obj_create(scr);
  lv_obj_set_size(hud_bar, 232, 28);
  lv_obj_align(hud_bar, LV_ALIGN_TOP_MID, 0, 4);
  lv_obj_set_style_pad_all(hud_bar, 2, 0);
  lv_obj_set_style_bg_opa(hud_bar, LV_OPA_80, 0);

  lv_obj_t * status_lbl = lv_label_create(hud_bar);
  lv_label_set_text(status_lbl, "RTOS: Mounting SD...");
  lv_obj_center(status_lbl);
#endif

  bool had_first_fix = false;

  for (;;)
  {
    bool need_redraw = false;

    /* 1. Drain all pending touch/sensor drag events and accumulate delta */
    InputEvent_t evt;
    int32_t sum_dx = 0;
    int32_t sum_dy = 0;
    while (xQueueReceive(g_q_input_evt, &evt, 0) == pdPASS)
    {
      sum_dx += evt.dx;
      sum_dy += evt.dy;
    }

    if (sum_dx != 0 || sum_dy != 0)
    {
      map_camera_pan_pixels(&g_camera, -sum_dx, -sum_dy);
      map_camera_get_visible_tiles(&g_camera, &g_visible_tiles);
      need_redraw = true;
    }

    /* 2. Check for new GPS fix / UTC time from GPS_UART_Task */
    GpsFix_t fix;
    if (xQueueReceive(g_q_gps_fix, &fix, 0) == pdPASS)
    {
      g_gps_fix = fix;
      if (fix.valid)
      {
        g_gps_pos = fix.pos;
        if (!had_first_fix)
        {
          had_first_fix = true;
          map_camera_set_center(&g_camera, g_gps_pos);
          map_camera_get_visible_tiles(&g_camera, &g_visible_tiles);
        }
      }
      need_redraw = true;
    }

    /* 3. Check async notifications from Tile_Loader_Task & Pathfinder_Task */
    uint32_t notify_bits = 0;
    if (xTaskNotifyWait(0, 0xFFFFFFFFUL, &notify_bits, 0) == pdPASS)
    {
#if APP_DEBUG_ENABLE
      if (notify_bits & NOTIFY_SD_ERROR)
      {
        lv_label_set_text(status_lbl, "SD Err (Retrying...)");
      }
#endif
      if (notify_bits & (NOTIFY_SD_MOUNTED | NOTIFY_TILE_LOADED | NOTIFY_ROUTE_READY))
      {
        need_redraw = true;
      }
    }

    /* 4. Re-blit visible tiles from SDRAM cache if camera moved or tile arrived */
    if (need_redraw && g_pak_ready)
    {
      (void)Render_Visible_Tiles_To_MapFB();
      lv_obj_invalidate(map_img);

#if APP_DEBUG_ENABLE
      if (!g_gps_fix.uart_ok)
      {
        lv_label_set_text_fmt(status_lbl, "GPS: No UART | z%ld",
                              (long)g_camera.zoom);
      }
      else if (g_gps_fix.time_valid)
      {
        lv_label_set_text_fmt(status_lbl, "%02u:%02u:%02u | %s (%u/%usat)",
                              (unsigned)g_gps_fix.utc_hour,
                              (unsigned)g_gps_fix.utc_min,
                              (unsigned)g_gps_fix.utc_sec,
                              g_gps_fix.valid ? "Fix" : "NoFix",
                              (unsigned)g_gps_fix.satellites,
                              (unsigned)g_gps_fix.sats_in_view);
      }
      else
      {
        lv_label_set_text_fmt(status_lbl, "--:--:-- | %s (%u/%usat)",
                              g_gps_fix.valid ? "Fix" : "NoFix",
                              (unsigned)g_gps_fix.satellites,
                              (unsigned)g_gps_fix.sats_in_view);
      }
#endif
    }

    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
