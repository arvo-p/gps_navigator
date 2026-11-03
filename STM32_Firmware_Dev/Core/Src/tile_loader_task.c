/**
  ******************************************************************************
  * @file    Core/Src/tile_loader_task.c
  * @brief   TASK 3: Tile_Loader_Task (Priority: BelowNormal)
  *          Owns microSD .pak streaming under g_mutex_sd so UI never blocks
  *          on SD card I/O.
  ******************************************************************************
  */

#include <string.h>
#include "app.h"

/* Tile Archive (.pak) State */
static FATFS         s_fatfs;
static FIL           s_pak_z14;
static DWORD         s_clmt_z14[64];
static TilePakHeader s_hdr_z14;

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

  g_sd_tiles_loaded++;
  return true;
}

void Tile_Loader_Task(void *argument)
{
  (void)argument;

  /* 1. Mount SD & open tiles_z14.pak once (retrying every 1s if card not inserted) */
  while (!g_pak_ready)
  {
    xSemaphoreTake(g_mutex_sd, portMAX_DELAY);
    FRESULT res = f_mount(&s_fatfs, "", 1);
    if (res == FR_OK)
    {
      if (Init_Tile_Pak_Locked(PATH_TILES_Z14, &s_pak_z14, &s_hdr_z14,
                               g_palette_rgb565, s_clmt_z14, 64U))
      {
        Tile_Palette_Hide_Pedestrian_Paths();
        g_pak_ready = true;
      }
    }
    xSemaphoreGive(g_mutex_sd);

    if (g_pak_ready)
    {
      xTaskNotify(g_ui_task_hdl, NOTIFY_SD_MOUNTED, eSetBits);
    }
    else
    {
      xTaskNotify(g_ui_task_hdl, NOTIFY_SD_ERROR, eSetBits);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  /* 2. Stream requested 64 KB tiles on demand from g_q_tile_req */
  TileRequest_t req;
  for (;;)
  {
    if (xQueueReceive(g_q_tile_req, &req, portMAX_DELAY) == pdPASS)
    {
      if (req.slot_idx < TILE_CACHE_SLOTS)
      {
        TileCacheSlot *slot = &g_tile_cache[req.slot_idx];

        xSemaphoreTake(g_mutex_sd, portMAX_DELAY);
        bool ok = Load_Tile_From_Pak_Locked(&s_pak_z14, &s_hdr_z14,
                                            req.tx, req.ty, slot->pixels);
        xSemaphoreGive(g_mutex_sd);

        slot->has_data = ok;
        slot->state    = TILE_SLOT_READY;

        /* Notify UI_LVGL_Task that a new tile is ready in SDRAM */
        xTaskNotify(g_ui_task_hdl, NOTIFY_TILE_LOADED, eSetBits);
      }
    }
  }
}
