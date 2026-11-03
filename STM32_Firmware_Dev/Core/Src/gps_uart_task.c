/**
  ******************************************************************************
  * @file    Core/Src/gps_uart_task.c
  * @brief   TASK 2: GPS_UART_Task (Priority: AboveNormal)
  *          Receives UART NMEA frames via GPS_UART_INSTANCE (configured in main.h)
  *          using GPS_DMA_RX_STREAM circular buffer + IDLE IRQ, sends ATGM336H-5N
  *          CASIC commands, parses UTC time & WGS84 coordinates, and publishes
  *          the latest GpsFix_t to UI_LVGL_Task.
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include "app.h"

#define GPS_DMA_RX_BUF_SIZE   512U
#define GPS_NMEA_MAX_LEN      128U

/* Must reside in internal SRAM1 (not CCMRAM) so GPS_DMA_RX_STREAM can access it */
static uint8_t s_gps_dma_rx_buf[GPS_DMA_RX_BUF_SIZE] __attribute__((aligned(4)));

/*----------------------------------------------------------------------------
 * 1. Interrupt Service Routines (Mapped via main.h #defines)
 *----------------------------------------------------------------------------*/
void GPS_UART_IRQHandler(void)
{
  uint32_t sr = GPS_UART_INSTANCE->SR;
  if (sr & (USART_SR_IDLE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
  {
    /* Reading SR followed by DR clears IDLE, ORE, FE, and NE flags */
    (void)GPS_UART_INSTANCE->DR;

    if ((sr & USART_SR_IDLE) && (g_gps_task_hdl != NULL))
    {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(g_gps_task_hdl, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

void GPS_DMA_RX_IRQHandler(void)
{
  uint32_t isr = GPS_DMA_RX_ISR_REG;
  if (isr & GPS_DMA_RX_ISR_FLAGS)
  {
    GPS_DMA_RX_IFCR_REG = GPS_DMA_RX_CLEAR_FLAGS;

    if (g_gps_task_hdl != NULL)
    {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(g_gps_task_hdl, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

/*----------------------------------------------------------------------------
 * 2. UART + Circular DMA Hardware Initialization
 *----------------------------------------------------------------------------*/
static void GPS_UART_DMA_Init(void)
{
  /* 1. Enable peripheral clocks for GPIO TX/RX, UART, and DMA */
  GPS_UART_TX_GPIO_CLK_ENABLE();
  GPS_UART_RX_GPIO_CLK_ENABLE();
  GPS_UART_CLK_ENABLE();
  GPS_DMA_CLK_ENABLE();

  /* 2. Configure TX pin (MCU TX -> GPS RX) and RX pin (MCU RX <- GPS TX) */
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FAST;
  gpio.Alternate = GPS_UART_AF;

  gpio.Pin = GPS_UART_TX_PIN;
  HAL_GPIO_Init(GPS_UART_TX_GPIO_PORT, &gpio);

  gpio.Pin = GPS_UART_RX_PIN;
  HAL_GPIO_Init(GPS_UART_RX_GPIO_PORT, &gpio);

  /* 3. Configure GPS_DMA_RX_STREAM in Circular Mode */
  GPS_DMA_RX_STREAM->CR &= ~DMA_SxCR_EN;
  while (GPS_DMA_RX_STREAM->CR & DMA_SxCR_EN) {}

  GPS_DMA_RX_IFCR_REG = GPS_DMA_RX_CLEAR_FLAGS;

  GPS_DMA_RX_STREAM->PAR  = (uint32_t)&GPS_UART_INSTANCE->DR;
  GPS_DMA_RX_STREAM->M0AR = (uint32_t)s_gps_dma_rx_buf;
  GPS_DMA_RX_STREAM->NDTR = GPS_DMA_RX_BUF_SIZE;
  GPS_DMA_RX_STREAM->FCR  = 0U;
  GPS_DMA_RX_STREAM->CR   = (GPS_DMA_RX_CHANNEL << DMA_SxCR_CHSEL_Pos) |
                            DMA_SxCR_PL_1                              | /* High priority */
                            DMA_SxCR_MINC                              | /* Increment memory address */
                            DMA_SxCR_CIRC                              | /* Circular buffer mode */
                            DMA_SxCR_HTIE                              | /* Half-transfer interrupt */
                            DMA_SxCR_TCIE;                               /* Transfer-complete interrupt */
  GPS_DMA_RX_STREAM->CR  |= DMA_SxCR_EN;

  /* 4. Configure GPS_UART_INSTANCE: 8N1, DMA Receiver + IDLE Line Interrupt */
  GPS_UART_INSTANCE->CR1 = 0U;
  GPS_UART_INSTANCE->CR2 = 0U;
  GPS_UART_INSTANCE->CR3 = USART_CR3_DMAR;

  uint32_t pclk = GPS_UART_GET_PCLK_FREQ();
  GPS_UART_INSTANCE->BRR = (pclk + (GPS_UART_BAUDRATE / 2U)) / GPS_UART_BAUDRATE;

  (void)GPS_UART_INSTANCE->SR;
  (void)GPS_UART_INSTANCE->DR;

  GPS_UART_INSTANCE->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_UE;

  /* 5. Enable NVIC IRQs at FreeRTOS-safe priority (>= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) */
  HAL_NVIC_SetPriority(GPS_DMA_RX_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(GPS_DMA_RX_IRQn);
  HAL_NVIC_SetPriority(GPS_UART_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(GPS_UART_IRQn);
}

/*----------------------------------------------------------------------------
 * 3. ATGM336H-5N (CASIC Protocol) Command Helpers via GPS_UART_INSTANCE TX
 *----------------------------------------------------------------------------*/
static void GPS_UART_SendBytes(const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0U; i < len; i++)
  {
    while (!(GPS_UART_INSTANCE->SR & USART_SR_TXE)) {}
    GPS_UART_INSTANCE->DR = data[i];
  }
  while (!(GPS_UART_INSTANCE->SR & USART_SR_TC)) {}
}

/* Formats "$<payload>*<XOR_HEX>\r\n" and sends over GPS_UART_INSTANCE TX */
static void GPS_SendCasicCmd(const char *payload)
{
  static const char hex_lut[16] = "0123456789ABCDEF";
  char frame[64];
  uint8_t crc = 0U;
  uint32_t idx = 0U;

  frame[idx++] = '$';
  while (*payload != '\0' && idx < (sizeof(frame) - 6U))
  {
    uint8_t ch = (uint8_t)(*payload++);
    crc ^= ch;
    frame[idx++] = (char)ch;
  }

  frame[idx++] = '*';
  frame[idx++] = hex_lut[(crc >> 4) & 0x0FU];
  frame[idx++] = hex_lut[crc & 0x0FU];
  frame[idx++] = '\r';
  frame[idx++] = '\n';

  GPS_UART_SendBytes((const uint8_t *)frame, idx);
}

/**
  * @brief  Select active GNSS constellations on ATGM336H-5N ($PCAS04).
  * @param  mode: ATGM336H_MODE_GPS_GLONASS (5 -> "$PCAS04,5*1C\r\n"), etc.
  */
void GPS_ATGM336H_SetConstellation(uint8_t mode)
{
  char cmd[16];
  (void)snprintf(cmd, sizeof(cmd), "PCAS04,%u", (unsigned)mode);
  GPS_SendCasicCmd(cmd);
}

/**
  * @brief  Put ATGM336H-5N into low-power standby/sleep mode ($PCAS12).
  * @param  standby_sec: Standby duration in seconds (1..65535).
  *                      Pass 0 to use maximum duration (65535 s).
  */
void GPS_ATGM336H_EnterSleep(uint16_t standby_sec)
{
  uint16_t sec = (standby_sec == 0U) ? 65535U : standby_sec;
  char cmd[24];
  (void)snprintf(cmd, sizeof(cmd), "PCAS12,%u", (unsigned)sec);
  GPS_SendCasicCmd(cmd);
}

/**
  * @brief  Wake ATGM336H-5N from standby/sleep mode ($PCAS10,0 hot start)
  *         and restore GPS + GLONASS constellation mode ($PCAS04,5).
  */
void GPS_ATGM336H_Wakeup(void)
{
  const uint8_t wake_pulse = 0xFFU;
  GPS_UART_SendBytes(&wake_pulse, 1U);
  vTaskDelay(pdMS_TO_TICKS(20));

  /* Hot start ($PCAS10,0*1C) and re-select GPS + GLONASS ($PCAS04,5*1C) */
  GPS_SendCasicCmd("PCAS10,0");
  vTaskDelay(pdMS_TO_TICKS(50));
  GPS_ATGM336H_SetConstellation(ATGM336H_MODE_GPS_GLONASS);
}

/*----------------------------------------------------------------------------
 * 4. Pure Fixed-Point NMEA 0183 Parser ($xxRMC, $xxGGA, $xxGSV)
 *----------------------------------------------------------------------------*/
static int32_t GPS_HexCharToNibble(char c)
{
  if (c >= '0' && c <= '9') return (int32_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (int32_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (int32_t)(c - 'a' + 10);
  return -1;
}

static bool GPS_VerifyNmeaChecksum(const char *line)
{
  if (line[0] != '$') return false;

  uint8_t calc = 0U;
  const char *p = line + 1;
  while (*p != '\0' && *p != '*' && *p != '\r' && *p != '\n')
  {
    calc ^= (uint8_t)(*p);
    p++;
  }

  if (*p != '*') return false;
  int32_t hi = GPS_HexCharToNibble(p[1]);
  int32_t lo = GPS_HexCharToNibble(p[2]);
  if (hi < 0 || lo < 0) return false;

  return (calc == (uint8_t)((hi << 4) | lo));
}

static const char *GPS_NextField(const char *p)
{
  while (*p != '\0' && *p != ',' && *p != '*')
  {
    p++;
  }
  return (*p == ',') ? (p + 1) : p;
}

static bool GPS_ParseUtcTime(const char *field, uint8_t *out_h, uint8_t *out_m, uint8_t *out_s)
{
  for (int i = 0; i < 6; i++)
  {
    if (field[i] < '0' || field[i] > '9') return false;
  }

  uint8_t h = (uint8_t)((field[0] - '0') * 10 + (field[1] - '0'));
  uint8_t m = (uint8_t)((field[2] - '0') * 10 + (field[3] - '0'));
  uint8_t s = (uint8_t)((field[4] - '0') * 10 + (field[5] - '0'));

  if (h > 23U || m > 59U || s > 60U) return false;

  *out_h = h;
  *out_m = m;
  *out_s = s;
  return true;
}

static bool GPS_ParseUtcDate(const char *field, uint8_t *out_d, uint8_t *out_mo, uint16_t *out_y)
{
  for (int i = 0; i < 6; i++)
  {
    if (field[i] < '0' || field[i] > '9') return false;
  }

  uint8_t  d  = (uint8_t)((field[0] - '0') * 10 + (field[1] - '0'));
  uint8_t  mo = (uint8_t)((field[2] - '0') * 10 + (field[3] - '0'));
  uint16_t y  = (uint16_t)(2000U + (uint16_t)((field[4] - '0') * 10 + (field[5] - '0')));

  if (d < 1U || d > 31U || mo < 1U || mo > 12U) return false;

  *out_d  = d;
  *out_mo = mo;
  *out_y  = y;
  return true;
}

/* Pure 64-bit integer NMEA coordinate parser ("ddmm.mmmm" / "dddmm.mmmm") -> 10^-7 degrees */
static bool GPS_ParseNmeaCoordE7(const char *field, char hemi, int32_t *out_e7)
{
  const char *dot = field;
  int32_t int_digits = 0;

  while (dot[int_digits] >= '0' && dot[int_digits] <= '9')
  {
    int_digits++;
  }

  if (int_digits < 4 || int_digits > 5 || dot[int_digits] != '.')
  {
    return false;
  }

  int32_t deg_digits = int_digits - 2;
  int32_t deg = 0;
  for (int32_t i = 0; i < deg_digits; i++)
  {
    deg = deg * 10 + (field[i] - '0');
  }

  int32_t min_int = (field[deg_digits] - '0') * 10 + (field[deg_digits + 1] - '0');
  if (min_int >= 60) return false;

  const char *frac = &field[int_digits + 1];
  int64_t min_frac_e7 = 0;
  int64_t scale       = 1000000LL;
  while (*frac >= '0' && *frac <= '9')
  {
    if (scale > 0)
    {
      min_frac_e7 += (int64_t)(*frac - '0') * scale;
      scale /= 10LL;
    }
    frac++;
  }

  int64_t total_min_e7 = (int64_t)min_int * 10000000LL + min_frac_e7;
  int64_t coord_e7     = (int64_t)deg * 10000000LL + (total_min_e7 + 30LL) / 60LL;

  if (hemi == 'S' || hemi == 'W')
  {
    coord_e7 = -coord_e7;
  }
  else if (hemi != 'N' && hemi != 'E')
  {
    return false;
  }

  *out_e7 = (int32_t)coord_e7;
  return true;
}

static uint32_t GPS_ParseDecimalX100(const char *field)
{
  uint32_t int_part = 0U;
  while (*field >= '0' && *field <= '9')
  {
    int_part = int_part * 10U + (uint32_t)(*field - '0');
    field++;
  }

  uint32_t frac_part = 0U;
  if (*field == '.')
  {
    field++;
    if (*field >= '0' && *field <= '9')
    {
      frac_part += (uint32_t)(*field - '0') * 10U;
      field++;
      if (*field >= '0' && *field <= '9')
      {
        frac_part += (uint32_t)(*field - '0');
      }
    }
  }

  return int_part * 100U + frac_part;
}

static bool GPS_ParseNmeaSentence(const char *line, GpsFix_t *fix)
{
  if (!GPS_VerifyNmeaChecksum(line))
  {
    return false;
  }

  /* Valid checksummed NMEA frame received -> UART link is active */
  fix->uart_ok = true;

  /* Match $GPRMC / $GNRMC / $GLRMC / $GARMC:
   * $xxRMC,hhmmss.ss,A,llll.ll,N,yyyyy.yy,E,spd_kn,cog_deg,ddmmyy,...*hh */
  if (strncmp(line + 3, "RMC,", 4) == 0)
  {
    const char *f_time   = line + 7;
    const char *f_status = GPS_NextField(f_time);
    const char *f_lat    = GPS_NextField(f_status);
    const char *f_ns     = GPS_NextField(f_lat);
    const char *f_lon    = GPS_NextField(f_ns);
    const char *f_ew     = GPS_NextField(f_lon);
    const char *f_spd    = GPS_NextField(f_ew);
    const char *f_cog    = GPS_NextField(f_spd);
    const char *f_date   = GPS_NextField(f_cog);

    if (GPS_ParseUtcTime(f_time, &fix->utc_hour, &fix->utc_min, &fix->utc_sec))
    {
      fix->time_valid = true;
    }

    (void)GPS_ParseUtcDate(f_date, &fix->utc_day, &fix->utc_month, &fix->utc_year);

    if (*f_status == 'A')
    {
      int32_t lat_e7 = 0;
      int32_t lon_e7 = 0;
      if (GPS_ParseNmeaCoordE7(f_lat, *f_ns, &lat_e7) &&
          GPS_ParseNmeaCoordE7(f_lon, *f_ew, &lon_e7))
      {
        fix->pos         = geo_coord_make(lat_e7, lon_e7);
        uint32_t kn_x100 = GPS_ParseDecimalX100(f_spd);
        fix->speed_kmh   = (uint16_t)((kn_x100 * 1852U + 50000U) / 100000U);
        fix->heading_deg = (uint16_t)((GPS_ParseDecimalX100(f_cog) + 50U) / 100U) % 360U;
        fix->valid       = true;
      }
    }
    else if (*f_status == 'V')
    {
      fix->valid = false;
    }

    return true;
  }

  /* Match $GPGGA / $GNGGA:
   * $xxGGA,hhmmss.ss,llll.ll,N,yyyyy.yy,E,qual,sats,hdop,alt,M,...*hh */
  if (strncmp(line + 3, "GGA,", 4) == 0)
  {
    const char *f_time = line + 7;
    const char *f_lat  = GPS_NextField(f_time);
    const char *f_ns   = GPS_NextField(f_lat);
    const char *f_lon  = GPS_NextField(f_ns);
    const char *f_ew   = GPS_NextField(f_lon);
    const char *f_qual = GPS_NextField(f_ew);
    const char *f_sats = GPS_NextField(f_qual);

    if (GPS_ParseUtcTime(f_time, &fix->utc_hour, &fix->utc_min, &fix->utc_sec))
    {
      fix->time_valid = true;
    }

    uint8_t sats = 0U;
    while (*f_sats >= '0' && *f_sats <= '9')
    {
      sats = (uint8_t)(sats * 10U + (uint8_t)(*f_sats - '0'));
      f_sats++;
    }
    fix->satellites = sats;

    if (*f_qual >= '1' && *f_qual <= '9')
    {
      int32_t lat_e7 = 0;
      int32_t lon_e7 = 0;
      if (GPS_ParseNmeaCoordE7(f_lat, *f_ns, &lat_e7) &&
          GPS_ParseNmeaCoordE7(f_lon, *f_ew, &lon_e7))
      {
        fix->pos   = geo_coord_make(lat_e7, lon_e7);
        fix->valid = true;
      }
    }
    else if (*f_qual == '0')
    {
      fix->valid = false;
    }

    return true;
  }

  /* Match $GPGSV / $GLGSV / $BDGSV (Satellites in View):
   * $xxGSV,num_msgs,msg_num,sats_in_view,...*hh */
  if (strncmp(line + 3, "GSV,", 4) == 0)
  {
    const char *f_num_msgs = line + 7;
    const char *f_msg_num  = GPS_NextField(f_num_msgs);
    const char *f_sats_vis = GPS_NextField(f_msg_num);

    uint8_t vis = 0U;
    while (*f_sats_vis >= '0' && *f_sats_vis <= '9')
    {
      vis = (uint8_t)(vis * 10U + (uint8_t)(*f_sats_vis - '0'));
      f_sats_vis++;
    }

    /* Keep Separate GPS ($GP) vs GLONASS ($GL) visible counts on message #1 */
    static uint8_t s_gp_vis = 0U;
    static uint8_t s_gl_vis = 0U;
    if (*f_msg_num == '1')
    {
      if (line[1] == 'G' && line[2] == 'P')
      {
        s_gp_vis = vis;
      }
      else if (line[1] == 'G' && line[2] == 'L')
      {
        s_gl_vis = vis;
      }
      fix->sats_in_view = (uint8_t)(s_gp_vis + s_gl_vis);
    }
    return true;
  }

  return false;
}

/*============================================================================
 * 5. GPS_UART_Task Entry Point
 *============================================================================*/
void GPS_UART_Task(void *argument)
{
  (void)argument;

  g_gps_task_hdl = xTaskGetCurrentTaskHandle();
  GPS_UART_DMA_Init();

  /* Allow ATGM336H-5N power-on boot, then select GPS + GLONASS ($PCAS04,5*1C) */
  vTaskDelay(pdMS_TO_TICKS(100));
  GPS_ATGM336H_SetConstellation(ATGM336H_MODE_GPS_GLONASS);

  uint32_t   rd_idx         = 0U;
  uint32_t   line_len       = 0U;
  char       line_buf[GPS_NMEA_MAX_LEN];
  GpsFix_t   cur_fix        = {0};
  TickType_t last_nmea_tick = xTaskGetTickCount();

  for (;;)
  {
    /* Wait for UART DMA / Idle interrupt notification when GPS hardware is connected */
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

    uint32_t wr_idx = (GPS_DMA_RX_BUF_SIZE - GPS_DMA_RX_STREAM->NDTR) % GPS_DMA_RX_BUF_SIZE;

    while (rd_idx != wr_idx)
    {
      char c = (char)s_gps_dma_rx_buf[rd_idx];
      rd_idx = (rd_idx + 1U) % GPS_DMA_RX_BUF_SIZE;

      if (c == '$')
      {
        line_len = 0U;
        line_buf[line_len++] = c;
      }
      else if (line_len > 0U)
      {
        if (c == '\r' || c == '\n')
        {
          line_buf[line_len] = '\0';
          if (GPS_ParseNmeaSentence(line_buf, &cur_fix))
          {
            last_nmea_tick = xTaskGetTickCount();
            xQueueOverwrite(g_q_gps_fix, &cur_fix);
          }
          line_len = 0U;
        }
        else if (line_len < (GPS_NMEA_MAX_LEN - 1U))
        {
          line_buf[line_len++] = c;
        }
        else
        {
          /* Sentence overflow -> reset until next '$' */
          line_len = 0U;
        }
      }
    }

    /* If no valid NMEA sentence arrived for > 2.5s, mark UART as disconnected */
    if (cur_fix.uart_ok && ((xTaskGetTickCount() - last_nmea_tick) > pdMS_TO_TICKS(2500)))
    {
      cur_fix.uart_ok = false;
      cur_fix.valid   = false;
      xQueueOverwrite(g_q_gps_fix, &cur_fix);
    }
  }
}
