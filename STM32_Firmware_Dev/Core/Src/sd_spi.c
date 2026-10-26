/**
  ******************************************************************************
  * @file    Core/Src/sd_spi.c
  * @brief   Bare-metal SPI5 microSD driver + FatFs diskio implementation.
  *          Supports SDSC, SDHC, and SDXC cards (FAT16 / FAT32 / exFAT).
  ******************************************************************************
  */

#include "sd_spi.h"
#include "stm32f429i_discovery.h"

/* SD SPI Commands */
#define CMD0    (0)         /* GO_IDLE_STATE */
#define CMD1    (1)         /* SEND_OP_COND (MMC) */
#define ACMD41  (0x80+41)   /* SEND_OP_COND (SDC) */
#define CMD8    (8)         /* SEND_IF_COND */
#define CMD9    (9)         /* SEND_CSD */
#define CMD12   (12)        /* STOP_TRANSMISSION */
#define CMD16   (16)        /* SET_BLOCKLEN */
#define CMD17   (17)        /* READ_SINGLE_BLOCK */
#define CMD18   (18)        /* READ_MULTIPLE_BLOCK */
#define ACMD23  (0x80+23)   /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24   (24)        /* WRITE_BLOCK */
#define CMD25   (25)        /* WRITE_MULTIPLE_BLOCK */
#define CMD55   (55)        /* APP_CMD */
#define CMD58   (58)        /* READ_OCR */

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType = 0;

/*----------------------------------------------------------------------------
 * Low-Level SPI5 Helpers
 *----------------------------------------------------------------------------*/
static inline void SD_CS_Low(void)
{
  /* Keep LCD D/CX (PD13) HIGH so ILI9341 never treats SD SPI bytes as commands */
  LCD_WRX_HIGH();
  HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

static inline void SD_CS_High(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET);
  LCD_WRX_HIGH();
}

static void SD_SPI_SetSlowClock(void)
{
  /* PCLK2 = 90 MHz -> /256 = 351.5 kHz (< 400 kHz required for SD init) */
  CLEAR_BIT(SPI5->CR1, SPI_CR1_SPE);
  MODIFY_REG(SPI5->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_256);
  SET_BIT(SPI5->CR1, SPI_CR1_SPE);
}

static void SD_SPI_SetFastClock(void)
{
  /* PCLK2 = 90 MHz -> /8 = 11.25 MHz */
  CLEAR_BIT(SPI5->CR1, SPI_CR1_SPE);
  MODIFY_REG(SPI5->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_8);
  SET_BIT(SPI5->CR1, SPI_CR1_SPE);
}

static uint8_t SD_SPI_TxRxByte(uint8_t data)
{
  while (!(SPI5->SR & SPI_SR_TXE)) {}
  *(__IO uint8_t *)&SPI5->DR = data;
  while (!(SPI5->SR & SPI_SR_RXNE)) {}
  return *(__IO uint8_t *)&SPI5->DR;
}

static void SD_SPI_InitBus(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* 1. Deselect onboard L3GD20 Gyroscope (PC1) so it never drives MISO (PF8) */
  GYRO_CS_GPIO_CLK_ENABLE();
  gpio.Pin   = GYRO_CS_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &gpio);
  GYRO_CS_HIGH();

  /* 2. Force LCD_WRX (PD13) HIGH so ILI9341 ignores MOSI when PC2 goes LOW */
  LCD_WRX_GPIO_CLK_ENABLE();
  LCD_WRX_HIGH();

  /* 3. Ensure SD_CS (PC2) is configured as push-pull output and starts HIGH */
  SD_CS_GPIO_CLK_ENABLE();
  gpio.Pin   = SD_CS_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(SD_CS_GPIO_PORT, &gpio);
  SD_CS_High();

  /* 4. Enable internal pull-up on SPI5 SCK/MISO/MOSI (PF7, PF8, PF9) */
  DISCOVERY_SPIx_GPIO_CLK_ENABLE();
  gpio.Pin       = DISCOVERY_SPIx_SCK_PIN | DISCOVERY_SPIx_MISO_PIN | DISCOVERY_SPIx_MOSI_PIN;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FAST;
  gpio.Alternate = DISCOVERY_SPIx_AF;
  HAL_GPIO_Init(DISCOVERY_SPIx_GPIO_PORT, &gpio);

  /* Ensure SPI5 peripheral is enabled */
  DISCOVERY_SPIx_CLK_ENABLE();
  SET_BIT(SPI5->CR1, SPI_CR1_SPE);
}

/*----------------------------------------------------------------------------
 * SD Card Protocol Helpers
 *----------------------------------------------------------------------------*/
static int SD_WaitReady(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint8_t d;
  do
  {
    d = SD_SPI_TxRxByte(0xFF);
    if (d == 0xFF)
    {
      return 1;
    }
  } while ((HAL_GetTick() - start) < timeout_ms);

  return 0;
}

static void SD_Deselect(void)
{
  SD_CS_High();
  SD_SPI_TxRxByte(0xFF); /* Provide 8 extra clocks for card to release MISO */
}

static int SD_Select(void)
{
  SD_CS_Low();
  SD_SPI_TxRxByte(0xFF);
  if (SD_WaitReady(500))
  {
    return 1;
  }
  SD_Deselect();
  return 0;
}

static int SD_ReceiveDataBlock(uint8_t *buff, UINT btr)
{
  uint8_t token;
  uint32_t start = HAL_GetTick();

  /* Wait for data packet token (0xFE) */
  do
  {
    token = SD_SPI_TxRxByte(0xFF);
  } while ((token == 0xFF) && ((HAL_GetTick() - start) < 200));

  if (token != 0xFE)
  {
    return 0;
  }

  /* Read data block */
  for (UINT i = 0; i < btr; i++)
  {
    buff[i] = SD_SPI_TxRxByte(0xFF);
  }

  /* Discard 2-byte CRC */
  SD_SPI_TxRxByte(0xFF);
  SD_SPI_TxRxByte(0xFF);

  return 1;
}

#if _USE_WRITE == 1
static int SD_SendDataBlock(const uint8_t *buff, uint8_t token)
{
  if (!SD_WaitReady(500))
  {
    return 0;
  }

  SD_SPI_TxRxByte(token);
  if (token != 0xFD)
  {
    for (UINT i = 0; i < 512; i++)
    {
      SD_SPI_TxRxByte(buff[i]);
    }
    /* Dummy CRC */
    SD_SPI_TxRxByte(0xFF);
    SD_SPI_TxRxByte(0xFF);

    uint8_t resp = SD_SPI_TxRxByte(0xFF);
    if ((resp & 0x1F) != 0x05)
    {
      return 0;
    }
  }
  return 1;
}
#endif

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
  uint8_t n, res;

  /* ACMD<n> is the command sequence of CMD55 + CMD<n> */
  if (cmd & 0x80)
  {
    cmd &= 0x7F;
    res = SD_SendCmd(CMD55, 0);
    if (res > 1)
    {
      return res;
    }
  }

  /* Select the card and wait for ready, except for CMD12/CMD0 */
  if (cmd != CMD12)
  {
    SD_Deselect();
    if (cmd == CMD0)
    {
      SD_CS_Low();
      SD_SPI_TxRxByte(0xFF);
    }
    else if (!SD_Select())
    {
      return 0xFF;
    }
  }

  /* Send 6-byte command packet */
  SD_SPI_TxRxByte(0x40 | cmd);
  SD_SPI_TxRxByte((uint8_t)(arg >> 24));
  SD_SPI_TxRxByte((uint8_t)(arg >> 16));
  SD_SPI_TxRxByte((uint8_t)(arg >> 8));
  SD_SPI_TxRxByte((uint8_t)arg);

  n = 0x01; /* Dummy CRC + Stop */
  if (cmd == CMD0) n = 0x95; /* Valid CRC for CMD0(0) */
  if (cmd == CMD8) n = 0x87; /* Valid CRC for CMD8(0x1AA) */
  SD_SPI_TxRxByte(n);

  /* Skip a stuff byte when stopping reading */
  if (cmd == CMD12)
  {
    SD_SPI_TxRxByte(0xFF);
  }

  /* Wait for R1 response (bit 7 == 0) within 10 bytes */
  n = 10;
  do
  {
    res = SD_SPI_TxRxByte(0xFF);
  } while ((res & 0x80) && --n);

  return res;
}

uint8_t SD_SPI_GetCardType(void)
{
  return CardType;
}

/*----------------------------------------------------------------------------
 * FatFs DiskIO Interface Functions
 *----------------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != 0) return STA_NOINIT;
  return Stat;
}

DSTATUS disk_initialize(BYTE pdrv)
{
  uint8_t n, cmd, ty, ocr[4];
  uint32_t start;

  if (pdrv != 0) return STA_NOINIT;

  SD_SPI_InitBus();
  SD_SPI_SetSlowClock();

  /* Send 80 dummy clock cycles with CS HIGH to wake card into native mode */
  SD_CS_High();
  for (n = 0; n < 10; n++)
  {
    SD_SPI_TxRxByte(0xFF);
  }

  ty = 0;
  /* Put card into SPI Idle State (CMD0) - retry up to 5 times */
  uint8_t r1 = 0xFF;
  for (n = 0; n < 5; n++)
  {
    r1 = SD_SendCmd(CMD0, 0);
    if (r1 == 1) break;
    HAL_Delay(10);
  }

  if (r1 == 1)
  {
    start = HAL_GetTick();
    if (SD_SendCmd(CMD8, 0x1AA) == 1)
    {
      /* SDv2 (SDSC or SDHC/SDXC) */
      for (n = 0; n < 4; n++) ocr[n] = SD_SPI_TxRxByte(0xFF);
      if (ocr[2] == 0x01 && ocr[3] == 0xAA)
      {
        /* Wait for leaving idle state (ACMD41 with HCS bit) */
        while (((HAL_GetTick() - start) < 1000) && SD_SendCmd(ACMD41, 1UL << 30)) {}
        if (((HAL_GetTick() - start) < 1000) && SD_SendCmd(CMD58, 0) == 0)
        {
          for (n = 0; n < 4; n++) ocr[n] = SD_SPI_TxRxByte(0xFF);
          ty = (ocr[0] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
        }
      }
    }
    else
    {
      /* SDv1 or MMCv3 */
      if (SD_SendCmd(ACMD41, 0) <= 1)
      {
        ty = CT_SD1;
        cmd = ACMD41;
      }
      else
      {
        ty = CT_MMC;
        cmd = CMD1;
      }
      while (((HAL_GetTick() - start) < 1000) && SD_SendCmd(cmd, 0)) {}
      if (((HAL_GetTick() - start) >= 1000) || SD_SendCmd(CMD16, 512) != 0)
      {
        ty = 0;
      }
    }
  }

  CardType = ty;
  SD_Deselect();

  if (ty)
  {
    Stat &= ~STA_NOINIT;
    SD_SPI_SetFastClock();
  }
  else
  {
    Stat = STA_NOINIT;
  }

  return Stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv != 0 || !count) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  /* Convert LBA to byte address for non-block-addressed (SDSC) cards */
  if (!(CardType & CT_BLOCK))
  {
    sector *= 512;
  }

  if (count == 1)
  {
    if ((SD_SendCmd(CMD17, sector) == 0) && SD_ReceiveDataBlock(buff, 512))
    {
      count = 0;
    }
  }
  else
  {
    if (SD_SendCmd(CMD18, sector) == 0)
    {
      do
      {
        if (!SD_ReceiveDataBlock(buff, 512)) break;
        buff += 512;
      } while (--count);
      SD_SendCmd(CMD12, 0);
    }
  }

  SD_Deselect();
  return count ? RES_ERROR : RES_OK;
}

#if _USE_WRITE == 1
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv != 0 || !count) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  if (!(CardType & CT_BLOCK))
  {
    sector *= 512;
  }

  if (count == 1)
  {
    if ((SD_SendCmd(CMD24, sector) == 0) && SD_SendDataBlock(buff, 0xFE))
    {
      count = 0;
    }
  }
  else
  {
    if (CardType & CT_SDC) SD_SendCmd(ACMD23, count);
    if (SD_SendCmd(CMD25, sector) == 0)
    {
      do
      {
        if (!SD_SendDataBlock(buff, 0xFC)) break;
        buff += 512;
      } while (--count);
      if (!SD_SendDataBlock(0, 0xFD)) count = 1;
    }
  }

  SD_Deselect();
  return count ? RES_ERROR : RES_OK;
}
#endif

#if _USE_IOCTL == 1
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;

  if (pdrv != 0) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
    case CTRL_SYNC:
      if (SD_Select()) res = RES_OK;
      break;
    case GET_SECTOR_SIZE:
      *(WORD *)buff = 512;
      res = RES_OK;
      break;
    case GET_BLOCK_SIZE:
      *(DWORD *)buff = 1;
      res = RES_OK;
      break;
    default:
      res = RES_PARERR;
      break;
  }

  SD_Deselect();
  return res;
}
#endif

DWORD get_fattime(void)
{
  return 0;
}
