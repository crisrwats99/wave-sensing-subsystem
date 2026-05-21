/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver for SD card via SPI
  *          Based on working signal processing code
  ******************************************************************************
  */
/* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */
#include <string.h>
#include "ff_gen_drv.h"

/* SD card SPI driver functions defined in main.c */
extern uint8_t sd_init(void);
extern uint8_t sd_read_block(uint32_t block_addr, uint8_t *buf);
extern uint8_t sd_write_block(uint32_t block_addr, const uint8_t *buf);

static volatile DSTATUS Stat = STA_NOINIT;
/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    (void)pdrv;

    /* If already initialized, return current status */
    if ((Stat & STA_NOINIT) == 0U) {
        return Stat;
    }

    /* Give SD card time to power up */
    HAL_Delay(200);

    /* Initialize SD card via SPI */
    if (sd_init() == 0U) {
        Stat = 0;  /* Success */
    } else {
        Stat = STA_NOINIT;  /* Initialization failed */
    }

    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    (void)pdrv;
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
    (void)pdrv;

    /* Parameter check */
    if (buff == NULL || count == 0U) {
        return RES_PARERR;
    }

    /* Check if drive is initialized */
    if (Stat & STA_NOINIT) {
        return RES_NOTRDY;
    }

    /* Read sectors one by one */
    for (UINT i = 0U; i < count; i++) {
        if (sd_read_block((uint32_t)(sector + i), &buff[i * 512U]) != 0U) {
            return RES_ERROR;
        }
    }

    return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
    (void)pdrv;

    /* Parameter check */
    if (buff == NULL || count == 0U) {
        return RES_PARERR;
    }

    /* Check if drive is initialized */
    if (Stat & STA_NOINIT) {
        return RES_NOTRDY;
    }

    /* Write sectors one by one */
    for (UINT i = 0U; i < count; i++) {
        if (sd_write_block((uint32_t)(sector + i), &buff[i * 512U]) != 0U) {
            return RES_ERROR;
        }
    }

    return RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    (void)pdrv;

    /* Check if drive is initialized */
    if (Stat & STA_NOINIT) {
        return RES_NOTRDY;
    }

    switch (cmd) {
        case CTRL_SYNC:
            /* Complete pending write process */
            return RES_OK;

        case GET_SECTOR_SIZE:
            /* Get sector size (512 bytes for SD cards) */
            if (buff == NULL) return RES_PARERR;
            *(WORD*)buff = 512U;
            return RES_OK;

        case GET_BLOCK_SIZE:
            /* Get erase block size (1 sector) */
            if (buff == NULL) return RES_PARERR;
            *(DWORD*)buff = 1U;
            return RES_OK;

        case GET_SECTOR_COUNT:
            /* Get number of sectors (fake value for now) */
            if (buff == NULL) return RES_PARERR;
            *(DWORD*)buff = 0x100000UL;  /* ~8GB */
            return RES_OK;

        default:
            return RES_PARERR;
    }
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */
