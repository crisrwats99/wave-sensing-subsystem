/* =============================================================================
 * FILE     : user_diskio.c (CLEAN VERSION)
 * BASED ON : https://github.com/MatveyMelnikov/SDCardDriver
 * BOARD    : STM32 NUCLEO-L4R5ZI-P
 * =============================================================================
 */

#include "ff_gen_drv.h"
#include "stm32l4xx_hal.h"
#include "diskio.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define SD_CS_PIN       GPIO_PIN_4
#define SD_CS_PORT      GPIOA
#define SD_CS_LOW()     HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)
#define SD_TIMEOUT      1000

/* Commands */
#define CMD0    0
#define CMD1    1
#define CMD8    8
#define CMD12   12
#define CMD16   16
#define CMD17   17
#define CMD18   18
#define CMD24   24
#define CMD25   25
#define CMD55   55
#define CMD58   58
#define ACMD23  23
#define ACMD41  41

#define R1_READY_STATE      0x00
#define R1_IDLE_STATE       0x01
#define DATA_START_BLOCK    0xFE
#define DATA_ACCEPT         0x05

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;

#define CT_MMC   0x01
#define CT_SD1   0x02
#define CT_SD2   0x04
#define CT_BLOCK 0x08

/* SPI helper */
static uint8_t SPI_TxRx(uint8_t data)
{
    uint8_t rxdata;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rxdata, 1, SD_TIMEOUT);
    return rxdata;
}

/* Wait for card ready */
static uint8_t SD_WaitReady(uint32_t timeout_ms)
{
    uint8_t res;
    uint32_t start = HAL_GetTick();
    do {
        res = SPI_TxRx(0xFF);
        if (res == 0xFF) return 1;
    } while ((HAL_GetTick() - start) < timeout_ms);
    return 0;
}

/* Deselect */
static void SD_Deselect(void)
{
    SD_CS_HIGH();
    SPI_TxRx(0xFF);
}

/* Select */
static uint8_t SD_Select(void)
{
    SD_CS_LOW();
    SPI_TxRx(0xFF);
    if (SD_WaitReady(500)) return 1;
    SD_Deselect();
    return 0;
}

/* Send command */
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    uint8_t res, n;

    /* ACMD: send CMD55 first */
    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = SD_SendCmd(CMD55, 0);
        if (res > 1) return res;
    }

    /* Select card (except CMD0) */
    if (cmd != CMD0) {
        SD_Deselect();
        if (!SD_Select()) return 0xFF;
    }

    /* Send command */
    SPI_TxRx(0x40 | cmd);
    SPI_TxRx((uint8_t)(arg >> 24));
    SPI_TxRx((uint8_t)(arg >> 16));
    SPI_TxRx((uint8_t)(arg >> 8));
    SPI_TxRx((uint8_t)arg);

    /* CRC */
    n = 0x01;
    if (cmd == CMD0) n = 0x95;
    if (cmd == CMD8) n = 0x87;
    SPI_TxRx(n);

    if (cmd == CMD12) SPI_TxRx(0xFF);

    /* Wait for response */
    n = 10;
    do {
        res = SPI_TxRx(0xFF);
    } while ((res & 0x80) && --n);

    return res;
}

/* Initialize */
DSTATUS USER_initialize(BYTE pdrv)
{
    uint8_t n, cmd, ty, ocr[4];
    uint16_t tmr;

    if (pdrv) return STA_NOINIT;

    SD_CS_HIGH();
    for (n = 0; n < 10; n++) SPI_TxRx(0xFF);

    ty = 0;
    if (SD_SendCmd(CMD0, 0) == R1_IDLE_STATE) {
        tmr = 1000;

        if (SD_SendCmd(CMD8, 0x1AA) == R1_IDLE_STATE) {
            /* SDv2 */
            for (n = 0; n < 4; n++) ocr[n] = SPI_TxRx(0xFF);

            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                /* ACMD41 with HCS */
                while (tmr-- && SD_SendCmd(ACMD41 | 0x80, 1UL << 30));

                if (tmr && SD_SendCmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = SPI_TxRx(0xFF);
                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        } else {
            /* SDv1 or MMC */
            if (SD_SendCmd(ACMD41 | 0x80, 0) <= 1) {
                ty = CT_SD1;
                cmd = ACMD41 | 0x80;
            } else {
                ty = CT_MMC;
                cmd = CMD1;
            }

            while (tmr-- && SD_SendCmd(cmd, 0));

            if (!tmr || SD_SendCmd(CMD16, 512) != 0) {
                ty = 0;
            }
        }
    }

    CardType = ty;
    SD_Deselect();

    if (ty) {
        Stat &= ~STA_NOINIT;
    } else {
        Stat = STA_NOINIT;
    }

    return Stat;
}

/* Status */
DSTATUS USER_status(BYTE pdrv)
{
    if (pdrv) return STA_NOINIT;
    return Stat;
}

/* Read */
DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) sector *= 512;

    if (count == 1) {
        if ((SD_SendCmd(CMD17, sector) == 0)) {
            uint8_t token;
            uint16_t timeout = 65000;

            do {
                token = SPI_TxRx(0xFF);
                if (token == DATA_START_BLOCK) break;
            } while (timeout--);

            if (token == DATA_START_BLOCK) {
                for (uint16_t i = 0; i < 512; i++) {
                    buff[i] = SPI_TxRx(0xFF);
                }
                SPI_TxRx(0xFF);
                SPI_TxRx(0xFF);
                count = 0;
            }
        }
    } else {
        if (SD_SendCmd(CMD18, sector) == 0) {
            do {
                uint8_t token;
                uint16_t timeout = 65000;

                do {
                    token = SPI_TxRx(0xFF);
                    if (token == DATA_START_BLOCK) break;
                } while (timeout--);

                if (token != DATA_START_BLOCK) break;

                for (uint16_t i = 0; i < 512; i++) {
                    *buff++ = SPI_TxRx(0xFF);
                }
                SPI_TxRx(0xFF);
                SPI_TxRx(0xFF);
            } while (--count);

            SD_SendCmd(CMD12, 0);
        }
    }

    SD_Deselect();
    return count ? RES_ERROR : RES_OK;
}

/* Write */
#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) sector *= 512;

    if (count == 1) {
        if ((SD_SendCmd(CMD24, sector) == 0)) {
            SPI_TxRx(DATA_START_BLOCK);

            for (uint16_t i = 0; i < 512; i++) {
                SPI_TxRx(buff[i]);
            }

            SPI_TxRx(0xFF);
            SPI_TxRx(0xFF);

            uint8_t res = SPI_TxRx(0xFF);
            if ((res & 0x1F) == DATA_ACCEPT) {
                if (SD_WaitReady(5000)) {  /* 5 second timeout for write */
                    count = 0;
                }
            }
        }
    } else {
        if (CardType & CT_SD2) {
            SD_SendCmd(ACMD23 | 0x80, count);
        }

        if (SD_SendCmd(CMD25, sector) == 0) {
            do {
                if (!SD_WaitReady(5000)) break;  /* 5 second timeout */

                SPI_TxRx(0xFC);

                for (uint16_t i = 0; i < 512; i++) {
                    SPI_TxRx(*buff++);
                }

                SPI_TxRx(0xFF);
                SPI_TxRx(0xFF);

                uint8_t res = SPI_TxRx(0xFF);
                if ((res & 0x1F) != DATA_ACCEPT) break;
            } while (--count);

            SPI_TxRx(0xFD);
            if (SD_WaitReady(5000)) {  /* 5 second timeout */
                count = 0;
            }
        }
    }

    SD_Deselect();
    return count ? RES_ERROR : RES_OK;
}
#endif

/* IOCTL */
#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;

    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd)
    {
        case CTRL_SYNC:
            if (SD_Select()) {
                SD_Deselect();
                res = RES_OK;
            }
            break;

        case GET_SECTOR_COUNT:
            *(DWORD*)buff = 1024000;
            res = RES_OK;
            break;

        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            res = RES_OK;
            break;

        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 1;
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
    }

    return res;
}
#endif

Diskio_drvTypeDef USER_Driver =
{
    USER_initialize,
    USER_status,
    USER_read,
#if _USE_WRITE == 1
    USER_write,
#endif
#if _USE_IOCTL == 1
    USER_ioctl,
#endif
};
