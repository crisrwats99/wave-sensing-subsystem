/* =============================================================================
 * FILE : MPU6050.c
 * ORIGINAL AUTHOR : Controllerstech / community STM32 HAL port
 * MODIFIED BY : EEE4113F Group 23, 2026
 *
 * CHANGES FROM ORIGINAL:
 *   1. Accel scaling changed from (raw * 100 / 16384.0) → (raw / 16384.0)
 *      The original multiplied by 100, giving % of g (e.g. 98.5 when flat).
 *      We want g directly (e.g. 0.985 when flat) so spectral analysis and
 *      the complementary filter work with standard units.
 *   2. Function renamed MPU6050_init → MPU6050_Init (capital I) for clarity.
 *   3. SMPLRT_DIV value changed from 0x07 (125 Hz) to 0x09 (100 Hz).
 *      See MPU6050_Init() comments for explanation.
 *   4. Added DLPF configuration (CONFIG register 0x1A).
 *
 * HOW I2C WORKS IN THIS FILE
 * ──────────────────────────
 * Every function here uses one of two HAL calls:
 *
 *   HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, 1, buffer, len, timeout)
 *   HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, &data, 1, timeout)
 *
 * These implement the full I2C "memory read/write" protocol:
 *
 *   WRITE: [START][0xD0,W][ACK][reg][ACK][data][ACK][STOP]
 *   READ:  [START][0xD0,W][ACK][reg][ACK][RESTART][0xD1,R][ACK][b0]...[STOP]
 *
 * Arguments:
 *   &hi2c1         — the I2C1 handle created by CubeIDE's MX_I2C1_Init()
 *   MPU6050_ADDR   — 0xD0 = 0x68 (7-bit) << 1 (HAL 8-bit format)
 *   reg            — register address inside the MPU-6050
 *   1              — register address is 8 bits wide (I2C_MEMADD_SIZE_8BIT)
 *   buffer/&data   — where to put received bytes / what byte to send
 *   len            — how many bytes to transfer
 *   1000           — timeout: give up after 1000 ms (should never reach this)
 *
 * All I2C activity happens on PB8 (SCL) and PB9 (SDA) as configured in
 * CubeIDE. The MPU-6050 listens at address 0x68 (AD0 pin tied to GND).
 * =============================================================================
 */

#include "MPU6050.h"

/* ─────────────────────────────────────────────────────────────────────────────
 * I2C ADDRESS
 *
 * The MPU-6050 has a 7-bit I2C address of 0x68 when AD0 is tied to GND.
 * STM32 HAL expects the 8-bit format (7-bit address left-shifted by 1).
 * 0x68 << 1 = 0xD0.
 *
 * The LSB (R/W bit) is added automatically by the HAL driver:
 *   0xD0 = write (R/W = 0)
 *   0xD1 = read  (R/W = 1)
 * We always pass 0xD0; the HAL sets the R/W bit internally.
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define MPU6050_ADDR        0xD0

/* ─────────────────────────────────────────────────────────────────────────────
 * REGISTER ADDRESSES
 *
 * Each #define is the 8-bit address of a register inside the MPU-6050.
 * These come from the MPU-6050 Register Map document (InvenSense RM-MPU-6000A).
 * ─────────────────────────────────────────────────────────────────────────────
 */

#define SMPLRT_DIV_REG      0x19
/*
 * Sample Rate Divider register.
 * Controls how often the sensor outputs new data.
 * Formula: Output Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
 * With DLPF active (see CONFIG_REG below), Gyro Output Rate = 1000 Hz.
 * We write 0x09 → Output Rate = 1000/(1+9) = 100 Hz.
 * The original code wrote 0x07 → 1000/8 = 125 Hz.
 * 100 Hz is cleaner and matches the polar wave sensing literature standard.
 */

#define CONFIG_REG          0x1A
/*
 * Configuration register — controls the Digital Low-Pass Filter (DLPF).
 * Bits [2:0] = DLPF_CFG:
 *   0x00 = DLPF off (gyro runs at 8 kHz — do NOT use with SMPLRT_DIV)
 *   0x01 = 188 Hz bandwidth
 *   0x02 =  98 Hz bandwidth
 *   0x03 =  42 Hz bandwidth  ← we use this
 *   0x04 =  20 Hz bandwidth
 *   0x05 =  10 Hz bandwidth
 *   0x06 =   5 Hz bandwidth
 *
 * Writing 0x03 sets 42 Hz bandwidth:
 *   - Removes all vibration noise above 42 Hz BEFORE data enters registers
 *   - Does not touch wave content (our waves are 0.05–0.5 Hz)
 *   - Drops internal gyro clock from 8 kHz to 1 kHz, making SMPLRT_DIV work
 * The original library did not configure this register (left at reset default
 * 0x00 = DLPF off, gyro at 8 kHz), which can produce noisy readings.
 */

#define GYRO_CONFIG_REG     0x1B
/*
 * Gyroscope Configuration register.
 * Bits [4:3] = FS_SEL set the full-scale range:
 *   0x00 → ±250°/s,  sensitivity = 131.0 LSB/°/s  ← we use this (0x00 written)
 *   0x08 → ±500°/s,  sensitivity =  65.5 LSB/°/s
 *   0x10 → ±1000°/s, sensitivity =  32.8 LSB/°/s
 *   0x18 → ±2000°/s, sensitivity =  16.4 LSB/°/s
 * ±250°/s: wave-induced angular rates rarely exceed ±50°/s — finest resolution.
 */

#define ACCEL_CONFIG_REG    0x1C
/*
 * Accelerometer Configuration register.
 * Bits [4:3] = AFS_SEL set the full-scale range:
 *   0x00 → ±2g,  sensitivity = 16384 LSB/g  ← we use this (0x00 written)
 *   0x08 → ±4g,  sensitivity =  8192 LSB/g
 *   0x10 → ±8g,  sensitivity =  4096 LSB/g
 *   0x18 → ±16g, sensitivity =  2048 LSB/g
 * ±2g: wave heave rarely exceeds ±1.5g. ±2g gives maximum resolution.
 * Note: if you mount this on a boat deck that sees occasional hard impacts,
 * consider ±4g to avoid clipping. If you change this, also change the
 * ACCEL_SENSITIVITY constant in imu_sd_logger.c.
 */

#define ACCEL_XOUT_H_REG    0x3B
/*
 * First accelerometer data register. Reading 6 bytes from here gives:
 *   0x3B = ACCEL_XOUT_H (high byte of X)
 *   0x3C = ACCEL_XOUT_L (low  byte of X)
 *   0x3D = ACCEL_YOUT_H
 *   0x3E = ACCEL_YOUT_L
 *   0x3F = ACCEL_ZOUT_H
 *   0x40 = ACCEL_ZOUT_L
 * The MPU-6050 auto-increments the register pointer during a burst read,
 * so one HAL_I2C_Mem_Read call with len=6 gets all three axes atomically.
 */

#define GYRO_XOUT_H_REG     0x43
/*
 * First gyroscope data register. Reading 6 bytes from here gives:
 *   0x43 = GYRO_XOUT_H
 *   0x44 = GYRO_XOUT_L
 *   0x45 = GYRO_YOUT_H
 *   0x46 = GYRO_YOUT_L
 *   0x47 = GYRO_ZOUT_H
 *   0x48 = GYRO_ZOUT_L
 */

#define PWR_MGMT_1_REG      0x6B
/*
 * Power Management register 1.
 * Bit 6 = SLEEP: 1 = sleep mode (default on power-on), 0 = awake.
 * Bit 2:0 = CLKSEL: clock source selection.
 * Writing 0x00 clears SLEEP and sets CLKSEL=000 (internal 8 MHz RC oscillator).
 * A better clock (PLL from gyro) is CLKSEL=001 (0x01), but 0x00 is sufficient
 * for our application and is what the original library uses.
 */

#define WHO_AM_I_REG        0x75
/*
 * WHO_AM_I register — always returns 0x68 on a genuine MPU-6050.
 * Reading this register confirms:
 *   (a) The I2C bus is working (PB8/PB9 wired correctly)
 *   (b) The device at address 0x68 is indeed an MPU-6050
 * If it returns 0x00 or 0xFF, there is a wiring fault.
 */

/* ─────────────────────────────────────────────────────────────────────────────
 * SENSITIVITY CONSTANTS
 *
 * These convert the 16-bit raw integer value to physical units.
 * They are determined by the full-scale range written to the config registers.
 *
 * Accel at ±2g:    16384 LSB/g   → divide raw by 16384.0 to get g
 * Gyro  at ±250°/s: 131  LSB/°/s → divide raw by 131.0   to get °/s
 *
 * If you change the range in the config registers, update these values:
 *   Accel ±4g  → 8192.0    Gyro ±500°/s  → 65.5
 *   Accel ±8g  → 4096.0    Gyro ±1000°/s → 32.8
 *   Accel ±16g → 2048.0    Gyro ±2000°/s → 16.4
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define ACCEL_SENSITIVITY   16384.0f   /* LSB/g   at ±2g range    */
#define GYRO_SENSITIVITY    131.0f     /* LSB/°/s at ±250°/s range */

/* ─────────────────────────────────────────────────────────────────────────────
 * RAW DATA BUFFERS
 *
 * int16_t: signed 16-bit integer, range -32768 to +32767.
 * These store the raw sensor counts before unit conversion.
 * Declared at file scope (not inside functions) so they persist between
 * calls — not strictly necessary here since they are always overwritten
 * before use, but keeps them accessible for debugging.
 * ─────────────────────────────────────────────────────────────────────────────
 */
static int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
static int16_t Gyro_X_RAW,  Gyro_Y_RAW,  Gyro_Z_RAW;

/* ─────────────────────────────────────────────────────────────────────────────
 * I2C HANDLE REFERENCE
 *
 * hi2c1 is defined and initialised by CubeIDE's generated MX_I2C1_Init()
 * inside main.c. 'extern' tells the compiler it lives in another file.
 * The linker connects this declaration to the actual variable at link time.
 *
 * If CubeIDE generates a different handle name (e.g. hi2c2 for I2C2),
 * change this line and the matching CubeMX configuration.
 * ─────────────────────────────────────────────────────────────────────────────
 */
extern I2C_HandleTypeDef hi2c1;


/* =============================================================================
 * MPU6050_Init
 *
 * Verifies the sensor is alive over I2C, then configures it for wave sensing.
 *
 * I2C transactions performed (in order):
 *   1. Read  1 byte  from WHO_AM_I_REG  (0x75) — identity check
 *   2. Write 1 byte  to   PWR_MGMT_1_REG (0x6B) — wake from sleep
 *   3. Write 1 byte  to   SMPLRT_DIV_REG (0x19) — set 100 Hz output rate
 *   4. Write 1 byte  to   CONFIG_REG     (0x1A) — set 42 Hz DLPF
 *   5. Write 1 byte  to   ACCEL_CONFIG_REG (0x1C) — set ±2g range
 *   6. Write 1 byte  to   GYRO_CONFIG_REG  (0x1B) — set ±250°/s range
 *
 * Total: 6 I2C transactions, each < 0.5 ms at 100 kHz = < 3 ms total.
 *
 * The WHO_AM_I check (value 104 = 0x68) prevents silent failure —
 * if init succeeds silently but the sensor is absent, every subsequent
 * read would return 0x00 for all axes (all zeros looks like "flat and still"
 * which is dangerous — you'd log garbage without knowing).
 * =============================================================================
 */
void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t data;

    /* ── Step 1: Identity check ──────────────────────────────────────────────
     * Read WHO_AM_I register. The MPU-6050 always returns 0x68 = 104 decimal.
     * If this fails (I2C error) or returns wrong value (wrong chip/address),
     * we skip the rest of init — the sensor is not usable.               */
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 104)  /* 104 = 0x68 */
    {
        /* ── Step 2: Wake from sleep ─────────────────────────────────────────
         * The MPU-6050 powers on in sleep mode (PWR_MGMT_1 bit 6 = 1).
         * Writing 0x00 to PWR_MGMT_1:
         *   - Clears SLEEP bit (bit 6)    → sensor wakes up
         *   - Clears TEMP_DIS bit (bit 3) → temperature sensor enabled
         *   - Sets CLKSEL=000             → internal 8 MHz RC oscillator
         * After this write, the sensor begins outputting data.           */
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

        /* ── Step 3: Sample rate divider → 100 Hz ───────────────────────────
         * Output Rate = 1000 Hz / (1 + SMPLRT_DIV)
         * Writing 0x09: Output Rate = 1000 / (1+9) = 100 Hz.
         * This requires DLPF to be active (step 4 sets that up).
         * Why 100 Hz: see CONFIG BLOCK in imu_sd_logger.c for full reasoning. */
        data = 0x09;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

        /* ── Step 4: Digital Low-Pass Filter → 42 Hz bandwidth ──────────────
         * Writing 0x03 to CONFIG register bits [2:0]:
         *   DLPF_CFG = 3 → accel bandwidth 44 Hz, gyro bandwidth 42 Hz.
         * Effect: hardware filter removes vibration noise above ~42 Hz.
         * Also switches the internal gyro clock from 8 kHz to 1 kHz,
         * which is required for SMPLRT_DIV to produce the correct rate.
         * NOT in the original library — added here for cleaner data.     */
        data = 0x03;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 1000);

        /* ── Step 5: Accelerometer range → ±2g ──────────────────────────────
         * Writing 0x00 to ACCEL_CONFIG register:
         *   Bits [4:3] = AFS_SEL = 00 → ±2g range, 16384 LSB/g.
         * Maximum resolution for wave sensing (heave < ±1.5g normally).  */
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

        /* ── Step 6: Gyroscope range → ±250°/s ──────────────────────────────
         * Writing 0x00 to GYRO_CONFIG register:
         *   Bits [4:3] = FS_SEL = 00 → ±250°/s range, 131 LSB/°/s.
         * Maximum resolution for wave angular rates (typically < ±50°/s). */
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
    }
    /* If check != 104: sensor absent or I2C fault. Init silently skips.
     * imu_sd_logger.c reads WHO_AM_I separately and calls fatal() if wrong. */
}


/* =============================================================================
 * MPU6050_Read_Accel
 *
 * Reads 6 bytes starting at ACCEL_XOUT_H (0x3B) in one I2C burst read.
 * Reconstructs three signed 16-bit integers (X, Y, Z).
 * Converts to g by dividing by ACCEL_SENSITIVITY (16384 LSB/g at ±2g).
 * Stores results at the three float pointers provided by the caller.
 *
 * I2C transaction:
 *   [START][0xD0,W][ACK][0x3B][ACK][RESTART][0xD1,R][ACK]
 *   [byte0][ACK][byte1][ACK][byte2][ACK][byte3][ACK][byte4][ACK][byte5][NACK][STOP]
 *   Total: 1 transaction, 6 data bytes, ~0.7 ms at 100 kHz.
 *
 * Byte layout (MSB first, big-endian):
 *   Rec_Data[0] = ACCEL_XOUT_H   Rec_Data[1] = ACCEL_XOUT_L  → X axis
 *   Rec_Data[2] = ACCEL_YOUT_H   Rec_Data[3] = ACCEL_YOUT_L  → Y axis
 *   Rec_Data[4] = ACCEL_ZOUT_H   Rec_Data[5] = ACCEL_ZOUT_L  → Z axis
 *
 * Reconstruction: (int16_t)(high_byte << 8 | low_byte)
 *   high_byte << 8 : places high byte in bits [15:8]
 *   | low_byte     : places low  byte in bits [7:0]
 *   (int16_t) cast : interprets bit 15 as the sign bit (two's complement)
 *   Example: Rec_Data = {0x3F, 0x00} → 0x3F00 = 16128 raw → 16128/16384 ≈ 0.984g
 *
 * Output units: g (gravitational acceleration).
 *   When sensor lies flat on a table:
 *     Ax ≈ 0.0g,  Ay ≈ 0.0g,  Az ≈ +1.0g  (gravity on Z axis)
 *   After calibration offset removal (in imu_sd_logger.c):
 *     Ax ≈ 0.0g,  Ay ≈ 0.0g,  Az ≈ 0.0g   (gravity removed, only wave motion)
 * =============================================================================
 */
void MPU6050_Read_Accel(float *Ax, float *Ay, float *Az)
{
    uint8_t Rec_Data[6];

    /* Single burst I2C read: register 0x3B, 6 bytes.
     * The MPU-6050 auto-increments its internal register pointer,
     * so bytes arrive in order: XH, XL, YH, YL, ZH, ZL.              */
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    /* Reconstruct 16-bit signed integers from two consecutive bytes */
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /* Convert raw counts to g.
     * CHANGED from original: removed the ×100 multiplier.
     * Original: *Ax = Accel_X_RAW * 100 / 16384.0  → gave % of g (e.g. 98.5)
     * Fixed:    *Ax = Accel_X_RAW / 16384.0          → gives g directly (e.g. 0.985)
     * We need g units for the complementary filter and CSV analysis.   */
    *Ax = (float)Accel_X_RAW / ACCEL_SENSITIVITY;
    *Ay = (float)Accel_Y_RAW / ACCEL_SENSITIVITY;
    *Az = (float)Accel_Z_RAW / ACCEL_SENSITIVITY;
}


/* =============================================================================
 * MPU6050_Read_Gyro
 *
 * Reads 6 bytes starting at GYRO_XOUT_H (0x43) in one I2C burst read.
 * Reconstructs three signed 16-bit integers (X, Y, Z).
 * Converts to °/s by dividing by GYRO_SENSITIVITY (131 LSB/°/s at ±250°/s).
 * Stores results at the three float pointers provided by the caller.
 *
 * I2C transaction: identical structure to MPU6050_Read_Accel, starting at 0x43.
 *
 * Output units: degrees per second (°/s).
 *   When sensor is stationary:
 *     Gx ≈ 0°/s,  Gy ≈ 0°/s,  Gz ≈ 0°/s  (plus small bias offset)
 *   After calibration offset removal (in imu_sd_logger.c), residual is < 0.1°/s.
 * =============================================================================
 */
void MPU6050_Read_Gyro(float *Gx, float *Gy, float *Gz)
{
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    *Gx = (float)Gyro_X_RAW / GYRO_SENSITIVITY;
    *Gy = (float)Gyro_Y_RAW / GYRO_SENSITIVITY;
    *Gz = (float)Gyro_Z_RAW / GYRO_SENSITIVITY;
}
