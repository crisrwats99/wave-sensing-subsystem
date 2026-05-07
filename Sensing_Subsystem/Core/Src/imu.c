/* =============================================================================
 * FILE     : imu.c
 * BOARD    : STM32 NUCLEO-L4R5ZI-P
 * SENSOR   : GY-87  (MPU6050 + HMC5883L)
 * PROTOCOL : I2C1   (PB8 = SCL, PB9 = SDA)
 *
 * HOW IT MAPS TO THE ARDUINO CODE:
 * ─────────────────────────────────────────────────────────────────────────────
 *  Arduino                              │ This STM32 code
 *  ─────────────────────────────────────┼──────────────────────────────────────
 *  Wire.begin()                         │ MX_I2C1_Init() in main.c
 *  accelgyro.initialize()               │ MPU6050_Init()
 *  accelgyro.testConnection()           │ HAL_I2C_Mem_Read WHO_AM_I
 *  accelgyro.setI2CBypassEnabled(true)  │ IMU_EnableBypass()   ← CRITICAL
 *  Wire.beginTransmission / write / end │ HAL_I2C_Mem_Write
 *  Wire.requestFrom / read              │ HAL_I2C_Mem_Read
 *  accelgyro.getMotion6()               │ MPU6050_Read_Accel + MPU6050_Read_Gyro
 *  atan2(-y, x)                         │ atan2f(-my, mx)
 *
 * WIRING:
 *   GY-87 SDA → PB9   GY-87 SCL → PB8   VCC → 3.3V   GND → GND
 *
 * LIBRARY USED: MPU6050.c / MPU6050.h  (the simple HAL-native version,
 *               already fixed from stm32f1xx → stm32l4xx)
 *
 * =============================================================================
 */

#include "imu.h"
#include "MPU6050.h"
#include "stm32l4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * I2C ADDRESSES  (7-bit shifted left by 1 for HAL)
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define MPU_ADDR    0xD0    /* MPU6050  : 0x68 << 1                          */
#define HMC_ADDR    0x3C    /* HMC5883L : 0x1E << 1                          */

/* ─────────────────────────────────────────────────────────────────────────────
 * MPU6050 REGISTER
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define MPU_REG_INT_PIN_CFG  0x37   /* INT_PIN_CFG register                  */
#define MPU_BYPASS_BIT       0x02   /* bit 1 = I2C_BYPASS_EN                 */

/* ─────────────────────────────────────────────────────────────────────────────
 * HMC5883L REGISTERS
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define HMC_MODE_REG         0x02   /* Mode register                         */
#define HMC_DATA_REG         0x03   /* First output byte (X MSB)             */
#define HMC_CONTINUOUS       0x00   /* Continuous measurement mode           */
#define HMC_IDLE             0x02   /* Idle / sleep mode                     */

/* ─────────────────────────────────────────────────────────────────────────────
 * BATCH BUFFER  (defined here, extern in imu.h)
 * ─────────────────────────────────────────────────────────────────────────────
 */
IMU_Sample_t imu_batch[IMU_BATCH_SIZE];
uint16_t     imu_batch_count = 0;
uint8_t      imu_batch_ready = 0;

/* ─────────────────────────────────────────────────────────────────────────────
 * PRIVATE STATE
 * ─────────────────────────────────────────────────────────────────────────────
 */
static float    off_ax, off_ay, off_az;     /* accel calibration offsets     */
static float    off_gx, off_gy, off_gz;     /* gyro  calibration offsets     */
static float    roll_est  = 0.0f;           /* complementary filter state    */
static float    pitch_est = 0.0f;
static uint8_t  filter_seeded = 0;

static uint32_t next_tick    = 0;
static uint8_t  tick_started = 0;

static float    last_heading = 0.0f;        /* last good compass reading     */
static float    last_mx = 0.0f;             /* last raw magnetometer X (µT)  */
static float    last_my = 0.0f;             /* last raw magnetometer Y (µT)  */
static float    last_mz = 0.0f;             /* last raw magnetometer Z (µT)  */
static uint8_t  mag_turn = 0;               /* read mag every other tick     */

static uint32_t last_debug_ms = 0;

/* HAL handles from main.c */
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef hlpuart1;


/* ─────────────────────────────────────────────────────────────────────────────
 * TINY HELPERS  (keep the main functions readable)
 * ─────────────────────────────────────────────────────────────────────────────
 */

/* Write one byte to one register on any I2C device */
static void i2c_write(uint16_t addr, uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, addr, reg, 1, &val, 1, 100);
}

/* Read len bytes starting at reg into buf */
static void i2c_read(uint16_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_I2C_Mem_Read(&hi2c1, addr, reg, 1, buf, len, 100);
}

/* Send a string to the serial terminal (LPUART1 → ST-LINK virtual COM port) */
static void dbg(const char *msg)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);
}


/* ─────────────────────────────────────────────────────────────────────────────
 * IMU_EnableBypass
 *
 * This is the equivalent of accelgyro.setI2CBypassEnabled(true) in Arduino.
 *
 * WHY THIS IS NEEDED:
 *   On the GY-87 PCB, the HMC5883L's SCL/SDA lines are wired to the MPU6050's
 *   auxiliary I2C pins (AUX_CL, AUX_DA), NOT directly to the main I2C bus.
 *   By default the MPU6050 acts as a gatekeeper — the STM32 cannot see the
 *   HMC5883L at all.
 *
 *   Writing bit 1 of register 0x37 (INT_PIN_CFG) to 1 enables I2C bypass:
 *   the auxiliary I2C pins are connected through to the main I2C bus, so
 *   the STM32 can address the HMC5883L directly at 0x1E (0x3C in HAL format).
 *
 *   Without this line: HAL_I2C_Mem_Read to the HMC5883L will always time out.
 * ─────────────────────────────────────────────────────────────────────────────
 */
static void IMU_EnableBypass(void)
{
    i2c_write(MPU_ADDR, MPU_REG_INT_PIN_CFG, MPU_BYPASS_BIT);
    /* MPU_BYPASS_BIT = 0x02 = 0b00000010
     * Bit 1 = I2C_BYPASS_EN.  Setting this to 1 opens the bypass. */
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Complementary filter
 * ─────────────────────────────────────────────────────────────────────────────
 */
static void filter_update(float ax, float ay, float az, float gx, float gy)
{
    float ar =  atan2f(ay, az) * (180.0f / 3.14159f);
    float ap = -atan2f(ax, sqrtf(ay*ay + az*az)) * (180.0f / 3.14159f);
    float dt = 1.0f / (float)IMU_SAMPLE_RATE_HZ;

    if (!filter_seeded) { roll_est = ar; pitch_est = ap; filter_seeded = 1; return; }

    roll_est  = 0.98f * (roll_est  + gx * dt) + 0.02f * ar;
    pitch_est = 0.98f * (pitch_est + gy * dt) + 0.02f * ap;
}


/* ─────────────────────────────────────────────────────────────────────────────
 * Read HMC5883L — direct I2C
 * ─────────────────────────────────────────────────────────────────────────────
 */
#define HMC_SENS_UT  0.092f   /* LSB to microtesla at default gain setting   */

static float read_magnetometer(float *mx_out, float *my_out, float *mz_out)
{
    uint8_t buf[6];
    i2c_read(HMC_ADDR, HMC_DATA_REG, buf, 6);

    /* HMC5883L byte order: X, Z, Y  (Z and Y are SWAPPED — datasheet quirk) */
    int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_z = (int16_t)((buf[2] << 8) | buf[3]);   /* Z is at [2:3]   */
    int16_t raw_y = (int16_t)((buf[4] << 8) | buf[5]);   /* Y is at [4:5]   */

    /* Convert raw counts to microtesla */
    *mx_out = raw_x * HMC_SENS_UT;
    *my_out = raw_y * HMC_SENS_UT;
    *mz_out = raw_z * HMC_SENS_UT;

    /* Heading */
    float angle = atan2f(-(float)raw_y, (float)raw_x) * (180.0f / 3.14159f);
    if (angle < 0.0f) angle += 360.0f;

    return angle;
}


/* =============================================================================
 * PUBLIC FUNCTIONS
 * =============================================================================
 */

/* ---------------------------------------------------------------------------
 * IMU_Init
 * Same as: accelgyro.initialize() + accelgyro.setI2CBypassEnabled(true)
 *          + Wire.write(hmcContinuousMode)
 * ---------------------------------------------------------------------------
 */
uint8_t IMU_Init(void)
{
    dbg("[IMU] Initialising GY-87...\r\n");

    /* Step 1: Wake and configure the MPU6050 (library handles this) */
    MPU6050_Init();
    HAL_Delay(100);

    /* Step 2: Verify MPU6050 is alive */
    uint8_t who = 0;
    i2c_read(MPU_ADDR, 0x75, &who, 1);  /* 0x75 = WHO_AM_I register*/
    if (who != 0x68)
    {
        dbg("[IMU] ERROR: MPU6050 not found. Check SDA=PB9, SCL=PB8.\r\n");
        return 0;
    }
    dbg("[IMU] MPU6050 OK\r\n");

    /* Step 3: Enable I2C bypass so STM32 can see the HMC5883L directly.*/
    IMU_EnableBypass();
    HAL_Delay(10);

    /* Step 4: Put HMC5883L into continuous measurement mode.*/
    i2c_write(HMC_ADDR, HMC_MODE_REG, HMC_CONTINUOUS);
    HAL_Delay(10);

    /* Verify HMC5883L is now visible (ID register 0x0A returns 'H' = 0x48) */
    uint8_t hid = 0;
    i2c_read(HMC_ADDR, 0x0A, &hid, 1);
    if (hid != 0x48)
    {
        dbg("[IMU] ERROR: HMC5883L not found. Bypass may not have worked.\r\n");
        return 0;
    }
    dbg("[IMU] HMC5883L OK\r\n");
    dbg("[IMU] Ready. 100Hz | +-2g | +-250dps\r\n");

    return 1;
}


/* ---------------------------------------------------------------------------
 * IMU_Calibrate — collect 500 still samples, measure sensor bias
 * ---------------------------------------------------------------------------
 */
uint8_t IMU_Calibrate(void)
{
    dbg("[IMU] Calibrating (5 s) — keep sensor still and flat...\r\n");

    double sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
    float ax, ay, az, gx, gy, gz;
    uint32_t t = HAL_GetTick();

    for (int i = 0; i < IMU_CALIB_SAMPLES; i++)
    {
        while ((int32_t)(HAL_GetTick() - t) < 0) {}
        t += IMU_PERIOD_MS;

        MPU6050_Read_Accel(&ax, &ay, &az);
        MPU6050_Read_Gyro (&gx, &gy, &gz);

        sax+=ax; say+=ay; saz+=az;
        sgx+=gx; sgy+=gy; sgz+=gz;
    }

    off_ax = (float)(sax/IMU_CALIB_SAMPLES);
    off_ay = (float)(say/IMU_CALIB_SAMPLES);
    off_az = (float)(saz/IMU_CALIB_SAMPLES) - 1.0f;  /* remove gravity      */
    off_gx = (float)(sgx/IMU_CALIB_SAMPLES);
    off_gy = (float)(sgy/IMU_CALIB_SAMPLES);
    off_gz = (float)(sgz/IMU_CALIB_SAMPLES);

    char msg[100];
    snprintf(msg, sizeof(msg),
        "[IMU] Offsets: ax=%.4f ay=%.4f az=%.4f | gx=%.4f gy=%.4f gz=%.4f\r\n",
        off_ax, off_ay, off_az, off_gx, off_gy, off_gz);
    dbg(msg);

    filter_seeded = 0;
    return 1;
}


/* ---------------------------------------------------------------------------
 * IMU_Sleep — both sensors into low-power mode
 * ---------------------------------------------------------------------------
 */
void IMU_Sleep(void)
{
    i2c_write(MPU_ADDR, 0x6B, 0x40);   /* MPU6050  PWR_MGMT_1: SLEEP = 1   */
    i2c_write(HMC_ADDR, HMC_MODE_REG, HMC_IDLE); /* HMC5883L: idle mode    */
    dbg("[IMU] Sleeping.\r\n");
}


/* ---------------------------------------------------------------------------
 * IMU_Wake — bring both sensors back from sleep
 * ---------------------------------------------------------------------------
 */
void IMU_Wake(void)
{
    i2c_write(MPU_ADDR, 0x6B, 0x01);   /* MPU6050: wake, use gyro PLL clock */
    HAL_Delay(100);
    IMU_EnableBypass();                 /* re-enable bypass after wake       */
    i2c_write(HMC_ADDR, HMC_MODE_REG, HMC_CONTINUOUS);
    HAL_Delay(10);

    tick_started  = 0;
    filter_seeded = 0;
    mag_turn      = 0;
    dbg("[IMU] Awake.\r\n");
}


/* ---------------------------------------------------------------------------
 * IMU_Tick — call from main loop every iteration, non-blocking
 * ---------------------------------------------------------------------------
 */
void IMU_Tick(void)
{
    if (imu_batch_ready) return;

    if (!tick_started) { next_tick = HAL_GetTick(); tick_started = 1; }

    if ((int32_t)(HAL_GetTick() - next_tick) < 0) return;
    next_tick += IMU_PERIOD_MS;

    /* Read accel and gyro */
    float ax, ay, az, gx, gy, gz;
    uint32_t ts = HAL_GetTick();
    MPU6050_Read_Accel(&ax, &ay, &az);
    MPU6050_Read_Gyro (&gx, &gy, &gz);

    /* Apply calibration */
    ax -= off_ax; ay -= off_ay; az -= off_az;
    gx -= off_gx; gy -= off_gy; gz -= off_gz;

    /* Update filter */
    filter_update(ax, ay, az, gx, gy);

    /* Read compass every other tick (HMC5883L max 75 Hz, we use 50 Hz) */
    mag_turn = !mag_turn;
    if (mag_turn) last_heading = read_magnetometer(&last_mx, &last_my, &last_mz);

    /* Store in batch */
    IMU_Sample_t *s = &imu_batch[imu_batch_count];
    s->timestamp_ms = ts;
    s->ax = ax; s->ay = ay; s->az = az;
    s->gx = gx; s->gy = gy; s->gz = gz;
    s->mx = last_mx; s->my = last_my; s->mz = last_mz;
    s->heading_deg = last_heading;
    s->roll_deg    = roll_est;
    s->pitch_deg   = pitch_est;
    imu_batch_count++;

    if (imu_batch_count >= IMU_BATCH_SIZE) imu_batch_ready = 1;

    /* ── DEBUG MONITOR — delete or comment this block when done testing ──
     * Prints to serial terminal every 1 second so you can watch live values.
     * Open serial terminal at 115200 baud on your PC.
     * Change IMU_DEBUG_PRINT to 0 in imu.h to disable without deleting.  */
#if IMU_DEBUG_PRINT
    if (HAL_GetTick() - last_debug_ms >= 1000)
    {
        last_debug_ms = HAL_GetTick();
        char buf[180];
        snprintf(buf, sizeof(buf),
            "ax=%.3f ay=%.3f az=%.3f | gx=%.2f gy=%.2f gz=%.2f | "
            "mx=%.1f my=%.1f mz=%.1f uT | "
            "roll=%.1f pitch=%.1f | hdg=%.1f deg\r\n",
            ax, ay, az, gx, gy, gz,
            last_mx, last_my, last_mz,
            roll_est, pitch_est, last_heading);
        dbg(buf);
    }
#endif
}


/* ---------------------------------------------------------------------------
 * IMU_ClearBatch — callrd after SD card finishes writing
 * ---------------------------------------------------------------------------
 */
void IMU_ClearBatch(void)
{
    imu_batch_count = 0;
    imu_batch_ready = 0;
}
