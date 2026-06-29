/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_BMI088

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi088.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

#define BMI088_MAX_SPI_CLK_HZ 10000000

#define BMI088_ACC_CHIP_ID 0x1E
#define BMI088_GYRO_CHIP_ID 0x0F

#define GYRO_EXTI_DETECT_THRESHOLD 1000

typedef enum {
    BMI088_ACC_REG_CHIP_ID = 0x00,
    BMI088_ACC_REG_STATUS = 0x03,
    BMI088_ACC_REG_DATA_X_LSB = 0x12,
    BMI088_ACC_REG_CONF = 0x40,
    BMI088_ACC_REG_RANGE = 0x41,
    BMI088_ACC_REG_PWR_CONF = 0x7C,
    BMI088_ACC_REG_PWR_CTRL = 0x7D,
    BMI088_ACC_REG_SOFTRESET = 0x7E,
} bmi088AccRegister_e;

typedef enum {
    BMI088_GYRO_REG_CHIP_ID = 0x00,
    BMI088_GYRO_REG_RATE_X_LSB = 0x02,
    BMI088_GYRO_REG_RANGE = 0x0F,
    BMI088_GYRO_REG_BANDWIDTH = 0x10,
    BMI088_GYRO_REG_LPM1 = 0x11,
    BMI088_GYRO_REG_SOFTRESET = 0x14,
    BMI088_GYRO_REG_INT_CTRL = 0x15,
    BMI088_GYRO_REG_INT3_INT4_IO_CONF = 0x16,
    BMI088_GYRO_REG_INT3_INT4_IO_MAP = 0x18,
} bmi088GyroRegister_e;

typedef enum {
    BMI088_ACC_CONF_ODR_1600HZ = 0x0C,
    BMI088_ACC_CONF_BWP_OSR4 = 0x08 << 4,
    BMI088_ACC_RANGE_12G = 0x02,
    BMI088_ACC_PWR_CONF_ACTIVE = 0x00,
    BMI088_ACC_PWR_CTRL_ON = 0x04,
    BMI088_VAL_CMD_SOFTRESET = 0xB6,

    BMI088_GYRO_RANGE_2000DPS = 0x00,
    BMI088_GYRO_BW_2000HZ_532HZ = 0x80,
    BMI088_GYRO_BW_2000HZ_230HZ = 0x81,
    BMI088_GYRO_BW_1000HZ_116HZ = 0x82,
    BMI088_GYRO_LPM_NORMAL = 0x00,
    BMI088_GYRO_INT_CTRL_DRDY = 0x80,
    BMI088_GYRO_INT3_INT4_IO_CONF_PP_ACTIVE_HIGH = 0x05,
    BMI088_GYRO_INT3_INT4_IO_MAP_DRDY_INT3 = 0x01,
} bmi088ConfigValue_e;

static const extDevice_t *bmi088AccDevice(const gyroDev_t *gyro)
{
    return gyro->accDev.bus ? &gyro->accDev : NULL;
}

static uint8_t bmi088AccRegisterRead(const extDevice_t *dev, bmi088AccRegister_e registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, sizeof(data))) {
        return data[1];
    }

    return 0;
}

static void bmi088RegisterWrite(const extDevice_t *dev, uint8_t registerId, uint8_t value, unsigned delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void bmi088AccEnableSPI(const extDevice_t *dev)
{
    IOLo(dev->busType_u.spi.csnPin);
    delay(1);
    IOHi(dev->busType_u.spi.csnPin);
    delay(1);

    // The first accelerometer SPI read after POR or soft reset only switches the interface to SPI.
    bmi088AccRegisterRead(dev, BMI088_ACC_REG_CHIP_ID);
    delay(1);
}

static uint8_t getBmi088GyroBandwidth(const gyroDev_t *gyro)
{
    switch (gyro->hardware_lpf) {
    case GYRO_HARDWARE_LPF_OPTION_1:
        return BMI088_GYRO_BW_2000HZ_532HZ;

    case GYRO_HARDWARE_LPF_OPTION_2:
        return BMI088_GYRO_BW_1000HZ_116HZ;

    case GYRO_HARDWARE_LPF_NORMAL:
    default:
        return BMI088_GYRO_BW_2000HZ_230HZ;
    }
}

static bool bmi088AccIsDetected(const extDevice_t *dev)
{
    if (!dev) {
        return false;
    }

    bmi088AccEnableSPI(dev);

    for (int attempts = 0; attempts < 5; attempts++) {
        delay(100);

        if (bmi088AccRegisterRead(dev, BMI088_ACC_REG_CHIP_ID) == BMI088_ACC_CHIP_ID) {
            return true;
        }
    }

    return false;
}

uint8_t bmi088Detect(const extDevice_t *dev)
{
    for (int attempts = 0; attempts < 5; attempts++) {
        delay(100);

        if (spiReadRegMsk(dev, BMI088_GYRO_REG_CHIP_ID) == BMI088_GYRO_CHIP_ID) {
            gyroDev_t *gyro = container_of(dev, gyroDev_t, dev);
            const extDevice_t *accDev = bmi088AccDevice(gyro);

            if (accDev && !bmi088AccIsDetected(accDev)) {
                return MPU_NONE;
            }

            return BMI_088_SPI;
        }
    }

    return MPU_NONE;
}

static void bmi088GyroConfig(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    bmi088RegisterWrite(dev, BMI088_GYRO_REG_SOFTRESET, BMI088_VAL_CMD_SOFTRESET, 100);
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_LPM1, BMI088_GYRO_LPM_NORMAL, 30);
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_RANGE, BMI088_GYRO_RANGE_2000DPS, 1);
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_BANDWIDTH, getBmi088GyroBandwidth(gyro), 1);
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_INT_CTRL, BMI088_GYRO_INT_CTRL_DRDY, 1);
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_INT4_IO_CONF_PP_ACTIVE_HIGH, 1);
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_INT3_INT4_IO_MAP, BMI088_GYRO_INT3_INT4_IO_MAP_DRDY_INT3, 1);
}

static void bmi088AccConfig(accDev_t *acc)
{
    const extDevice_t *dev = bmi088AccDevice(acc->gyro);

    if (!dev) {
        return;
    }

    bmi088RegisterWrite(dev, BMI088_ACC_REG_SOFTRESET, BMI088_VAL_CMD_SOFTRESET, 100);
    bmi088AccEnableSPI(dev);
    bmi088RegisterWrite(dev, BMI088_ACC_REG_PWR_CONF, BMI088_ACC_PWR_CONF_ACTIVE, 100);
    bmi088RegisterWrite(dev, BMI088_ACC_REG_PWR_CTRL, BMI088_ACC_PWR_CTRL_ON, 100);
    bmi088RegisterWrite(dev, BMI088_ACC_REG_CONF, BMI088_ACC_CONF_BWP_OSR4 | BMI088_ACC_CONF_ODR_1600HZ, 1);
    bmi088RegisterWrite(dev, BMI088_ACC_REG_RANGE, BMI088_ACC_RANGE_12G, 1);

    acc->acc_1G = 2048;
}

#ifdef USE_DMA
static busStatus_e bmi088IntCallback(uintptr_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    const int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}
#endif

static void bmi088ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    extDevice_t *dev = &gyro->dev;

    const uint32_t nowCycles = getCycleCounter();
    gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    gyro->gyroLastEXTI = nowCycles;

    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(dev, gyro->segments);
    }

    gyro->detectedEXTI++;
}

static void bmi088IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi088ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}

static bool bmi088AccRead(accDev_t *acc)
{
    const extDevice_t *dev = bmi088AccDevice(acc->gyro);
    uint8_t data[7];

    if (!dev || !(bmi088AccRegisterRead(dev, BMI088_ACC_REG_STATUS) & 0x80) || !spiReadRegMskBufRB(dev, BMI088_ACC_REG_DATA_X_LSB, data, sizeof(data))) {
        return false;
    }

    acc->ADCRaw[X] = ((int16_t)((data[2] << 8) | data[1])) * 3 / 4;
    acc->ADCRaw[Y] = ((int16_t)((data[4] << 8) | data[3])) * 3 / 4;
    acc->ADCRaw[Z] = ((int16_t)((data[6] << 8) | data[5])) * 3 / 4;

    return true;
}

static bool bmi088GyroRead(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    int16_t *gyroData = (int16_t *)dev->rxBuf;

    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
        memset(dev->txBuf, 0x00, 8);

        gyro->gyroDmaMaxDuration = 5;
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
#ifdef USE_DMA
            if (spiUseDMA(dev)) {
                dev->callbackArg = (uintptr_t)gyro;
                dev->txBuf[1] = BMI088_GYRO_REG_RATE_X_LSB | 0x80;
                gyro->segments[0].len = 7;
                gyro->segments[0].callback = bmi088IntCallback;
                gyro->segments[0].u.buffers.txData = &dev->txBuf[1];
                gyro->segments[0].u.buffers.rxData = &dev->rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else
#endif
            {
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        dev->txBuf[1] = BMI088_GYRO_REG_RATE_X_LSB | 0x80;

        busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 7, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &dev->txBuf[1];
        segments[0].u.buffers.rxData = &dev->rxBuf[1];

        spiSequence(dev, &segments[0]);
        spiWait(dev);

        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];
        break;

    default:
        break;
    }

    return true;
}

static void bmi088SpiGyroInit(gyroDev_t *gyro)
{
    bmi088GyroConfig(gyro);
    bmi088IntExtiInit(gyro);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));
}

static void bmi088SpiAccInit(accDev_t *acc)
{
    bmi088AccConfig(acc);
    spiSetClkDivisor(&acc->gyro->accDev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));
}

bool bmi088SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_088_SPI) {
        return false;
    }

    const extDevice_t *dev = bmi088AccDevice(acc->gyro);
    if (!bmi088AccIsDetected(dev)) {
        return false;
    }

    busDeviceRegister(dev);

    acc->initFn = bmi088SpiAccInit;
    acc->readFn = bmi088AccRead;

    return true;
}

bool bmi088SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_088_SPI) {
        return false;
    }

    gyro->initFn = bmi088SpiGyroInit;
    gyro->readFn = bmi088GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

#endif // USE_ACCGYRO_BMI088
