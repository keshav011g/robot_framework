#include "SparkFunLIS3DH.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>

//****************************************************************************//
//  LIS3DHCore (Modified for Raspberry Pi I2C)
//****************************************************************************//
LIS3DHCore::LIS3DHCore(uint8_t i2cAddress) : I2CAddress(i2cAddress)
{
    i2c_fd = -1;
}

status_t LIS3DHCore::beginCore(void)
{
    const char *device = "/dev/i2c-1";
    if((i2c_fd = open(device, O_RDWR)) < 0) {
        return IMU_HW_ERROR;
    }
    
    if(ioctl(i2c_fd, I2C_SLAVE, I2CAddress) < 0) {
        close(i2c_fd);
        return IMU_HW_ERROR;
    }

    // Verify device ID
    uint8_t whoami;
    status_t ret = readRegister(&whoami, LIS3DH_WHO_AM_I);
    if(ret != IMU_SUCCESS || whoami != 0x33) {
        close(i2c_fd);
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

status_t LIS3DHCore::readRegisterRegion(uint8_t *outputPointer, uint8_t offset, uint8_t length)
{
    uint8_t writeBuf[1] = {static_cast<uint8_t>(offset | 0x80)}; // Auto-increment
    
    if(write(i2c_fd, writeBuf, 1) != 1) {
        return IMU_HW_ERROR;
    }
    
    ssize_t bytesRead = read(i2c_fd, outputPointer, length);
    return (bytesRead == length) ? IMU_SUCCESS : IMU_HW_ERROR;
}

status_t LIS3DHCore::readRegister(uint8_t *outputPointer, uint8_t offset)
{
    if(write(i2c_fd, &offset, 1) != 1) {
        return IMU_HW_ERROR;
    }
    
    ssize_t bytesRead = read(i2c_fd, outputPointer, 1);
    return (bytesRead == 1) ? IMU_SUCCESS : IMU_HW_ERROR;
}

status_t LIS3DHCore::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
    uint8_t buffer[2] = {offset, dataToWrite};
    ssize_t bytesWritten = write(i2c_fd, buffer, sizeof(buffer));
    return (bytesWritten == sizeof(buffer)) ? IMU_SUCCESS : IMU_HW_ERROR;
}

//****************************************************************************//
//  LIS3DH (Main Class - Minimal Changes Needed)
//****************************************************************************//
LIS3DH::LIS3DH(uint8_t i2cAddress) : LIS3DHCore(i2cAddress)
{
    // Original settings initialization
    settings.adcEnabled = 1;
    settings.tempEnabled = 1;
    settings.accelSampleRate = 50;
    settings.accelRange = 2;
    settings.xAccelEnabled = 1;
    settings.yAccelEnabled = 1;
    settings.zAccelEnabled = 1;
    settings.fifoEnabled = 0;
    settings.fifoThreshold = 20;
    settings.fifoMode = 0;
    allOnesCounter = 0;
    nonSuccessCounter = 0;
}

status_t LIS3DH::begin(void)
{
    status_t returnError = beginCore();
    applySettings();
    return returnError;
}

// Remaining LIS3DH class methods remain unchanged from original
// (readRawAccelX/Y/Z, calcAccel, FIFO methods, etc.)

