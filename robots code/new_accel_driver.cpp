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
void LIS3DH::applySettings( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable

	//Build TEMP_CFG_REG
	dataToWrite = 0; //Start Fresh!
	dataToWrite = ((settings.tempEnabled & 0x01) << 6) | ((settings.adcEnabled & 0x01) << 7);
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_TEMP_CFG_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_TEMP_CFG_REG, dataToWrite);
	
	//Build CTRL_REG1
	dataToWrite = 0; //Start Fresh!
	//  Convert ODR
	switch(settings.accelSampleRate)
	{
		case 1:
		dataToWrite |= (0x01 << 4);
		break;
		case 10:
		dataToWrite |= (0x02 << 4);
		break;
		case 25:
		dataToWrite |= (0x03 << 4);
		break;
		case 50:
		dataToWrite |= (0x04 << 4);
		break;
		case 100:
		dataToWrite |= (0x05 << 4);
		break;
		case 200:
		dataToWrite |= (0x06 << 4);
		break;
		default:
		case 400:
		dataToWrite |= (0x07 << 4);
		break;
		case 1600:
		dataToWrite |= (0x08 << 4);
		break;
		case 5000:
		dataToWrite |= (0x09 << 4);
		break;
	}
	
	dataToWrite |= (settings.zAccelEnabled & 0x01) << 2;
	dataToWrite |= (settings.yAccelEnabled & 0x01) << 1;
	dataToWrite |= (settings.xAccelEnabled & 0x01);
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_CTRL_REG1: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_CTRL_REG1, dataToWrite);

	//Build CTRL_REG4
	dataToWrite = 0; //Start Fresh!
	//  Convert scaling
	switch(settings.accelRange)
	{
		case 2:
		dataToWrite |= (0x00 << 4);
		break;
		case 4:
		dataToWrite |= (0x01 << 4);
		break;
		case 8:
		dataToWrite |= (0x02 << 4);
		break;
		default:
		case 16:
		dataToWrite |= (0x03 << 4);
		break;
	}
	dataToWrite |= 0x80; //set block update
	dataToWrite |= 0x08; //set high resolution
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_CTRL_REG4: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	//Now, write the patched together data
	writeRegister(LIS3DH_CTRL_REG4, dataToWrite);

}
//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t LIS3DH::readRawAccelX( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS3DH_OUT_X_L );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LIS3DH::readFloatAccelX( void )
{
	float output = calcAccel(readRawAccelX());
	return output;
}

int16_t LIS3DH::readRawAccelY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS3DH_OUT_Y_L );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}

float LIS3DH::readFloatAccelY( void )
{
	float output = calcAccel(readRawAccelY());
	return output;
}

int16_t LIS3DH::readRawAccelZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS3DH_OUT_Z_L );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;

}

float LIS3DH::readFloatAccelZ( void )
{
	float output = calcAccel(readRawAccelZ());
	return output;
}

float LIS3DH::calcAccel( int16_t input )
{
	float output;
	switch(settings.accelRange)
	{
		case 2:
		output = (float)input / 15987;
		break;
		case 4:
		output = (float)input / 7840;
		break;
		case 8:
		output = (float)input / 3883;
		break;
		case 16:
		output = (float)input / 1280;
		break;
		default:
		output = 0;
		break;
	}
	return output;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
uint16_t LIS3DH::read10bitADC1( void )
{
	int16_t intTemp;
	uint16_t uintTemp;
	readRegisterInt16( &intTemp, LIS3DH_OUT_ADC1_L );
	intTemp = 0 - intTemp;
	uintTemp = intTemp + 32768;
	return uintTemp >> 6;
}

uint16_t LIS3DH::read10bitADC2( void )
{
	int16_t intTemp;
	uint16_t uintTemp;
	readRegisterInt16( &intTemp, LIS3DH_OUT_ADC2_L );
	intTemp = 0 - intTemp;
	uintTemp = intTemp + 32768;
	return uintTemp >> 6;
}

uint16_t LIS3DH::read10bitADC3( void )
{
	int16_t intTemp;
	uint16_t uintTemp;
	readRegisterInt16( &intTemp, LIS3DH_OUT_ADC3_L );
	intTemp = 0 - intTemp;
	uintTemp = intTemp + 32768;
	return uintTemp >> 6;
}

//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//
void LIS3DH::fifoBegin( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable

	//Build LIS3DH_FIFO_CTRL_REG
	readRegister( &dataToWrite, LIS3DH_FIFO_CTRL_REG ); //Start with existing data
	dataToWrite &= 0x20;//clear all but bit 5
	dataToWrite |= (settings.fifoMode & 0x03) << 6; //apply mode
	dataToWrite |= (settings.fifoThreshold & 0x1F); //apply threshold
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_CTRL_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_FIFO_CTRL_REG, dataToWrite);

	//Build CTRL_REG5
	readRegister( &dataToWrite, LIS3DH_CTRL_REG5 ); //Start with existing data
	dataToWrite &= 0xBF;//clear bit 6
	dataToWrite |= (settings.fifoEnabled & 0x01) << 6;
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_CTRL_REG5: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_CTRL_REG5, dataToWrite);
}

void LIS3DH::fifoClear( void ) {
	//Drain the fifo data and dump it
	while( (fifoGetStatus() & 0x20 ) == 0 ) {
		readRawAccelX();
		readRawAccelY();
		readRawAccelZ();
	}
}

void LIS3DH::fifoStartRec( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable
	
	//Turn off...
	readRegister( &dataToWrite, LIS3DH_FIFO_CTRL_REG ); //Start with existing data
	dataToWrite &= 0x3F;//clear mode
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_CTRL_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_FIFO_CTRL_REG, dataToWrite);	
	//  ... then back on again
	readRegister( &dataToWrite, LIS3DH_FIFO_CTRL_REG ); //Start with existing data
	dataToWrite &= 0x3F;//clear mode
	dataToWrite |= (settings.fifoMode & 0x03) << 6; //apply mode
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_CTRL_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_FIFO_CTRL_REG, dataToWrite);
}

uint8_t LIS3DH::fifoGetStatus( void )
{
	//Return some data on the state of the fifo
	uint8_t tempReadByte = 0;
	readRegister(&tempReadByte, LIS3DH_FIFO_SRC_REG);
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_SRC_REG: 0x");
	Serial.println(tempReadByte, HEX);
#endif
	return tempReadByte;  
}

void LIS3DH::fifoEnd( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable

	//Turn off...
	readRegister( &dataToWrite, LIS3DH_FIFO_CTRL_REG ); //Start with existing data
	dataToWrite &= 0x3F;//clear mode
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_CTRL_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_FIFO_CTRL_REG, dataToWrite);	
}

