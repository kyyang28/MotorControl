#ifndef __ACCGYRO_MPU_H
#define __ACCGYRO_MPU_H

#include <stdint.h>
#include <stdbool.h>
#include "target.h"

#if defined(USE_GYRO_SPI_MPU9250)
#define GYRO_USES_SPI
#endif

// MPU6050
//#define MPU_RA_WHO_AM_I         	0x75
//#define MPU_RA_WHO_AM_I_LEGACY  	0x00

// RA = Register Address

#define MPU_RA_XG_OFFS_TC       	0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       	0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       	0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      	0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      	0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      	0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        	0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     	0x07
#define MPU_RA_YA_OFFS_H        	0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     	0x09
#define MPU_RA_ZA_OFFS_H        	0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     	0x0B
#define MPU_RA_PRODUCT_ID       	0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     	0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     	0x14
#define MPU_RA_YG_OFFS_USRH     	0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     	0x16
#define MPU_RA_ZG_OFFS_USRH     	0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     	0x18
#define MPU_RA_SMPLRT_DIV       	0x19
#define MPU_RA_CONFIG           	0x1A
#define MPU_RA_GYRO_CONFIG      	0x1B
#define MPU_RA_ACCEL_CONFIG     	0x1C
#define MPU_RA_FF_THR           	0x1D
#define MPU_RA_FF_DUR           	0x1E
#define MPU_RA_MOT_THR          	0x1F
#define MPU_RA_MOT_DUR          	0x20
#define MPU_RA_ZRMOT_THR        	0x21
#define MPU_RA_ZRMOT_DUR        	0x22
#define MPU_RA_FIFO_EN          	0x23
#define MPU_RA_I2C_MST_CTRL     	0x24
#define MPU_RA_I2C_SLV0_ADDR    	0x25
#define MPU_RA_I2C_SLV0_REG     	0x26
#define MPU_RA_I2C_SLV0_CTRL    	0x27
#define MPU_RA_I2C_SLV1_ADDR    	0x28
#define MPU_RA_I2C_SLV1_REG     	0x29
#define MPU_RA_I2C_SLV1_CTRL    	0x2A
#define MPU_RA_I2C_SLV2_ADDR    	0x2B
#define MPU_RA_I2C_SLV2_REG     	0x2C
#define MPU_RA_I2C_SLV2_CTRL    	0x2D
#define MPU_RA_I2C_SLV3_ADDR    	0x2E
#define MPU_RA_I2C_SLV3_REG     	0x2F
#define MPU_RA_I2C_SLV3_CTRL    	0x30
#define MPU_RA_I2C_SLV4_ADDR    	0x31
#define MPU_RA_I2C_SLV4_REG     	0x32
#define MPU_RA_I2C_SLV4_DO      	0x33
#define MPU_RA_I2C_SLV4_CTRL    	0x34
#define MPU_RA_I2C_SLV4_DI      	0x35
#define MPU_RA_I2C_MST_STATUS   	0x36
#define MPU_RA_INT_PIN_CFG      	0x37
#define MPU_RA_INT_ENABLE       	0x38
#define MPU_RA_DMP_INT_STATUS   	0x39
#define MPU_RA_INT_STATUS       	0x3A
#define MPU_RA_ACCEL_XOUT_H     	0x3B
#define MPU_RA_ACCEL_XOUT_L     	0x3C
#define MPU_RA_ACCEL_YOUT_H     	0x3D
#define MPU_RA_ACCEL_YOUT_L     	0x3E
#define MPU_RA_ACCEL_ZOUT_H     	0x3F
#define MPU_RA_ACCEL_ZOUT_L     	0x40
#define MPU_RA_TEMP_OUT_H       	0x41
#define MPU_RA_TEMP_OUT_L       	0x42
#define MPU_RA_GYRO_XOUT_H      	0x43
#define MPU_RA_GYRO_XOUT_L      	0x44
#define MPU_RA_GYRO_YOUT_H      	0x45
#define MPU_RA_GYRO_YOUT_L      	0x46
#define MPU_RA_GYRO_ZOUT_H      	0x47
#define MPU_RA_GYRO_ZOUT_L      	0x48
#define MPU_RA_EXT_SENS_DATA_00 	0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      	0x63
#define MPU_RA_I2C_SLV1_DO      	0x64
#define MPU_RA_I2C_SLV2_DO      	0x65
#define MPU_RA_I2C_SLV3_DO      	0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        	0x6A
#define MPU_RA_PWR_MGMT_1       	0x6B
#define MPU_RA_PWR_MGMT_2       	0x6C
#define MPU_RA_BANK_SEL         	0x6D
#define MPU_RA_MEM_START_ADDR   	0x6E
#define MPU_RA_MEM_R_W          	0x6F
#define MPU_RA_DMP_CFG_1        	0x70
#define MPU_RA_DMP_CFG_2        	0x71
#define MPU_RA_FIFO_COUNTH      	0x72
#define MPU_RA_FIFO_COUNTL      	0x73
#define MPU_RA_FIFO_R_W         	0x74
#define MPU_RA_WHO_AM_I         	0x75

typedef enum {
	MPU_NONE,					// 0
	MPU_60x0,					// 1
	MPU_9250_I2C,				// 2
	MPU_9250_SPI				// 3
}detectedMPUSensor_e;

typedef enum {
	MPU_HALF_RESOLUTION,
	MPU_FULL_RESOLUTION
}mpu6050Resolution_e;

typedef struct mpuDetectionResult_s {
	detectedMPUSensor_e sensor;
	mpu6050Resolution_e resolution;
}mpuDetectionResult_t;

typedef bool (*mpuReadRegisterFunc)(uint8_t reg, uint8_t length, uint8_t *data);
typedef bool (*mpuWriteRegisterFunc)(uint8_t reg, uint8_t data);
typedef void (*mpuResetFuncPtr)(void);

extern mpuResetFuncPtr mpuReset;

typedef struct mpuConfiguration_s {
	mpuReadRegisterFunc read;
	mpuWriteRegisterFunc write;
	mpuReadRegisterFunc slowRead;
	mpuWriteRegisterFunc verifyWrite;
	mpuResetFuncPtr reset;
	uint8_t gyroReadXRegister;			// Y and Z must registers follow this, 2 words each
	uint8_t temperatureReadRegister;	// temperature registers (MPU_RA_TEMP_OUT_H (0x41) and MPU_RA_TEMP_OUT_L (0x42))
}mpuConfiguration_t;

enum gyro_fsr_e {
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_GYRO_FSR
};

enum fchoice_b {
	FCB_DISABLED = 0,
	FCB_8800_32,
	FCB_3600_32
};

enum clock_sel_e {
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
};

enum accel_fsr_e {
	INV_FSR_2G = 0,			// 0
	INV_FSR_4G,				// 1
	INV_FSR_8G,				// 2
	INV_FSR_16G,			// 3
	NUM_ACCEL_FSR			// 4
};

struct gyroDev_s;
struct accDev_s;
void mpuGyroInit(struct gyroDev_s *gyro);
mpuDetectionResult_t *mpuDetect(struct gyroDev_s *gyro);
bool mpuTemperatureRead(struct gyroDev_s *gyro);
bool mpuGyroRead(struct gyroDev_s *gyro);
bool mpuAccRead(struct accDev_s *acc);
bool mpuCheckDataReady(struct gyroDev_s *gyro);

#endif	// __ACCGYRO_MPU_H
