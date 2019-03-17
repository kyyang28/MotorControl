#ifndef __MPU6050_SOFT_I2C_H
#define __MPU6050_SOFT_I2C_H

#include <stdint.h>

//#define MPU_ACCEL_OFFS_REG		0x06	//accel_offs�Ĵ���,�ɶ�ȡ�汾��,�Ĵ����ֲ�δ�ᵽ
//#define MPU_PROD_ID_REG			0x0C	//prod id�Ĵ���,�ڼĴ����ֲ�δ�ᵽ
#define MPU_SELF_TESTX_REG		0x0D	//�Լ�Ĵ���X
#define MPU_SELF_TESTY_REG		0x0E	//�Լ�Ĵ���Y
#define MPU_SELF_TESTZ_REG		0x0F	//�Լ�Ĵ���Z
#define MPU_SELF_TESTA_REG		0x10	//�Լ�Ĵ���A
#define MPU_SAMPLE_RATE_REG		0x19	//����Ƶ�ʷ�Ƶ��
#define MPU_CFG_REG				0x1A	//���üĴ���
#define MPU_GYRO_CFG_REG		0x1B	//���������üĴ���
#define MPU_ACCEL_CFG_REG		0x1C	//���ٶȼ����üĴ���
#define MPU_MOTION_DET_REG		0x1F	//�˶���ֵⷧ���üĴ���
#define MPU_FIFO_EN_REG			0x23	//FIFOʹ�ܼĴ���
#define MPU_I2CMST_CTRL_REG		0x24	//IIC�������ƼĴ���
#define MPU_I2CSLV0_ADDR_REG	0x25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU_I2CSLV0_REG			0x26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU_I2CSLV0_CTRL_REG	0x27	//IIC�ӻ�0���ƼĴ���
#define MPU_I2CSLV1_ADDR_REG	0x28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU_I2CSLV1_REG			0x29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU_I2CSLV1_CTRL_REG	0x2A	//IIC�ӻ�1���ƼĴ���
#define MPU_I2CSLV2_ADDR_REG	0x2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU_I2CSLV2_REG			0x2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU_I2CSLV2_CTRL_REG	0x2D	//IIC�ӻ�2���ƼĴ���
#define MPU_I2CSLV3_ADDR_REG	0x2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU_I2CSLV3_REG			0x2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU_I2CSLV3_CTRL_REG	0x30	//IIC�ӻ�3���ƼĴ���
#define MPU_I2CSLV4_ADDR_REG	0x31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU_I2CSLV4_REG			0x32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU_I2CSLV4_DO_REG		0x33	//IIC�ӻ�4д���ݼĴ���
#define MPU_I2CSLV4_CTRL_REG	0x34	//IIC�ӻ�4���ƼĴ���
#define MPU_I2CSLV4_DI_REG		0x35	//IIC�ӻ�4�����ݼĴ���

#define MPU_I2CMST_STA_REG		0x36	//IIC����״̬�Ĵ���
#define MPU_INTBP_CFG_REG		0x37	//�ж�/��·���üĴ���
#define MPU_INT_EN_REG			0x38	//�ж�ʹ�ܼĴ���
#define MPU_INT_STA_REG			0x3A	//�ж�״̬�Ĵ���

#define MPU_ACCEL_XOUTH_REG		0x3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_XOUTL_REG		0x3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_YOUTH_REG		0x3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_YOUTL_REG		0x3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_ZOUTH_REG		0x3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU_ACCEL_ZOUTL_REG		0x40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU_TEMP_OUTH_REG		0x41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU_TEMP_OUTL_REG		0x42	//�¶�ֵ��8λ�Ĵ���

#define MPU_GYRO_XOUTH_REG		0x43	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_XOUTL_REG		0x44	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_YOUTH_REG		0x45	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_YOUTL_REG		0x46	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_ZOUTH_REG		0x47	//������ֵ,Z���8λ�Ĵ���
#define MPU_GYRO_ZOUTL_REG		0x48	//������ֵ,Z���8λ�Ĵ���

#define MPU_I2CSLV0_DO_REG		0x63	//IIC�ӻ�0���ݼĴ���
#define MPU_I2CSLV1_DO_REG		0x64	//IIC�ӻ�1���ݼĴ���
#define MPU_I2CSLV2_DO_REG		0x65	//IIC�ӻ�2���ݼĴ���
#define MPU_I2CSLV3_DO_REG		0x66	//IIC�ӻ�3���ݼĴ���

#define MPU_I2CMST_DELAY_REG	0x67	//IIC������ʱ����Ĵ���
#define MPU_SIGPATH_RST_REG		0x68	//�ź�ͨ����λ�Ĵ���
#define MPU_MDETECT_CTRL_REG	0x69	//�˶������ƼĴ���
#define MPU_USER_CTRL_REG		0x6A	//�û����ƼĴ���
#define MPU_PWR_MGMT1_REG		0x6B	//��Դ����Ĵ���1
#define MPU_PWR_MGMT2_REG		0x6C	//��Դ����Ĵ���2 
#define MPU_FIFO_CNTH_REG		0x72	//FIFO�����Ĵ����߰�λ
#define MPU_FIFO_CNTL_REG		0x73	//FIFO�����Ĵ����Ͱ�λ
#define MPU_FIFO_RW_REG			0x74	//FIFO��д�Ĵ���
#define MPU_DEVICE_ID_REG		0x75	//����ID�Ĵ���
 
//���AD0��(9��)�ӵ�,IIC��ַΪ0x68(���������λ).
//�����V3.3,��IIC��ַΪ0x69(���������λ).
#define MPU_ADDR_AD0			0x68
#define MPU_ADDR_AD1			0x69

uint8_t MPU6050_Set_Gyro_FSR(uint8_t fsr);
uint8_t MPU6050_Set_Accel_FSR(uint8_t fsr);
uint8_t MPU6050_Set_LPF(uint16_t lpf);
uint8_t MPU6050_Set_Rate(uint16_t rate);

int MPU6050_Soft_I2C_Init(void);
uint8_t MPU6050_Get_Gyroscope_Data(short *gx, short *gy, short *gz);
uint8_t MPU6050_Get_Accelerometer_Data(short *ax, short *ay, short *az);
float MPU6050_Get_Temperature_Data(void);

void mpu6050_send_data(short accx, short accy, short accz, short gyrox, short gyroy, short gyroz);
void usart3_report_imu(short accx, short accy, short accz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw);

#endif	// __MPU6050_SOFT_I2C_H
