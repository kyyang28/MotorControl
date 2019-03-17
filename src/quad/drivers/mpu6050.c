
#include "mpu6050.h"
#include "bitband_i2c_soft.h"
#include "system.h"
#include "stm32f4xx_usart.h"
#include "rxSerial3Test.h"
#include <stdio.h>				// debugging purposes

// IICдһ���ֽ� 

// reg:�Ĵ�����ַ

// data:����

// ����ֵ:0,����

// ����,�������
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR_AD0 << 1) | 0);			// �����豸��ַ + д����
	
	if (IIC_Wait_Ack()) {						// �ȴ�Ӧ��
		IIC_Stop();								// I2C����ֹͣ
		return 1;
	}
	
	IIC_Send_Byte(reg);							// д�Ĵ�����ַ
	IIC_Wait_Ack();								// �ȴ�Ӧ��
	IIC_Send_Byte(data);						// ��������
	
	if (IIC_Wait_Ack()) {						// �ȴ�Ack
		IIC_Stop();								// I2C����ֹͣ
		return 1;
	}
	
	IIC_Stop();									// I2C����ֹͣ
	return 0;
}

//IIC��һ���ֽ� 

//reg:�Ĵ�����ַ 

//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
	
	IIC_Start();							// ��ʼIIC����
	
	IIC_Send_Byte((MPU_ADDR_AD0 << 1) | 0);		// �����豸��ַ + д����
	
	IIC_Wait_Ack();							// �ȴ�Ӧ��
	
	IIC_Send_Byte(reg);						// ���ͼĴ�����ַ
	
	IIC_Wait_Ack();							// �ȴ�Ӧ��
	
	IIC_Start();							// �ٴο�ʼIIC����
	
	IIC_Send_Byte((MPU_ADDR_AD0 << 1) | 1);		// �����豸��ַ + ������
	
	IIC_Wait_Ack();							// �ȴ�Ӧ��
	
	res = IIC_Read_Byte(0);					// ��ȡ���ݣ� ����nAck
		
	IIC_Stop();								// ����һ��ֹͣ����
	
	return res;
}

//����MPU6050�����Ǵ����������̷�Χ

//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps

//����ֵ:0,���óɹ�

//    ����,����ʧ�� 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);		// ���������������̷�Χ
}

//����MPU6050���ٶȴ����������̷�Χ

//fsr:0,��2g;1,��4g;2,��8g;3,��16g

//����ֵ:0,���óɹ�

//    ����,����ʧ�� 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);		// ���ü��ٶȼƴ����������̷�Χ
}

//����MPU6050�����ֵ�ͨ�˲���

//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)

//����ֵ:0,���óɹ�

//    ����,����ʧ�� 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	
	if (lpf >= 188) {
		data = 1;
	}else if (lpf >= 98) {
		data = 2;
	}else if (lpf >= 42) {
		data = 3;
	}else if (lpf >= 20) {
		data = 4;
	}else if (lpf >= 10) {
		data = 5;
	}else {
		data = 6;
	}
	
	return MPU_Write_Byte(MPU_CFG_REG, data);				// �������ֵ�ͨ�˲���
}

//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)

//rate:4~1000(Hz)

//����ֵ:0,���óɹ�

//    ����,����ʧ�� 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	
	if (rate > 1000) {
		rate = 1000;
	}
	
	if (rate < 4) {
		rate = 4;
	}
	
	data = 1000 / rate - 1;
	
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);		// �������ֵ�ͨ�˲���
	
	return MPU_Set_LPF(rate / 2);							// �Զ�����LPFΪ�����ʵ�һ��
}

uint8_t MPU_Set_Fifo(uint8_t sens)
{
	return 0;
}

//IIC����д (Burst write)
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);			// send device address + write cmd
	
	if (IIC_Wait_Ack()) {					// wait ack
		IIC_Stop();
		return 1;
	}
	
	IIC_Send_Byte(reg);						// send register address
	IIC_Wait_Ack();							// wait ack
	
	for (i = 0; i < len; i++) {
		IIC_Send_Byte(buf[i]);				// send data one at a time
		
		if (IIC_Wait_Ack()) {				// wait ack
			IIC_Stop();
			return 1;
		}
	}
	
	IIC_Stop();
	return 0;
}

//IIC������ (Burst read)
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	IIC_Start();							// start IIC
	IIC_Send_Byte((addr << 1) | 0);			// send device address + write cmd
	if (IIC_Wait_Ack()) {					// wait response
		IIC_Stop();
		return 1;
	}
	
	IIC_Send_Byte(reg);						// send register address
	IIC_Wait_Ack();							// wait ack
	
	IIC_Start();							// start IIC
	IIC_Send_Byte((addr << 1) | 1);			// send device address + read cmd
	IIC_Wait_Ack();							// wait ack
	
	while (len) {
		if (len == 1) {
			*buf = IIC_Read_Byte(0);		// read data, send nAck
		}else {
			*buf = IIC_Read_Byte(1);		// read data, send ack
		}
		
		len--;
		buf++;
	}
	
	IIC_Stop();								// generate a IIC stop signal
	return 0;
}

// ��ʼ��MPU6050
uint8_t MPU_Init(void)
{
	uint8_t resID;
	
//	IIC_Init();									// ��ʼ��IIC����
	
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80);	// ��λMPU6050
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 0x80);			// reset MPU6050
	
	delay(30);									// �ӳ�30 ms
//	delay(100);									// �ӳ�100 ms
	
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);	// ����MPU6050
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 0x00);			// wake up MPU6050
	
	MPU_Set_Gyro_Fsr(3);						// ���������Ǵ�����, ��2000dps
//	MPU6050_Set_Gyro_FSR(3);												// ��2000dps
	
	MPU_Set_Accel_Fsr(0);						// ���ü��ٶȼƴ�����, ��2g
//	MPU6050_Set_Accel_FSR(0);												// ��2g
		
	MPU_Set_Rate(50);							// ���ò�����50Hz
//	MPU6050_Set_Rate(50);													// sampling rate 50Hz
	
	MPU_Write_Byte(MPU_INT_EN_REG, 0x00);		// �ر������ж�
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_INT_EN_REG, 0x00);				// disable all interrupt
	
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0x00);	// I2C��ģʽ�ر�
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_USER_CTRL_REG, 0x00);			// disable I2C master mode
	
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0x00);		// �ر�FIFO
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_FIFO_EN_REG, 0x00);				// disable FIFO
	
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0x80);	// INT���ŵ͵�ƽ��Ч
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_INTBP_CFG_REG, 0x80);			// INT low voltage level
	
	resID = MPU_Read_Byte(MPU_DEVICE_ID_REG);
//	i2cRead(I2C_DEVICE, MPU_ADDR_AD0, MPU_DEVICE_ID_REG, 1, &resID);
	
	printf("MPU_ADDR_resID: 0x%x, %s, %d\r\n", resID, __FUNCTION__, __LINE__);
	
	if (resID == MPU_ADDR_AD0) {						// �豸ID��ȷ
		printf("MPU6050 is initialised successfully!, %s, %d\r\n", __FUNCTION__, __LINE__);
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x01);	// ����CLKSEL, PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0x00);	// ���ٶȼ��������Ƕ�����
		MPU_Set_Rate(50);							// ���ò�����50Hz
	}else {
		printf("Failed to initialise MPU6050!, %s, %d\r\n", __FUNCTION__, __LINE__);
		return 1;
	}
	
	return 0;
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ
float MPU_Get_Temperature(void)
//short MPU_Get_Temperature(void)
{
	uint8_t buf[2];
	short raw;
	float temp;
	
	MPU_Read_Len(MPU_ADDR_AD0, MPU_TEMP_OUTH_REG, 2, buf);
	
	raw = ((uint16_t)buf[0] << 8) | buf[1];
	temp = ((double)raw)/340 + 36.53;
	
	return temp;
//	return temp*100;
}

//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	uint8_t buf[6], res;
	
	res = MPU_Read_Len(MPU_ADDR_AD0, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0) {
		*gx = ((uint16_t)buf[0] << 8) | buf[1];
		*gy = ((uint16_t)buf[2] << 8) | buf[3];
		*gz = ((uint16_t)buf[4] << 8) | buf[5];
	}
	
	return res;
}

//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	uint8_t buf[6], res;
	
	res = MPU_Read_Len(MPU_ADDR_AD0, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0) {
		*ax = ((uint16_t)buf[0] << 8) | buf[1];
		*ay = ((uint16_t)buf[2] << 8) | buf[3];
		*az = ((uint16_t)buf[4] << 8) | buf[5];
	}
	
	return res;
}

#if 0
//����3����1���ַ� 
//c:Ҫ���͵��ַ�
void usart3_send_char(uint8_t c)
{
//	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {
//		USART_SendData(USART3, c);
//	}
	rxSerial3TestWrite(c);
}

//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart3_ano_report(uint8_t fun, uint8_t *data, uint8_t len)
{
	uint8_t send_buf[32];
	uint8_t i;
	
	if (len > 28)							// ���28�ֽ�����
		return;
	
	send_buf[len+3] = 0;					// У��������
	send_buf[0] = 0x88;						// ֡ͷ
	send_buf[1] = fun;						// ������
	send_buf[2] = len;						// ���ݳ���
	for (i = 0; i < len; i++) {
		send_buf[3+i] = data[i];			// ��������
	}
	
	for (i = 0; i < len+3; i++) {
		send_buf[len+3] += send_buf[i];		// ����У���
	}
	
	for (i = 0; i < len+4; i++) {
		usart3_send_char(send_buf[i]);		// �������ݵ�����3
	}
}

//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void mpu6050_send_data(short accx, short accy, short accz, short gyrox, short gyroy, short gyroz)
{
	uint8_t tbuf[12];
	uint8_t i;
	
	for (i = 0; i < 13; i++) {
		tbuf[i] = 0;
	}
	
	tbuf[0] = (accx >> 8) & 0xff;
	tbuf[1] = accx & 0xff;
	tbuf[2] = (accy >> 8) & 0xff;
	tbuf[3] = accy & 0xff;
	tbuf[4] = (accz >> 8) & 0xff;
	tbuf[5] = accz & 0xff;
	tbuf[6] = (gyrox >> 8) & 0xff;
	tbuf[7] = gyrox & 0xff;
	tbuf[8] = (gyroy >> 8) & 0xff;
	tbuf[9] = gyroy & 0xff;
	tbuf[10] = (gyroz >> 8) & 0xff;
	tbuf[11] = gyroz & 0xff;
	usart3_ano_report(0xA1, tbuf, 12);		// �Զ���֡,0xA1
}

//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart3_report_imu(short accx, short accy, short accz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
	uint8_t tbuf[28];
	uint8_t i;
	
	for (i = 0; i < 28; i++) {
		tbuf[i] = 0;
	}
	
	tbuf[0] = (accx >> 8) & 0xff;
	tbuf[1] = accx & 0xff;
	tbuf[2] = (accy >> 8) & 0xff;
	tbuf[3] = accy & 0xff;
	tbuf[4] = (accz >> 8) & 0xff;
	tbuf[5] = accz & 0xff;
	tbuf[6] = (gyrox >> 8) & 0xff;
	tbuf[7] = gyrox & 0xff;
	tbuf[8] = (gyroy >> 8) & 0xff;
	tbuf[9] = gyroy & 0xff;
	tbuf[10] = (gyroz >> 8) & 0xff;
	tbuf[11] = gyroz & 0xff;
	tbuf[18] = (roll >> 8) & 0xff;
	tbuf[19] = roll & 0xff;
	tbuf[20] = (pitch >> 8) & 0xff;
	tbuf[21] = pitch & 0xff;
	tbuf[22] = (yaw >> 8) & 0xff;
	tbuf[23] = yaw & 0xff;
	usart3_ano_report(0xAF, tbuf, 28);
}
#endif
