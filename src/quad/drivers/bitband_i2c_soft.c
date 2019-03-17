
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "bitband_i2c_soft.h"
#include "system.h"

void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// enable GPIOB clock
	
	/* Initialise GPIOB 8 and 9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			// Open-drain
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// Push-pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;						// Low speed
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		// Just for testing
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	IIC_SCL = 1;		// PB8
	IIC_SDA = 1;		// PB9
}

void IIC_Start(void)
{
	SDA_OUT();			// SDA output mode
	IIC_SDA = 1;
	IIC_SCL = 1;
//	delay(500);			// For testing using oscilloscope
	delayMicroseconds(4);
	IIC_SDA = 0;		// START: when CLK line is high, DATA line changes from high to low
//	delay(500);			// For testing using oscilloscope
	delayMicroseconds(4);
	IIC_SCL = 0;
}

void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL = 0;
	IIC_SDA = 0;		// STOP: when CLK line is high, DATA line changes from low to high
//	delay(500);			// For testing using oscilloscope
	delayMicroseconds(4);
	IIC_SCL = 1;
	IIC_SDA = 1;		// Send I2C terminal signal
//	delay(500);			// For testing using oscilloscope
	delayMicroseconds(4);
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
	SDA_IN();			// SDA����Ϊ����
	IIC_SDA = 1;
	delayMicroseconds(1);
	IIC_SCL = 1;
	delayMicroseconds(1);
	while (READ_SDA) {
		ucErrTime++;
		if (ucErrTime > 250) {
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL = 0;		// ʱ�����0
	return 0;
}

void IIC_Ack(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 0;
//	delay(200);			// For testing using oscilloscope
	delayMicroseconds(2);
	IIC_SCL = 1;
//	delay(200);			// For testing using oscilloscope
	delayMicroseconds(2);
	IIC_SCL = 0;
}

void IIC_NAck(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 1;
//	delay(200);			// For testing using oscilloscope
	delayMicroseconds(2);
	IIC_SCL = 1;
//	delay(200);			// For testing using oscilloscope
	delayMicroseconds(2);
	IIC_SCL = 0;
}

// IIC����һ���ֽ�
// ���شӻ�����Ӧ��
// 1����Ӧ��
// 0����Ӧ��		
void IIC_Send_Byte(uint8_t txd)
{
	uint8_t i;
	SDA_OUT();

	/* All SDA changes should take place when SCL is low, with the exception of start and stop conditions. */
	IIC_SCL = 0;		// ����ʱ�ӿ�ʼ���ݴ���

	for (i = 0; i < 8; i++) {
		IIC_SDA = (txd & 0x80) >> 7;
		txd <<= 1;
		delayMicroseconds(2);
		IIC_SCL = 1;
		delayMicroseconds(2);
		IIC_SCL = 0;
		delayMicroseconds(2);
	}
}

// ��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN();			// SDA����Ϊ����
	for (i = 0; i < 8; i++) {
		IIC_SCL = 0;
		delayMicroseconds(2);
		IIC_SCL = 1;
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delayMicroseconds(1);
	}
	if (!ack) {
		IIC_NAck();		// ����NAck
	}else {
		IIC_Ack();		// ����Ack
	}
	
	return receive;
}

uint8_t IIC_Read_Byte2(void)
{
	unsigned char i, receive = 0;
	SDA_IN();			// SDA����Ϊ����
	for (i = 0; i < 8; i++) {
		IIC_SCL = 0;
		delayMicroseconds(2);
		IIC_SCL = 1;
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delayMicroseconds(1);
	}
	
	return receive;
}

//void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr, uint8_t data)
//{
//	
//}

//uint8_t IIC_Read_One_Byte(uint8_t daddr, uint8_t addr)
//{
//	return 0;
//}
