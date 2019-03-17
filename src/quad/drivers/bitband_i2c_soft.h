#ifndef __BITBAND_I2C_SOFT_H
#define __BITBAND_I2C_SOFT_H

#include <stdint.h>
#include "stm32f4xx.h"

//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) 			((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  				*((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   		MEM_ADDR(BITBAND(addr, bitnum)) 

#define GPIOB_ODR_Addr    (GPIOB_BASE+20) 			//0x40020414
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) 			//0x40020410

//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PBout(n)   			BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    			BIT_ADDR(GPIOB_IDR_Addr,n)  //����

/* configuration of IO direction */
#define SDA_IN()  { GPIOB->MODER &= ~(3<<(9*2)); GPIOB->MODER |= 0<<9*2; }	// Input mode of PB9
#define SDA_OUT() { GPIOB->MODER &= ~(3<<(9*2)); GPIOB->MODER |= 1<<9*2;} 	// Output mode of PB9

/* IO operations */
#define IIC_SCL			PBout(8)			// SCL
#define IIC_SDA			PBout(9)			// SDA
#define READ_SDA		PBin(9)				// SDA input

//IIC���в�������
void IIC_Init(void);                		//��ʼ��IIC��IO��				 
void IIC_Start(void);						//����IIC��ʼ�ź�
void IIC_Stop(void);	  					//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char ack);	//IIC��ȡһ���ֽ�
uint8_t IIC_Read_Byte2(void);
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);							//IIC����ACK�ź�
void IIC_NAck(void);						//IIC������ACK�ź�

//void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr, uint8_t data);
//uint8_t IIC_Read_One_Byte(uint8_t daddr, uint8_t addr);

#endif	// __BITBAND_I2C_SOFT_H
