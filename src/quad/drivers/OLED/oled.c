#include "oled.h"
//#include "stdlib.h"
#include "oledfont.h"  	 
#include "system.h"
#include "configMaster.h"

/* OLED IO Definition */
#define OLED_RST_Clr() 			IOLo(IOGetByTag(OLEDConfig()->RST))   		// RST
#define OLED_RST_Set() 			IOHi(IOGetByTag(OLEDConfig()->RST))   		// RST
#define OLED_RS_Clr() 			IOLo(IOGetByTag(OLEDConfig()->DC))    		// DC
#define OLED_RS_Set() 			IOHi(IOGetByTag(OLEDConfig()->DC))    		// DC
#define OLED_SCLK_Clr()  		IOLo(IOGetByTag(OLEDConfig()->SCL))  		// SCL
#define OLED_SCLK_Set()  		IOHi(IOGetByTag(OLEDConfig()->SCL))   		// SCL
#define OLED_SDIN_Clr()  		IOLo(IOGetByTag(OLEDConfig()->SDA))   		// SDA
#define OLED_SDIN_Set()  		IOHi(IOGetByTag(OLEDConfig()->SDA))   		// SDA

//#define OLED_RST_Clr() 		PCout(13)=0   		// RST
//#define OLED_RST_Set() 		PCout(13)=1   		// RST
//#define OLED_RS_Clr() 		PBout(4)=0    		// DC
//#define OLED_RS_Set() 		PBout(4)=1    		// DC
//#define OLED_SCLK_Clr()  		PCout(15)=0  		// SCL
//#define OLED_SCLK_Set()  		PCout(15)=1   		// SCL
//#define OLED_SDIN_Clr()  		PCout(14)=0   		// SDA
//#define OLED_SDIN_Set()  		PCout(14)=1   		// SDA

#define OLED_CMD  				0					// Write command
#define OLED_DATA 				1					// Write data

static IO_t g_OLED_RST;
static IO_t g_OLED_DC;
static IO_t g_OLED_SCL;
static IO_t g_OLED_SDA;

uint8_t OLED_GRAM[128][8];	 

void OLED_Refresh_Gram(void)
{
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

//��OLEDд��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{	
	uint8_t i;			  
	if(cmd)
	  OLED_RS_Set();
	else 
	  OLED_RS_Clr();		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_RS_Set();   	  
} 

	  	  
//����OLED��ʾ    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{  
	uint8_t i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//������ʾ
}
//���� 
//x:0~127
//y:0~63
//t:1 ��� 0,���				   
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t pos,bx,temp=0;
	if(x>127||y>63)return;//������Χ��.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/12 
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{      			    
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' ';//�õ�ƫ�ƺ��ֵ				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];  //����1206����
		else temp=oled_asc2_1608[chr][t];		 //����1608���� 	                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
//m^n����
uint8_t oled_pow(uint8_t m,uint8_t n)
{
	uint8_t result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 		  
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
//��ʾ�ַ���
//x,y:�������  
//*p:�ַ�����ʼ��ַ
//��16����
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	   
//��ʼ��OLED					    
void oledInit(oledConfig_t *oledConfig)
{ 	
	g_OLED_RST = IOGetByTag(oledConfig->RST);		// PC13
	g_OLED_DC = IOGetByTag(oledConfig->DC);			// PB4
	g_OLED_SCL = IOGetByTag(oledConfig->SCL);		// PC15
	g_OLED_SDA = IOGetByTag(oledConfig->SDA);		// PC14

	IOInit(g_OLED_RST, OWNER_OLED, 0);
	IOInit(g_OLED_DC, OWNER_OLED, 1);
	IOInit(g_OLED_SCL, OWNER_OLED, 2);
	IOInit(g_OLED_SDA, OWNER_OLED, 3);

	IOConfigGPIO(g_OLED_RST, IOCFG_OUT_PP);
	IOConfigGPIO(g_OLED_DC, IOCFG_OUT_PP);
	IOConfigGPIO(g_OLED_SCL, IOCFG_OUT_PP);
	IOConfigGPIO(g_OLED_SDA, IOCFG_OUT_PP);
	
// 	RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��
//	GPIOB->CRL&=0XFFF0FFFF;
//	GPIOB->CRL|=0X00020000;//PB4 �������   

//	RCC->APB2ENR|=1<<4;     	//ʹ��PORTCʱ��  
//	RCC->APB2ENR|=1<<0;     	//ʹ��AFIOʱ��	
//	GPIOC->CRH&=0X000FFFFF;	//PC13,14,15���ó���� 2MHz �������   
//	GPIOC->CRH|=0X22200000; 
	PWR->CR |= 1<<8;	//
	RCC->BDCR &= 0xFFFFFFFE;	//�ⲿ���������ر� PC14��PC15��Ϊ��ͨIO	 	
//	BKP->CR&=0xFFFFFFFE; 	//������TAMPER������Ϊͨ��IO��ʹ�� 
	PWR->CR &= 0xFFFFFEFF;	//ȡ��������д���� 

	OLED_RST_Clr();
	delay(100);				// delay 100 ms
	OLED_RST_Set(); 
					  
	OLED_WR_Byte(0xAE,OLED_CMD); //�ر���ʾ
	OLED_WR_Byte(0xD5,OLED_CMD); //����ʱ�ӷ�Ƶ����,��Ƶ��
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],��Ƶ����;[7:4],��Ƶ��
	OLED_WR_Byte(0xA8,OLED_CMD); //��������·��
	OLED_WR_Byte(0X3F,OLED_CMD); //Ĭ��0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //������ʾƫ��
	OLED_WR_Byte(0X00,OLED_CMD); //Ĭ��Ϊ0

	OLED_WR_Byte(0x40,OLED_CMD); //������ʾ��ʼ�� [5:0],����.
													    
	OLED_WR_Byte(0x8D,OLED_CMD); //��ɱ�����
	OLED_WR_Byte(0x14,OLED_CMD); //bit2������/�ر�
	OLED_WR_Byte(0x20,OLED_CMD); //�����ڴ��ַģʽ
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
	OLED_WR_Byte(0xA1,OLED_CMD); //���ض�������,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
	OLED_WR_Byte(0xDA,OLED_CMD); //����COMӲ����������
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]����
		 
	OLED_WR_Byte(0x81,OLED_CMD); //�Աȶ�����
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;Ĭ��0X7F (��������,Խ��Խ��)
	OLED_WR_Byte(0xD9,OLED_CMD); //����Ԥ�������
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //����VCOMH ��ѹ����
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
	OLED_WR_Byte(0xA6,OLED_CMD); //������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //������ʾ	 
	
	OLED_Clear();
}  




