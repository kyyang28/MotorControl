
#include <stdio.h>
#include <string.h>
#include "configMaster.h"
#include "stm32f4xx.h"
#include "rcc.h"
#include "dma.h"
#include "target.h"
#include "adc_impl.h"			// including adc.h
#include "resource.h"
#include "io_def_generated.h"

#ifdef USE_ADC

#ifndef ADC_INSTANCE
#define ADC_INSTANCE			ADC1
#endif

#ifndef ADC1_DMA_STREAM
#define ADC1_DMA_STREAM			DMA2_Stream4
#endif

const adcDevice_t adcHardware[] = {
	{ .ADCx = ADC1, .rccAdc = RCC_APB2(ADC1), .DMAy_Streamx = ADC1_DMA_STREAM, .channel = DMA_Channel_0 },
};

const adcTagMap_t adcTagMap[] = {
	{ DEFIO_TAG_E__PC0, ADC_Channel_10 },
	{ DEFIO_TAG_E__PC1, ADC_Channel_11 },
	{ DEFIO_TAG_E__PC2, ADC_Channel_12 },
	{ DEFIO_TAG_E__PC3, ADC_Channel_13 },
	{ DEFIO_TAG_E__PC4, ADC_Channel_14 },
	{ DEFIO_TAG_E__PC5, ADC_Channel_15 },
	{ DEFIO_TAG_E__PB0, ADC_Channel_8 },
	{ DEFIO_TAG_E__PB1, ADC_Channel_9 },
	{ DEFIO_TAG_E__PA0, ADC_Channel_0 },
	{ DEFIO_TAG_E__PA1, ADC_Channel_1 },
	{ DEFIO_TAG_E__PA2, ADC_Channel_2 },
	{ DEFIO_TAG_E__PA3, ADC_Channel_3 },
	{ DEFIO_TAG_E__PA4, ADC_Channel_4 },
	{ DEFIO_TAG_E__PA5, ADC_Channel_5 },
	{ DEFIO_TAG_E__PA6, ADC_Channel_6 },
	{ DEFIO_TAG_E__PA7, ADC_Channel_7 },
};

AdcDevice adcDeviceByInstance(ADC_TypeDef *instance)
{
	if (instance == ADC1)
		return ADCDEV_1;
	
	if (instance == ADC2)
		return ADCDEV_2;
	
	if (instance == ADC3)
		return ADCDEV_3;
	
	return ADCINVALID;
}

void adcInit(adcConfig_t *adcConfig)
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	uint8_t i;
	uint8_t configuredAdcChannels = 0;
	
	memset(&adcOperationConfig, 0, sizeof(adcOperationConfig));
	
	if (adcConfig->motorCurrentMeter1.enabled) {
		adcOperationConfig[ADC_MOTOR_CURRENT1].tag = adcConfig->motorCurrentMeter1.ioTag;
	}
	
	if (adcConfig->motorCurrentMeter2.enabled) {
		adcOperationConfig[ADC_MOTOR_CURRENT2].tag = adcConfig->motorCurrentMeter2.ioTag;
	}
	
	AdcDevice device = adcDeviceByInstance(ADC_INSTANCE);		// ADC_INSTANCE = ADC1
	if (device == ADCINVALID)
		return;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	adcDevice_t adcDevice = adcHardware[device];
	
	bool adcActive = false;
	for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
		if (!adcOperationConfig[i].tag)
			continue;
		
		adcActive = true;
		IOInit(IOGetByTag(adcOperationConfig[i].tag), OWNER_ADC_MOTOR_CURRENT1 + i, 0);
		IOConfigGPIO(IOGetByTag(adcOperationConfig[i].tag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
		adcOperationConfig[i].adcChannel 	= adcChannelByTag(adcOperationConfig[i].tag);
		adcOperationConfig[i].dmaIndex 		= configuredAdcChannels++;
		adcOperationConfig[i].sampleTime 	= ADC_SampleTime_28Cycles;
//		adcOperationConfig[i].sampleTime 	= ADC_SampleTime_480Cycles;
		adcOperationConfig[i].enabled 		= true;
	}
	
//	printf("adcChannel[%d]: %u\r\n", 0, adcOperationConfig[0].adcChannel);		// PC1 -- ADC_Channel_11
//	printf("dmaIndex[%d]: %u\r\n", 0, adcOperationConfig[0].dmaIndex);			// dmaIndex = 0
//	printf("sampleTime[%d]: %u\r\n", 0, adcOperationConfig[0].sampleTime);		// sampleTime = ADC_SampleTime_480Cycles (0x07)
//	printf("enabled[%d]: %u\r\n", 0, adcOperationConfig[0].enabled);			// enabled = true
//	printf("adcChannel[%d]: %u\r\n", 1, adcOperationConfig[1].adcChannel);		// PC2 -- ADC_Channel_12
//	printf("dmaIndex[%d]: %u\r\n", 1, adcOperationConfig[1].dmaIndex);			// dmaIndex = 1
//	printf("sampleTime[%d]: %u\r\n", 1, adcOperationConfig[1].sampleTime);		// sampleTime = ADC_SampleTime_480Cycles (0x07)
//	printf("enabled[%d]: %u\r\n", 1, adcOperationConfig[1].enabled);			// enabled = true
	
	if (!adcActive) {
		return;
	}
	
	/* Enable ADC1 clock */
	RCC_ClockCmd(adcDevice.rccAdc, ENABLE);
	
	/* DMA initialisation
	 * 
	 * adcDevice.DMAy_Streamx = DMA2_Stream4
	 */
	dmaInit(dmaGetIdentifier(adcDevice.DMAy_Streamx), OWNER_ADC, 0);
	
	DMA_DeInit(adcDevice.DMAy_Streamx);
	
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr 							= (uint32_t)&adcDevice.ADCx->DR;
	DMA_InitStructure.DMA_Channel 										= adcDevice.channel;
	DMA_InitStructure.DMA_Memory0BaseAddr 								= (uint32_t)adcValues;
	DMA_InitStructure.DMA_DIR 											= DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize 									= configuredAdcChannels;
	DMA_InitStructure.DMA_PeripheralInc 								= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 									= configuredAdcChannels > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize 							= DMA_PeripheralDataSize_HalfWord;			// 16-bit based on the size of ADC->DR register (16 bits)
	DMA_InitStructure.DMA_MemoryDataSize 								= DMA_MemoryDataSize_HalfWord;					// 16-bit based on the size of ADC->DR register (16 bits)
	DMA_InitStructure.DMA_Mode 											= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority 										= DMA_Priority_High;
	
	/* DMA InitStructure init
	 *
	 * adcDevice.DMAy_Streamx = DMA2_Stream4
	 */
	DMA_Init(adcDevice.DMAy_Streamx, &DMA_InitStructure);
	
	/* Enable DMA
	 *
	 * adcDevice.DMAy_Streamx = DMA2_Stream4
	 */
	DMA_Cmd(adcDevice.DMAy_Streamx, ENABLE);
	
	/*
	 * HCLK   = SYSCLK / 1 = 168 MHz
	 * PCLK2  = HCLK / 2 = 84 MHz
	 * ADCCLK = PCLK2 / 4 = 21 MHz 
	 * ADC Sampling Rate = Sampling Time + Conversion Time = 480 + 12 cycles (fixed) = 492 cycle
	 * Conversion Time = 21 MHz / 492 cycle = 42.68 ksample/sec
	 * ADC Sampling Rate = Sampling Time + Conversion Time = 144 + 12 cycles (fixed) = 156 cycle
	 * Conversion Time = 21 MHz / 156 cycle = 134.6 ksample/s. 
	*/
	
	/* ADC CommonInitTypeDef init */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	ADC_CommonStructInit(&ADC_CommonInitStructure);
	
	/* ADCCLK = 21 MHz, ADC_SampleTime_3Cycles = 3 cycles, Conversion sample = 21 MHz / (3 + 12) cycles = 21000 / 15 = 1400 ksamples/sec
	 * 
	 * Conversion time = (3 + 12) cycles / 21 MHz = 0.714 microseconds
	 */
	ADC_CommonInitStructure.ADC_Mode 				= ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler 			= ADC_Prescaler_Div4;	// SystemCoreClock / 2 = 84 MHz = PCLK2 / 4 = ADCCLK = 84 MHz / 4 = 21 MHz
//	ADC_CommonInitStructure.ADC_Prescaler 			= ADC_Prescaler_Div8;	// SystemCoreClock / 2 = 84 MHz = PCLK2 / 8 = ADCCLK = 84 MHz / 8 = 10.5 MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode 		= ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay 	= ADC_TwoSamplingDelay_5Cycles;
	
	/* ADC Common Initialisation */
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	/* ADC Structure Init */
	ADC_StructInit(&ADC_InitStructure);
	
	ADC_InitStructure.ADC_ContinuousConvMode 		= ENABLE;
	ADC_InitStructure.ADC_Resolution 				= ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConv 			= ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge		= ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign					= ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion			= configuredAdcChannels;
	ADC_InitStructure.ADC_ScanConvMode				= configuredAdcChannels > 1 ? ENABLE : DISABLE;	// When enabled, scan more than one channel in group

	/* ADC initialisation */
	ADC_Init(adcDevice.ADCx, &ADC_InitStructure);
	
	uint8_t rank = 1;
	for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
		if (!adcOperationConfig[i].enabled)
			continue;
		
		ADC_RegularChannelConfig(adcDevice.ADCx, adcOperationConfig[i].adcChannel, rank++, adcOperationConfig[i].sampleTime);
	}
	
	/* Enable new DMA request after last transfer (circular mode) */
	ADC_DMARequestAfterLastTransferCmd(adcDevice.ADCx, ENABLE);
	
	/* Enable ADC DMA */
	ADC_DMACmd(adcDevice.ADCx, ENABLE);
	
	/* Enable ADC */
	ADC_Cmd(adcDevice.ADCx, ENABLE);
	
	/* Start ADC Conversion, set by software, clear by hardware as soon as the conversion is started */
	ADC_SoftwareStartConv(adcDevice.ADCx);
}

#endif
