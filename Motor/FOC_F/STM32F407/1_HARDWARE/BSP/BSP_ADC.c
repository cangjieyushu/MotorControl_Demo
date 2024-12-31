/**************************************************************************************************
*     File Name :                        BSP_ADC.c
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ADC��ʼ����Ӧ�ò�ӿ�Դ�ļ�
**************************************************************************************************/
#include "BSP_ADC.h"

/**********************************************************************************************
Function: BSP_ADC_Init
Description: ���������ADC��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_ADC_Init(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	ADC_TempSensorVrefintCmd(ENABLE);/*ʹ���ڲ��¶ȴ�����*/

	/*ͨ�ÿ��ƼĴ���������*/
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             /*DMA��*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                            /*����ģʽ*/
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;                         /*APB2��2��Ƶ*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;        /*���������׶ε���ʱ5��ʱ��*/
	ADC_CommonInit(&ADC_CommonInitStructure);
    
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                      /*12λģʽ*/
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                /*ɨ��ģʽ*/
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                          /*����ת��*/
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; /*��ֹ������� ʹ���������*/
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                      /*�Ҷ���*/
	ADC_InitStructure.ADC_NbrOfConversion = 1;                                  /*ֻʹ��1ͨ�� ����ͨΪ1*/
	ADC_Init(ADC1,&ADC_InitStructure);
    
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_84Cycles);
    
    ADC_InjectedSequencerLengthConfig(ADC1, 4);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_15Cycles);
    
    ADC_InjectedDiscModeCmd(ADC1, DISABLE);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    
	ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConv(ADC1);
}

uint16_t ADC_INT_RAW_DATA[4] = {0,0,0,0};

void BSP_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	while(DMA_GetCmdStatus(DMA2_Stream0) != DISABLE);                               /*�ȴ�DMA��������*/
	 
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                                  /*DMAͨ��0*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0;                         /*�����ַ*/
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC_INT_RAW_DATA;             /*��ȡ����ַ*/
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         /*��������赽�ڴ�*/
	DMA_InitStructure.DMA_BufferSize = 1;                                           /*���ݴ��������Ϊ1*/
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                /*��ַ������*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         /*��ַ������*/
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;     /*���ݳ��Ȱ���*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;             /*���ݳ��Ȱ���*/
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                             /*�����ȼ�*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                                 /*ѭ��ģʽ*/
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          /*��ֹFIFO*/
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;               /*FIFO��ֵ*/
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     /*���δ���*/
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             /*���δ���*/
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);/**/
    
    DMA_Cmd(DMA2_Stream0, ENABLE);
}
