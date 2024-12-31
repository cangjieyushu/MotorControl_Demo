/**************************************************************************************************
*     File Name :                        BSP_ADC.h
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ADC��ʼ����Ӧ�ò�ӿ�ͷ�ļ�
**************************************************************************************************/
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "MotorHal_cfg.h"

#define BSP_ADC_READ_DATA_VBUS              (ADC1->JDR4)
#define BSP_ADC_READ_DATA_TEMP              (0U)
#define BSP_ADC_READ_DATA_VR                (4095U)
#define BSP_ADC_READ_DATA_VBG               (0U)

/**********************************************************************************************
Function: BSP_ADC_Init
Description: ���������ADC��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_ADC_Init(void);
void BSP_DMA_Init(void);
	
#endif /* BSP_ADC_H */
