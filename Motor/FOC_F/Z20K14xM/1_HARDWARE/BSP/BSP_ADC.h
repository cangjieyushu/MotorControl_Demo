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

#define ADC_DATA_NUM_S          (3)     //Ӧ�ò�ADC����ͨ������

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

/**********************************************************************************************
Function: BSP_ADC_Init_S
Description: Ӧ�ò�ADC��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_ADC_Init_S(void);

#endif /* BSP_ADC_H */
