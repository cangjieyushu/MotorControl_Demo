/**************************************************************************************************
*     File Name :                        BSP_WDG.h
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             WDG��ʼ����Ӧ�ò�ӿ�ͷ�ļ�
**************************************************************************************************/
#ifndef BSP_WDG_H
#define BSP_WDG_H

#include "MotorHal_cfg.h"

#define IWDG_TIM            1000  //ms
#define IWDGPrescaler       IWDG_PRESCALER_16
#define ReloadCounter       (32000*IWDG_TIM/(4<<IWDGPrescaler)/1000)

/**********************************************************************************************
Function: BSP_WDG_Init
Description: WDG��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_WDG_Init(void);

#endif /* BSP_WDG_H */
