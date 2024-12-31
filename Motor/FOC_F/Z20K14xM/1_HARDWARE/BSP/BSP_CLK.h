/**************************************************************************************************
*     File Name :                        BSP_CLK.h
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ʱ�ӳ�ʼ����Ӧ�ò�ӿ�ͷ�ļ�
**************************************************************************************************/
#ifndef BSP_CLK_H
#define BSP_CLK_H

#include "MotorHal_cfg.h"

#define HAL_M1PWM_CLK_MODULE            CLK_MCPWM0
#define HAL_M1PWM_SYSCTRL_MODULE        SYSCTRL_MCPWM0
#define HAL_M2PWM_CLK_MODULE            CLK_MCPWM1
#define HAL_M2PWM_SYSCTRL_MODULE        SYSCTRL_MCPWM1

/**********************************************************************************************
Function: BSP_CLK_Init
Description: ʱ�ӳ�ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_CLK_Init(void);

#endif /* BSP_CLK_H */
