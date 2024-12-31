/**************************************************************************************************
*     File Name :                        BSP_TIM.h
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             TIM��ʼ����Ӧ�ò�ӿ�ͷ�ļ�
**************************************************************************************************/
#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "MotorHal_cfg.h"

/**********************************************************************************************
Function: BSP_TIM_Init
Description: 1ms�ж�TIM��ʼ��
Input: �ٶȻ��жϺ���
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_TIM_Init(isr_cb_t *StimIntCbf);

/**********************************************************************************************
Function: BSP_HALL_Init
Description: HALL����ʱ���������ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_HALL_Init(void);

#endif /* BSP_TIM_H */
