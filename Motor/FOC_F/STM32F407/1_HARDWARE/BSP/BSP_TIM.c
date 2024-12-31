/**************************************************************************************************
*     File Name :                        BSP_TIM.c
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             TIM��ʼ����Ӧ�ò�ӿ�Դ�ļ�
**************************************************************************************************/
#include "BSP_TIM.h"

/**********************************************************************************************
Function: BSP_TIM_Init
Description: HALL����ʱ���������ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_TIM_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
    
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4; 
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
    
    TIM_Cmd(TIM2,ENABLE);
}
