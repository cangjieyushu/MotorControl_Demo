/**************************************************************************************************
*     File Name :                        BSP_ISR.c
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             �ж����ȼ���ʼ��
**************************************************************************************************/
#include "BSP_ISR.h"

/**********************************************************************************************
Function: BSP_ISR_Init
Description: �ж����ȼ���ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_ISR_Init(void)
{
    NVIC_SetPriority(SysTick_IRQn, 5);
    
	NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
