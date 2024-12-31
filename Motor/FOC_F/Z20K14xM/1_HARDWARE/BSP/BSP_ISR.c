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
    INT_SetPriority(STIM_IRQn, 2);
    INT_SetPriority(DMA_Ch0_IRQn, 1);
    INT_SetPriority(MCPWM1_Fault_IRQn, 0);
    INT_EnableIRQ(STIM_IRQn);
    INT_EnableIRQ(DMA_Ch0_IRQn);
    INT_EnableIRQ(MCPWM1_Fault_IRQn);
}
