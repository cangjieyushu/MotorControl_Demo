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
	NVIC_SetPriority(TIM8_BRK_UP_TRG_COM_IRQn, 0);	//�����ж����ȼ�
	NVIC_EnableIRQ(TIM8_BRK_UP_TRG_COM_IRQn);		//ʹ�� ɲ�� �ж�
	
	NVIC_SetPriority(TIM3_IRQn, 1);			        //�����ж����ȼ�
	NVIC_EnableIRQ(TIM3_IRQn);				        //ʹ�� �Ƚ� �ж�
    
	NVIC_SetPriority(ADC_IRQn, 1);			        //�����ж����ȼ�
	NVIC_EnableIRQ(ADC_IRQn);				        //ʹ�� ADC �ж�
    
	NVIC_SetPriority(TIM8_CC_IRQn, 2);	            //�����ж����ȼ�
	NVIC_EnableIRQ(TIM8_CC_IRQn);		            //ʹ�� ���� �ж�

	NVIC_SetPriority(UART1_IRQn, 4);			    //�����ж����ȼ�
	NVIC_EnableIRQ(UART1_IRQn);				        //ʹ�� ���� �ж�
	NVIC_SetPriority(UART2_IRQn, 4);			    //�����ж����ȼ�
	NVIC_EnableIRQ(UART2_IRQn);				        //ʹ�� ���� �ж�
    
	NVIC_SetPriority(SysTick_IRQn, 5);			    //�����ж����ȼ�
	NVIC_EnableIRQ(SysTick_IRQn);				    //ʹ�� SYS �ж�
}
