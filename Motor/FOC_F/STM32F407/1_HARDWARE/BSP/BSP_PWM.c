/**************************************************************************************************
*     File Name :                        BSP_PWM.c
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             PWM��ʼ����Ӧ�ò�ӿ�Դ�ļ�
**************************************************************************************************/
#include "BSP_PWM.h"

/**********************************************************************************************
Function: BSP_PWM_Init
Description: ���������PWM��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_PWM_Init(void)
{	
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;	
    TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
    
    TIM_Cmd(TIM1,ENABLE);

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;                     // 1��Ƶ
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;     // �������ĶԳ�
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;                                    // Timer clock = sysclock /(TIM_Prescaler+1) = 168M
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 1;
    TIM_TimeBaseInitStructure.TIM_Period = (HAL_PWM_SET_COUNT_U-1);               // Period = (TIM counter clock / TIM output clock) - 1 = 20K
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
        
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                               // pwmģʽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;                      // TIM_OCNPolarity_High 
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1,&TIM_OCInitStructure);
    TIM_OC2Init(TIM1,&TIM_OCInitStructure);
    TIM_OC3Init(TIM1,&TIM_OCInitStructure);
    
    TIM_OCInitStructure.TIM_Pulse = (HAL_ADC_SAMPLE_VALUE - HAL_ADC_DELAY_VALUE)/2;
    TIM_OC4Init(TIM1,&TIM_OCInitStructure);
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);
    
    TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; 
    TIM1_BDTRInitStructure.TIM_DeadTime = (0x7FU & HAL_PWM_DEADTIME_VALUE);
    TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;                    // ��������ͣ��������PWM  TIM_Break_Disable TIM_Break_Enable
    TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;  
    TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);
    
    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);      //���жϱ�־λ
    TIM_ITConfig(TIM1,TIM_IT_Break, ENABLE);        //���ж�

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1,ENABLE);	
}
