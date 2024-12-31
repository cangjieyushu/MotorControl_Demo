/**************************************************************************************************
*     File Name :                        MotorHal.c
*     Library/Module Name :              MotorHal
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             �������HAL��Դ�ļ�
**************************************************************************************************/
#include "MotorHal.h"

ST_MH_ADC_DATA  Adc_Data;

Q32U_ Q32U_ADC_Data_Lsb[20] = {0};

/**********************************************************************************************
Function: MH_Read_ADC_Data
Description: ��ȡADC��������
Input: ��
Output: ��
Input_Output: ADC����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_Read_ADC_Data(ST_MH_ADC_DATA *pADC_DATA)
{
    Q32U_ADC_Data_Lsb[0] = ADC1->JDR1;
    Q32U_ADC_Data_Lsb[1] = ADC1->JDR2;
    Q32U_ADC_Data_Lsb[2] = ADC1->JDR3;
    
    pADC_DATA->Ia = (float)Q32U_ADC_Data_Lsb[0];
    pADC_DATA->Ib = (float)Q32U_ADC_Data_Lsb[1];
    pADC_DATA->Ic = (float)Q32U_ADC_Data_Lsb[2];
}


/**********************************************************************************************
Function: MH_PWM_Output_Enable
Description: PWM���ͨ����ͨ
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_PWM_Output_Enable(void) 
{
    TIM1->CCER |= 0x5555;
    TIM1->BDTR |= TIM_BDTR_MOE;
}
 
/**********************************************************************************************
Function: MH_PWM_Output_Disable
Description: PWM���ͨ���ر�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_PWM_Output_Disable(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCER &= 0xAAAA;
    TIM1->BDTR &= (Q16U_)~TIM_BDTR_MOE;
}


/**********************************************************************************************
Function: MH_PWM_Duty_Set_Three
Description: PWM�������ռ�ձ����
Input: ABC�����л�ʱ��㣬0~0.5��Ӧ��ʼ���Գ�����
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_PWM_Duty_Set_Three(float Ta, float Tb, float Tc)
{
	TIM1->CCR1 = (Q32U_)(Ta*HAL_PWM_ALL_COUNT_F);
    TIM1->CCR2 = (Q32U_)(Tb*HAL_PWM_ALL_COUNT_F);
    TIM1->CCR3 = (Q32U_)(Tc*HAL_PWM_ALL_COUNT_F);
}

/**********************************************************************************************
Function: MH_Read_Hall_Count
Description: ��ȡ����HALL��ƽ
Input: ��
Output: ��
Input_Output: ��
Return: HALL��ƽ
Author: CJYS
***********************************************************************************************/
Q32U_ MH_Read_Hall_State(void)
{
    return ((GPIOH->IDR&(GPIO_Pin_12|GPIO_Pin_11|GPIO_Pin_10))>>10);
}

/**********************************************************************************************
Function: MH_Read_Hall_Count
Description: ��ȡ����HALL����������ĵ�ǰֵ
Input: ��
Output: ��
Input_Output: ��
Return: ������ֵ
Author: CJYS
***********************************************************************************************/
Q32U_ MH_Read_Hall_Count(void)
{
   return TIM2->CNT;
}

