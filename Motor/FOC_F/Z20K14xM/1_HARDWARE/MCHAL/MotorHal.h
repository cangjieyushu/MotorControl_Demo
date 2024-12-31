/**************************************************************************************************
*     File Name :                        MotorHal.h
*     Library/Module Name :              MotorHal
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             �������HAL��ͷ�ļ�
**************************************************************************************************/
#ifndef MotorHal_H
#define MotorHal_H

#include "BSP.h"
#include "MotorHal_cfg.h"

typedef struct
{
    float   Ia;
    float   Ib;
    float   Ic;
    
    float   Ishunt_1;
    float   Ishunt_2;
    
    float   Vdc;
}ST_MH_ADC_DATA;

/**********************************************************************************************
Function: MH_Read_ADC_Data
Description: ��ȡADC��������
Input: ��
Output: ��
Input_Output: ADC����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_Read_ADC_Data(ST_MH_ADC_DATA *pADC_DATA);

/**********************************************************************************************
Function: MH_PWM_Output_En
Description: PWM���ͨ����ͨ��ر�
Input: ʹ�ܻ�رգ�true or false
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_PWM_Output_En(bool IsEnabled);

/**********************************************************************************************
Function: MH_PWM_Duty_Set_Three
Description: PWM�������ռ�ձ����
Input: ABC�����л�ʱ��㣬0~0.5��Ӧ��ʼ���Գ�����
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_PWM_Duty_Set_Three(float Ta, float Tb, float Tc);

/**********************************************************************************************
Function: MH_PWM_Duty_Set_One
Description: PWM�������ռ�ձ������������
Input: ABC����ռ�ձȣ�0~0.5��Ӧ��ʼ���Գ����ģ�Ta1���������������е��л�ʱ��㣬Ta2���½����������е��л�ʱ��㣬
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_PWM_Duty_Set_One(float Ta1, float Ta2, float Tb1, float Tb2, float Tc1, float Tc2);

/**********************************************************************************************
Function: MH_ADC_TrigTime_Set
Description: ��������£�ADC�Ĳ���ʱ������
Input: 0~0.5��Ӧ��ʼ���Գ����ģ�ch1�ǵ�һ�β���ʱ�̣�Ta2�ǵڶ��β���ʱ�̣�
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_ADC_TrigTime_Set(float Ch1, float Ch2);

/**********************************************************************************************
Function: MH_DMA0_ClearChannel0Int
Description: DMA��0��ͨ���жϱ�־λ���
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_DMA0_ClearChannel0Int(void);

/**********************************************************************************************
Function: MH_Read_Hall_Count
Description: ��ȡ����HALL����������ĵ�ǰֵ
Input: ��
Output: ��
Input_Output: ��
Return: ������ֵ
Author: CJYS
***********************************************************************************************/
Q32U_ MH_Read_Hall_Count(void);

extern ST_MH_ADC_DATA   Adc_Data;
extern Q32U_ Q32U_ADC_Data_Lsb[20];

#endif /* HAL_H */
