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
Function: MH_PWM_Output_Enable
Description: PWM���ͨ����ͨ
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_PWM_Output_Enable(void);

/**********************************************************************************************
Function: MH_PWM_Output_Disable
Description: PWM���ͨ���ر�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MH_PWM_Output_Disable(void);

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
Function: MH_Read_Hall_Count
Description: ��ȡ����HALL��ƽ
Input: ��
Output: ��
Input_Output: ��
Return: HALL��ƽ
Author: CJYS
***********************************************************************************************/
Q32U_ MH_Read_Hall_State(void);

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

#endif /* MotorHal_H */
