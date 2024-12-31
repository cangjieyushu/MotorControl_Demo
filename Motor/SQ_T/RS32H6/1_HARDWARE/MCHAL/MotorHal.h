/**************************************************************************************************
*     File Name :                        MotorHal.h
*     Library/Module Name :              MotorHal
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             �������HAL��ͷ�ļ�
**************************************************************************************************/
#ifndef MotorHal_H
#define MotorHal_H

#include "MotorHal_cfg.h"                     

#define ADC_DATA_READ_U_BEMF        ((Q32U_)HAL_MOTOR_ADC->JDR1)
#define ADC_DATA_READ_V_BEMF        ((Q32U_)HAL_MOTOR_ADC->JDR2)
#define ADC_DATA_READ_W_BEMF        ((Q32U_)HAL_MOTOR_ADC->JDR3)
#define ADC_DATA_READ_CURRENT       ((Q32U_)HAL_MOTOR_ADC->JDR4)

/**********************************************************************************************
Function: MH_ADC_Soft_Trigger
Description: �������ADC
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MH_ADC_Soft_Trigger(void);

/**********************************************************************************************
Function: MH_ADC_Trigger_Delay_Time
Description: �ӳٴ���ADC,�ܿ�����ƽ̨
Input: �ӳٴ�������������ֵ
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MH_ADC_Trigger_Delay_Time(Q32U_ count);

/**********************************************************************************************
Function: MH_PWM_Freq_Set
Description: ������Ƶ
Input: PWM��Ƶ���Ƽ�����ֵ
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MH_PWM_Freq_Set(Q32U_ count);

/**********************************************************************************************
Function: MH_HPWM_LGPIO_Init
Description: ��ͬ��������ʼ��
Input: PWM��Ƶ���Ƽ�����ֵ
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MH_HPWM_LGPIO_Init(Q32U_ count);

/**********************************************************************************************
Function: MH_HPWM_LPWM_Init
Description: ͬ��������ʼ��
Input: PWM��Ƶ���Ƽ�����ֵ
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MH_HPWM_LPWM_Init(Q32U_ count);

/**********************************************************************************************
Function: MH_HPWM_LGPIO_XXXX
Description: ��ͬ���������ƣ�UpVnΪ��ͨU�Ϻ�V�£������Դ����ƣ�HOpenΪ�������Ϲ������£�LOpenΪ�������Ͽ�������,CLOSEΪ����ȫ��
Input: PWMռ�ձȿ��Ƽ�����ֵ
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MH_HPWM_LGPIO_UpVn(Q32U_ duty);
void MH_HPWM_LGPIO_UpWn(Q32U_ duty);
void MH_HPWM_LGPIO_VpWn(Q32U_ duty);
void MH_HPWM_LGPIO_VpUn(Q32U_ duty);
void MH_HPWM_LGPIO_WpUn(Q32U_ duty);
void MH_HPWM_LGPIO_WpVn(Q32U_ duty);
void MH_HPWM_LGPIO_HOpen(Q32U_ duty);
void MH_HPWM_LGPIO_LOpen(Q32U_ duty);
void MH_HPWM_LGPIO_Close(void);

/**********************************************************************************************
Function: MH_HPWM_LPWM_XXXX
Description: ͬ���������ƣ�UpVnΪ��ͨU�Ϻ�V�£������Դ����ƣ�HOpenΪ�������Ϲ������£�LOpenΪ�������Ͽ�������,CLOSEΪ����ȫ��
Input: PWMռ�ձȿ��Ƽ�����ֵ
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MH_HPWM_LPWM_UpVn(Q32U_ duty);
void MH_HPWM_LPWM_UpWn(Q32U_ duty);
void MH_HPWM_LPWM_VpWn(Q32U_ duty);
void MH_HPWM_LPWM_VpUn(Q32U_ duty);
void MH_HPWM_LPWM_WpUn(Q32U_ duty);
void MH_HPWM_LPWM_WpVn(Q32U_ duty);
void MH_HPWM_LPWM_HOpen(Q32U_ duty);
void MH_HPWM_LPWM_LOpen(Q32U_ duty);
void MH_HPWM_LPWM_Close(void);

/**********************************************************************************************
Function: MH_PWM_Read_Count
Description: ��ȡ��ǰPWM������ֵ
Input: ��
Output: ��
Input_Output: ��
Return: PWM������ֵ
Author: CJYS
***********************************************************************************************/
Q32U_ MH_PWM_Read_Count(void);

/**********************************************************************************************
Function: MH_HALL_TIM_Read_Count
Description: ��ȡ��ǰHALL���������ֵ
Input: ��
Output: ��
Input_Output: ��
Return: ALL���������ֵ
Author: CJYS
***********************************************************************************************/
Q32U_ MH_HALL_TIM_Read_Count(void);

/**********************************************************************************************
Function: MH_Switch_TIM_Delay
Description: �����ӳٻ��������ֵ�������ж�
Input: ���������ֵ
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MH_Switch_TIM_Delay(Q32U_ count);

/**********************************************************************************************
Function: MH_Switch_TIM_Stop
Description: ֹͣ�ӳٻ��������ֵ�������ж�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MH_Switch_TIM_Stop(void);

#endif /* MotorHal_H */
