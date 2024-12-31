/**************************************************************************************************
*     File Name :                        MotorTask.h
*     Library/Module Name :              Motor
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ���������ͷ�ļ�
**************************************************************************************************/
#ifndef MotorTask_H
#define MotorTask_H

#include "Math.h"
#include "MotorHal.h"
#include "MotorSQ.h"
#include "MotorPara.h"

typedef void(*pFUN_HPWMLGPIO_OUT)(Q32U_);
typedef void(*pFUN_HPWMLPWM_OUT)(Q32U_);

/**********************************************************************************************
Function: Motor_Start
Description: �������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void Motor_Start(void)
{
    Motor.Motor_State_Flag.bit.motor_run_flag = 1U;
}

/**********************************************************************************************
Function: Motor_Stop
Description: ���ͣ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void Motor_Stop(void)
{
    Motor.Motor_State_Flag.bit.motor_run_flag = 0U;
}


/**********************************************************************************************
Function: Motor_Set_Dir
Description: ���õ�����з���
Input: 0����ת����1����ת��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void Motor_Set_Dir(Q32U_ Dir)
{
    if(Dir == 0)
    {
        Motor.MS_CTRL.DIR_Target = CW;
    }
    else
    {
        Motor.MS_CTRL.DIR_Target = CCW;
    }
}


/**********************************************************************************************
Function: Motor_Read_Dir
Description: ��ȡ������з���
Input: ��
Output: 0����ת����1����ת��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline Q32U_ Motor_Read_Dir(void)
{
    if(Motor.MS_CTRL.DIR_Set == CW)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/**********************************************************************************************
Function: Motor_Get_Run_State
Description: ��ȡ����Ƿ�Ϊ����״̬
Input: ��
Output: 1,0
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline Q32U_ Motor_Read_Run_State(void)
{
    if(Motor.Motor_Flow == MOTOR_STATE_RUN)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**********************************************************************************************
Function: Motor_Set_Target_Freq
Description: ����Ŀ��ռ�ձ�
Input: ռ�ձȣ�0~4095��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void Motor_Set_Target_Freq(Q32U_ Duty)
{
    Motor.MS_CTRL.PWM_CTRL._I_Q14I_duty_vr = Duty;
}

/**********************************************************************************************
Function: Motor_Read_Freq
Description: ��ȡ���Ƶ��
Input: ��
Output: ���Ƶ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline Q32U_ Motor_Read_Freq(void)
{
    return Motor.MS_CTRL.FL_Freq.Q16I_Filter_out;
}

/**********************************************************************************************
Function: Motor_Set_Vbus
Description: ����ĸ�ߵ�ѹֵ
Input: ĸ�ߵ�ѹ��lsb��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void Motor_Set_Vbus(Q32U_ Vbus_Val)
{
    Motor.MS_CTRL.Q12I_VBUS_VAL = Vbus_Val;
}

/**********************************************************************************************
Function: Motor_Read_Current_Max
Description: ��ȡ��������ֵ������Ϊ�ú��������õ�����
Input: ��
Output: ��������ֵ
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline Q32U_ Motor_Read_Current_Max(void)
{
    Q32U_ iphase_max_tmp = Motor.Q14I_IPHASE_MAX_PU;
    Motor.Q14I_IPHASE_MAX_PU = 0;
    return iphase_max_tmp;
}

/**********************************************************************************************
Function: Motor_Read_Error
Description: ��ȡ���������
Input: ��
Output: ���������
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline Q32U_ Motor_Read_Error(void)
{
    return Motor.Motor_Error_Flag.all;
}

/**********************************************************************************************
Function: Motor_Clear_Error
Description: ����������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void Motor_Clear_Error(void)
{
    Motor.Motor_Error_Flag.all = 0U;
}

/**********************************************************************************************
Function: MotorTask_Freq_Flow
Description: ��������ٶȻ�
Input: ��
Output: ��
Input_Output: �������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorTask_Freq_Flow(ST_MOTOR_TASK* pMotor);
/**********************************************************************************************
Function: MotorTask_Current_Flow
Description: ������Ƶ�����
Input: ��
Output: ��
Input_Output: �������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorTask_Current_Flow(ST_MOTOR_TASK* pMotor);

/**********************************************************************************************
Function: MotorTask_Switch_Flow
Description: ������ƻ���
Input: ��
Output: ��
Input_Output: �������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorTask_Switch_Flow(ST_MOTOR_TASK* pMotor);

/**********************************************************************************************
Function: MotorTask_PWM_Start_ADC_Flow
Description: ��������״δ���ADC
Input: ��
Output: ��
Input_Output: �������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorTask_PWM_Start_ADC_Flow(ST_MOTOR_TASK* pMotor);

/**********************************************************************************************
Function: MotorTask_Shut_Flow
Description: ������ƹ��Ϲض�
Input: ��
Output: ��
Input_Output: �������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorTask_Shut_Flow(ST_MOTOR_TASK* pMotor);

#endif /* MotorTask_H */
