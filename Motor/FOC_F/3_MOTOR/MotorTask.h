/**************************************************************************************************
*     File Name :                        MotorTask.h
*     Library/Module Name :              Motor
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ���������ͷ�ļ�
**************************************************************************************************/
#ifndef MotorTask_H
#define MotorTask_H

#include "MotorPara.h"
#include "MotorHal.h"
    
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
Input: 1.0f����ת����-1.0f����ת��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void Motor_Set_Dir(float Dir)
{
    Motor.SRAD_CTRL._I_F_DIR_Target = Dir;
    Motor.IF_CTRL._I_F_DIR_Target = Dir;
    Motor.VF_CTRL._I_F_DIR_Target = Dir;
}

/**********************************************************************************************
Function: Motor_Read_Dir
Description: ��ȡ������з���
Input: ��
Output: 1.0f����ת����-1.0f����ת��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline float Motor_Read_Dir(void)
{
    return Motor.SRAD_CTRL._O_F_DIR_Set;
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
Function: Motor_Set_Target_SRAD
Description: ���õ��ת��
Input: ���ת�٣�����ٶȣ������ƣ�
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void Motor_Set_Target_SRAD(float SRAD)
{
    Motor.SRAD_CTRL._I_F_SRAD_Target = SRAD;
}

/**********************************************************************************************
Function: Motor_Read_SRAD
Description: ��ȡ���ת��
Input: ��
Output: ���ת�٣�����ٶȣ������ƣ�
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline float Motor_Read_SRAD(void)
{
    return Motor.SRAD_CTRL._I_F_SRAD;
}

/**********************************************************************************************
Function: Motor_Set_Vbus
Description: ����FOC�㷨��ĸ�ߵ�ѹֵ
Input: ĸ�ߵ�ѹ��V��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void Motor_Set_Vbus(float Vbus_Val)
{
    Motor.SVPWM_CTRL._I_F_Vbus = Vbus_Val;
    Motor.SVPWM_CTRL._I_F_One_Over_Vbus = 1.0f/Vbus_Val;
    Motor.SRAD_CTRL._I_F_Vbus = Vbus_Val;
    Motor.CURRENT_CTRL._I_F_Vbus = Vbus_Val;
}

/**********************************************************************************************
Function: Motor_Read_Current_Max
Description: ��ȡ��������ֵ������Ϊ�ú��������õ�����
Input: ��
Output: ��������ֵ��A��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline float Motor_Read_Current_Max(void)
{
    float iphase_max_tmp = Motor.F_Iphase_Max;
    Motor.F_Iphase_Max = 0.0f;
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
Function: MotorTask_SRAD_Flow
Description: ��������ٶȻ�
Input: ��
Output: ��
Input_Output: �������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorTask_SRAD_Flow(ST_MOTOR_TASK* pMotor);

/**********************************************************************************************
Function: MotorTask_Current_Flow
Description: ������Ƶ�����
Input: ��
Output: ��
Input_Output: �������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorTask_Current_Flow(ST_MOTOR_TASK* pMotor);

#endif /* MotorTask_H */
