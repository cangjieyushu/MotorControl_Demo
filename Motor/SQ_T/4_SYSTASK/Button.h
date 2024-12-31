/**************************************************************************************************
*     File Name :                        Button.h
*     Library/Module Name :              SysTask
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ��������ͷ�ļ�
**************************************************************************************************/
#ifndef Button_H
#define Button_H

#include "SysTask.h"


//����ģʽ
#define BUTTON_MODE                     BUTTON_MODE_VR
#define BUTTON_MODE_BUTTON              0                   //����������ֹͣ
#define BUTTON_MODE_VR                  1                   //VR������ֹͣ������
#define BUTTON_MODE_BUTTON_VR           2                   //����������ֹͣ��VR����


//BUTTON_MODE_BUTTON

//BUTTON_MODE_VR
#define VR_START_TL                     (Q16I_)(100.0f)             //VR������ֵ
#define VR_STOP_TL                      (Q16I_)(50.0f)              //VRֹͣ��ֵ

#define VR_MAX_LIMIT                    (Q16I_)(3800.0f)            //VR�����ֵ
#define VR_MIN_LIMIT                    (Q16I_)(200.0f)             //VR��С��ֵ

#define VR_MAX_DUTY                     (Q32I_)(MOTOR_Q14_PU)           //VR���ռ�ձ�

//BUTTON_MODE_BUTTON_VR

typedef struct{
    Q32U_ Q16U_vr_start_tl;
    Q32U_ Q16U_vr_stop_tl;
    
    Q32U_ Q16U_vr_max_limit;
    Q32U_ Q16U_vr_min_limit;
    
    Q32U_ Q16U_vr_duty_max;
}ST_BUTTON_CONTROL;

extern ST_BUTTON_CONTROL Button_Ctrl;

/**********************************************************************************************
Function: Button_Control
Description: �����߼�����
Input: ��
Output: ��
Input_Output: ��������ָ�룬ϵͳ״ָ̬��
Return: ��
Author: CJYS
***********************************************************************************************/
void Button_Control(ST_BUTTON_CONTROL* pButton, ST_SYSTEM_TASK*  pST);

#endif /* Button_H */
