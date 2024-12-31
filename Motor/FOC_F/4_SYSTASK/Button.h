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
#define VR_START_TL                     (100.0f)                    //VR������ֵ
#define VR_STOP_TL                      (50.0f)                     //VRֹͣ��ֵ

#define VR_MAX_LIMIT                    (3800.0f)                   //VR�����ֵ
#define VR_MIN_LIMIT                    (200.0f)                    //VR��С��ֵ

//BUTTON_MODE_BUTTON_VR

typedef struct{
    float F_vr_start_tl;
    float F_vr_stop_tl;
    
    float F_vr_max_limit;
    float F_vr_min_limit;
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
