/**************************************************************************************************
*     File Name :                        Button.c
*     Library/Module Name :              SysTask
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ��������Դ�ļ�
**************************************************************************************************/
#include "Button.h"

ST_BUTTON_CONTROL Button_Ctrl = {
    .F_vr_start_tl = VR_START_TL,
    .F_vr_stop_tl = VR_STOP_TL,
    
    .F_vr_max_limit = VR_MAX_LIMIT,
    .F_vr_min_limit = VR_MIN_LIMIT,
};

/**********************************************************************************************
Function: Button_Control
Description: �����߼�����
Input: ��
Output: ��
Input_Output: ��������ָ�룬ϵͳ״ָ̬��
Return: ��
Author: CJYS
***********************************************************************************************/
void Button_Control(ST_BUTTON_CONTROL* pButton, ST_SYSTEM_TASK*  pST)
{
    float VRtmp = 0U;
    
    VRtmp = pST->F_FL_VR.F_Filter_out;
    if(VRtmp > pButton->F_vr_start_tl)
    {
        pST->System_State_Flag.BIT.system_runflag = 1U;
    }
    else if(VRtmp < pButton->F_vr_stop_tl)
    {
        pST->System_State_Flag.BIT.system_runflag = 0U;
    }
    
    if(VRtmp > pButton->F_vr_max_limit)
    {
        VRtmp = pButton->F_vr_max_limit;
    }
    else if(VRtmp < pButton->F_vr_min_limit)
    {
        VRtmp = pButton->F_vr_min_limit;
    }
    
    pST->F_Duty_Target = (VRtmp - pButton->F_vr_min_limit)/(pButton->F_vr_max_limit - pButton->F_vr_min_limit);
}

