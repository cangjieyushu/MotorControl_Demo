/**************************************************************************************************
*     File Name :                        MotorSQ.c
*     Library/Module Name :              Motor
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             �޸з���Դ�ļ�
**************************************************************************************************/
 
#include "MotorSQ.h"

EM_CHANNEL_NUM ADC_VAL_Table[6][3] = 
{
    U_CHANNEL_NUM, W_CHANNEL_NUM, V_CHANNEL_NUM, 
    U_CHANNEL_NUM, V_CHANNEL_NUM, W_CHANNEL_NUM,
    V_CHANNEL_NUM, U_CHANNEL_NUM, W_CHANNEL_NUM, 
    V_CHANNEL_NUM, W_CHANNEL_NUM, U_CHANNEL_NUM,
    W_CHANNEL_NUM, V_CHANNEL_NUM, U_CHANNEL_NUM, 
    W_CHANNEL_NUM, U_CHANNEL_NUM, V_CHANNEL_NUM,
};

EM_SECTOR_NUM Position_CW[6][2] = {sector_3, sector_5,
                                   sector_4, sector_6,
                                   sector_5, sector_1,
                                   sector_6, sector_2,
                                   sector_1, sector_3,
                                   sector_2, sector_4};

EM_SECTOR_NUM Last_Sector[6] = {sector_6, sector_1, sector_2, sector_3, sector_4, sector_5};
EM_SECTOR_NUM Next_Sector[6] = {sector_2, sector_3, sector_4, sector_5, sector_6, sector_1};

/**********************************************************************************************
Function: MotorSQ_Init
Description: �����㷨��ʼ��
Input: ��
Output: ��
Input_Output: ��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_Init(ST_MS_CONTROL* pMS_CTRL)
{
    pMS_CTRL->SW_Math = SWITCH_CURRENT;
    pMS_CTRL->SQ_Flow = SQUARE_CROSS_ING;
    pMS_CTRL->DIR_Set = pMS_CTRL->DIR_Target;
    
    Ramp_Init(&pMS_CTRL->Ramp_Freq, pMS_CTRL->Ramp_Freq.Q32I_Init);
    
    Filter_Init(&pMS_CTRL->FL_Iphase, 0);
    Filter_Init(&pMS_CTRL->FL_Freq, 0);
    Filter_Init(&pMS_CTRL->FL_Ibus, 0);
    Filter_Init(&pMS_CTRL->FL_Ibrake, 0);
    
    PID_Inc_Init(&pMS_CTRL->PID_Iphase, 0);
    PID_Inc_Init(&pMS_CTRL->PID_Freq, 0);
    PID_Inc_Init(&pMS_CTRL->PID_Ibus, 0);
    PID_Inc_Init(&pMS_CTRL->PID_Ibrake, 0);
    
    MotorSQ_DIAG_Init(&pMS_CTRL->MS_DIAG);
    MotorSQ_CURRENT_Init(&pMS_CTRL->MS_CURRENT);
    MotorSQ_FLUX_Init(&pMS_CTRL->MS_FLUX);
    MotorSQ_BEMF_Init(&pMS_CTRL->MS_BEMF);
    MotorSQ_CMP_Init(&pMS_CTRL->MS_CMP);
    MotorSQ_Freq_Cal_Init(&pMS_CTRL->FREQ_CAL);
    MotorSQ_PWM_Freq_Switch_Init(&pMS_CTRL->PWM_CTRL);
    MotorSQ_Stall_Check_Init(&pMS_CTRL->STALL_CTRL, pMS_CTRL);
    
    pMS_CTRL->Q12I_BEMF_ADC_tmp[0] = 0;
    pMS_CTRL->Q12I_BEMF_ADC_tmp[1] = 0;
    pMS_CTRL->Q12I_BEMF_ADC_tmp[2] = 0;
    pMS_CTRL->Q12I_VBUS_VAL = 0;
    pMS_CTRL->Q12I_IPHASE_ADC = 0;
    pMS_CTRL->Q12I_IPHASE_OFFSET = 0;
    pMS_CTRL->Q14I_IPHASE_PU = 0;
    
    pMS_CTRL->Q32U_switch_cnt = 0;
}

/**********************************************************************************************
Function: MotorSQ_Flying_Init
Description: ˳��������ʼ��
Input: ��
Output: ��
Input_Output: ��������ָ�룬˳����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_Flying_Init(ST_MS_CONTROL* pMS_CTRL, ST_MS_FLYING* pMS_FLYING)
{
}

/**********************************************************************************************
Function: MotorSQ_Offset_Check_Init
Description: ƫ�ü���ʼ��
Input: ��
Output: ��
Input_Output: ƫ�ü��ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Offset_Check_Init(ST_MS_OFFSET* pMS_OFFSET)
{
    Q32U_ flag_tmp = ING;
    
    if(pMS_OFFSET->Flag.bit.b0_init == 0U)
    {
        pMS_OFFSET->Flag.all = 0U;
        
        pMS_OFFSET->_V_Q32U_cnt = 0U;

        pMS_OFFSET->Flag.bit.b0_init = 1U;
    }
    else
    {
        flag_tmp = SUCC;
    }
    
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Offset_Check
Description: ƫ�ü�����
Input: ��
Output: ��
Input_Output: ƫ�ü��ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Offset_Check(ST_MS_OFFSET* pMS_OFFSET)
{
    Q32U_ flag_tmp = ING;
    
    pMS_OFFSET->_O_Q12I_IPHASE_OFFSET += pMS_OFFSET->_I_Q12I_IPHASE_ADC;
    pMS_OFFSET->_V_Q32U_cnt++;
    
    if(pMS_OFFSET->_V_Q32U_cnt == pMS_OFFSET->_P_Q16U_check_num)
    {
        pMS_OFFSET->_O_Q12I_IPHASE_OFFSET /= pMS_OFFSET->_V_Q32U_cnt;
        if((pMS_OFFSET->_O_Q12I_IPHASE_OFFSET > pMS_OFFSET->_P_Q16U_offset_max)
        || (pMS_OFFSET->_O_Q12I_IPHASE_OFFSET < pMS_OFFSET->_P_Q16U_offset_min))
        {
            pMS_OFFSET->Flag.bit.b2_fail = 1U;
        }
        else
        {
            pMS_OFFSET->Flag.bit.b1_succ = 1U;
        }
    }
    
    if(pMS_OFFSET->Flag.bit.b1_succ == 1U)
    {
        pMS_OFFSET->Flag.bit.b0_init = 0U;
        flag_tmp = SUCC;
    }
    else if(pMS_OFFSET->Flag.bit.b2_fail == 1U)
    {
        pMS_OFFSET->Flag.bit.b0_init = 0U;
        flag_tmp = FAIL;
    }
    
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Flying_Check_Init
Description: ˳�����ʼ��
Input: ��
Output: ��
Input_Output: ˳����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Flying_Check_Init(ST_MS_FLYING* pMS_FLYING)
{
    Q32U_ flag_tmp = ING;
    
    if(pMS_FLYING->Flag.bit.b0_init == 0U)
    {
        pMS_FLYING->Flag.all = 0U;

        pMS_FLYING->_V_Q32U_cnt = 0U;
        pMS_FLYING->_V_Q32U_time_cnt = 0U;
        pMS_FLYING->_V_Q32U_switch_cnt = 0U;
        pMS_FLYING->_V_Q32U_phase_cnt = 0U;
        
        pMS_FLYING->Flag.bit.b0_init = 1U;
    }
    else
    {
        flag_tmp = SUCC;
    }
    
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Flying_Check
Description: ˳�������
Input: ��
Output: ��
Input_Output: ˳����ָ�룬Ƶ�ʼ���ָ�룬��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Flying_Check(ST_MS_FLYING* pMS_FLYING, ST_FREQ_CAL* pFREQ_CAL, ST_MS_CONTROL* pMS_CTRL)
{
    Q32U_ flag_tmp = ING;
    
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Boot_Check_Init
Description: �Ծٿ��Ƴ�ʼ��
Input: ��
Output: ��
Input_Output: �Ծٿ���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Boot_Check_Init(ST_MS_BOOT* pMS_BOOT)
{
    Q32U_ flag_tmp = ING;
    
    if(pMS_BOOT->Flag.bit.b0_init == 0U)
    {
        pMS_BOOT->Flag.all = 0U;

        pMS_BOOT->_V_Q32U_cnt = 0U;
        pMS_BOOT->_V_Q32U_time_cnt = 0U;
        
        pMS_BOOT->Flag.bit.b0_init = 1U;
    }
    else
    {
        flag_tmp = SUCC;
    }
    
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Boot_Check
Description: �Ծٿ��Ƽ���
Input: ��
Output: ��
Input_Output: �Ծٿ���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Boot_Check(ST_MS_BOOT* pMS_BOOT)
{
    Q32U_ flag_tmp = ING;
    
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Pluse_Positon_Init
Description: ���嶨λ��ʼ��
Input: ��
Output: ��
Input_Output: ���嶨λָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Pluse_Positon_Init(ST_MS_POSITION* pMS_POSITION)
{
    Q32U_ flag_tmp = ING;
    
    if(pMS_POSITION->Flag.bit.b0_init == 0U)
    {
        pMS_POSITION->Flag.all = 0U;

        pMS_POSITION->_V_Q32U_cnt = 7U;
        
        pMS_POSITION->Flag.bit.b0_init = 1U;
    }
    else
    {
        if(pMS_POSITION->_V_Q32U_cnt == 7U)
        {
            pMS_POSITION->_V_Q32U_cnt = 0U;
        }
        else
        {
            flag_tmp = SUCC;
        }
    }
    
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Pluse_Positon
Description: ���嶨λ����
Input: ��
Output: ��
Input_Output: ���嶨λָ�룬��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Pluse_Positon(ST_MS_POSITION* pMS_POSITION, ST_MS_CONTROL* pMS_CTRL)
{
    Q32U_ flag_tmp = ING;
    
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Brake_Init
Description: ɲ�����Ƴ�ʼ��
Input: ��
Output: ��
Input_Output: ɲ������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Brake_Init(ST_BRAKE_CONTROL* pBRAKE_CTRL, ST_MS_CONTROL* pMS_CTRL)
{
    Q32U_ flag_tmp = ING;
    
    if(pBRAKE_CTRL->Flag.bit.b0_init == 0U)
    {
        pBRAKE_CTRL->Flag.all = 0U;

        pBRAKE_CTRL->_V_Q32U_cnt = 0U;
        Ramp_Init(&pBRAKE_CTRL->Ramp_Brake_Duty, pBRAKE_CTRL->Ramp_Brake_Duty.Q32I_Init);
        PID_Inc_Init(&pMS_CTRL->PID_Ibrake, pMS_CTRL->PID_Ibrake.Q14I_OutMin);
        
        pBRAKE_CTRL->Flag.bit.b0_init = 1U;
    }
    else
    {
        flag_tmp = SUCC;
    }
    
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Brake
Description: ɲ������ռ�ձȼ���
Input: ��
Output: ��
Input_Output: ɲ������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Brake(ST_BRAKE_CONTROL* pBRAKE_CTRL, ST_MS_CONTROL* pMS_CTRL)
{
    Q32U_ flag_tmp = ING;
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_DIAG_Init
Description: ��������ʼ��
Input: ��
Output: ��
Input_Output: �������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_DIAG_Init(ST_MS_DIAG* pMS_DIAG)
{
    pMS_DIAG->_V_Q32U_cnt = 0U;
    pMS_DIAG->_V_Q32U_time_cnt = 0U;
}

/**********************************************************************************************
Function: MotorSQ_DIAG_Zero_Cross
Description: ����������
Input: ��
Output: ��
Input_Output: �������ָ�룬��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_DIAG_Zero_Cross(ST_MS_DIAG* pMS_DIAG, ST_MS_CONTROL* pMS_CTRL)
{ 
}

/**********************************************************************************************
Function: MotorSQ_CURRENT_Init
Description: ���������ʼ��
Input: ��
Output: ��
Input_Output: ��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_CURRENT_Init(ST_MS_CURRENT* pMS_CURRENT)
{
    pMS_CURRENT->_V_Q32U_cnt = 0U;
    pMS_CURRENT->_V_Q32U_time_cnt = 0U;
    pMS_CURRENT->_V_Q32U_switch_cnt = 0U;
}

/**********************************************************************************************
Function: MotorSQ_CURRENT_Zero_Cross
Description: �����������
Input: ��
Output: ��
Input_Output: ��������ָ�룬��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_CURRENT_Zero_Cross(ST_MS_CURRENT* pMS_CURRENT, ST_MS_CONTROL* pMS_CTRL)
{
}

/**********************************************************************************************
Function: MotorSQ_FLUX_Init
Description: ���������ʼ��
Input: ��
Output: ��
Input_Output: ��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_FLUX_Init(ST_MS_FLUX* pMS_FLUX)
{
    pMS_FLUX->_V_Q32U_cnt = 0U;
    pMS_FLUX->_V_Q32U_time_cnt = 0U;
    pMS_FLUX->_V_Q32U_switch_cnt = 0U;
}

/**********************************************************************************************
Function: MotorSQ_FLUX_Zero_Cross
Description: �����������
Input: ��
Output: ��
Input_Output: ��������ָ�룬��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_FLUX_Zero_Cross(ST_MS_FLUX* pMS_FLUX, ST_MS_CONTROL* pMS_CTRL)
{
}

/**********************************************************************************************
Function: MotorSQ_BEMF_Init
Description: ���綯�ƻ����ʼ��
Input: ��
Output: ��
Input_Output: ���綯�ƻ���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_BEMF_Init(ST_MS_BEMF* pMS_BEMF)
{
    pMS_BEMF->_V_Q32U_cnt = 0U;
    pMS_BEMF->_V_Q32U_time_cnt = 0U;
}

/**********************************************************************************************
Function: MotorSQ_BEMF_Zero_Cross
Description: ���綯�ƻ������
Input: ��
Output: ��
Input_Output: ���綯�ƻ���ָ�룬��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_BEMF_Zero_Cross(ST_MS_BEMF* pMS_BEMF, ST_MS_CONTROL* pMS_CTRL)
{
}

/**********************************************************************************************
Function: MotorSQ_CMP_Init
Description: �Ƚ��������ʼ��
Input: ��
Output: ��
Input_Output: �Ƚ�������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_CMP_Init(ST_MS_CMP* pMS_CMP)
{
}

/**********************************************************************************************
Function: MotorSQ_CMP_Zero_Cross
Description: �Ƚ����������
Input: ��
Output: ��
Input_Output: �Ƚ�������ָ�룬��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_CMP_Zero_Cross(ST_MS_CMP* pMS_CMP, ST_MS_CONTROL* pMS_CTRL)
{
}

/**********************************************************************************************
Function: MotorSQ_Freq_Cal_Init
Description: Ƶ�ʼ����ʼ��
Input: ��
Output: ��
Input_Output: Ƶ�ʼ���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_Freq_Cal_Init(ST_FREQ_CAL* pFREQ_CAL)
{
    pFREQ_CAL->_V_Q32U_60_degree_cnt_last = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[0] = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[1] = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[2] = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[3] = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[4] = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[5] = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[6] = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[7] = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[8] = 0U;
    pFREQ_CAL->_V_Q32U_60_degree_cnt_tmp[9] = 0U;
    pFREQ_CAL->_O_Q32U_60_degree_cnt = 0U;
}

/**********************************************************************************************
Function: MotorSQ_Freq_Cal
Description: Ƶ�ʼ���
Input: ��
Output: ��
Input_Output: Ƶ�ʼ���ָ�룬��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_Freq_Cal(ST_FREQ_CAL* pFREQ_CAL, ST_MS_CONTROL* pMS_CTRL)
{
}

/**********************************************************************************************
Function: MotorSQ_PWM_Freq_Switch_Init
Description: ��Ƶ���Ƴ�ʼ��
Input: ��
Output: ��
Input_Output: ��Ƶ����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_PWM_Freq_Switch_Init(ST_PWM_CONTROL* pPWM_CTRL)
{
    pPWM_CTRL->Flag.all = 0U;
    pPWM_CTRL->Ramp_Duty.Q32I_Output = 0U;
    pPWM_CTRL->_O_Q16U_arr_set = pPWM_CTRL->_P_Q14U_low_pwm_freq;
}

/**********************************************************************************************
Function: MotorSQ_PWM_Freq_Switch
Description: ��Ƶ���Ƽ���
Input: ��
Output: ��
Input_Output: ��Ƶ����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_PWM_Freq_Switch(ST_PWM_CONTROL* pPWM_CTRL)
{
}

/**********************************************************************************************
Function: MotorSQ_Stall_Check_Init
Description: ��ת����ʼ��
Input: ��
Output: ��
Input_Output: ��ת���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_Stall_Check_Init(ST_STALL_CONTROL* pSTALL_CTRL, ST_MS_CONTROL* pMS_CTRL)
{
    pSTALL_CTRL->Flag.all = 0U;
    pSTALL_CTRL->_V_Q32U_current_cnt = 0U;
    pSTALL_CTRL->_V_Q32U_cnt = 0U;
    pSTALL_CTRL->_V_Q32U_switch_cnt = 0U;
    
    pMS_CTRL->Q32U_switch_cnt = 0U;
}

/**********************************************************************************************
Function: MotorSQ_Stall_Check
Description: ��ת������
Input: ��
Output: ��
Input_Output: ��ת���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q32U_ MotorSQ_Stall_Check(ST_STALL_CONTROL* pSTALL_CTRL, ST_MS_CONTROL* pMS_CTRL)
{
    Q32U_ flag_tmp = ING;
    return flag_tmp;
}

/**********************************************************************************************
Function: MotorSQ_Ibus_Cal
Description: ĸ�ߵ�������
Input: ��
Output: ��
Input_Output: ��������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorSQ_Ibus_Cal(ST_MS_CONTROL* pMS_CTRL)
{
    pMS_CTRL->FL_Ibus.Q16I_Filter_in = Q32I_RHT_12(pMS_CTRL->PWM_CTRL._O_Q12I_duty_set*pMS_CTRL->Q14I_IPHASE_PU);
    Filter_Cal(&pMS_CTRL->FL_Ibus);
}
