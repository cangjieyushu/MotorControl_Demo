/**************************************************************************************************
*     File Name :                        MotorEst.c
*     Library/Module Name :              Motor
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             �ٶȽǶȹ۲���Դ�ļ�
**************************************************************************************************/
#include "MotorEst.h"

/**********************************************************************/

//void Est_HFI_Init(ST_HFI_CONTROL* pCTRL)
//{
//    pCTRL->ElecFreqHz = 0.0f;
//    pCTRL->ElecFreqHz_Filter = 0.0f;
//    pCTRL->AngleRad = 0.0f;
//    pCTRL->AngleSpeed = 0.0f;
//    pCTRL->Id_LPF = 0.0f;
//    pCTRL->Iq_LPF = 0.0f;
//    pCTRL->Id_HPF = 0.0f;
//    pCTRL->Iq_HPF = 0.0f;
//    pCTRL->AngleRad_HFI = 0.0f;  
//    pCTRL->AngleRad_ERROR = 0.0f;    
//    pCTRL->Ud_HFI = 0.0f;     
//    pCTRL->cnt = 0U;
//    PID_POS_Init(&pCTRL->Pll_Pid, 0.0f);
//}

//Ram_Func void Est_HFI(ST_HFI_CONTROL* pCTRL)
//{
//    /* Clarke transform */   
//    Clarke_Transform(pFoc);
//    
//    pCTRL->cnt++;
//    if(pCTRL->cnt == 5U)
//    {
//        pCTRL->cnt = 0U;
//        pCTRL->Id_LPF = 0.5f*(pFoc->Ialpha + pCTRL->Id_Last);
//        pCTRL->Id_HPF = 0.5f*(pFoc->Ialpha - pCTRL->Id_Last)*pCTRL->SIGN;
//        pCTRL->Iq_LPF = 0.5f*(pFoc->Ibeta + pCTRL->Iq_Last);
//        pCTRL->Iq_HPF = 0.5f*(pFoc->Ibeta - pCTRL->Iq_Last)*pCTRL->SIGN;
//        
//        pCTRL->Id_Last = pFoc->Ialpha;
//        pCTRL->Iq_Last = pFoc->Ibeta;
//        pFoc->Ialpha = pCTRL->Id_LPF;
//        pFoc->Ibeta = pCTRL->Iq_LPF;
//        
//        pCTRL->Pll_Pid.Ref = pFoc->CosValue*pCTRL->Iq_HPF;
//        pCTRL->Pll_Pid.Fdb = pFoc->SinValue*pCTRL->Id_HPF;
//        PID_POS_Cal(&pCTRL->Pll_Pid);
//        
//        pCTRL->AngleSpeed = pCTRL->Pll_Pid.Output;
//        pCTRL->ElecFreqHz = ZXMATH_ONE_OVER_2PI*pCTRL->AngleSpeed;
//        pCTRL->ElecFreqHz_Filter = 0.001f*(USER_PLL_SPEED_LPF_COEFF*pCTRL->ElecFreqHz + (1000.0f-USER_PLL_SPEED_LPF_COEFF)*pCTRL->ElecFreqHz_Filter);
//        
//        pCTRL->AngleRad += 10.0f*pCTRL->Ts*pCTRL->AngleSpeed;
//        while(pCTRL->AngleRad > MATH_2PI_F)
//        {
//            pCTRL->AngleRad -= MATH_2PI_F;
//        }
//        while(pCTRL->AngleRad < 0.0f)
//        {
//            pCTRL->AngleRad += MATH_2PI_F;
//        }
//        
//        if(pCTRL->cnt_1 == 0U)
//        {
//            pCTRL->cnt_1 = 1U;
//            pCTRL->SIGN = 1.0f;
//        }
//        else if(pCTRL->cnt_1 == 1U)
//        {
//            pCTRL->cnt_1 = 0U;
//            pCTRL->SIGN = -1.0f;
//        }
//        pCTRL->Ud_HFI = pCTRL->SIGN*pCTRL->Ud_Ref;
//    }
//    
//    pFoc->AngleRad = pCTRL->AngleRad;
//    /* Park transform */
//    pFoc->SinValue = ZxMath_SinF32(pFoc->AngleRad);
//    pFoc->CosValue = ZxMath_CosF32(pFoc->AngleRad);
//    Park_Transform(pFoc);
//    
//    pFoc->VsMax = pFoc->RealVdc * pFoc->VsMaxScale;
//    /* Id PID */
//    pFoc->PidId.OutMax = pCTRL->Udq_Coeff*pFoc->VsMax;
//    pFoc->PidId.OutMin = -pCTRL->Udq_Coeff*pFoc->VsMax;
//    pFoc->PidId.Ref = pFoc->IdRef;
//    pFoc->PidId.Fdb = pFoc->Id;
//    PID_POS_Cal(&pFoc->PidId);
//    /* Iq PID */
//    pFoc->PidIq.OutMax = pCTRL->Udq_Coeff*pFoc->VsMax;
//    pFoc->PidIq.OutMin = -pCTRL->Udq_Coeff*pFoc->VsMax;
//    pFoc->PidIq.Ref = pFoc->IqRef;
//    pFoc->PidIq.Fdb = pFoc->Iq;
//    PID_POS_Cal(&pFoc->PidIq);
//    
//    pFoc->Ud = pFoc->PidId.Output + pCTRL->Ud_HFI;
//    pFoc->Uq = pFoc->PidIq.Output;
//    /* IPark transform */	
//    Ipark_Transform(pFoc);
//    /* SVGEN */
//    SVPWM_Cal(pFoc);
//}

/**********************************�����۲���************************************/

/**********************************************************************************************
Function: Est_Flux_Init_F
Description: �����۲�����ʼ��
Input: ��
Output: ��
Input_Output: �����۲���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void Est_Flux_Init_F(ST_FLUX_CONTROL_F* pCTRL)
{
    PID_Pos_Init_F(&pCTRL->PID_PLL, 0.0f);
    Filter_Init_F(&pCTRL->FL_SRAD, 0.0f);
    pCTRL->TG_Triangle.F_Angle = 0.0f;
    
    pCTRL->_V_F_Xalfa = 0.0f;
    pCTRL->_V_F_Xbeta = 0.0f;
}

/**********************************************************************************************
Function: Est_Flux_F
Description: �����۲�������
Input: ��
Output: ��
Input_Output: �����۲���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void Est_Flux_F(ST_FLUX_CONTROL_F* pCTRL)
{
}

/**********************************��ģ�۲���************************************/

/**********************************************************************************************
Function: Est_SMO_Init_F
Description: ��ģ�۲�����ʼ��
Input: ��
Output: ��
Input_Output: ��ģ�۲���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void Est_SMO_Init_F(ST_SMO_CONTROL_F* pCTRL)
{
    PID_Pos_Init_F(&pCTRL->PID_PLL, 0.0f);
    Filter_Init_F(&pCTRL->FL_SRAD, 0.0f);
    pCTRL->TG_Triangle.F_Angle = 0.0f;
    
    pCTRL->_V_F_Aalfa = 0.0f;
    pCTRL->_V_F_Abeta = 0.0f;
    pCTRL->_V_F_Ealfa = 0.0f;
    pCTRL->_V_F_Ebeta = 0.0f;
}

/**********************************************************************************************
Function: Est_SMO_F
Description: ��ģ�۲�������
Input: ��
Output: ��
Input_Output: ��ģ�۲���ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void Est_SMO_F(ST_SMO_CONTROL_F* pCTRL)
{
}

/**********************************************************************/

void Hallest_Positon_Control(ST_POSITION_CONTROL* pCTRL)
{
    float tmp1;
    if(pCTRL->I_Position_Error > pCTRL->P_K1)
    {
        tmp1 = pCTRL->P_K1;
    }
    else if(pCTRL->I_Position_Error < -pCTRL->P_K1)
    {
        tmp1 = -pCTRL->P_K1;
    }
    else
    {
        tmp1 = pCTRL->I_Position_Error;
    }
    
    tmp1 += pCTRL->P_K2*pCTRL->I_Position_Error;
    
    pCTRL->O_Follow_Speed = pCTRL->I_Target_Speed - tmp1;
}

/**********************************************************************/

float Hallest_Angle_Mean(float* DATA, ST_HALL_CONTROL_F* pCTRL)
{
    float tmp = 0.0f;
    for(Q08U_ i=0;i<pCTRL->P_HS_num;i++)
    {
        tmp += (*(DATA+3+i));
    }
    tmp /= (float)pCTRL->P_HS_num;
    return tmp;
}

void Hallest_Study_Task_Flow(ST_HALL_CONTROL_F* pCTRL)
{
    switch(pCTRL->HS_Flow)
    {
    case EM_HALL_STUDY_INIT:
        {
            Ramp_Init_F(&pCTRL->AngleRadRamp, pCTRL->AngleRadRamp.F_Init);
            pCTRL->Set_Dir = 1.0f;
            pCTRL->HS_Sector_cnt = 0;
            pCTRL->HS_cnt = 0;
            pCTRL->HS_Flow = EM_HALL_STUDY_CW;
        }break;
    case EM_HALL_STUDY_CW:
        {
            Ramp_Cal_F(&pCTRL->AngleRadRamp);
            if(pCTRL->HS_cnt >= pCTRL->P_HS_num+5)
            {
                pCTRL->HS_Sector_Tab_Init[0] = pCTRL->HS_Sector_tmp[0];
                pCTRL->HS_Sector_Tab_Init[1] = pCTRL->HS_Sector_tmp[1];
                pCTRL->HS_Sector_Tab_Init[2] = pCTRL->HS_Sector_tmp[2];
                pCTRL->HS_Sector_Tab_Init[3] = pCTRL->HS_Sector_tmp[3];
                pCTRL->HS_Sector_Tab_Init[4] = pCTRL->HS_Sector_tmp[4];
                pCTRL->HS_Sector_Tab_Init[5] = pCTRL->HS_Sector_tmp[5];
                
                pCTRL->HS_Angle_Tab_CW_Init[0] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_1, pCTRL);
                pCTRL->HS_Angle_Tab_CW_Init[1] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_2, pCTRL);
                pCTRL->HS_Angle_Tab_CW_Init[2] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_3, pCTRL);
                pCTRL->HS_Angle_Tab_CW_Init[3] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_4, pCTRL);
                pCTRL->HS_Angle_Tab_CW_Init[4] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_5, pCTRL);
                pCTRL->HS_Angle_Tab_CW_Init[5] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_6, pCTRL);
                
                Ramp_Init_F(&pCTRL->AngleRadRamp, pCTRL->AngleRadRamp.F_Init);
                pCTRL->Set_Dir = -1.0f;
                pCTRL->HS_Sector_cnt = 0;
                pCTRL->HS_cnt = 0;
                pCTRL->HS_Flow = EM_HALL_STUDY_CCW;
            }
        }break;
    case EM_HALL_STUDY_CCW:
        {
            Ramp_Cal_F(&pCTRL->AngleRadRamp);
            if(pCTRL->HS_cnt >= pCTRL->P_HS_num+5)
            {
                pCTRL->HS_Angle_Tab_CCW_Init[0] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_1, pCTRL);
                pCTRL->HS_Angle_Tab_CCW_Init[1] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_2, pCTRL);
                pCTRL->HS_Angle_Tab_CCW_Init[2] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_3, pCTRL);
                pCTRL->HS_Angle_Tab_CCW_Init[3] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_4, pCTRL);
                pCTRL->HS_Angle_Tab_CCW_Init[4] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_5, pCTRL);
                pCTRL->HS_Angle_Tab_CCW_Init[5] = Hallest_Angle_Mean(pCTRL->HS_Angle_tmp_6, pCTRL);
                
                for(Q08U_ i=0;i<=5;i++)
                {
                    for(Q08U_ j=0;j<=5;j++)
                    {
                        if(pCTRL->HS_Sector_Tab_Init[i] == pCTRL->HS_Sector_tmp[j])
                        {
                            pCTRL->HS_Angle_Tab_Init[i] = 0.5f*(pCTRL->HS_Angle_Tab_CW_Init[i] + pCTRL->HS_Angle_Tab_CCW_Init[j]);
                        }
                    }
                }
                
                if(pCTRL->HS_Angle_Tab_Init[0] > pCTRL->HS_Angle_Tab_Init[1])
                {
                    pCTRL->HS_Angle_Tab_Init[0] -= MATH_PI_F;
                }
                
                if(pCTRL->HS_Angle_Tab_Init[4] > pCTRL->HS_Angle_Tab_Init[5])
                {
                    pCTRL->HS_Angle_Tab_Init[5] += MATH_PI_F;
                }
                
                pCTRL->HS_Flow = EM_HALL_STUDY_FINISH;
            }
        }break;
    default:break;
    }
}

void Hallest_Study_VF(ST_HALL_CONTROL_F* pCTRL)
{
    pCTRL->AngleRad += pCTRL->Set_Dir*pCTRL->P_Ts*pCTRL->AngleRadRamp.F_Output*MATH_2PI_F;
    if(pCTRL->AngleRad > MATH_2PI_F)
    {
        pCTRL->HS_Sector_cnt = 0;
        pCTRL->HS_cnt++;
        pCTRL->AngleRad -= MATH_2PI_F;
    }
    else if(pCTRL->AngleRad < 0.0f)
    {
        pCTRL->HS_Sector_cnt = 0;
        pCTRL->HS_cnt++;
        pCTRL->AngleRad += MATH_2PI_F;
    }
}

void Hallest_Study_Self(ST_HALL_CONTROL_F* pCTRL)
{
    pCTRL->HS_Sector_tmp[pCTRL->HS_Sector_cnt] = pCTRL->HallCurrentLevel;
    switch(pCTRL->HS_Sector_cnt)
    {
    case 0:
        {
            pCTRL->HS_Angle_tmp_1[pCTRL->HS_cnt] = pCTRL->AngleRad;
        }break;
    case 1:
        {
            pCTRL->HS_Angle_tmp_2[pCTRL->HS_cnt] = pCTRL->AngleRad;
        }break;
    case 2:
        {
            pCTRL->HS_Angle_tmp_3[pCTRL->HS_cnt] = pCTRL->AngleRad;
        }break;
    case 3:
        {
            pCTRL->HS_Angle_tmp_4[pCTRL->HS_cnt] = pCTRL->AngleRad;
        }break;
    case 4:
        {
            pCTRL->HS_Angle_tmp_5[pCTRL->HS_cnt] = pCTRL->AngleRad;
        }break;
    case 5:
        {
            pCTRL->HS_Angle_tmp_6[pCTRL->HS_cnt] = pCTRL->AngleRad;
        }break;
    default:break;
    }
    pCTRL->HS_Sector_cnt++;
}

void Hallest_Angle_Initial(ST_HALL_CONTROL_F* pCTRL)
{
    pCTRL->HS_Sector_Tab[0] = pCTRL->HS_Sector_Tab_Init[5];
    pCTRL->HS_Sector_Tab[1] = pCTRL->HS_Sector_Tab_Init[0];
    pCTRL->HS_Sector_Tab[2] = pCTRL->HS_Sector_Tab_Init[1];
    pCTRL->HS_Sector_Tab[3] = pCTRL->HS_Sector_Tab_Init[2];
    pCTRL->HS_Sector_Tab[4] = pCTRL->HS_Sector_Tab_Init[3];
    pCTRL->HS_Sector_Tab[5] = pCTRL->HS_Sector_Tab_Init[4];
    pCTRL->HS_Sector_Tab[6] = pCTRL->HS_Sector_Tab_Init[5];
    pCTRL->HS_Sector_Tab[7] = pCTRL->HS_Sector_Tab_Init[0];
    
    pCTRL->HS_Angle_Tab[0] = pCTRL->HS_Angle_Tab_Init[5];
    pCTRL->HS_Angle_Tab[1] = pCTRL->HS_Angle_Tab_Init[0];
    pCTRL->HS_Angle_Tab[2] = pCTRL->HS_Angle_Tab_Init[1];
    pCTRL->HS_Angle_Tab[3] = pCTRL->HS_Angle_Tab_Init[2];
    pCTRL->HS_Angle_Tab[4] = pCTRL->HS_Angle_Tab_Init[3];
    pCTRL->HS_Angle_Tab[5] = pCTRL->HS_Angle_Tab_Init[4];
    pCTRL->HS_Angle_Tab[6] = pCTRL->HS_Angle_Tab_Init[5];
    pCTRL->HS_Angle_Tab[7] = pCTRL->HS_Angle_Tab_Init[0];
    
    pCTRL->HS_Angle_Tab_CW[0] = 0.5f*(pCTRL->HS_Angle_Tab[6] + pCTRL->HS_Angle_Tab[5]);
    pCTRL->HS_Angle_Tab_CW[1] = 0.5f*(pCTRL->HS_Angle_Tab[1] + pCTRL->HS_Angle_Tab[6]) - MATH_PI_F;
    pCTRL->HS_Angle_Tab_CW[2] = 0.5f*(pCTRL->HS_Angle_Tab[2] + pCTRL->HS_Angle_Tab[1]);
    pCTRL->HS_Angle_Tab_CW[3] = 0.5f*(pCTRL->HS_Angle_Tab[3] + pCTRL->HS_Angle_Tab[2]);
    pCTRL->HS_Angle_Tab_CW[4] = 0.5f*(pCTRL->HS_Angle_Tab[4] + pCTRL->HS_Angle_Tab[3]);
    pCTRL->HS_Angle_Tab_CW[5] = 0.5f*(pCTRL->HS_Angle_Tab[5] + pCTRL->HS_Angle_Tab[4]);
    pCTRL->HS_Angle_Tab_CW[6] = 0.5f*(pCTRL->HS_Angle_Tab[6] + pCTRL->HS_Angle_Tab[5]);
    pCTRL->HS_Angle_Tab_CW[7] = 0.5f*(pCTRL->HS_Angle_Tab[1] + pCTRL->HS_Angle_Tab[6]) - MATH_PI_F;
    for(Q08U_ i=0;i<=7;i++)
    {
        while(pCTRL->HS_Angle_Tab_CW[i] > MATH_2PI_F)
        {
            pCTRL->HS_Angle_Tab_CW[i] -= MATH_2PI_F;
        }
        while(pCTRL->HS_Angle_Tab_CW[i] < 0.0f)
        {
            pCTRL->HS_Angle_Tab_CW[i] += MATH_2PI_F;
        }
    }
    
    pCTRL->HS_Angle_Tab_CCW[0] = 0.5f*(pCTRL->HS_Angle_Tab[6] + pCTRL->HS_Angle_Tab[1]) - MATH_PI_F;
    pCTRL->HS_Angle_Tab_CCW[1] = 0.5f*(pCTRL->HS_Angle_Tab[1] + pCTRL->HS_Angle_Tab[2]);
    pCTRL->HS_Angle_Tab_CCW[2] = 0.5f*(pCTRL->HS_Angle_Tab[2] + pCTRL->HS_Angle_Tab[3]);
    pCTRL->HS_Angle_Tab_CCW[3] = 0.5f*(pCTRL->HS_Angle_Tab[3] + pCTRL->HS_Angle_Tab[4]);
    pCTRL->HS_Angle_Tab_CCW[4] = 0.5f*(pCTRL->HS_Angle_Tab[4] + pCTRL->HS_Angle_Tab[5]);
    pCTRL->HS_Angle_Tab_CCW[5] = 0.5f*(pCTRL->HS_Angle_Tab[5] + pCTRL->HS_Angle_Tab[6]);
    pCTRL->HS_Angle_Tab_CCW[6] = 0.5f*(pCTRL->HS_Angle_Tab[6] + pCTRL->HS_Angle_Tab[1]) - MATH_PI_F;
    pCTRL->HS_Angle_Tab_CCW[7] = 0.5f*(pCTRL->HS_Angle_Tab[1] + pCTRL->HS_Angle_Tab[2]);
    for(Q08U_ i=0;i<=7;i++)
    {
        while(pCTRL->HS_Angle_Tab_CCW[i] > MATH_2PI_F)
        {
            pCTRL->HS_Angle_Tab_CCW[i] -= MATH_2PI_F;
        }
        while(pCTRL->HS_Angle_Tab_CCW[i] < 0.0f)
        {
            pCTRL->HS_Angle_Tab_CCW[i] += MATH_2PI_F;
        }
    }
}

void Hallest_Init(ST_HALL_CONTROL_F* pCTRL)
{
    pCTRL->Set_Dir = pCTRL->I_Target_Dir;
    pCTRL->Hall_Dir = pCTRL->I_Target_Dir;  
    
    pCTRL->Switch_Phase_Flag = 0;  
    
    pCTRL->HallCount_tmp[0] = 0;
    pCTRL->HallCount_tmp[1] = 0;
    pCTRL->HallCount_tmp[2] = 0;
    pCTRL->HallCount_tmp[3] = 0;
    pCTRL->HallCount_tmp[4] = 0;
    pCTRL->HallCount_tmp[5] = 0;
    pCTRL->HallLastCount = 0;  
    pCTRL->HallCurrentCount = 0;  
    pCTRL->HallSpeedCount = 0;      
    
    pCTRL->HallStallCount = 0;             
    pCTRL->HallStallLastCount = 0;         
    pCTRL->HallStall_cnt = 0;     
    
    pCTRL->AngleRad = 0.0f;                 
    pCTRL->AngleRad_Hall = 0.0f;        
    
    pCTRL->ElecFreqHz = 0.0f;                  
    pCTRL->ElecFreqHz_Filter = 0.0f;   
    
    Ramp_Init_F(&pCTRL->AngleRadRamp, pCTRL->AngleRadRamp.F_Init);
    pCTRL->HS_Flow = EM_HALL_STUDY_INIT;
    pCTRL->HS_Sector_cnt = 0;  
    pCTRL->HS_cnt = 0;
    
    for(Q08U_ i=1;i<=6;i++)
    {
        if(pCTRL->HallCurrentLevel == pCTRL->HS_Sector_Tab[i])
        {
            pCTRL->HS_Sector_cnt = i;
            break;
        }
    }
    pCTRL->AngleRad_Hall = pCTRL->HS_Angle_Tab[pCTRL->HS_Sector_cnt];
}

void Hallest_High_Speed_Anlge(ST_HALL_CONTROL_F* pCTRL)
{
    for(Q08U_ i=1;i<=6;i++)
    {
        if(pCTRL->HallCurrentLevel == pCTRL->HS_Sector_Tab[i])
        {
            pCTRL->HS_Sector_cnt = i;
            break;
        }
    }
    
    if(pCTRL->Set_Dir == 1.0f)
    {
        pCTRL->AngleRad_Hall = pCTRL->HS_Angle_Tab_CW[pCTRL->HS_Sector_cnt];
    }
    else if(pCTRL->Set_Dir == -1.0f)
    {
        pCTRL->AngleRad_Hall = pCTRL->HS_Angle_Tab_CCW[pCTRL->HS_Sector_cnt];
    }
    
    if(pCTRL->HallLastLevel == pCTRL->HS_Sector_Tab[pCTRL->HS_Sector_cnt-1])
    {
        pCTRL->Hall_Dir = 1.0f;
    }
    else if(pCTRL->HallLastLevel == pCTRL->HS_Sector_Tab[pCTRL->HS_Sector_cnt+1])
    {
        pCTRL->Hall_Dir = -1.0f;
    }
}

void Hallest_Speed_Cal(ST_HALL_CONTROL_F* pCTRL)
{
    pCTRL->HallStallCount++;
    
    if(pCTRL->HallCurrentCount >  pCTRL->HallLastCount)
    {
        pCTRL->HallCount_tmp[0] = pCTRL->HallCurrentCount - pCTRL->HallLastCount;
    }
    else
    {
        pCTRL->HallCount_tmp[0] =  (pCTRL->P_TIM_Max_Count - pCTRL->HallLastCount) + pCTRL->HallCurrentCount;
    }
    
    pCTRL->ElecFreqHz = pCTRL->P_TIM_FreqHz/((float)pCTRL->HallCount_tmp[0] + (float)pCTRL->HallCount_tmp[1] + (float)pCTRL->HallCount_tmp[2]
                                           + (float)pCTRL->HallCount_tmp[3] + (float)pCTRL->HallCount_tmp[4] + (float)pCTRL->HallCount_tmp[5]);
    
    pCTRL->HallCount_tmp[5] = pCTRL->HallCount_tmp[4];
    pCTRL->HallCount_tmp[4] = pCTRL->HallCount_tmp[3];
    pCTRL->HallCount_tmp[3] = pCTRL->HallCount_tmp[2];
    pCTRL->HallCount_tmp[2] = pCTRL->HallCount_tmp[1];
    pCTRL->HallCount_tmp[1] = pCTRL->HallCount_tmp[0];
    
    pCTRL->ElecFreqHz_Filter = 0.001f*(pCTRL->P_Freq_Filter_Coeff*pCTRL->ElecFreqHz_Filter + (1000.0f-pCTRL->P_Freq_Filter_Coeff)*pCTRL->ElecFreqHz);
    
    pCTRL->HallLastCount = pCTRL->HallCurrentCount;
    
    pCTRL->Switch_Phase_Flag = 1;
}

void Hallest_Angle_Inc(ST_HALL_CONTROL_F* pCTRL)
{
    float AngleRad_Hall_tmp = 0.0f;
    float AngleRad_Add_tmp = 0.0f;
    Q32U_ HallCurrentCount_tmp = 0;
    Q32U_ HallDelta_tmp = 0;
    
    pCTRL->Switch_Phase_Flag = 0U;
    
    AngleRad_Hall_tmp = pCTRL->AngleRad_Hall;
    HallCurrentCount_tmp = pCTRL->HallCurrentCount;
    
    if(pCTRL->HallSpeedCount >  HallCurrentCount_tmp)
    {
        HallDelta_tmp = pCTRL->HallSpeedCount - HallCurrentCount_tmp;
    }
    else
    {
        HallDelta_tmp = (pCTRL->P_TIM_Max_Count - HallCurrentCount_tmp) + pCTRL->HallSpeedCount;
    }
    
    AngleRad_Add_tmp = ((float)HallDelta_tmp)*MATH_2PI_F*pCTRL->ElecFreqHz_Filter/pCTRL->P_TIM_FreqHz;
    
    if(AngleRad_Add_tmp > MATH_PI_OVER_TWO_F)
    {
        AngleRad_Add_tmp = MATH_PI_OVER_TWO_F;
    }
    if(AngleRad_Add_tmp < -MATH_PI_OVER_TWO_F)
    {
        AngleRad_Add_tmp = -MATH_PI_OVER_TWO_F;
    }
    
    if(pCTRL->Switch_Phase_Flag == 1U)
    {
        pCTRL->AngleRad = pCTRL->AngleRad_Hall;
    }
    else
    {
        pCTRL->AngleRad = AngleRad_Hall_tmp + AngleRad_Add_tmp;
    }
}
