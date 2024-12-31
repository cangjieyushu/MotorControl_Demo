/**************************************************************************************************/
/**
 * @copyright : 
 **************************************************************************************************/
 
#include "MotorFoc.h"

EM_SECTOR_NUM Position_CW[6][2] = {sector_3, sector_5,
                                   sector_4, sector_6,
                                   sector_5, sector_1,
                                   sector_6, sector_2,
                                   sector_1, sector_3,
                                   sector_2, sector_4};

EM_SECTOR_NUM Last_Sector[6] = {sector_6, sector_1, sector_2, sector_3, sector_4, sector_5};
EM_SECTOR_NUM Next_Sector[6] = {sector_2, sector_3, sector_4, sector_5, sector_6, sector_1};

void Motor_Current_Offset_Check(ST_COMMON_CONTROL* pCOM, ST_FOC_PARAMETER* pFocPara)
{
    pFocPara->Q12I_I_Data_tmp += pFocPara->Q12I_I_Data[3];
    if(++pCOM->sq_cnt == pCOM->Q08U_iphase_offset_num)
    {
        pFocPara->Q12I_I_Offset = pFocPara->Q12I_I_Data_tmp/pCOM->sq_cnt;
        if((pFocPara->Q12I_I_Offset > pCOM->Q16U_iphase_offset_min)
        && (pFocPara->Q12I_I_Offset < pCOM->Q16U_iphase_offset_max))
        {
            pCOM->Motor_Flag.bit.current_offset_Succ = 1U;
        }
        else
        {
            pCOM->Motor_Flag.bit.current_offset_Fail = 1U;
        }
        pCOM->sq_cnt = 0U;
        pFocPara->Q12I_I_Data_tmp = 0;
    }
}

void Six_Pluse_Positon(ST_COMMON_CONTROL* pCOM)
{
	Q16U_ max_tmp = pCOM->Q12U_Position_Current_VAL[sector_1];
    Q08U_ max_index = sector_1;
    for(Q08U_ i=1U;i<6U;i++)
    {
        if(pCOM->Q12U_Position_Current_VAL[i] > max_tmp)
        {
            max_index = i;
            max_tmp = pCOM->Q12U_Position_Current_VAL[i];
        }
    }
    
    pCOM->Sector = Position_CW[max_index][1];
    if(pCOM->Q12U_Position_Current_VAL[Next_Sector[pCOM->Sector]] > pCOM->Q12U_Position_Current_VAL[Last_Sector[pCOM->Sector]])
    {
        pCOM->Sector = Next_Sector[pCOM->Sector];
    }
    else
    {
        pCOM->Sector = Last_Sector[pCOM->Sector];
    }
    
    pCOM->Motor_Flag.bit.motor_position_Fail = 0U;
    pCOM->Motor_Flag.bit.motor_position_Succ = 1U;
    for(Q08U_ j=0U;j<6U;j++)
    {
        if(pCOM->Q12U_Position_Current_VAL[j] < pCOM->Q16U_position_tl)
        {
            pCOM->Motor_Flag.bit.motor_position_Fail = 1U;
            pCOM->Motor_Flag.bit.motor_position_Succ = 0U;
        }
    }
}

void MotorFoc_Clark(ST_FOC_PARAMETER* pFocPara)
{
    pFocPara->Q14I_Ialfa = pFocPara->Q14I_Ia;
    pFocPara->Q14I_Ibeta = MATH_ONE_OVER_SQRT_THREE_U(pFocPara->Q14I_Ib - pFocPara->Q14I_Ic);
}

void MotorFoc_Park(ST_FOC_PARAMETER* pFocPara)
{
    pFocPara->Q14I_Id = Q32I_RHT_14( pFocPara->Q14I_Ialfa*pFocPara->Angle.Q14I_Cos + pFocPara->Q14I_Ibeta*pFocPara->Angle.Q14I_Sin); 
    pFocPara->Q14I_Iq = Q32I_RHT_14(-pFocPara->Q14I_Ialfa*pFocPara->Angle.Q14I_Sin + pFocPara->Q14I_Ibeta*pFocPara->Angle.Q14I_Cos);
}

void MotorFoc_Ipark(ST_FOC_PARAMETER* pFocPara)
{
    pFocPara->Q14I_Ualfa = Q32I_RHT_14(pFocPara->Q14I_Ud*pFocPara->Angle.Q14I_Cos - pFocPara->Q14I_Uq*pFocPara->Angle.Q14I_Sin);
    pFocPara->Q14I_Ubeta = Q32I_RHT_14(pFocPara->Q14I_Ud*pFocPara->Angle.Q14I_Sin + pFocPara->Q14I_Uq*pFocPara->Angle.Q14I_Cos);
}

static const Q08U_ ADC_Table[3][8] = 
{{0U,2U,0U,0U,1U,1U,2U,0U},
 {0U,0U,1U,2U,2U,0U,1U,0U},
 {0U,1U,2U,1U,0U,2U,0U,0U}};
void MotorFoc_Simple_Cal(ST_FOC_PARAMETER* pFocPara)
{
}

static const Q08U_ Txyz_Table[3][8] = 
{{0U,1U,0U,0U,2U,2U,1U,0U},
 {0U,0U,2U,1U,1U,0U,2U,0U},
 {0U,2U,1U,2U,0U,1U,0U,0U}};
void MotorFoc_SVPWM(ST_FOC_PARAMETER* pFocPara, ST_CURRENT_CONTROL* pCurrentCtrl)
{
}

void MotorFoc_WEAK_Init(ST_WEAK_CONTROL* pWEAK)
{
    PID_Pos_Init(&pWEAK->PidV, 0);
}

void MotorFoc_WEAK_Control(ST_FOC_PARAMETER* pFocPara, ST_SRAD_CONTROL* pSpeed, ST_WEAK_CONTROL* pWEAK)
{
    Q32I_ Idtmp = 0;
    Q32I_ Iqtmp = 0;
    
    pWEAK->PidV.Q14I_Rf = pFocPara->Q14I_Vs;
//    pWEAK->PidV.Q14I_Fb = Math_Sqrt(pFocPara->Q14I_Ud*pFocPara->Q14I_Ud + pFocPara->Q14I_Uq*pFocPara->Q14I_Uq);
    PID_Pos_Cal(&pWEAK->PidV);
    pWEAK->Q12I_Theta = pWEAK->PidV.Q14I_Output + MATH_2PI_U;
    
//    Idtmp = pSpeed->Q14I_IqRef * Math_Sin(pWEAK->Q12I_Theta);
//    Iqtmp = pSpeed->Q14I_IqRef * Math_Cos(pWEAK->Q12I_Theta);
    
    if(pWEAK->Q12I_Theta >= MATH_2PI_U)
    {
//        pWEAK->Q14I_IdRef = pMTPA->Q14I_IdRef;
//        pWEAK->Q14I_IqRef = pMTPA->Q14I_IqRef;
    }
    else
    {
        pWEAK->Q14I_IdRef = Idtmp;
        pWEAK->Q14I_IqRef = Iqtmp;
    }
}

void MotorFoc_IF_Init(ST_IF_CONTROL* pCTRL)
{
    pCTRL->Angle.Q12U_Angle = 0;
    pCTRL->Est_AngleRad = 0;
    pCTRL->AngleRad_cnt = 0U;
    pCTRL->IF_Success_Flag = 0U;
    Ramp_Init(&pCTRL->SRad_Ramp, pCTRL->SRad_Ramp.Q32I_Init);
}

void MotorFoc_IF_Control(ST_PMSM_PARAMETER* pPMSMPara, ST_IF_CONTROL* pCTRL)
{
    pCTRL->Q28I_Angle_tmp += pPMSMPara->Q14I_Motor_HTs*pCTRL->SRad_Ramp.Q32I_Output;
    MATH_ANGLE_TMP_U(pCTRL->Q28I_Angle_tmp);
    pCTRL->Angle.Q12U_Angle = Q32I_RHT_16(pCTRL->Q28I_Angle_tmp);
    Math_SinCos(&pCTRL->Angle);
}

void MotorFoc_Flux_Init(ST_FLUX_CONTROL* pCTRL)
{
    pCTRL->Angle.Q12U_Angle = 0;
    pCTRL->Q28I_Angle_tmp = 0;
    pCTRL->FL_SRAD.Q16I_Filter_out = 0;
    pCTRL->FL_SRAD.Q32I_Filter_tmp = 0;
    
    pCTRL->Q28I_Ref_Yalfa = 0;
    pCTRL->Q28I_Ref_Ybeta = 0;
    
    pCTRL->Q14I_Nn_alfa = 0;
    pCTRL->Q14I_Nn_beta = 0;
    pCTRL->Q14I_Nn_2 = 0;
    
    pCTRL->Q28I_Xalfa_addtmp = 0;
    pCTRL->Q28I_Xbeta_addtmp = 0;
    
    pCTRL->Q28I_Est_Xalfatmp1 = 0;
    pCTRL->Q28I_Est_Xbetatmp1 = 0;
    pCTRL->Q28I_Est_Xalfatmp2 = 0;
    pCTRL->Q28I_Est_Xbetatmp2 = 0;
    pCTRL->Q14I_Est_Xalfa = 0;
    pCTRL->Q14I_Est_Xbeta = 0;
    
    PID_Pos_Init(&pCTRL->PLL_PID, 0);
}

void MotorFoc_Flux_Control(ST_PMSM_PARAMETER* pPMSMPara, ST_FOC_PARAMETER* pFocPara, ST_FLUX_CONTROL* pCTRL)
{
}

void MotorFoc_SMO_Init(ST_SMO_CONTROL* pCTRL)
{
    pCTRL->Angle.Q12U_Angle = 0;
    pCTRL->Q28I_Angle_tmp = 0;
    pCTRL->FL_SRAD.Q16I_Filter_out = 0;
    pCTRL->FL_SRAD.Q32I_Filter_tmp = 0;
    
    pCTRL->Q28I_Est_Ialfa_tmp = 0;
    pCTRL->Q28I_Est_Ibeta_tmp = 0;
    pCTRL->Q14I_Est_Ialfa = 0;
    pCTRL->Q14I_Est_Ibeta = 0;  
    pCTRL->Q14I_Est_Ealfa = 0;
    pCTRL->Q14I_Est_Ebeta = 0;  
    
    PID_Pos_Init(&pCTRL->PLL_PID, 0);
}

void MotorFoc_SMO_Control(ST_PMSM_PARAMETER* pPMSMPara, ST_FOC_PARAMETER* pFocPara, ST_SMO_CONTROL* pCTRL)
{
}

void MotorFoc_Speed_Init(ST_SRAD_CONTROL* pSpeed)
{
    pSpeed->Q14I_SRAD_Rf = 0;
    pSpeed->Q14I_SRAD_Fb = 0;
    pSpeed->Q14I_IdRef = 0;
    pSpeed->Q14I_IqRef = 0;
    
    Ramp_Init(&pSpeed->Id_Ramp, pSpeed->Id_Ramp.Q32I_Init);
    Ramp_Init(&pSpeed->Iq_Ramp, 0);
    Ramp_Init(&pSpeed->SRad_Ramp, 0);
    PID_Pos_Init(&pSpeed->PidSpd, 0);
}

void MotorFoc_Speed_Loop(ST_SRAD_CONTROL* pSpeed)
{
}

void MotorFoc_Current_Init(ST_CURRENT_CONTROL* pCurrentCtrl)
{
    pCurrentCtrl->Q14I_IdRef = 0;
    pCurrentCtrl->Q14I_IqRef = 0;
    
    PID_Pos_Init(&pCurrentCtrl->PidId, 0);
    PID_Pos_Init(&pCurrentCtrl->PidIq, 0);
}

void MotorFoc_Current_Loop(ST_FOC_PARAMETER* pFocPara, ST_CURRENT_CONTROL* pCurrentCtrl)
{
}
