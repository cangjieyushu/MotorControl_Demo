/**************************************************************************************************
*     File Name :                        MotorFoc.c
*     Library/Module Name :              Motor
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             FOC�㷨Դ�ļ�
**************************************************************************************************/
#include "MotorFoc.h"

/**********************************IF����************************************/

/**********************************************************************************************
Function: MotorFoc_IF_Init_F
Description: IF��ʼ��
Input: ��
Output: ��
Input_Output: IF����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_IF_Init_F(ST_IF_CONTROL_F* pCTRL)
{
    pCTRL->_O_F_Angle = 0.0f;
    Ramp_Init_F(&pCTRL->Ramp_Iq, 0.0f);
    Ramp_Init_F(&pCTRL->Ramp_SRAD, 0.0f);
    Ramp_Init_F(&pCTRL->Ramp_AngleERR, 0.0f);
}

/**********************************************************************************************
Function: MotorFoc_IF_OPEN_F
Description: IF�������ƺ���
Input: ��
Output: ��
Input_Output: IF����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorFoc_IF_OPEN_F(ST_IF_CONTROL_F* pCTRL)
{
    Ramp_Cal_F(&pCTRL->Ramp_Iq);
    Ramp_Cal_F(&pCTRL->Ramp_SRAD);
}

/**********************************************************************************************
Function: MotorFoc_IF_CLOSE_F
Description: IF�ջ����ƺ���
Input: ��
Output: ��
Input_Output: IF����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorFoc_IF_CLOSE_F(ST_IF_CONTROL_F* pCTRL)
{
    Ramp_Cal_F(&pCTRL->Ramp_AngleERR);
}

/**********************************************************************************************
Function: MotorFoc_IF_CURRENT_F
Description: IF�������жϿ��ƺ���
Input: ��
Output: ��
Input_Output: IF����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_IF_CURRENT_F(ST_IF_CONTROL_F* pCTRL)
{
    pCTRL->_O_F_Angle += pCTRL->_I_F_DIR_Target*pCTRL->_P_F_Ts*pCTRL->Ramp_SRAD.F_Output;
    MATH_ANGLE_MOD_F(pCTRL->_O_F_Angle);
}

/**********************************VF����************************************/

/**********************************************************************************************
Function: MotorFoc_VF_Init_F
Description: VF��ʼ��
Input: ��
Output: ��
Input_Output: VF����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_VF_Init_F(ST_VF_CONTROL_F* pCTRL)
{
    pCTRL->_O_F_Angle = 0.0f;
    Ramp_Init_F(&pCTRL->Ramp_Vq, 0.0f);
    Ramp_Init_F(&pCTRL->Ramp_SRAD, 0.0f);
    Ramp_Init_F(&pCTRL->Ramp_AngleERR, 0.0f);
}

/**********************************************************************************************
Function: MotorFoc_VF_OPEN_F
Description: VF�������ƺ���
Input: ��
Output: ��
Input_Output: VF����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorFoc_VF_OPEN_F(ST_VF_CONTROL_F* pCTRL)
{
    Ramp_Cal_F(&pCTRL->Ramp_Vq);
    Ramp_Cal_F(&pCTRL->Ramp_SRAD);
}

/**********************************************************************************************
Function: MotorFoc_VF_CLOSE_F
Description: VF�ջ����ƺ���
Input: ��
Output: ��
Input_Output: VF����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorFoc_VF_CLOSE_F(ST_VF_CONTROL_F* pCTRL)
{
    Ramp_Cal_F(&pCTRL->Ramp_AngleERR);
}

/**********************************************************************************************
Function: MotorFoc_VF_CURRENT_F
Description: VF�������жϿ��ƺ���
Input: ��
Output: ��
Input_Output: VF����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_VF_CURRENT_F(ST_VF_CONTROL_F* pCTRL)
{
    pCTRL->_O_F_Angle += pCTRL->_I_F_DIR_Target*pCTRL->_P_F_Ts*pCTRL->Ramp_SRAD.F_Output;
    MATH_ANGLE_MOD_F(pCTRL->_O_F_Angle);
}

/*********************************����任*************************************/

/**********************************************************************************************
Function: MotorFoc_Clark_F
Description: Clark����任����
Input: ��
Output: ��
Input_Output: SVPWM����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_Clark_F(ST_SVPWM_CONTROL_F* pCTRL)
{
    pCTRL->_O_F_Ialfa = MATH_ONE_OVER_THREE_F*(2.0f*pCTRL->_I_F_Ia - pCTRL->_I_F_Ib - pCTRL->_I_F_Ic);
    pCTRL->_O_F_Ibeta = MATH_ONE_OVER_SQRT_THREE_F*(pCTRL->_I_F_Ib - pCTRL->_I_F_Ic);
}

/**********************************************************************************************
Function: MotorFoc_Park_F
Description: Park����任����
Input: ��
Output: ��
Input_Output: SVPWM����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_Park_F(ST_SVPWM_CONTROL_F* pCTRL)
{
    pCTRL->_O_F_Id =   pCTRL->_O_F_Ialfa*pCTRL->TG_Triangle.F_Cos + pCTRL->_O_F_Ibeta*pCTRL->TG_Triangle.F_Sin;
    pCTRL->_O_F_Iq = - pCTRL->_O_F_Ialfa*pCTRL->TG_Triangle.F_Sin + pCTRL->_O_F_Ibeta*pCTRL->TG_Triangle.F_Cos;
}

/**********************************************************************************************
Function: MotorFoc_Ipark_F
Description: Ipark����任����
Input: ��
Output: ��
Input_Output: SVPWM����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_Ipark_F(ST_SVPWM_CONTROL_F* pCTRL)
{
    pCTRL->_O_F_Ualfa = pCTRL->_I_F_Ud*pCTRL->TG_Triangle.F_Cos - pCTRL->_I_F_Uq*pCTRL->TG_Triangle.F_Sin;
    pCTRL->_O_F_Ubeta = pCTRL->_I_F_Ud*pCTRL->TG_Triangle.F_Sin + pCTRL->_I_F_Uq*pCTRL->TG_Triangle.F_Cos;
}

/*********************************SVPWM*************************************/

/**********************************************************************************************
Function: MotorFoc_SVPWM_ThreeShunt_F
Description: ����SVPWM
Input: ��
Output: ��
Input_Output: SVPWM����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_SVPWM_ThreeShunt_F(ST_SVPWM_CONTROL_F* pCTRL)
{
}

/********************************����������ع�**************************************/

/**********************************************************************************************
Function: MotorFoc_OneShunt_Cal_F
Description: ����������������
Input: ��
Output: ��
Input_Output: SVPWM����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_OneShunt_Cal_F(ST_SVPWM_CONTROL_F* pCTRL)
{
}

/**********************************************************************************************
Function: MotorFoc_SVPWM_OneShunt_F
Description: ����SVPWM
Input: ��
Output: ��
Input_Output: SVPWM����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_SVPWM_OneShunt_F(ST_SVPWM_CONTROL_F* pCTRL)
{
}

/********************************�ٶȻ�**************************************/

/**********************************************************************************************
Function: MotorFoc_SRAD_Init_F
Description: ����SVPWM
Input: ��
Output: ��
Input_Output: �ٶȻ�����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_SRAD_Init_F(ST_SRAD_CONTROL_F* pCTRL)
{
    PID_Pos_Init_F(&pCTRL->PID_SRAD, 0.0f);
    PID_Pos_Init_F(&pCTRL->PID_WEAK, 0.0f);
    Ramp_Init_F(&pCTRL->Ramp_SRAD, 0.0f);
}

/**********************************************************************************************
Function: MotorFoc_SRAD_Loop_F
Description: �ٶȻ�����
Input: ��
Output: ��
Input_Output: �ٶȻ�����ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
void MotorFoc_SRAD_Loop_F(ST_SRAD_CONTROL_F* pCTRL)
{
}

/*******************************������***************************************/

/**********************************************************************************************
Function: MotorFoc_Current_Init_F
Description: ����������
Input: ��
Output: ��
Input_Output: ����������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_Current_Init_F(ST_CURRENT_CONTROL_F* pCTRL)
{
    PID_Pos_Init_F(&pCTRL->PID_Id, 0.0f);
    PID_Pos_Init_F(&pCTRL->PID_Iq, 0.0f);
}

/**********************************************************************************************
Function: MotorFoc_Current_Loop_F
Description: ����������
Input: ��
Output: ��
Input_Output: ����������ָ��
Return: ��
Author: CJYS
***********************************************************************************************/
Ram_Func void MotorFoc_Current_Loop_F(ST_CURRENT_CONTROL_F* pCTRL)
{
}
