/**************************************************************************************************
*     File Name :                        BSP_TMU.c
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             TMU��ʼ����Ӧ�ò�ӿ�Դ�ļ�
**************************************************************************************************/
#include "BSP_TMU.h"

/**********************************************************************************************
Function: BSP_TMU_Init
Description: ���������TMU��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_TMU_Init(void)
{
    TMU_SetUnlockForModule(TMU_MODULE_TDG0_TRIG_IN);
    TMU_SetSourceForModule(TMU_SOURCE_MCPWM1_INIT_TRIG0, TMU_MODULE_TDG0_TRIG_IN);
    TMU_ModuleCmd(TMU_MODULE_TDG0_TRIG_IN, ENABLE);
    TMU_SetLockForModule(TMU_MODULE_TDG0_TRIG_IN);
}
