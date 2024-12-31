/**************************************************************************************************
*     File Name :                        BSP_WDG.h
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             WDG��ʼ����Ӧ�ò�ӿ�ͷ�ļ�
**************************************************************************************************/
#ifndef BSP_WDG_H
#define BSP_WDG_H

#include "MotorHal_cfg.h"

#define HAL_WDOG_WIN_WINDOWVALUE (36000U)
#define HAL_WDOG_WIN_TIMEOUTVALUE (400000U)     //10ms�����4msιһ�ι�

/**********************************************************************************************
Function: BSP_FeedWatchDog
Description: ι������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void BSP_FeedWatchDog(void)
{
    if(WDOG_GetCounter() > HAL_WDOG_WIN_WINDOWVALUE)
    {
        WDOG_Refresh();
    }
}

/**********************************************************************************************
Function: BSP_WDG_Init
Description: WDG��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_WDG_Init(void);

#endif /* BSP_WDG_H */
