/**************************************************************************************************
*     File Name :                        BSP_WDG.c
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             WDG��ʼ����Ӧ�ò�ӿ�Դ�ļ�
**************************************************************************************************/
#include "BSP_WDG.h"

/**********************************************************************************************
Function: BSP_WDG_Init
Description: WDG��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_WDG_Init(void)
{
    WDOG_Config_t WDOGInitConfig;
    /* Choose clock source of the WDOG */
    WDOGInitConfig.clkSource = WDOG_OSC_CLOCK;
    /* Disable wait mode */
    WDOGInitConfig.wait = DISABLE;
    /* Disable stop mode */
    WDOGInitConfig.stop = DISABLE;
    /* Disable debug mode */
    WDOGInitConfig.debug = DISABLE;
    /* Enable window mode */
    WDOGInitConfig.winEnable = ENABLE;
    /* The timeout value */
    WDOGInitConfig.timeoutValue = HAL_WDOG_WIN_TIMEOUTVALUE;
    /* The window value */
    WDOGInitConfig.windowValue = HAL_WDOG_WIN_WINDOWVALUE;
    /* Normal mode */
    WDOGInitConfig.testMode = WDOG_TST_NORMAL;

    /* Initialize WDOG */
    WDOG_Init(&WDOGInitConfig);

    /* Mask WDOG interrupt */
    WDOG_IntMask(WDOG_INT_BEFORERESET, MASK);
    
    WDOG_Enable();
}
