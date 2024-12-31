/**************************************************************************************************
*     File Name :                        BSP_GPIO.h
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             GPIO��ʼ����Ӧ�ò�ӿ�ͷ�ļ�
**************************************************************************************************/
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "MotorHal_cfg.h"

/**********************************************************************************************
Function: BSP_GPIO_Init
Description: GPIO��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_GPIO_Init(void);

/**********************************************************************************************
Function: BSP_GPIO_Read_SW0_State
Description: ����0״̬��ȡ
Input: ��
Output: ��
Input_Output: ��
Return: ����0״̬
Author: CJYS
***********************************************************************************************/
Q32U_ BSP_GPIO_Read_SW0_State(void);

/**********************************************************************************************
Function: BSP_GPIO_Read_SW1_State
Description: ����1״̬��ȡ
Input: ��
Output: ��
Input_Output: ��
Return: ����1״̬
Author: CJYS
***********************************************************************************************/
Q32U_ BSP_GPIO_Read_SW1_State(void);

/**********************************************************************************************
Function: BSP_GPIO_Read_SW2_State
Description: ����2״̬��ȡ
Input: ��
Output: ��
Input_Output: ��
Return: ����2״̬
Author: CJYS
***********************************************************************************************/
Q32U_ BSP_GPIO_Read_SW2_State(void);

#endif /* BSP_GPIO_H */
