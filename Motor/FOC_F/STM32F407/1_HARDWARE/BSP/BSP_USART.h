/**************************************************************************************************
*     File Name :                        BSP_USART.h
*     Library/Module Name :              BSP
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             USART��ʼ����Ӧ�ò�ӿ�ͷ�ļ�
**************************************************************************************************/
#ifndef BSP_USART_H
#define BSP_USART_H

#include "MotorHal_cfg.h"

#define USART1_RESCEIVE_DATA        USART1->DR
#define USART1_TRANSMISSION_DATA    USART1->DR

#define USART2_RESCEIVE_DATA        USART2->DR
#define USART2_TRANSMISSION_DATA    USART2->DR

/**********************************************************************************************
Function: USART1_Enable_Rx
Description: USART1�򿪽���
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void USART1_Enable_Rx(void)
{
    SET_BIT(USART1->CR1, USART_CR1_RE);
}

/**********************************************************************************************
Function: USART1_Disable_Rx
Description: USART1�رս���
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void USART1_Disable_Rx(void)
{
    CLEAR_BIT(USART1->CR1, USART_CR1_RE);
}

/**********************************************************************************************
Function: USART1_Enable_Tx
Description: USART1�򿪷���
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void USART1_Enable_Tx(void)
{
    SET_BIT(USART1->CR1, USART_CR1_TE);
}

/**********************************************************************************************
Function: USART1_Disable_Tx
Description: USART1�رշ���
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void USART1_Disable_Tx(void)
{
    CLEAR_BIT(USART1->CR1, USART_CR1_TE);
}

/**********************************************************************************************
Function: USART2_Enable_Rx
Description: USART2�򿪽���
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void USART2_Enable_Rx(void)
{
    SET_BIT(USART2->CR1, USART_CR1_RE);
}

/**********************************************************************************************
Function: USART2_Disable_Rx
Description: USART2�رս���
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void USART2_Disable_Rx(void)
{
    CLEAR_BIT(USART2->CR1, USART_CR1_RE);
}

/**********************************************************************************************
Function: USART2_Enable_Tx
Description: USART2�򿪷���
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void USART2_Enable_Tx(void)
{
    SET_BIT(USART2->CR1, USART_CR1_TE);
}

/**********************************************************************************************
Function: USART2_Disable_Tx
Description: USART2�رշ���
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
static inline void USART2_Disable_Tx(void)
{
    CLEAR_BIT(USART2->CR1, USART_CR1_TE);
}

/**********************************************************************************************
Function: BSP_USART_Init
Description: USART��ʼ��
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void BSP_USART_Init(void);

#endif /* BSP_USART_H */
