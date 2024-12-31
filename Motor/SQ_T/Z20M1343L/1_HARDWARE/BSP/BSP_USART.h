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

/*PRQA S 0303 ++*/
static Reg_Uart_WType* const Uart_RegWPtr = (Reg_Uart_WType*)UART_BASE_ADDR;
/*PRQA S 0303 --*/

#define USART1_RESCEIVE_DATA        Uart_RegWPtr->UART_RAR
#define USART1_TRANSMISSION_DATA    Uart_RegWPtr->UART_TAR

#define USART2_RESCEIVE_DATA        Uart_RegWPtr->UART_RAR
#define USART2_TRANSMISSION_DATA    Uart_RegWPtr->UART_TAR

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
