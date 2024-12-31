/**************************************************************************************************
*     File Name :                        USART.h
*     Library/Module Name :              COMMON
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ����ͨѶͷ�ļ�
**************************************************************************************************/
#ifndef USART_H
#define USART_H

#include "Math.h"
#include "BSP.h"

#define RESCEIVE_DATA_LENGTH        (30U)
#define TRANSMISSION_DATA_LENGTH    (30U)

typedef enum{
    USART_STATE_IDLE,
    USART_STATE_RUN,
    USART_STATE_END,
    USART_STATE_ERROR,
}EM_USART_STATE_FLOW;

typedef union{
    ALL ALL;
    struct{
        BIT        resceive_enable         :1;
        BIT        transmission_enable     :1;
    }BIT;
}UN_USART_STATE_FLAG;

typedef struct{
    EM_USART_STATE_FLOW USART_Resceive_Flow;
    EM_USART_STATE_FLOW USART_Transmission_Flow;
    UN_USART_STATE_FLAG USART_State;
    
    Q08U_ rxdata[RESCEIVE_DATA_LENGTH];
    Q08U_ txdata[TRANSMISSION_DATA_LENGTH];

    Q32U_ rxdata_length_tmp;
    Q32U_ txdata_length_tmp;
    
    Q32U_ rxdata_length_last;
    Q32U_ txdata_length_last;

    Q32U_ rxdata_maxlength;
    Q32U_ txdata_maxlength;
    
    Q32U_ rxdata_cnt;
    Q32U_ txdata_cnt;
    
    Q32U_ error_cnt;
}ST_USART_CONTROL;

/**********************************************************************************************
Function: USART_Get_Resceive_Data_1
Description: ����1��������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Get_Resceive_Data_1(void);

/**********************************************************************************************
Function: USART_Send_Transmission_Data_1
Description: ����1��������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Send_Transmission_Data_1(void);
/**********************************************************************************************
Function: USART_Resceive_Int_1
Description: ����1���������ж�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Resceive_Int_1(void);

/**********************************************************************************************
Function: USART_Transmission_Int_1
Description: ����1���������ж�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Transmission_Int_1(void);

/**********************************************************************************************
Function: USART_Get_Resceive_Data_2
Description: ����2��������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Get_Resceive_Data_2(void);

/**********************************************************************************************
Function: USART_Send_Transmission_Data_2
Description: ����2��������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Send_Transmission_Data_2(void);

/**********************************************************************************************
Function: USART_Resceive_Int_2
Description: ����2���������ж�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Resceive_Int_2(void);

/**********************************************************************************************
Function: USART_Transmission_Int_2
Description: ����2���������ж�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Transmission_Int_2(void);
    
#endif /* USART_H */
