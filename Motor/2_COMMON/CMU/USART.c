/**************************************************************************************************
*     File Name :                        USART.c
*     Library/Module Name :              COMMON
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ����ͨѶԴ�ļ�
**************************************************************************************************/
#include "USART.h"

ST_USART_CONTROL USART_Ctrl_1 = {
    .rxdata_maxlength = RESCEIVE_DATA_LENGTH,
    .txdata_maxlength = TRANSMISSION_DATA_LENGTH,
};

ST_USART_CONTROL USART_Ctrl_2 = {
    .rxdata_maxlength = RESCEIVE_DATA_LENGTH,
    .txdata_maxlength = TRANSMISSION_DATA_LENGTH,
};

/**********************************************************************************************
Function: Cal_CRC8
Description: CRC8У��
Input: ����
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
Q08U_ Cal_CRC8(const Q08U_ data)
{
    Q08U_ i, crc;
    crc = data;
    /* ������������8λ����Ҫ����8�� */
    for (i = 8; i > 0; i--) {
        /* �ж����λ�Ƿ�Ϊ1 */
        if(crc & 0x80)
        {
        /* ���λΪ1������Ҫ���������һλ��Ȼ����0x2f��� */
        /* 0x12f(����ʽ��x8 + x5 + x3 + x2 + x + 1,  100101111)�����λ����Ҫ���ֱ��ȥ�� */
            crc = (crc << 1) ^ 0x2f;
        } 
        else
        {
            /* ���λΪ0ʱ������Ҫ�����������������һλ */
            crc = (crc << 1);
        }
    }
    return crc;
}

/**********************************************************************************************
Function: USART_Get_Resceive_Data_1
Description: ����1��������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Get_Resceive_Data_1(void)
{
    ST_USART_CONTROL* pUC = &USART_Ctrl_1;
    
    switch(pUC->USART_Resceive_Flow)
    {
        case USART_STATE_IDLE:
        {
            if(pUC->USART_State.BIT.resceive_enable == 1U)
            {
                if(++pUC->rxdata_cnt > 3U)
                {
                    pUC->rxdata_cnt = 0U;
                    USART1_Enable_Rx();
                    pUC->USART_Resceive_Flow = USART_STATE_RUN;
                }
            }
        }break;
        case USART_STATE_RUN:
        {
            if(pUC->rxdata_length_tmp >= pUC->rxdata_maxlength)
            {
                USART1_Disable_Rx();
                pUC->USART_Resceive_Flow = USART_STATE_ERROR;
            }
            if((pUC->rxdata_length_tmp == pUC->rxdata_length_last)
            && (pUC->rxdata_length_tmp != 0U))
            {
                if(++pUC->rxdata_cnt > 3)
                {
                    pUC->rxdata_cnt = 0;
                    USART1_Disable_Rx();
                    pUC->USART_Resceive_Flow = USART_STATE_END;
                }
            }
            pUC->rxdata_length_last = pUC->rxdata_length_tmp;
        }break;
        case USART_STATE_END:
        {
//            pUC->txdata[0] = pUC->rxdata[0];
//            pUC->txdata[1] = pUC->rxdata[1];
//            pUC->txdata[2] = pUC->rxdata[2];
//            pUC->txdata[3] = pUC->rxdata[3];
//            pUC->txdata[4] = pUC->rxdata[4];
            
            pUC->rxdata_length_last = 0U;
            pUC->rxdata_length_tmp = 0U;
            
            pUC->USART_State.BIT.resceive_enable = 0U;
            pUC->USART_State.BIT.transmission_enable = 1U;
            pUC->USART_Resceive_Flow = USART_STATE_IDLE;
        }break;
        case USART_STATE_ERROR:
        {
            pUC->rxdata_length_tmp = 0U;
            pUC->rxdata_length_last = 0U;
            pUC->rxdata_cnt = 0U;
            pUC->USART_Resceive_Flow = USART_STATE_IDLE;
        }break;
        default:
            break;
    }
}

/**********************************************************************************************
Function: USART_Send_Transmission_Data_1
Description: ����1��������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Send_Transmission_Data_1(void)
{
    ST_USART_CONTROL* pUC = &USART_Ctrl_1;
    
    switch(pUC->USART_Transmission_Flow)
    {
        case USART_STATE_IDLE:
        {
            if(pUC->USART_State.BIT.transmission_enable == 1U)
            {
                if(++pUC->txdata_cnt > 500U)
                {
                    pUC->txdata_cnt = 0U;
                    pUC->txdata_length_tmp = 0U;
                    pUC->txdata_length_last = 0U;
                    
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[0];
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[1];
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[2];
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[3];
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[4];
                    
                    pUC->txdata[pUC->txdata_length_tmp++] = 1;
                    pUC->txdata[pUC->txdata_length_tmp++] = 3;
                    pUC->txdata[pUC->txdata_length_tmp++] = 5;
                    pUC->txdata[pUC->txdata_length_tmp++] = 7;
                    pUC->txdata[pUC->txdata_length_tmp++] = 9;
                    
                    USART1_TRANSMISSION_DATA = pUC->txdata[0];
                    USART1_Enable_Tx(); 
                    
                    pUC->USART_Transmission_Flow = USART_STATE_RUN;
                }
            }
            if(++pUC->error_cnt > 1000U)
            {
                pUC->error_cnt = 0U;
                pUC->USART_Transmission_Flow = USART_STATE_ERROR;
            }
        }break;
        case USART_STATE_RUN:
        {
            if(pUC->rxdata_length_last == pUC->rxdata_length_tmp)
            {
                pUC->USART_Transmission_Flow = USART_STATE_END;
            }
        }break;
        case USART_STATE_END:
        {
            pUC->USART_State.BIT.resceive_enable = 1U;
            pUC->USART_Transmission_Flow = USART_STATE_IDLE;
        }break;
        case USART_STATE_ERROR:
        {
            pUC->USART_State.BIT.resceive_enable = 1U;
            pUC->USART_Transmission_Flow = USART_STATE_IDLE;
        }break;
        default:
            break;
    }
}

/**********************************************************************************************
Function: USART_Get_Resceive_Data_2
Description: ����2��������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Get_Resceive_Data_2(void)
{
    ST_USART_CONTROL* pUC = &USART_Ctrl_2;
    
    switch(pUC->USART_Resceive_Flow)
    {
        case USART_STATE_IDLE:
        {
            if(pUC->USART_State.BIT.resceive_enable == 1U)
            {
                if(++pUC->rxdata_cnt > 3U)
                {
                    pUC->rxdata_cnt = 0U;
                    USART2_Enable_Rx();
                    pUC->USART_Resceive_Flow = USART_STATE_RUN;
                }
            }
        }break;
        case USART_STATE_RUN:
        {
            if(pUC->rxdata_length_tmp >= pUC->rxdata_maxlength)
            {
                USART2_Disable_Rx();
                pUC->USART_Resceive_Flow = USART_STATE_ERROR;
            }
            if((pUC->rxdata_length_tmp == pUC->rxdata_length_last)
            && (pUC->rxdata_length_tmp != 0U))
            {
                if(++pUC->rxdata_cnt > 3)
                {
                    pUC->rxdata_cnt = 0;
                    USART2_Disable_Rx();
                    pUC->USART_Resceive_Flow = USART_STATE_END;
                }
            }
            pUC->rxdata_length_last = pUC->rxdata_length_tmp;
        }break;
        case USART_STATE_END:
        {
//            pUC->txdata[0] = pUC->rxdata[0];
//            pUC->txdata[1] = pUC->rxdata[1];
//            pUC->txdata[2] = pUC->rxdata[2];
//            pUC->txdata[3] = pUC->rxdata[3];
//            pUC->txdata[4] = pUC->rxdata[4];
            
            pUC->rxdata_length_last = 0U;
            pUC->rxdata_length_tmp = 0U;
            
            pUC->USART_State.BIT.resceive_enable = 0U;
            pUC->USART_State.BIT.transmission_enable = 1U;
            pUC->USART_Resceive_Flow = USART_STATE_IDLE;
        }break;
        case USART_STATE_ERROR:
        {
            pUC->rxdata_length_tmp = 0U;
            pUC->rxdata_length_last = 0U;
            pUC->rxdata_cnt = 0U;
            pUC->USART_Resceive_Flow = USART_STATE_IDLE;
        }break;
        default:
            break;
    }
}

/**********************************************************************************************
Function: USART_Send_Transmission_Data_2
Description: ����2��������
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Send_Transmission_Data_2(void)
{
    ST_USART_CONTROL* pUC = &USART_Ctrl_2;
    
    switch(pUC->USART_Transmission_Flow)
    {
        case USART_STATE_IDLE:
        {
            if(pUC->USART_State.BIT.transmission_enable == 1U)
            {
                if(++pUC->txdata_cnt > 500U)
                {
                    pUC->txdata_cnt = 0U;
                    pUC->txdata_length_tmp = 0U;
                    pUC->txdata_length_last = 0U;
                    
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[0];
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[1];
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[2];
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[3];
//                    pUC->txdata[pUC->txdata_length_tmp++] = pUC->rxdata[4];
                    
                    pUC->txdata[pUC->txdata_length_tmp++] = 1;
                    pUC->txdata[pUC->txdata_length_tmp++] = 3;
                    pUC->txdata[pUC->txdata_length_tmp++] = 5;
                    pUC->txdata[pUC->txdata_length_tmp++] = 7;
                    pUC->txdata[pUC->txdata_length_tmp++] = 9;
                    
                    USART1_TRANSMISSION_DATA = pUC->txdata[0];
                    USART2_Enable_Tx(); 
                    
                    pUC->USART_Transmission_Flow = USART_STATE_RUN;
                }
            }
            if(++pUC->error_cnt > 1000U)
            {
                pUC->error_cnt = 0U;
                pUC->USART_Transmission_Flow = USART_STATE_ERROR;
            }
        }break;
        case USART_STATE_RUN:
        {
            if(pUC->rxdata_length_last == pUC->rxdata_length_tmp)
            {
                pUC->USART_Transmission_Flow = USART_STATE_END;
            }
        }break;
        case USART_STATE_END:
        {
            pUC->USART_State.BIT.resceive_enable = 1U;
            pUC->USART_Transmission_Flow = USART_STATE_IDLE;
        }break;
        case USART_STATE_ERROR:
        {
            pUC->USART_State.BIT.resceive_enable = 1U;
            pUC->USART_Transmission_Flow = USART_STATE_IDLE;
        }break;
        default:
            break;
    }
}

/**********************************************************************************************
Function: USART_Resceive_Int_1
Description: ����1���������ж�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Resceive_Int_1(void)
{
    ST_USART_CONTROL* pUC = &USART_Ctrl_1;
    pUC->rxdata[pUC->rxdata_length_tmp++] = USART1_RESCEIVE_DATA; 
}

/**********************************************************************************************
Function: USART_Transmission_Int_1
Description: ����1���������ж�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Transmission_Int_1(void)
{
    ST_USART_CONTROL* pUC = &USART_Ctrl_1;
    USART1_TRANSMISSION_DATA = pUC->txdata[++pUC->txdata_length_last];
    if(pUC->txdata_length_last == pUC->txdata_length_tmp)
    {
        USART1_Disable_Tx();
    }
}

/**********************************************************************************************
Function: USART_Resceive_Int_2
Description: ����2���������ж�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Resceive_Int_2(void)
{
    ST_USART_CONTROL* pUC = &USART_Ctrl_2;
    pUC->rxdata[pUC->rxdata_length_tmp++] = USART2_RESCEIVE_DATA; 
}

/**********************************************************************************************
Function: USART_Transmission_Int_2
Description: ����2���������ж�
Input: ��
Output: ��
Input_Output: ��
Return: ��
Author: CJYS
***********************************************************************************************/
void USART_Transmission_Int_2(void)
{
    ST_USART_CONTROL* pUC = &USART_Ctrl_2;
    USART1_TRANSMISSION_DATA = pUC->txdata[++pUC->txdata_length_last];
    if(pUC->txdata_length_last == pUC->txdata_length_tmp)
    {
        USART2_Disable_Tx();
    }
}
