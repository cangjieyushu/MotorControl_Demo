/**************************************************************************************************
*     File Name :                        Voltage.h
*     Library/Module Name :              SysTask
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ��ѹ����ͷ�ļ�
**************************************************************************************************/
#ifndef Voltage_H
#define Voltage_H

#include "SysTask.h"

//��ѹ����
//����ʱ����ź����ʱ�����������ڣ�ֻ��Ҫ�����֮ǰ��ʱ�伴��
//��32˵��������32msִ��һ�Σ���С�ֱ��ʾ���32ms��Ȼ��ϵͳ��ȡ��
//������96������96ms��������120Ҳ��96ms������ֻ����128������128ms����
#define LOW_VOLTAGE_PROTECT_LEVEL_TL        (10.0f)                                     //V����ѹ������ֵ
#define LOW_VOLTAGE_PROTECT_LEVEL_TIME      ((Q32U_)(1000.0f/16.0f))                    //ms����ѹ����ʱ��

#define OVER_VOLTAGE_PROTECT_LEVEL_TL       (28.0f)                                     //V����ѹ������ֵ
#define OVER_VOLTAGE_PROTECT_LEVEL_TIME     ((Q32U_)(100.0f/16.0f))                     //ms����ѹ����ʱ��

typedef struct{
    float F_voltage_protect_tl;
    Q32U_ Q16U_voltage_protect_time;
    
    Q32U_ Q16U_voltage_protect_cnt;
}ST_VOLTAGE_PROTECT;

/**********************************************************************************************
Function: Voltage_Protect_Flow
Description: ��ѹ��������
Input: ��
Output: ��
Input_Output: ϵͳ״ָ̬��
Return: ��
Author: CJYS
***********************************************************************************************/
void Voltage_Protect_Flow(ST_SYSTEM_TASK*  pST);
    
#endif /* Voltage_H */
