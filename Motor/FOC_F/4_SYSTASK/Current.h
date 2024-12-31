/**************************************************************************************************
*     File Name :                        Current.h
*     Library/Module Name :              SysTask
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ��������ͷ�ļ�
**************************************************************************************************/
#ifndef Current_H
#define Current_H

#include "SysTask.h"

//��������
//����ʱ����ź����ʱ�����������ڣ�ֻ��Ҫ�����֮ǰ��ʱ�伴��
//��32˵��������32msִ��һ�Σ���С�ֱ��ʾ���32ms��Ȼ��ϵͳ��ȡ��
//������96������96ms��������120Ҳ��96ms������ֻ����128������128ms����
#define CURRENT_PROTECT_LEVEL           3                                   //����������λ

#define CURRENT_PROTECT_LEVEL_1_TL      (30.0f)                             //A������������ֵ
#define CURRENT_PROTECT_LEVEL_1_TIME    ((Q32U_)(100.0f /32.0f))            //ms����������ʱ��

#define CURRENT_PROTECT_LEVEL_2_TL      (35.0f)                             //A������������ֵ
#define CURRENT_PROTECT_LEVEL_2_TIME    ((Q32U_)(1000.0f /32.0f))           //ms����������ʱ��

#define CURRENT_PROTECT_LEVEL_3_TL      (40.0f)                             //A������������ֵ
#define CURRENT_PROTECT_LEVEL_3_TIME    ((Q32U_)(10000.0f /32.0f))          //ms����������ʱ��

typedef struct{
    float F_current_protect_tl;
    Q32U_ Q16U_current_protect_time;
    
    Q32U_ Q16U_current_protect_cnt;
}ST_CURRENT_PROTECT;

/**********************************************************************************************
Function: Current_Protect_Flow
Description: ������������
Input: ��
Output: ��
Input_Output: ϵͳ״ָ̬��
Return: ��
Author: CJYS
***********************************************************************************************/
void Current_Protect_Flow(ST_SYSTEM_TASK*  pST);
    
#endif /* Current_H */
