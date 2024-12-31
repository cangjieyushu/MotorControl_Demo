/**************************************************************************************************
*     File Name :                        Temperature.h
*     Library/Module Name :              SysTask
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             �¶ȱ���ͷ�ļ�
**************************************************************************************************/
#ifndef Temperature_H
#define Temperature_H

#include "SysTask.h"

//�¶ȱ���
//����ʱ����ź����ʱ�����������ڣ�ֻ��Ҫ�����֮ǰ��ʱ�伴��
//��32˵��������32msִ��һ�Σ���С�ֱ��ʾ���32ms��Ȼ��ϵͳ��ȡ��
//������96������96ms��������120Ҳ��96ms������ֻ����128������128ms����
#define OVER_TEMP_PROTECT_LEVEL_TL       ((Q32U_)(10.0f))                   //lsb�����±�����ֵ
#define OVER_TEMP_PROTECT_LEVEL_TIME     ((Q32U_)(100.0f/128.0f))           //ms�����±���ʱ��

#define LOW_TEMP_PROTECT_LEVEL_TL        ((Q32U_)(15.0f))                   //lsb�����±�����ֵ
#define LOW_TEMP_PROTECT_LEVEL_TIME      ((Q32U_)(1000.0f/128.0f))          //ms�����±���ʱ��

typedef struct{
    Q32I_ Q16I_temp_protect_tl;
    Q32U_ Q16U_temp_protect_time;
    
    Q32U_ Q16U_temp_protect_cnt;
}ST_TEMP_PROTECT;

/**********************************************************************************************
Function: Temperature_Protect_Flow
Description: �¶ȱ�������
Input: ��
Output: ��
Input_Output: ϵͳ״ָ̬��
Return: ��
Author: CJYS
***********************************************************************************************/
void Temperature_Protect_Flow(ST_SYSTEM_TASK*  pST);
    
#endif /* Temperature_H */
