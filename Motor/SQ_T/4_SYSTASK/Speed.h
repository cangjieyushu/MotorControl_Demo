/**************************************************************************************************
*     File Name :                        Speed.h
*     Library/Module Name :              SysTask
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             �ٶȱ���ͷ�ļ�
**************************************************************************************************/
#ifndef Speed_H
#define Speed_H

#include "SysTask.h"

//�ٶȱ���
//����ʱ����ź����ʱ�����������ڣ�ֻ��Ҫ�����֮ǰ��ʱ�伴��
//��32˵��������32msִ��һ�Σ���С�ֱ��ʾ���32ms��Ȼ��ϵͳ��ȡ��
//������96������96ms��������120Ҳ��96ms������ֻ����128������128ms����
#define OVER_SPEED_PROTECT_LEVEL_TL         (Q14I_FREQ_MOTOR_TO_PU(2000.0f))    //Hz�����ٱ�����ֵ
#define OVER_SPEED_PROTECT_LEVEL_TIME       ((Q32U_)(1000.0f/64.0f))            //ms�����ٱ���ʱ��

#define LOW_SPEED_PROTECT_LEVEL_TL          (Q14I_FREQ_MOTOR_TO_PU(10.0f))      //Hz�����ٱ�����ֵ
#define LOW_SPEED_PROTECT_LEVEL_TIME        ((Q32U_)(1000.0f/64.0f))            //ms�����ٱ���ʱ��

typedef struct{
    Q32I_ Q16I_speed_protect_tl;
    Q32U_ Q16U_speed_protect_time;
    
    Q32U_ Q16U_speed_protect_cnt;
}ST_SPEED_PROTECT;

/**********************************************************************************************
Function: Speed_Protect_Flow
Description: �ٶȱ�������
Input: ��
Output: ��
Input_Output: ϵͳ״ָ̬��
Return: ��
Author: CJYS
***********************************************************************************************/
void Speed_Protect_Flow(ST_SYSTEM_TASK*  pST);
    
#endif /* Speed_H */
