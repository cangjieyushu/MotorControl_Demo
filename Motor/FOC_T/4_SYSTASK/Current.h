/**************************************************************************************************/
/**
 * @copyright : 
 **************************************************************************************************/
#ifndef Current_H
#define Current_H

#include "BSP.h"
#include "SysTask.h"
#include "User_Pmsm.h"
#include "MotorTask.h"

//��������
//����ʱ����ź����ʱ�����������ڣ�ֻ��Ҫ�����֮ǰ��ʱ�伴��
//��32˵��������32msִ��һ�Σ���С�ֱ��ʾ���32ms��Ȼ��ϵͳ��ȡ��
//������96������96ms��������120Ҳ��96ms������ֻ����128������128ms����
#define CURRENT_PROTECT_LEVEL           3                                   //����������λ

#define CURRENT_PROTECT_LEVEL_1_TL      (Q14I_CURRENT_MOTOR_TO_PU(30.0f))   //A������������ֵ
#define CURRENT_PROTECT_LEVEL_1_TIME    ( 100.0f /32.0f)                    //ms����������ʱ��

#define CURRENT_PROTECT_LEVEL_2_TL      (Q14I_CURRENT_MOTOR_TO_PU(35.0f))   //A������������ֵ
#define CURRENT_PROTECT_LEVEL_2_TIME    ( 1000.0f /32.0f)                   //ms����������ʱ��

#define CURRENT_PROTECT_LEVEL_3_TL      (Q14I_CURRENT_MOTOR_TO_PU(40.0f))   //A������������ֵ
#define CURRENT_PROTECT_LEVEL_3_TIME    ( 10000.0f /32.0f)                   //ms����������ʱ��

typedef struct{
    Q32U_ Q16U_current_protect_tl;
    Q32U_ Q16U_current_protect_time;
    
    Q32U_ Q16U_current_protect_cnt;
}ST_CURRENT_PROTECT;

void Current_Protect_Flow(ST_SYSTEM_TASK*  pST);
    
#endif /* Current_H */
