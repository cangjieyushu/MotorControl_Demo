/**************************************************************************************************/
/**
 * @copyright : 
 **************************************************************************************************/
#ifndef Voltage_H
#define Voltage_H

#include "BSP.h"
#include "SysTask.h"
#include "User_Pmsm.h"

//��ѹ����
//����ʱ����ź����ʱ�����������ڣ�ֻ��Ҫ�����֮ǰ��ʱ�伴��
//��32˵��������32msִ��һ�Σ���С�ֱ��ʾ���32ms��Ȼ��ϵͳ��ȡ��
//������96������96ms��������120Ҳ��96ms������ֻ����128������128ms����
#define LOW_VOLTAGE_PROTECT_LEVEL_TL        (Q14I_VOLTAGE_MOTOR_TO_PU(10.0f))    //V����ѹ������ֵ
#define LOW_VOLTAGE_PROTECT_LEVEL_TIME      (1000.0f/16.0f)                      //ms����ѹ����ʱ��

#define OVER_VOLTAGE_PROTECT_LEVEL_TL       (Q14I_VOLTAGE_MOTOR_TO_PU(28.0f))    //V����ѹ������ֵ
#define OVER_VOLTAGE_PROTECT_LEVEL_TIME     (100.0f/16.0f)                       //ms����ѹ����ʱ��

typedef struct{
    Q32I_ Q16I_voltage_protect_tl;
    Q32U_ Q16U_voltage_protect_time;
    
    Q32U_ Q16U_voltage_protect_cnt;
}ST_VOLTAGE_PROTECT;

void Voltage_Protect_Flow(ST_SYSTEM_TASK*  pST);
    
#endif /* Voltage_H */
