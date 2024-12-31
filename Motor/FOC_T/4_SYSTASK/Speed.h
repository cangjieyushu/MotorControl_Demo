/**************************************************************************************************/
/**
 * @copyright : 
 **************************************************************************************************/
#ifndef Speed_H
#define Speed_H

#include "BSP.h"
#include "SysTask.h"
#include "User_Pmsm.h"
#include "MotorTask.h"

//�ٶȱ���
//����ʱ����ź����ʱ�����������ڣ�ֻ��Ҫ�����֮ǰ��ʱ�伴��
//��32˵��������32msִ��һ�Σ���С�ֱ��ʾ���32ms��Ȼ��ϵͳ��ȡ��
//������96������96ms��������120Ҳ��96ms������ֻ����128������128ms����
#define OVER_SPEED_PROTECT_LEVEL_TL         (Q14I_SRAD_MOTOR_TO_PU(60000.0f))
#define OVER_SPEED_PROTECT_LEVEL_TIME       (1000.0f/64.0f)

#define LOW_SPEED_PROTECT_LEVEL_TL          (Q14I_SRAD_MOTOR_TO_PU(10.0f))
#define LOW_SPEED_PROTECT_LEVEL_TIME        (1000.0f/64.0f)

typedef struct{
    Q32I_ Q16I_speed_protect_tl;
    Q32U_ Q16U_speed_protect_time;
    
    Q32U_ Q16U_speed_protect_cnt;
}ST_SPEED_PROTECT;

void Speed_Protect_Flow(ST_SYSTEM_TASK*  pST);
    
#endif /* Speed_H */
