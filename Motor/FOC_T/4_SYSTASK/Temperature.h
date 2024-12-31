/**************************************************************************************************/
/**
 * @copyright : 
 **************************************************************************************************/
#ifndef Temperature_H
#define Temperature_H

#include "BSP.h"
#include "SysTask.h"

//�¶ȱ���
//����ʱ����ź����ʱ�����������ڣ�ֻ��Ҫ�����֮ǰ��ʱ�伴��
//��32˵��������32msִ��һ�Σ���С�ֱ��ʾ���32ms��Ȼ��ϵͳ��ȡ��
//������96������96ms��������120Ҳ��96ms������ֻ����128������128ms����
#define OVER_TEMP_PROTECT_LEVEL_TL       (10.0f)            //lsb�����±�����ֵ
#define OVER_TEMP_PROTECT_LEVEL_TIME     (100.0f/128.0f)    //ms�����±���ʱ��

#define LOW_TEMP_PROTECT_LEVEL_TL        (15.0f)            //lsb�����±�����ֵ
#define LOW_TEMP_PROTECT_LEVEL_TIME      (1000.0f/128.0f)   //ms�����±���ʱ��

typedef struct{
    Q32I_ Q16I_temp_protect_tl;
    Q32U_ Q16U_temp_protect_time;
    
    Q32U_ Q16U_temp_protect_cnt;
}ST_TEMP_PROTECT;

void Temperature_Protect_Flow(ST_SYSTEM_TASK*  pST);
    
#endif /* Temperature_H */
