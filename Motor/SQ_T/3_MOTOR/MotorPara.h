/**************************************************************************************************
*     File Name :                        MotorPara.h
*     Library/Module Name :              Motor
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ������Ʋ�����ʼ��ͷ�ļ�
**************************************************************************************************/
#ifndef MotorPara_H
#define MotorPara_H

#include "MotorHal_cfg.h"
#include "MotorSQ.h"
#include "PmsmPara.h"

#define SYS_RECTIFICAITON   1       //0����ͬ��������1��ͬ������

#define BRAKE_EN            1       //ɲ��ʹ��

#define SPEED_CLOSE_EN      1       //0��������1��ת�ٻ�
#define IBUS_CLOSE_EN       1       //ĸ�ߵ�������ʹ�ܣ�0��δʹ�ܣ�1��ĸ�ߵ�������2�����ʻ�
#define IPHASE_CLOSE_EN     1       //���������ʹ��

//��������ƫ�ü��
#define CURRENT_OFFSET_VOLTAGE_V        (0.5f)                                              //V����������ƫ�õ�ѹ
#define CURRENT_OFFSET_lsb              (Q32U_)(CURRENT_OFFSET_VOLTAGE_V*HAL_ADC_SCALE_BIT/HAL_ADC_REF_VOLTAGE_V)//lsb����������ƫ��
#define CURRENT_OFFSET_TL_lsb           (200U)                                              //lsb����������ƫ��ƫ����ֵ
#define CURRENT_OFFSET_MAX_lsb          (CURRENT_OFFSET_lsb + CURRENT_OFFSET_TL_lsb)        //lsb����������ƫ������
#define CURRENT_OFFSET_MIN_lsb          (CURRENT_OFFSET_lsb - CURRENT_OFFSET_TL_lsb)        //lsb����������ƫ������
#define CURRENT_OFFSET_NUM              (20U)                                               //��������ƫ�ü�����
    
//˳����
#define FREE_FLYING_TL                  (50U)               //lsb��˳�����ѹ��ֵ
#define FREE_FLYING_NUM                 (20U)               //˳�����ѹ�жϴ���
#define FREE_FLYING_FILTER              (2U)                //�˲�����
#define FREE_FLYING_TIME                (1000U)             //˳����ÿ�������������

//�����ֹ���  
#define BOOT_CHECK_DUTY                 (HAL_PWM_DUTY_50_PERCENT)   //�����ֹ���ռ�ձ�
#define BOOT_CHECK_TL_lsb               (50U)                       //�����ֹ��ⷴ�綯����ֵ
#define BOOT_CHECK_NUM                  (10U)                       //�����ֹ����жϴ���
#define BOOT_CHECK_TIME                 (10000U)                    //�����ֹ����ܴ���

//���嶨λ 
#define POSITION_DUTY                   (HAL_PWM_DUTY_5_PERCENT)   //1kHz�����嶨λռ�ձ�
#define POSITION_TL_lsb                 (1000U)                      //���嶨λ�Ƿ�ɹ��ж�

//�˲���ϵ��
#define IPHASE_FILTER_COEFF             (25U) //0~256
#define FREQ_FILTER_COEFF               (25U) //0~256
#define IBUS_FILTER_COEFF               (25U) //0~256
#define IBRAKE_FILTER_COEFF             (25U) //0~256


//����ϵ��
#define DIAG_CROSS_RISE_TL              (51U)//base64
#define DIAG_CROSS_FALL_TL              (13U)//base64
#define CURRENT_CROSS_RISE_TL           (37U)//base64
#define CURRENT_CROSS_FALL_TL           (27U)//base64
#define FLUX_CROSS_RISE_TL              (37U)//base64
#define FLUX_CROSS_FALL_TL              (27U)//base64

#define DIAG_CROSS_FILTER               (2U)//�˲�����
#define CURRENT_CROSS_FILTER            (2U)//�˲�����
#define FLUX_CROSS_FILTER               (2U)//�˲�����
#define BEMF_CROSS_FILTER               (2U)//�˲�����

#define BEMF_CROSS_DELAY_COEFF          (16U)//base64���ӳٻ������32Ϊ���۵�30��

#define CURRENT_CROSS_FAIL_FILTER       (1000U) //���������˲�����
#define CURRENT_CROSS_DIAG_FILTER       (500U)  //�������������˲�����

//�����㷨�л�
#define CURRENT_TO_FLUX_FREQ            (Q32U_)(Q14I_FREQ_MOTOR_TO_PU(0.20f * MOTOR_MAX_FREQ))
#define CURRENT_TO_FLUX_NUM             (10U)

#define FLUX_TO_CURRENT_FREQ            (Q32U_)(Q14I_FREQ_MOTOR_TO_PU(0.15f * MOTOR_MAX_FREQ))
#define FLUX_TO_CURRENT_NUM             (5U)

#define FLUX_TO_BEMF_FREQ               (Q32U_)(Q14I_FREQ_MOTOR_TO_PU(0.25f * MOTOR_MAX_FREQ))
#define FLUX_TO_BEMF_DUTY               (HAL_PWM_DUTY_20_PERCENT)
#define FLUX_TO_BEMF_NUM                (5U)

#define BEMF_TO_FLUX_FREQ               (Q32U_)(Q14I_FREQ_MOTOR_TO_PU(0.20f * MOTOR_MAX_FREQ))
#define BEMF_TO_FLUX_DUTY               (HAL_PWM_DUTY_15_PERCENT)
#define BEMF_TO_FLUX_NUM                (5U)

//��Ƶ�л�
#define PWM_FREQ_START                  (HAL_PWM_INIT_SET)
#define PWM_FREQ_LOW                    (HAL_PWM_RUN1_SET)
#define PWM_FREQ_HIGH                   (HAL_PWM_RUN2_SET)
#define PWM_FREQ_LOW_TO_HIGH_DUTY       ((Q32U_)(40.0f*HAL_PWM_DUTY_MAX*HAL_PWM_RUN1_FREQ/1000.0f))     //40us
#define PWM_FREQ_HIGH_TO_LOW_DUTY       ((Q32U_)(30.0f*HAL_PWM_DUTY_MAX*HAL_PWM_RUN2_FREQ/1000.0f))     //30us


//���ռ�ձȣ���Сռ�ձ�
#define DUTY_RAMP_ADDSTEP               (Q32I_)( 0.020f * HAL_PWM_DUTY_MAX)
#define DUTY_RAMP_SUBSTEP               (Q32I_)(-0.020f * HAL_PWM_DUTY_MAX)

#define DUTY_CTRL_MAX                   (Q32I_)(HAL_PWM_DUTY_100_PERCENT)
#define DUTY_CTRL_MIN                   (Q32I_)(HAL_PWM_DUTY_5_PERCENT)

//ת��PID
#define FREQ_RAMP_ADDSTEP               (Q32I_)( Q14I_FREQ_MOTOR_TO_PU(0.001f * MOTOR_MAX_FREQ))
#define FREQ_RAMP_SUBSTEP               (Q32I_)(-Q14I_FREQ_MOTOR_TO_PU(0.001f * MOTOR_MAX_FREQ))

#define FREQ_PID_KP                     (Q32I_)(0.0001f * MATH_PID_MAX)
#define FREQ_PID_KI                     (Q32I_)(0.0010f * MATH_PID_MAX)
#define FREQ_PID_KD                     (Q32I_)(0.0001f * MATH_PID_MAX)
#define FREQ_PID_STEPMAX                (Q32I_)( 0.010f * HAL_PWM_DUTY_MAX)
#define FREQ_PID_STEPMIN                (Q32I_)(-0.010f * HAL_PWM_DUTY_MAX)
#define FREQ_PID_OUTMAX                 (DUTY_CTRL_MAX)
#define FREQ_PID_OUTMIN                 (DUTY_CTRL_MIN)
    
//ĸ�ߵ���PID
#define IBUS_GAIN                       (2)//ĸ�ߵ����Ŵ�����δʹ��
#define IBUS_PID_RF                     (Q32I_)(Q14I_CURRENT_MOTOR_TO_PU(0.5f * MOTOR_CURRENT_BUS_A))

#define IBUS_PID_KP                     (Q32I_)(0.010f * MATH_PID_MAX)
#define IBUS_PID_KI                     (Q32I_)(0.010f * MATH_PID_MAX)
#define IBUS_PID_KD                     (Q32I_)(0.000f * MATH_PID_MAX)
#define IBUS_PID_STEPMAX                (Q32I_)( 0.010f * HAL_PWM_DUTY_MAX)
#define IBUS_PID_STEPMIN                (Q32I_)(-0.010f * HAL_PWM_DUTY_MAX)
#define IBUS_PID_OUTMAX                 (DUTY_CTRL_MAX)
#define IBUS_PID_OUTMIN                 (DUTY_CTRL_MIN)

//�����PID
#define IPHSAE_PID_RF                   (Q32I_)(Q14I_CURRENT_MOTOR_TO_PU(0.5f * MOTOR_CURRENT_PHASE_A))

#define IPHASE_PID_KP                   (Q32I_)(0.010f * MATH_PID_MAX)
#define IPHASE_PID_KI                   (Q32I_)(0.010f * MATH_PID_MAX)
#define IPHASE_PID_KD                   (Q32I_)(0.000f * MATH_PID_MAX)
#define IPHASE_PID_STEPMAX              (Q32I_)( 0.010f * HAL_PWM_DUTY_MAX)
#define IPHASE_PID_STEPMIN              (Q32I_)(-0.010f * HAL_PWM_DUTY_MAX)
#define IPHASE_PID_OUTMAX               (DUTY_CTRL_MAX)
#define IPHASE_PID_OUTMIN               (DUTY_CTRL_MIN)

//ɲ��ĸ�ߵ���PID
#define BRAKE_DUTY_RAMP_ADDSTEP         (Q32I_)( 0.020f * HAL_PWM_DUTY_MAX)
#define BRAKE_DUTY_RAMP_SUBSTEP         (Q32I_)(-0.020f * HAL_PWM_DUTY_MAX)

#define BRAKE_DUTY_CTRL_MAX             (Q32I_)(0.400f * HAL_PWM_DUTY_MAX)
#define BRAKE_DUTY_CTRL_MIN             (Q32I_)(0.200f * HAL_PWM_DUTY_MAX)

#define IBRAKE_GAIN                     (5)//ɲ�������Ŵ�����δʹ��
#define IBRAKE_PID_RF                   (Q32I_)(Q14I_CURRENT_MOTOR_TO_PU(0.5f * MOTOR_CURRENT_BRAKE_A))

#define IBRAKE_PID_KP                   (Q32I_)(0.010f * MATH_PID_MAX)
#define IBRAKE_PID_KI                   (Q32I_)(0.010f * MATH_PID_MAX)
#define IBRAKE_PID_KD                   (Q32I_)(0.000f * MATH_PID_MAX)
#define IBRAKE_PID_STEPMAX              (Q32I_)( 0.010f * HAL_PWM_DUTY_MAX)
#define IBRAKE_PID_STEPMIN              (Q32I_)(-0.010f * HAL_PWM_DUTY_MAX)
#define IBRAKE_PID_OUTMAX               (DUTY_CTRL_MAX)
#define IBRAKE_PID_OUTMIN               (DUTY_CTRL_MIN)

//ɲ��ʱ��
#define NO_BRAKE_TIME                   (100U)              //ms����1�����ɻ���
#define SLOW_BRAKE_TIME                 (100U)              //ms����2������ɲ�������û��ʹ�ܣ���������2�׶�
#define SHORT_BRAKE_TIME                (1000U)             //ms����3�ζ̽�ɲ��

//��ת��������
#define MOTOR_STALL_SWITCH_COEFF        (31U)  //base64
#define MOTOR_STALL_ERROR_TIME          (200U)  //ms����תʱ��
#define MOTOR_STALL_CURRENT_ERROR_TIME  (50U)


typedef enum{
    MOTOR_STATE_PRE,
    MOTOR_STATE_INIT,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_BOOT,
    MOTOR_STATE_POSITION,
    MOTOR_STATE_RUN,
    MOTOR_STATE_BRAKE,
}EM_MOTOR_STATE_FLOW;

typedef union{
    ALL all;
    struct{
        BIT motor_run_flag      :1;//������б�־λ
    }bit;
}UN_MOTOR_STATE_FLAG;

typedef union{
    ALL all;
    struct{
        BIT current_offset      :1;//ƫ�ù���
        BIT current_short       :1;//��·����
        BIT mos_fault           :1;//mos���ϣ������ϵ������ڣ��������ζ�·��������������״̬��
        BIT motor_stall         :1;//�����ת����
        BIT position_error      :1;//�����λ����
    }bit;
}UN_MOTOR_ERROR_FLAG;

typedef struct{
    EM_MOTOR_STATE_FLOW         Motor_Flow;
    UN_MOTOR_STATE_FLAG         Motor_State_Flag;
    UN_MOTOR_ERROR_FLAG         Motor_Error_Flag;
    
    ST_MS_OFFSET                MS_OFFSET;
    ST_MS_FLYING                MS_FLYING;
    ST_MS_BOOT                  MS_BOOT;
    ST_MS_POSITION              MS_POSITION;
    ST_BRAKE_CONTROL            BRAKE_CTRL;
    
    ST_MS_CONTROL               MS_CTRL;
    
    Q32I_                       Q14I_IPHASE_MAX_PU;
    Q32U_                       flow_cnt;
}ST_MOTOR_TASK;

extern ST_MOTOR_TASK  Motor;

#endif /* MotorPara_H */
