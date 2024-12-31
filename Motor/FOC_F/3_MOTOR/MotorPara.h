/**************************************************************************************************
*     File Name :                        MotorPara.h
*     Library/Module Name :              Motor
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             ������Ʋ�����ʼ��ͷ�ļ�
**************************************************************************************************/
#ifndef MotorPara_H
#define MotorPara_H

#include "Math.h"
#include "PmsmPara.h"
#include "MotorHal_cfg.h"
#include "MotorEst.h"
#include "MotorFoc.h"


//���openloop��ز��� 
#define MOTOR_OPENLOOP_SRAD                 (5.0f * MATH_2PI_F)             //Hz,���OPENLOOP�л�closeloop1ת��
#define MOTOR_OPENLOOP_SWITCH_TIME          (50)                            //ms,���OPENLOOP�л�closeloop1ʱ��

//���closeloop1��ز������ջ���ʼ�׶Σ�ʹ�÷����нǶ�  
#define MOTOR_CLOSELOOP1_STEP               (0.1f * MATH_2PI_F)             //Hz/s,���closeloop1���ٲ���
#define MOTOR_CLOSELOOP1_SRAD               (30.0f * MATH_2PI_F)            //Hz,���closeloop1�л�closeloop2ת��
#define MOTOR_CLOSELOOP1_SWITCH_TIME        (100)                           //ms,���closeloop1�л�closeloop2��ʱ��

//���closeloop2��ز������ջ����н׶Σ�ʹ�����໷�Ƕ�           
#define MOTOR_CLOSELOOP2_SRAD_TARGET        (MOTOR_MAX_SRAD)                //Hz,���closeloop2Ŀ��ת��
#define MOTOR_CLOSELOOP2_STEP               (0.025f * MATH_2PI_F)           //Hz/s,���closeloop2���ٲ��� 

//���closeloop3��ز������ջ������׶Σ������趨ת�ٺ�ͣ�� 
#define MOTOR_CLOSELOOP3_SRAD_TARGET        (30.0f * MATH_2PI_F)            //Hz,���closeloop3Ŀ��ת��
#define MOTOR_CLOSELOOP3_STEP               (0.1f * MATH_2PI_F)             //Hz/s,���closeloop3���ٲ���
#define MOTOR_CLOSELOOP3_SRAD               (60.0f * MATH_2PI_F)            //Hz,���closeloop3�л�ֹͣ�ٶ�
#define MOTOR_CLOSELOOP3_SWITCH_TIME        (20)                            //ms,���closeloop3�л�ֹͣʱ��


//λ�ô�����
#define MOTOR_SENSE_MASK                (0F00U)
#define MOTOR_SENSE_NO                  (0100U)                     //�޸�
#define MOTOR_SENSE_HALL                (0200U)                     //HALL���ش�����

//�޸������㷨
#define MOTOR_START_MASK                (00F0U)
#define MOTOR_START_IF                  (0010U)                     //IF
#define MOTOR_START_VF                  (0020U)                     //VF
#define MOTOR_START_FLUX                (0030U)                     //�����Դ���
#define MOTOR_START_HFI                 (0040U)                     //HFI

//�޸������㷨
#define MOTOR_SPEED_MASK                (000FU)
#define MOTOR_SPEED_FLUX                (0001U)                     //�����Դ���
#define MOTOR_SPEED_SMO                 (0002U)                     //SMO

//�������ģʽ
#define MOTOR_SENSE_MODE                (MOTOR_SENSE_NO)
#define MOTOR_START_MODE                (MOTOR_START_IF)
#define MOTOR_SPEED_MODE                (MOTOR_SPEED_FLUX)


//IF
#define MOTOR_IF_IQRAMP_INIT                (0.0f)                          //A,Iq��ʼֵ
#define MOTOR_IF_IQRAMP_TARGET              (4.0f)                          //A,IqĿ��ֵ
#define MOTOR_IF_IQRAMP_STEP                (1.0f / 1000.0f)                //A/s,Iqÿ�����Ӳ���

#define MOTOR_IF_ANGLERAMP_INIT             (0.0f * MATH_2PI_F)             //Hz,IF�ٶȳ�ʼֵ
#define MOTOR_IF_ANGLERAMP_TARGET           (8.0f * MATH_2PI_F)             //Hz,IF�ٶ�Ŀ��ֵ
#define MOTOR_IF_ANGLERAMP_STEP             (2.0f * MATH_2PI_F / 1000.0f)   //Hz/s,IF�ٶ�ÿ�����Ӳ���

#define MOTOR_IF_ANGLE_ERROR                (3.0f)                          //rad,IF��۲����Ƕ�ƫ�������л�ֵ
#define MOTOR_IF_ANGLE_ERROR_RAMP_STEP      (1.0f / 1000.0f)                //Hz,���IF�۲����Ƕ���������

//VF
#define MOTOR_VF_IQRAMP_INIT                (0.1f)                          //V,Vq��ʼֵ
#define MOTOR_VF_IQRAMP_TARGET              (0.5f)                          //V,VqĿ��ֵ
#define MOTOR_VF_IQRAMP_STEP                (0.2f / 1000.0f)                //V/s,Vqÿ�����Ӳ���

#define MOTOR_VF_ANGLERAMP_INIT             (0.0f * MATH_2PI_F)             //Hz,VF�ٶȳ�ʼֵ
#define MOTOR_VF_ANGLERAMP_TARGET           (8.0f * MATH_2PI_F)             //Hz,VF�ٶ�Ŀ��ֵ
#define MOTOR_VF_ANGLERAMP_STEP             (2.0f * MATH_2PI_F / 1000.0f)   //Hz/s,VF�ٶ�ÿ�����Ӳ���

#define MOTOR_VF_ANGLE_ERROR                (3.0f)                          //rad,VF��۲����Ƕ�ƫ�������л�ֵ
#define MOTOR_VF_ANGLE_ERROR_RAMP_STEP      (1.0f / 1000.0f)                //Hz,���VF�۲����Ƕ���������


//ת�ٻ�PID    
#define MOTOR_SPD_PID_Coeff                 (0.25f)                          //ת�ٻ�PID����ϵ��
#define MOTOR_SPD_KP_GAIN                   (MOTOR_SPD_PID_Coeff * MOTOR_CURRENT_PHASE_A / MOTOR_MAX_SRAD)
#define MOTOR_SPD_KI_GAIN                   (MOTOR_CURRENT_PHASE_A * MOTOR_CLOSELOOP2_STEP * MOTOR_LTs / MATH_2PI_F / MATH_2PI_F)
#define MOTOR_SPD_KD_GAIN                   (0.0f)
#define MOTOR_SPD_PID_MAX                   (MOTOR_CURRENT_PHASE_A)       //A,ת�ٻ����q������޷�
#define MOTOR_SPD_PID_MIN                   (-MOTOR_CURRENT_PHASE_A)      //A,ת�ٻ����q������޷�

//����PID 
#define MOTOR_FOC_P_Coeff                   (0.05f)                         //������P����ϵ��
#define MOTOR_FOC_KP_GAIN                   (MOTOR_FOC_P_Coeff * MOTOR_Ls * MATH_2PI_F / MOTOR_HTs)
#define MOTOR_FOC_KI_GAIN                   (MOTOR_FOC_KP_GAIN * MOTOR_HTs * MOTOR_Rs / MOTOR_Ls)
#define MOTOR_FOC_KD_GAIN                   (0.0f)
//dq�������ѹ���ƣ������֤��ѹʸ��ΪԲ�Σ�����Ϊ0.5774f�������Ҫ�����ƣ������Ϊ0.6667f
#define MOTOR_VS_SCALE                      (0.5774f)


//�۲���PLLϵ��
#define MOTOR_PLL_KI_Coeff                  (0.1f)
#define MOTOR_PLL_KP_Coeff                  (MOTOR_PLL_KI_Coeff*MOTOR_PLL_KI_Coeff)
#define USER_PLL_SPEED_LPF_COEFF            (0.05f)                     //0~1��ԽС�˲�Խ��

//�����Դ����۲���  
#define MOTOR_FLUX_KT                       (2.0f / MOTOR_VOLTAGE_V / MOTOR_FLUX / MOTOR_FLUX / MOTOR_FLUX) //����ϵ��

#define MOTOR_FLUX_PLL_KP                   (MOTOR_PLL_KP_Coeff * MOTOR_MAX_SRAD * MOTOR_MAX_SRAD / MOTOR_FLUX)  //���໷����ϵ��
#define MOTOR_FLUX_PLL_KI                   (MOTOR_PLL_KI_Coeff * MOTOR_MAX_SRAD / MOTOR_FLUX)                   //���໷����ϵ��
#define MOTOR_FLUX_PLL_KD                   (0.0f)                    //���໷΢��ϵ��
#define MOTOR_FLUX_PLL_MAX                  (10.0f * MOTOR_MAX_SRAD)   //���໷������
#define MOTOR_FLUX_PLL_MIN                  (-10.0f * MOTOR_MAX_SRAD)  //���໷��С���

//SMO�۲���            
#define MOTOR_SMO_K1                        (8.00f * MOTOR_VOLTAGE_V / MOTOR_CURRENT_PHASE_A)           //����ϵ��1
#define MOTOR_SMO_K2                        (0.02f * MOTOR_VOLTAGE_V / MOTOR_CURRENT_PHASE_A)           //����ϵ��2

#define MOTOR_SMO_PLL_KP                    (MOTOR_PLL_KP_Coeff * MOTOR_MAX_SRAD * MOTOR_MAX_SRAD / MOTOR_VOLTAGE_V)    //���໷����ϵ��
#define MOTOR_SMO_PLL_KI                    (MOTOR_PLL_KI_Coeff * MOTOR_MAX_SRAD / MOTOR_VOLTAGE_V)                     //���໷����ϵ��
#define MOTOR_SMO_PLL_KD                    (0.0f)                    //���໷΢��ϵ��
#define MOTOR_SMO_PLL_MAX                   (10.0f * MOTOR_MAX_SRAD)   //���໷������
#define MOTOR_SMO_PLL_MIN                   (-10.0f * MOTOR_MAX_SRAD)  //���໷��С���


//ĸ�ߵ���PID 
#define MOTOR_BUS_P_REf                     (MOTOR_CURRENT_BUS_A)           //ĸ�ߵ����ο�ֵ
#define MOTOR_BUS_KP_GAIN                   (2.00f)
#define MOTOR_BUS_KI_GAIN                   (5.0f)
#define MOTOR_BUS_KD_GAIN                   (0.0f)


typedef enum{
    MOTOR_STATE_PRE,
    MOTOR_STATE_INIT,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_BOOT,
    MOTOR_STATE_POSITION,
    MOTOR_STATE_RUN,
    MOTOR_STATE_BRAKE,
}EM_MOTOR_STATE_FLOW;

typedef enum
{
    MOTOR_ALIGNLOOP,   
    MOTOR_OPENLOOP,  
    MOTOR_CLOSELOOP1,
    MOTOR_CLOSELOOP2,
}EM_MOTOR_LOOP_MODE;

typedef union{
    ALL all;
    struct{
        BIT motor_run_flag      :1;//������б�־λ
        BIT pwm_output_flag     :1;//pwm���ʹ�ܱ�־λ
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
    EM_MOTOR_LOOP_MODE          Motor_Loop_Mode;
    UN_MOTOR_STATE_FLAG         Motor_State_Flag;
    UN_MOTOR_ERROR_FLAG         Motor_Error_Flag;
    
    ST_IF_CONTROL_F             IF_CTRL;
    ST_VF_CONTROL_F             VF_CTRL;
    ST_SVPWM_CONTROL_F          SVPWM_CTRL;
    ST_SRAD_CONTROL_F           SRAD_CTRL;
    ST_CURRENT_CONTROL_F        CURRENT_CTRL;
    
    ST_FLUX_CONTROL_F           FLUX_CTRL;
    ST_SMO_CONTROL_F            SMO_CTRL;
    
    Q32U_                       flow_cnt;
    float                       F_Iphase_Max;
}ST_MOTOR_TASK;


extern ST_MOTOR_TASK  Motor;

#endif /* MotorPara_H */
