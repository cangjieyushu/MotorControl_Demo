/**************************************************************************************************/
/**
 * @copyright : 
 **************************************************************************************************/
#ifndef USER_PMSM_H
#define USER_PMSM_H

#include "Math.h"
#include "MotorHal_cfg.h"   

//��������
#define MOTOR_VOLTAGE_V                     (24.0f)             //V��ĸ�ߵ�ѹ
#define MOTOR_CURRENT_PHASE_A               (30.0f)             //A���������ֵ
#define MOTOR_CURRENT_BUS_A                 (10.0f)             //A��ĸ�ߵ���
#define MOTOR_CURRENT_BRAKE_A               (2.0f)              //A��ɲ������

#define MOTOR_HTs                           (1.0f/1000.0f/HAL_PWM_FREQ_SET)
#define MOTOR_LTs                           (HAL_SLOW_TIMER_FREQ/2.0f/1000.0f)
#define MOTOR_POLE_PAIR                     (2.0f)                          //ת�Ӽ�����
#define MOTOR_Rs                            (0.31f)                         //���������
#define MOTOR_Ld                            (0.37f*0.001f)                  //H��d����
#define MOTOR_Lq                            (0.44f*0.001f)                  //H��q���У�q����������Ҫ��d���д�10uH
#define MOTOR_Ls                            (0.5f*(MOTOR_Ld + MOTOR_Lq))    //H������
#define MOTOR_FLUX                          (0.017f)                        //Wb

#define MOTOR_MAX_SPEED                     (4000.0f)             //rpm�����ת��
#define MOTOR_MIN_SPEED                     (1000.0f)             //rpm�����ת��

#define MOTOR_MAX_SRAD                      (MATH_2PI_F*MOTOR_POLE_PAIR*MOTOR_MAX_SPEED/60.0f)        //Hz����߽�Ƶ��
#define MOTOR_MIN_SRAD                      (MATH_2PI_F*MOTOR_POLE_PAIR*MOTOR_MIN_SPEED/60.0f)        //Hz����ͽ�Ƶ��

//dq�������ѹ���ƣ������֤��ѹʸ��ΪԲ�Σ�����Ϊ0.5774f�������Ҫ�����ƣ������Ϊ0.6667f
#define MOTOR_VS_SCALE                      (0.5774f)

//���ۻ�
#define V_BASE                              (0.5f*HAL_ADC_VOLTAGE_MAX)      //V����ѹ
#define I_BASE                              (0.5f*HAL_ADC_CURRENT_MAX)      //A������
#define F_BASE                              (1.0f*MOTOR_POLE_PAIR*MOTOR_MAX_SPEED/60.0f)    //Hz��Ƶ��

#define W_BASE                              (F_BASE*MATH_2PI_F)     //Hz����Ƶ��
#define R_BASE                              (V_BASE/I_BASE)         //��������

#define L_BASE                              (V_BASE/W_BASE/I_BASE)  //H�����
#define P_BASE                              (V_BASE/W_BASE)         //wb������
#define T_BASE                              (MATH_2PI_F/W_BASE)     //s,ʱ��

#define MOTOR_Q14_PU                        (Q14U_MAX)      //16384
#define MOTOR_Q24_PU                        (Q24U_MAX)      //

#define Q24I_VOLTAGE_SCALE_PU               (Q32U_)(MOTOR_Q24_PU*HAL_ADC_VOLTAGE_SCALE/V_BASE)          //��ѹ
#define Q24I_CURRENT_SCALE_PU               (Q32U_)(MOTOR_Q24_PU*HAL_ADC_CURRENT_SCALE/I_BASE)          //����

#define Q14I_VOLTAGE_LSB_TO_PU(a)           (Q32I_RHT_10(Q24I_VOLTAGE_SCALE_PU*(a)))          //��ѹ
#define Q14I_CURRENT_LSB_TO_PU(a)           (Q32I_RHT_10(Q24I_CURRENT_SCALE_PU*(a)))          //����
     
#define Q14I_HTs_PU                         (Q32U_)(MOTOR_Q14_PU*MOTOR_HTs/T_BASE)
#define Q14I_LTs_PU                         (Q32U_)(MOTOR_Q14_PU*MOTOR_LTs/T_BASE)
#define Q14I_Rs_PU                          (Q32U_)(MOTOR_Q14_PU*MOTOR_Rs/R_BASE)       //���������
#define Q14I_Ld_PU                          (Q32U_)(MOTOR_Q14_PU*MOTOR_Ld/L_BASE)       //H��d����
#define Q14I_Lq_PU                          (Q32U_)(MOTOR_Q14_PU*MOTOR_Lq/L_BASE)       //H��q����
#define Q14I_Ls_PU                          (Q32U_)(MOTOR_Q14_PU*MOTOR_Ls/L_BASE)       //H������
#define Q14I_FLUX_PU                        (Q32U_)(MOTOR_Q14_PU*MOTOR_FLUX/P_BASE)     //V*S��Wb

#define Q14I_VOLTAGE_MOTOR_TO_PU(a)         (Q32U_)(MOTOR_Q14_PU*(a)/V_BASE)              //��ѹ����ת��
#define Q14I_CURRENT_MOTOR_TO_PU(a)         (Q32U_)(MOTOR_Q14_PU*(a)/I_BASE)              //��������ת��
#define Q14I_SRAD_MOTOR_TO_PU(a)            (Q32U_)(MOTOR_Q14_PU*(a)/W_BASE)              //Ƶ�ʱ���ת��

#define Q14I_VOLTAGE_PU                     (Q32U_)(MOTOR_Q14_PU*MOTOR_VOLTAGE_V/V_BASE)        //���ѹ����ֵ
#define Q14I_CURRENT_PHASE_A_PU             (Q32U_)(MOTOR_Q14_PU*MOTOR_CURRENT_PHASE_A/I_BASE)  //����������ֵ
#define Q14I_CURRENT_BUS_A_IPU              (Q32U_)(MOTOR_Q14_PU*MOTOR_CURRENT_BUS_A/I_BASE)    //�ĸ�ߵ�������ֵ
#define Q14I_CURRENT_BRAKE_A_PU             (Q32U_)(MOTOR_Q14_PU*MOTOR_CURRENT_BRAKE_A/I_BASE)  //�ɲ����������ֵ
#define Q14I_MAX_SRAD_PU                    (Q32U_)(MOTOR_Q14_PU*MOTOR_MAX_SRAD/W_BASE)         //�Ƶ�ʱ���ֵ
#define Q14I_MIN_SRAD_PU                    (Q32U_)(MOTOR_Q14_PU*MOTOR_MIN_SRAD/W_BASE)         //�Ƶ�ʱ���ֵ

#endif /* USER_PMSM_H */
