/**************************************************************************************************
*     File Name :                        PmsmPara.h
*     Library/Module Name :              Motor
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             �������ͷ�ļ�
**************************************************************************************************/
#ifndef PmsmPara_H
#define PmsmPara_H

#include "Math.h"
#include "MotorHal_cfg.h"   

//��������������ԭ��
//#define MOTOR_VOLTAGE_V                     (24.0f)             //V��ĸ�ߵ�ѹ
//#define MOTOR_CURRENT_PHASE_A               (12.0f)             //A���������ֵ
//#define MOTOR_CURRENT_BUS_A                 (8.0f)              //A��ĸ�ߵ���
//#define MOTOR_CURRENT_BRAKE_A               (2.0f)              //A��ɲ������
//
//#define MOTOR_HTs                           (1.0f/1000.0f/HAL_PWM_SET_FREQ)
//#define MOTOR_LTs                           (HAL_SLOW_TIMER_FREQ/1000.0f)
//#define MOTOR_POLE_PAIR                     (2.0f)                          //ת�Ӽ�����
//#define MOTOR_Rs                            (0.31f)                         //���������
//#define MOTOR_Ld                            (0.37f*0.001f)                  //H��d����
//#define MOTOR_Lq                            (0.44f*0.001f)                  //H��q���У�q����������Ҫ��d���д�10uH
//#define MOTOR_Ls                            (0.5f*(MOTOR_Ld + MOTOR_Lq))    //H������
//#define MOTOR_FLUX                          (0.017f)                        //Wb
//
//#define MOTOR_MAX_SPEED                     (3300.0f)             //rpm�����ת��
//#define MOTOR_MIN_SPEED                     (100.0f)              //rpm�����ת��
//
//#define MOTOR_MAX_SRAD                      (MATH_2PI_F*MOTOR_POLE_PAIR*MOTOR_MAX_SPEED/60.0f)        //Hz����߽�Ƶ��
//#define MOTOR_MIN_SRAD                      (MATH_2PI_F*MOTOR_POLE_PAIR*MOTOR_MIN_SPEED/60.0f)        //Hz����ͽ�Ƶ��

//������������ɫ���
#define MOTOR_VOLTAGE_V                     (12.0f)             //V��ĸ�ߵ�ѹ
#define MOTOR_CURRENT_PHASE_A               (10.0f)             //A���������ֵ
#define MOTOR_CURRENT_BUS_A                 (6.0f)              //A��ĸ�ߵ���
#define MOTOR_CURRENT_BRAKE_A               (2.0f)              //A��ɲ������

#define MOTOR_HTs                           (1.0f/1000.0f/HAL_PWM_SET_FREQ)
#define MOTOR_LTs                           (HAL_SLOW_TIMER_FREQ/1000.0f)
#define MOTOR_POLE_PAIR                     (4.0f)                          //ת�Ӽ�����
#define MOTOR_Rs                            (0.36f)                         //���������
#define MOTOR_Ld                            (0.225f*0.001f)                 //H��d����
#define MOTOR_Lq                            (0.245f*0.001f)                 //H��q���У�q����������Ҫ��d���д�10uH
#define MOTOR_Ls                            (0.5f*(MOTOR_Ld + MOTOR_Lq))    //H������
#define MOTOR_FLUX                          (0.00577f)                      //Wb

#define MOTOR_MAX_SPEED                     (3500.0f)             //rpm�����ת��
#define MOTOR_MIN_SPEED                     (100.0f)              //rpm�����ת��

#define MOTOR_MAX_SRAD                      (MATH_2PI_F*MOTOR_POLE_PAIR*MOTOR_MAX_SPEED/60.0f)        //Hz����߽�Ƶ��
#define MOTOR_MIN_SRAD                      (MATH_2PI_F*MOTOR_POLE_PAIR*MOTOR_MIN_SPEED/60.0f)        //Hz����ͽ�Ƶ��

////������������ɫ����
//#define MOTOR_VOLTAGE_V                     (12.0f)             //V��ĸ�ߵ�ѹ
//#define MOTOR_CURRENT_PHASE_A               (12.0f)             //A���������ֵ
//#define MOTOR_CURRENT_BUS_A                 (8.0f)              //A��ĸ�ߵ���
//#define MOTOR_CURRENT_BRAKE_A               (2.0f)              //A��ɲ������
//
//#define MOTOR_HTs                           (1.0f/1000.0f/HAL_PWM_SET_FREQ)
//#define MOTOR_LTs                           (HAL_SLOW_TIMER_FREQ/1000.0f)
//#define MOTOR_POLE_PAIR                     (5.0f)                          //ת�Ӽ�����
//#define MOTOR_Rs                            (0.175f)                         //���������
//#define MOTOR_Ld                            (0.31f*0.001f)                  //H��d����
//#define MOTOR_Lq                            (0.43f*0.001f)                  //H��q���У�q����������Ҫ��d���д�10uH
//#define MOTOR_Ls                            (0.5f*(MOTOR_Ld + MOTOR_Lq))    //H������
//#define MOTOR_FLUX                          (0.00975f)                        //Wb
//
//#define MOTOR_MAX_SPEED                     (3300.0f)             //rpm�����ת��
//#define MOTOR_MIN_SPEED                     (100.0f)              //rpm�����ת��
//
//#define MOTOR_MAX_SRAD                      (MATH_2PI_F*MOTOR_POLE_PAIR*MOTOR_MAX_SPEED/60.0f)        //Hz����߽�Ƶ��
//#define MOTOR_MIN_SRAD                      (MATH_2PI_F*MOTOR_POLE_PAIR*MOTOR_MIN_SPEED/60.0f)        //Hz����ͽ�Ƶ��

#endif /* PmsmPara_H */
