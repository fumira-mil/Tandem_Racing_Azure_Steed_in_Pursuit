/*
 * Motor.c
 *
 *  Created on: 2025��1��11��
 *      Author: zhuji
 */
#include "Motor.h"

/*-------------------------------------------------------------------------------------------------------------------
  @brief     �����ʼ������
  @param     NULL
  @return    null
  Sample     Motor_Left(pwm_L);
  @note      ������������������������������
                                      �ڸ����ǰ���޷������
-------------------------------------------------------------------------------------------------------------------*/
void Motor_Init(void)
{
    pwm_init(MOTOR01_SPEED_PIN, 17000, 0);
    pwm_init(MOTOR02_SPEED_PIN, 17000, 0);
    pwm_set_duty(MOTOR01_SPEED_PIN,0);
    pwm_set_duty(MOTOR02_SPEED_PIN,0);
    gpio_init(MOTOR01_DIR_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR02_DIR_PIN, GPO, 0, GPO_PUSH_PULL);
    PID_Init(&sptr1);
    PID_Init(&sptr2);
        //PID��ʼ��
        //����ʽPID
    imu_PID_Init(&imu);
//    if(flag)
    sptr1.P=2.5;////��Ӧspeed2����������
    sptr1.I=1.25;
    sptr1.D=0;

    sptr2.P=2.5;//��Ӧspeed1����������
    sptr2.I=1.2;
    sptr2.D=0;

    imu.KP_1=60;
    imu.KD_1=0;
    imu.GKD=0;
}
    //       setspeed1 = 2000;//��������Ӧ�ٶ�2.7m/s
    //       setspeed2 = 2000;
    //       sptr1.P=1.5;////��Ӧspeed2����������
    //           sptr1.P=1.5;////��Ӧspeed2����������
    //           sptr1.I=1.24;
    //           sptr1.D=0;
    //
    //           sptr2.P=2;//��Ӧspeed1����������
    //           sptr2.I=1.17;
    //           sptr2.D=0;

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ����������
  @param     pwm_L �������ռ�ձȣ������ɸ�
  @return    null
  Sample     Motor_Left(pwm_L);
  @note      ������������������������������
                                      �ڸ����ǰ���޷������
-------------------------------------------------------------------------------------------------------------------*/
void Motor_Left(int pwm_L)
{
    if(pwm_L>=SPEED_MAX)//�޷�����
        pwm_L=SPEED_MAX;
    else if(pwm_L<=SPEED_MIN)
             pwm_L=SPEED_MIN;

    if(pwm_L>=0)//��ת
    {
        pwm_set_duty(MOTOR01_SPEED_PIN, pwm_L);//������ת
        gpio_set_level(MOTOR01_DIR_PIN, 0);//01������D15,0��ת��1��ת
    }
    else
    {
        pwm_set_duty(MOTOR01_SPEED_PIN, -pwm_L);//������ת
        gpio_set_level(MOTOR01_DIR_PIN, 1);//01������D15,0��ת��1��ת
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ����������
  @param     pwm_R �������ռ�ձȣ������ɸ�
  @return    null
  Sample     Motor_Right(pwm_R);
  @note      ������������������������������
                                      �ڸ����ǰ���޷������
-------------------------------------------------------------------------------------------------------------------*/
void Motor_Right(int pwm_R)
{
    if(pwm_R>=SPEED_MAX)//�޷�����
        pwm_R=SPEED_MAX;
    else if(pwm_R<=SPEED_MIN)
            pwm_R=SPEED_MIN;

    if(pwm_R>=0)//��ת
    {
        pwm_set_duty(MOTOR02_SPEED_PIN, pwm_R);//�ҵ����ת
        gpio_set_level(MOTOR02_DIR_PIN, 0);//02������D14,0��ת��1��ת
    }
    else
   {
        pwm_set_duty(MOTOR02_SPEED_PIN, -pwm_R);//�ҵ����ת
        gpio_set_level(MOTOR02_DIR_PIN, 1);//02������D14,0��ת��1��ת
   }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ����������ת��
  @param     pwm_R �������ռ�ձȣ������ɸ�
  @return    null
  Sample     Motor_Right(pwm_R);
  @note      ������������������������������
                                      �ڸ����ǰ���޷������
-------------------------------------------------------------------------------------------------------------------*/
void wu_shua(void)
{
    pwm_set_duty(ATOM0_CH2_P21_4,1000);
    pwm_set_duty(ATOM1_CH3_P21_5,1000);


}
