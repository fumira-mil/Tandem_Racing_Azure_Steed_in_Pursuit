/*
 * PID.c
 *
 *  Created on: 2025��1��25��
 *      Author: zhuji
 */
#include "PID.h"

int setspeed1=1200;
int setspeed2=1200;
int speed1;//����
int speed2;
int Increase1=0;
int Increase2=0;
float divertion;
float erspeed=0;

float gyro_filtered = 0; // �˲����������ֵ

PID_Datatypedef sptr1,sptr2;
PID_imu_Datatypedef imu;
PID_Angeltypedef angel;
//float P_L=0.1;
//float I_L=0;
//float P_R=0.1;
//float I_R=0;

int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

void PID_Init(PID_Datatypedef*sptr)
{
    sptr->P=0;
    sptr->I=0;
    sptr->D=0;
    sptr->LastError=0;
    sptr->PrevError=0;
}
void imu_PID_Init(PID_imu_Datatypedef*imu)
{
    imu->KP_1=0;
    imu->KD_1=0;
    imu->GKD=0;
    imu->lasterror=0;
    imu->KP_2 = 0;
    imu->integrator = 0;   // �������ʼ��Ϊ0
}
int MotorPID_Output(PID_Datatypedef*sptr,float NowSpeed,int ExpectSpeed)
{
    int Increase;
    int iError;
    iError=ExpectSpeed-NowSpeed;
    Increase=(int)(sptr->P*(iError-sptr->LastError)+sptr->I*iError+sptr->D*(iError-2*sptr->LastError+sptr->PrevError));
    sptr->PrevError=sptr->LastError;
    sptr->LastError=iError;
    return Increase;
}
float imuPID_Output(float erspeed,PID_imu_Datatypedef*imu)
{
    float imu_out;

    imu_out = (erspeed * imu->KP_1) + (erspeed * my_abs(erspeed) * imu->KP_2) + (erspeed - imu->lasterror) * imu->KD_1 + imu660ra_gyro_x * imu->GKD;
    imu->lasterror = erspeed; // ������һ�����
    if (imu_out > 5000) imu_out =5000;
    else if (imu_out < -5000) imu_out = -5000;
    return imu_out;

}
//int ang_pid(float b,int c)
//{
//    int t;
//    int temp_speed;
//
//    temp_speed=(speed1+speed2)/2;
//
//    t=angel.kP*(temp_speed-b)+angel.kD*(temp_speed-angel.LastError1);
//    angel.LastError1=temp_speed-b;
//
//    return t;
//
//}
void PID_output(void)
{

//    static float last_Increase1 = 0;
//    static float last_Increase2 = 0;

//            ����ʽPID����С��ֱ��
    Increase1=MotorPID_Output(&sptr1,speed1,setspeed1);
    Increase2=MotorPID_Output(&sptr2,speed2,setspeed2);
            //����ֱ��ŤתС�����з���
    divertion=imuPID_Output(center_line_error,&imu);
//    Increase1=-divertion;
//    Increase2=+divertion;
    Increase1=Increase1-divertion;
    Increase2=Increase2+divertion;
    Motor_Left(Increase1);
    Motor_Right(Increase2);
//    Motor_Left(1000);
//    Motor_Right(1000);
//    if(Increase1-Increase2>-6000 && Increase1-Increase2<6000)
//    {
//    float diff = Increase1 - Increase2;
//    if (diff > 3500 || diff < -3500) {         // ���ת��
//        if (Increase1 < 0) Increase1 = -1.8 * Increase1;
//        if (Increase2 < 0) Increase2 = -1.8 * Increase2;
//    }
//    else if (diff > 2500 || diff < -2500) {    // ��ת��
//        if (Increase1 < 0) Increase1 = -1.5 * Increase1;
//        if (Increase2 < 0) Increase2 = -1.5 * Increase2;
//    }
//    else if (diff > 1500 || diff < -1500) {    // �е�ת��
//        if (Increase1 < 0) Increase1 = -1.0 * Increase1;
//        if (Increase2 < 0) Increase2 = -1.0 * Increase2;
//    }
//    else if (diff > 1000 || diff < -1000) {    // Сת��
//        if (Increase1 < 0) Increase1 = -0.5 * Increase1;
//        if (Increase2 < 0) Increase2 = -0.5 * Increase2;
//    }
//    else {
//        Increase1 = 0.8 * last_Increase1 + 0.2 * Increase1;
//        Increase2 = 0.8 * last_Increase2 + 0.2 * Increase2;
//    }

}

//void PID_select(void)
//{
//    if(flag)
//    {
//    setspeed1=1000;
//    setspeed2=1000;
//    sptr1.P=1.0;//pwm1000ʱ�Ĳ���
//    sptr1.I=1.13;
//    sptr1.D=0;
//
//    sptr2.P=1.0;
//    sptr2.I=1.3;
//    sptr2.D=0;
//        //ת��PID
//    imu.KP_1=80;
//    imu.KD_1=10;
//    imu.GKD=0.2;}
//
//    if(flag1)
//{   setspeed1=1000;
//    setspeed2=1000;
//    sptr1.P=2.5;
//    sptr1.I=1.13;
//    sptr1.D=0;
//
//    sptr2.P=1.5;
//    sptr2.I=1.28;
//    sptr2.D=0;
//    }
//}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     PID����
  @param     int set_speed ,int speed,����ֵ��ʵ��ֵ
  @return    ���ռ�ձ�
  Sample     pwm_R= PID_R(set_speed_right,right_wheel);//pid���Ƶ��ת��
             pwm_L= PID_L(set_speed_left,left_wheel );//pid���Ƶ��ת��
  @note      ������
-------------------------------------------------------------------------------------------------------------------*/
//int PID_L(int set_speed ,float speed)//pid���Ƶ��ת��
//{
//    volatile static int out;
//    volatile static int out_increment;
//    volatile static int ek,ek1;
//    volatile static int speed_bb;
//
//    ek1 = ek;
//    ek = set_speed - speed;
//
//    if(ek>80) speed_bb=SPEED_MAX;
//    else if(ek<-80) speed_bb=SPEED_MIN;
//    else speed_bb=0;
//
//    out_increment= (int)(P_L*(ek-ek1) + I_L*ek + speed_bb);
//    out+= out_increment;
//
//    if(out>=SPEED_MAX)//�޷�����
//        out=SPEED_MAX;
//    else if(out<=SPEED_MIN)
//        out=SPEED_MIN;
//    return (int) out;
//}
//
///*-------------------------------------------------------------------------------------------------------------------
//  @brief     PID����
//  @param     int set_speed ,int speed,����ֵ��ʵ��ֵ
//  @return    ���ռ�ձ�
//  Sample     pwm_R= PID_R(set_speed_right,right_wheel);//pid���Ƶ��ת��
//             pwm_L= PID_L(set_speed_left,left_wheel );//pid���Ƶ��ת��
//  @note      ������
//-------------------------------------------------------------------------------------------------------------------*/
//int PID_R(int set_speed ,float speed)//pid���Ƶ��ת��
//{
//    volatile static int  out;
//    volatile static int  out_increment;
//    volatile static int  ek,ek1;
//    volatile static int speed_bb;
//
//    ek1 = ek;
//    ek = set_speed - speed;
//
//    if(ek>80) speed_bb=SPEED_MAX;
//    else if(ek<-80) speed_bb=SPEED_MIN;
//    else speed_bb=0;
//
//    out_increment= (int)(P_R*(ek-ek1) + I_R*ek + speed_bb);
//    out+= out_increment;
//
//    if(out>=SPEED_MAX)//�޷�����
//        out=SPEED_MAX;
//    else if(out<=SPEED_MIN)
//        out=SPEED_MIN;
//    return (int) out;
//}



