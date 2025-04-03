/*
 * PID.c
 *
 *  Created on: 2025年1月25日
 *      Author: zhuji
 */
#include "PID.h"

int setspeed1=1500;
int setspeed2=1500;
int speed1;//左电机
int speed2;
int Increase1=0;
int Increase2=0;
float divertion;
float erspeed=0;



PID_Datatypedef sptr1,sptr2;
PID_imu_Datatypedef imu;
//float P_L=0.1;
//float I_L=0;
//float P_R=0.1;
//float I_R=0;

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
    imu_out=erspeed*imu->KP_1+(erspeed-imu->lasterror)*imu->KD_1-imu660ra_gyro_x*imu->GKD;
    imu->lasterror=erspeed;
    if (imu_out > 5000) imu_out = 5000;
    else if (imu_out < -5000) imu_out = -5000;
    return imu_out;

}
void PID_output(void)
{
    CurveInfo curve;
//    static float last_Increase1 = 0;
//    static float last_Increase2 = 0;

            //增量式PID控制小车直行
    Increase1=MotorPID_Output(&sptr1,speed1,setspeed1);
    Increase2=MotorPID_Output(&sptr2,speed2,setspeed2);
            //方向环直接扭转小车运行方向
    divertion=imuPID_Output(erspeed,&imu);

    Increase1=Increase1-divertion;
    Increase2=Increase2+divertion;
    Motor_Left(Increase1);
    Motor_Right(Increase2);
//    if(Increase1-Increase2>-6000 && Increase1-Increase2<6000)
//    {
//    float diff = Increase1 - Increase2;
//    if (diff > 3500 || diff < -3500) {         // 最大转向
//        if (Increase1 < 0) Increase1 = -1.8 * Increase1;
//        if (Increase2 < 0) Increase2 = -1.8 * Increase2;
//    }
//    else if (diff > 2500 || diff < -2500) {    // 大转向
//        if (Increase1 < 0) Increase1 = -1.5 * Increase1;
//        if (Increase2 < 0) Increase2 = -1.5 * Increase2;
//    }
//    else if (diff > 1500 || diff < -1500) {    // 中等转向
//        if (Increase1 < 0) Increase1 = -1.0 * Increase1;
//        if (Increase2 < 0) Increase2 = -1.0 * Increase2;
//    }
//    else if (diff > 1000 || diff < -1000) {    // 小转向
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
//    sptr1.P=1.0;//pwm1000时的参数
//    sptr1.I=1.13;
//    sptr1.D=0;
//
//    sptr2.P=1.0;
//    sptr2.I=1.3;
//    sptr2.D=0;
//        //转向环PID
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
  @brief     PID控制
  @param     int set_speed ,int speed,期望值，实际值
  @return    电机占空比
  Sample     pwm_R= PID_R(set_speed_right,right_wheel);//pid控制电机转速
             pwm_L= PID_L(set_speed_left,left_wheel );//pid控制电机转速
  @note      调参呗
-------------------------------------------------------------------------------------------------------------------*/
//int PID_L(int set_speed ,float speed)//pid控制电机转速
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
//    if(out>=SPEED_MAX)//限幅处理
//        out=SPEED_MAX;
//    else if(out<=SPEED_MIN)
//        out=SPEED_MIN;
//    return (int) out;
//}
//
///*-------------------------------------------------------------------------------------------------------------------
//  @brief     PID控制
//  @param     int set_speed ,int speed,期望值，实际值
//  @return    电机占空比
//  Sample     pwm_R= PID_R(set_speed_right,right_wheel);//pid控制电机转速
//             pwm_L= PID_L(set_speed_left,left_wheel );//pid控制电机转速
//  @note      调参呗
//-------------------------------------------------------------------------------------------------------------------*/
//int PID_R(int set_speed ,float speed)//pid控制电机转速
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
//    if(out>=SPEED_MAX)//限幅处理
//        out=SPEED_MAX;
//    else if(out<=SPEED_MIN)
//        out=SPEED_MIN;
//    return (int) out;
//}



