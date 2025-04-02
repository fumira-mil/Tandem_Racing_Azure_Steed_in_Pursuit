/*
 * imu.c
 *
 *  Created on: 2025��3��6��
 *      Author: �Ժؼ�
 */
#include "imu.h"

float X, Y, Z;                  // ��������ƯУ׼ֵ
float acc_x, acc_y, acc_z;      // ���ٶȼ��˲�ֵ (��λ: g)
float gyro_x, gyro_y, gyro_z;   // �����ǽ��ٶ� (��λ: rad/s)
float Q0 = 1, Q1 = 0, Q2 = 0, Q3 = 0; // ��Ԫ������ʼ��Ϊ 1 0 0 0
float I_ex, I_ey, I_ez;         // ������
float icm_kp = 0.17;            // ���ٶȼ��������ʱ�������
float icm_ki = 0.004;           // �������������ʻ�������
float pitch, roll, yaw;         // ŷ���� (��λ: ��)
float ang_z = 0;                // Z ����ٶȻ��ֽǶ�

uint8 GyroOn = 0;               // �Ƿ�����־
uint8 GyroINT = 1;              // �жϴ�����־

// IMU660RA ԭʼ���� (������ٶȺ����������ݶ��� IMU660RA ��ȡ)
extern int16 imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z;  // ���ٶȼ�ԭʼ����
extern int16 imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z; // ������ԭʼ����

void imu660_Read(void)
{
    float alpha = 0.3;  // һ�׵�ͨ�˲�ϵ��

    // ��ȡ IMU660RA ���ݣ������Ѷ��� imu660ra_get_acc ������
    imu660ra_get_gyro();  // ��ȡ����������
    // imu660ra_get_acc();  // ��ȡ���ٶ����ݣ�������ʵ�֣�

    // һ�׵�ͨ�˲�����λ g (�������� ��8g�������� 4096 LSB/g)
    acc_x = (((float)imu660ra_acc_x) * alpha) / 4096 + acc_x * (1 - alpha);
    acc_y = (((float)imu660ra_acc_y) * alpha) / 4096 + acc_y * (1 - alpha);
    acc_z = (((float)imu660ra_acc_z) * alpha) / 4096 + acc_z * (1 - alpha);

    // �����ǽ��ٶ�ת��Ϊ������ (���� ��2000��/s�������� 16.4 LSB/(��/s))
    gyro_x = ((float)imu660ra_gyro_x - X) * PI / 180 / 16.4f;
    gyro_y = ((float)imu660ra_gyro_y - Y) * PI / 180 / 16.4f;
    gyro_z = ((float)imu660ra_gyro_z - Z) * PI / 180 / 16.4f;
//    imu660ra_gyro_transition(imu660ra_gyro_x) ;
//    imu660ra_gyro_transition(imu660ra_gyro_y) ;
//    imu660ra_gyro_transition(imu660ra_gyro_z) ;
//    imu660ra_gyro_x=imu660ra_gyro_x-1;
//    imu660ra_gyro_y=imu660ra_gyro_y-4;
//    imu660ra_gyro_z=imu660ra_gyro_z+1;
//    imu660ra_acc_x=imu660ra_acc_x+43.2;
//    imu660ra_acc_y=imu660ra_acc_y+125.5;
//    imu660ra_acc_z=imu660ra_acc_z+4100.5;
//    ips200_show_float(0,90,imu660ra_gyro_z,3,2);
//ips200_show_int(0, 190, imu660ra_gyro_x, 3);
//ips200_show_int(7, 210, imu660ra_gyro_y, 3);
//ips200_show_int(14, 230, imu660ra_gyro_z, 3);
//ips200_show_int(0, 250, imu660ra_acc_x, 3);
//ips200_show_int(7, 270, imu660ra_acc_y, 3);
//ips200_show_int(14, 290, imu660ra_acc_z, 3);
//   put_int32(3, imu660ra_gyro_x);
//   put_int32(4, imu660ra_gyro_y);
//    put_int32(5, imu660ra_gyro_z);
//   put_int32(6, imu660ra_acc_x);
//   put_int32(7, imu660ra_acc_y);
//   put_int32(8, imu660ra_acc_z);
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�� IMU660RA ��������Ư
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     gyroOffsetInit();  // ���ú� X, Y, Z ���洢��������Ưֵ
// ��ע��Ϣ     ͨ�� 100 �β���ƽ��������Ư
//-------------------------------------------------------------------------------------------------------------------
void gyroOffsetInit(void)
{
    uint8 i;
    X = -1;
    Y = -2;
    Z = -1;
    for (i = 0; i < 100; ++i)
    {
        imu660ra_get_gyro(); // ��ȡ����������
        X += imu660ra_gyro_x;
        Y += imu660ra_gyro_y;
        Z += imu660ra_gyro_z;
        system_delay_ms(5);  // 5ms ���������Ƶ��Լ 200Hz
    }

    X /= 100;  // ����ƽ����Ư
    Y /= 100;
    Z /= 100;
}
float myRsqrt(float num)
{
    float halfx = 0.5f * num;
    float y = num;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1);

    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���� IMU660RA ��Ԫ�� (AHRS �㷨)
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     ImuAHRSupdate();  // ���� Q0-Q3
// ��ע��Ϣ     �趨��ȫ�ֱ��� delta_T (�������ڣ���λ: ��)
//-------------------------------------------------------------------------------------------------------------------
void ImuAHRSupdate(void)
{
    float halfT = 0.5f * delta_T;  // ��������һ��
    float vx, vy, vz;              // ��������
    float ex, ey, ez;              // �������
    float q0 = Q0, q1 = Q1, q2 = Q2, q3 = Q3; // ��ǰ��Ԫ��
    float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;
    float norm;

    // ���ٶȹ�һ��
    norm = myRsqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    acc_x = acc_x * norm;
    acc_y = acc_y * norm;
    acc_z = acc_z * norm;

    // ������������������ϵ��
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // ������
    ex = acc_y * vz - acc_z * vy;
    ey = acc_z * vx - acc_x * vz;
    ez = acc_x * vy - acc_y * vx;

    // ������
    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    // PI ������У��������
    gyro_x = gyro_x + icm_kp * ex + icm_ki * I_ex;
    gyro_y = gyro_y + icm_kp * ey + icm_ki * I_ey;
    gyro_z = gyro_z + icm_kp * ez + icm_ki * I_ez;

    // һ���������������Ԫ��
    q0 = q0 + (-q1 * gyro_x - q2 * gyro_y - q3 * gyro_z) * halfT;
    q1 = q1 + (q0 * gyro_x + q2 * gyro_z - q3 * gyro_y) * halfT;
    q2 = q2 + (q0 * gyro_y - q1 * gyro_z + q3 * gyro_x) * halfT;
    q3 = q3 + (q0 * gyro_z + q1 * gyro_y - q2 * gyro_x) * halfT;

    // ��Ԫ����һ��
    norm = myRsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q0 = q0 * norm;
    Q1 = q1 * norm;
    Q2 = q2 * norm;
    Q3 = q3 * norm;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ IMU660RA ���������ݲ�ת��Ϊ����λ (��/s)
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     get_gyro();  // ���� ang_z
// ��ע��Ϣ     �� Z�������޷� (��2500 LSB)����ת��Ϊ ��/s
//-------------------------------------------------------------------------------------------------------------------
void get_gyro(void)
{
    imu660ra_get_gyro();  // ��ȡ����������
//    imu660ra_gyro_x=imu660ra_gyro_x;
    // �޷� (��2500 LSB�������޷�ֵ��������̵���)
//    if (imu660ra_gyro_z > 2500) imu660ra_gyro_z = 2500;
//    else if (imu660ra_gyro_z < -2500) imu660ra_gyro_z = -2500;
//
//    // ת��Ϊ����λ (��/s)
//    ang_z = imu660ra_gyro_transition(imu660ra_gyro_z);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Ԫ������ŷ����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     ReadGyro();  // ���� pitch, roll, yaw
// ��ע��Ϣ     �����λΪ��
//-------------------------------------------------------------------------------------------------------------------
void ReadGyro(void)
{
    pitch = asin(2 * Q0 * Q2 - 2 * Q1 * Q3) * 180 / PI;                   // ������
    roll = atan2(2 * Q2 * Q3 + 2 * Q0 * Q1, -2 * Q1 * Q1 - 2 * Q2 * Q2 + 1) * 180 / PI; // �����
    yaw = atan2(2 * Q1 * Q2 + 2 * Q0 * Q3, -2 * Q2 * Q2 - 2 * Q3 * Q3 + 1) * 180 / PI;  // ƫ����
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ������Ԫ����������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     ClearGyro();
// ��ע��Ϣ     ����Ԫ��������������
//-------------------------------------------------------------------------------------------------------------------
void ClearGyro(void)
{
    Q0 = 1;
    Q1 = 0;
    Q2 = 0;
    Q3 = 0;
    I_ex = 0;
    I_ey = 0;
    I_ez = 0;
}
//-------------------------------------------------------------------------------------------------------------------
// �������     IMU660RA ��Ԫ����̬����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     GyroResolve();  // ���жϻ���ѭ���е���
// ��ע��Ϣ     ���� GyroINT ��������
//-------------------------------------------------------------------------------------------------------------------
void GyroResolve(void)
{
    if (GyroINT)
    {
        get_gyro();

        if (GyroOn)
        {
            imu660_Read();
            ImuAHRSupdate();
            ReadGyro();
        }
        GyroINT = 0;
    }
}

