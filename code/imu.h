/*
 * imu.h
 *
 *  Created on: 2025��3��6��
 *      Author: �Ժؼ�
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_
#include "zf_common_headfile.h"

#define delta_T 0.00003f //1ms
extern float pitch, roll, yaw,ang_z;                // Z ����ٶȻ��ֽǶ�        // ŷ���� (��λ: ��)

void imu660_Read(void);
void gyroOffsetInit(void);
float myRsqrt(float num);
void ImuGetValues(void);
void ImuAHRSupdate(void);
void ReadGyro(void);
void ClearGyro(void);
void GyroResolve(void);



#endif /* CODE_IMU_H_ */
