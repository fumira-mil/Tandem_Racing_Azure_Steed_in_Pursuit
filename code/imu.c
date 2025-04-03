/*
 * imu.c
 *
 *  Created on: 2025Äê3ÔÂ6ÈÕ
 *      Author: ÕÔºØ¼Î
 */
#include "imu.h"

void imu660_Read(void)
{
    imu660ra_get_gyro ();
    imu660ra_get_acc ();
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


