/*
 * Encoder.c
 *
 *  Created on: 2025��1��11��
 *      Author: zhuji
 */
#include "Encoder.h"

//�ٶȱ���
volatile float Speed_Left_Real=0;   //����ʵ��ֵ
volatile float Speed_Right_Real=0;

void Encoder_Init()
{
    encoder_dir_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);//������1
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);//������2
}
void Encoder_Read(void)
{   speed1=-encoder_get_count(TIM2_ENCODER);//��
    encoder_clear_count(TIM2_ENCODER);
    speed2=encoder_get_count(TIM6_ENCODER);//��
    encoder_clear_count(TIM6_ENCODER);

    //system_delay_ms(100);
};
