#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"

extern volatile float Speed_Left_Real;   //����ʵ��ֵ
extern volatile float Speed_Right_Real;

void Encoder_Init();
void Encoder_Read(void);
#endif /* CODE_ENCODER_H_ */
