/*
 * image_dealwith.h
 *
 *  Created on: 2025??2??16??
 *      Author: zhuji
 */

#ifndef CODE_IMAGE_DEALWITH_H_
#define CODE_IMAGE_DEALWITH_H_
#include "zf_common_headfile.h"

//???????  ???????????????????????????????????
#define uesr_RED     0XF800    //???
#define uesr_GREEN   0X07E0    //???
#define uesr_BLUE    0X001F    //???

#define white_pixel 255
#define black_pixel 0

//????
#define image_h 120//?????
#define image_w 188//??????

#define ERROR_COUNT 80

#define bin_jump_num    1//?????????
#define border_max  image_w-2 //???????
#define border_min  1   //?????С?

extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//???????

void image_process(void); //??????ж??????????????????????????

#define IMG_BLACK     0X00      //0x00???
#define IMG_WHITE     0Xff      //0xff???

// 道路类型枚举
typedef enum {
    STRAIGHT,
    LEFT_CURVE,
    RIGHT_CURVE,
    UNKNOWN
} RoadType;

// 曲率计算结构体
typedef struct {
    double a0;        // 常数项
    double a1;        // 一次项系数
    double a2;        // 二次项系数
    double curvature; // 曲率值
} CurveInfo;
//typedef struct {
//    double curvature;  // 曲率
//    double a2;         // 二阶系数（决定弯曲方向）
//    int trend_score;   // 宽度趋势
//    int jump_direction; // 边界跳变方向
//} SegmentInfo;
//#define ERROR_COUNT1 120  // 数组长度为 120
//
//uint8 jiaquan_[ERROR_COUNT1] = {
//    10, 10, 10, 10, 10, 10, 10, 10,  // 119-112
//    9,  9,  9,  9,  9,  9,  9,  9,   // 111-104
//    8,  8,  8,  8,  8,  8,  8,  8,   // 103-96
//    7,  7,  7,  7,  7,  7,  7,  7,   // 95-88
//    6,  6,  6,  6,  6,  6,  6,  6,   // 87-80
//    5,  5,  5,  5,  5,  5,  5,  5,   // 79-72
//    4,  4,  4,  4,  4,  4,  4,  4,   // 71-64
//    3,  3,  3,  3,  3,  3,  3,  3,   // 63-56
//    2,  2,  2,  2,  2,  2,  2,  2,   // 55-48
//    1,  1,  1,  1,  1,  1,  1,  1,   // 47-40
//    1,  1,  1,  1,  1,  1,  1,  1,   // 39-32
//    1,  1,  1,  1,  1,  1,  1,  1,   // 31-24
//    1,  1,  1,  1,  1,  1,  1,  1,   // 23-16
//    1,  1,  1,  1,  1,  1,  1,  1,   // 15-8
//    1,  1,  1,  1,  1,  1,  1,  1    // 7-0
//};
// 道路识别参数配置
#define CURVATURE_THRESHOLD     0.003  // 曲率阈值（调试后设定）
#define WIDTH_VARIATION_THRESH  20     // 宽度变化阈值
#define MIN_VALID_ROWS          40     // 最小有效行数
#define LOOK_AHEAD_Y            60     // 前瞻行（中部）
#define STRAIGHT_CONFIRM_COUNT  5      // 直线确认计数
//#define SEGMENT_SIZE 30  // 每段 30 行（可调整）
//#define NUM_SEGMENTS (MT9V03X_H / SEGMENT_SIZE)  // 分成 4 段
//#define CONFIRM_THRESHOLD 3  // 确认次数阈值
//#define WEIGHT_BASE 1.0  // 基础权重
//#define WEIGHT_INCREMENT 0.5  // 每段权重增量（底部权重最高）
//
//
//static RoadType current_road_type = STRAIGHT;
//static uint8_t straight_counter = 0;
//static uint8_t left_curve_counter = 0;
//static uint8_t right_curve_counter = 0;


//Type：0=直线，1=左弯，2=右弯，3=未知。
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];
extern uint8 image_dealwith[MT9V03X_H][MT9V03X_W];
int get_curve_info(uint8_t* center_line, int start_row, int end_row, CurveInfo* info);
int analyze_width_trend(int* trend_score);
int detect_border_jump(int* direction);
RoadType recognize_road_type(void);


void Binarization_Algorithm(void);

float get_center_error(void);

extern uint8 l_border[image_h];//????????
extern uint8 r_border[image_h];//????????
extern uint8 center_line[image_h];//????????
extern uint8 hightest;//最高点
#endif /* CODE_IMAGE_DEALWITH_H_ */
