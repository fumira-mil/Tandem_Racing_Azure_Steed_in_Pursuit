/*
 * iamge_dealwith.c
 *
 *  Created on: 2025年2月16日
 *      Author: zhuji
 */

#include "image_dealwith.h"

 const uint8 Standard_Road_Wide[MT9V03X_H]=
 { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
   39, 41, 43, 44, 45, 46, 47, 48, 49, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61,
   62, 63, 65, 66, 67, 68, 69, 70, 71, 72,
   73, 74, 76, 77, 79, 80, 81, 82, 83, 84,
   85, 86, 87, 88, 89, 90, 91, 92, 93, 95,
   96, 97, 98, 99, 100, 101, 102, 103, 105, 106,
   107, 108, 109, 110,111,112,113,114,116,117};

 const uint8 Standard_Road_Wide02[MT9V03X_H]=//标准赛宽
 { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
   10, 12, 14, 16, 18, 20, 22, 24, 26, 28,
   30, 32, 34, 36, 38, 40, 42, 44, 46, 48,
   50, 52, 54, 56, 58, 60, 62, 64, 66, 68,
   70, 72, 74, 76, 78, 80, 82, 84, 86, 88,
   90, 92, 94, 96, 98,100,102,104,106,108,
  110,112,114,116,118,120,122,124,126,128,
  130,132,134,136,138,140,142,144,146,148};

uint8 Threshold = 0 ;
extern uint8_t l_border[MT9V03X_H];
extern uint8_t r_border[MT9V03X_H];
extern uint8_t center_line[MT9V03X_H];
extern uint16 data_stastics_l, data_stastics_r;
extern uint8 hightest;
extern float err_road, steer, erspeed;
static RoadType current_road_type = STRAIGHT;
static uint8_t straight_counter = 0;
 //-------------------------------------------------------------------------------------------------------------------
 //  @brief      阈值处理算法
 //  @param      image  图像数组
 //  @param      clo    宽
 //  @param      row    高
 //  @param      pixel_threshold 阈值分离
 //  @return     uint8
 //  @since      2021.6.23
 //  @note       动态阈值，大津法
 //-------------------------------------------------------------------------------------------------------------------
 static uint8 Threshold_deal(uint8* image,
                      uint16 col,
                      uint16 row,
                      uint32 pixel_threshold) {
 #define GrayScale 256
   uint16 width = col;
   uint16 height = row;
   int pixelCount[GrayScale];    //灰度直方图
   float pixelPro[GrayScale];    //不同灰度值在图像中所占比例
   int i, j, pixelSum = width * height;     //pixelSum为整幅图像像素总和
   uint8 threshold = 0;
   uint8* data = image;  //指向像素数据的指针
   for (i = 0; i < GrayScale; i++) {
     pixelCount[i] = 0;
     pixelPro[i] = 0;
   }
   uint32 gray_sum = 0;
   //统计灰度级中每个像素在整幅图像中的个数
   for (i = 0; i < height; i += 1) {
     for (j = 0; j < width; j += 1) {
       pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
       gray_sum += (int)data[i * width + j];  //灰度值总和
     }
   }
   //计算每个像素值的点在整幅图像中的比例
   for (i = 0; i < GrayScale; i++) {
     pixelPro[i] = (float)pixelCount[i] / pixelSum;
   }
   //遍历灰度级[0,255]
   float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
   w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
   for (j = 0; j < pixel_threshold; j++) {
     w0 +=
         pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
     u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
     w1 = 1 - w0;
     u1tmp = gray_sum / pixelSum - u0tmp;
     u0 = u0tmp / w0;    //背景平均灰度
     u1 = u1tmp / w1;    //前景平均灰度
     u = u0tmp + u1tmp;  //全局平均灰度
     deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
     if (deltaTmp > deltaMax) {
       deltaMax = deltaTmp;
       threshold = (uint8)j;
     }
     if (deltaTmp < deltaMax) {
       break;
     }
   }
   return threshold;
 }
 //-------------------------------------------------------------------------------------------------------------------
 //  @brief      自适应阈值
 //  @param      img_data  输入图像数组
 //  @param      width    宽
 //  @param      height    高
 //  @param      block 小块区域的大小 面积block*block 即处理图像的次数 必须为奇数！！！
 //  @param      clip_value  一个经验值一般为 2-5
 //  @since      2025.2.16
 //  @note       那么自然我们得出一个简单的处理方法，对于每个像素点，取它周围一小片区域计算得出一个二值化阈值，并对这个像素进行二值化。参考之前的讲过的大津法，一小片区域的二值化阈值可以通过中位数，平均数或者大津法确定。在我们的代码实现中，采用了7x7的区域和平均数的确定方法另外，当这一小片区域亮度差异不大时，我们可能并不希望算法“强行”找到一个阈值，将这片区域分割开来，所以对于算法找出的局部阈值，我们将其减去一个经验性的参数（通常取2-5即可），避免算法将其强行分割。
 //-------------------------------------------------------------------------------------------------------------------
 void adaptiveThreshold(uint8_t* img_data, uint8_t* output_data, int width, int height, int block, uint8_t clip_value){

   int half_block = block / 2;
   for(int y=half_block; y<height-half_block; y++){
     for(int x=half_block; x<width-half_block; x++){
       // 计算局部阈值
       int thres = 0;
       for(int dy=-half_block; dy<=half_block; dy++){
         for(int dx=-half_block; dx<=half_block; dx++){
           thres += img_data[(x+dx)+(y+dy)*width];
         }
       }
       thres = thres / (block * block) - clip_value;
       // 进行二值化
       output_data[x+y*width] = img_data[x+y*width]>thres ? 255 : 0;
     }
   }
 }
 //-------------------------------------------------------------------------------------------------------------------
 //  @brief      图像二值化
 //  @since      2025.2.16
 //  @note       model设置 0 为大津法 1 为上交自适应阈值算法
 //-------------------------------------------------------------------------------------------------------------------

void Binarization_Algorithm(void)
{
    static uint8 model = 0 ;
    if(model == 0)
    {
        uint8 i = 0 ,j = 0;
        Threshold = Threshold_deal(image_copy[0], MT9V03X_W, MT9V03X_H, 190);
        for (i =0 ;i<MT9V03X_H ;i++){
            for (j = 0; j < MT9V03X_W; j++) {
               if (image_copy[i][j] >= (Threshold))         //数值越大，显示的内容越多，较浅的图像也能显示出来
                   image_dealwith[i][j] = IMG_WHITE;  //白
               else
                   image_dealwith[i][j] = IMG_BLACK;  //黑
             }
        }
    }
    else
    {
        adaptiveThreshold(image_copy[0], image_dealwith[0],MT9V03X_W, MT9V03X_H, 7, 2);
    }

}

/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  my_abs( x)；
 */
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

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}


/*变量声明*/
uint8 image_thereshold;//图像分割阈值
//------------------------------------------------------------------------------------------------------------------
//  @brief      获得一副灰度图像
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8(*image_copy)[image_w])
{
#define use_num     1   //1就是不压缩，2就是压缩一倍
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
            line++;
        }
        line = 0;
        row++;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;


    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }




    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------

void turn_to_bin(void)
{
  uint8 i,j;
 image_thereshold = otsuThreshold(original_image[0], image_w, image_h);
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
}


/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(image_h-2)
 */
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //从中间往左边，先找起点
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            //printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            //printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if(l_found&&r_found)return 1;
    else {
        //printf("未找到起点\n");
        return 0;
    }
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[image_w]       ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic              ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic              ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x               ：左边起点横坐标
l_start_y               ：左边起点纵坐标
r_start_x               ：右边起点横坐标
r_start_y               ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num image_h*3   //定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
            }

            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            //printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }

        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录生长方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }


    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;

}
/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */

void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //初始化
    for (i = 0;i<image_h;i++)
    {
        l_border[i] = border_min;
    }
    h = image_h - 2;
    //左边
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l[j][0]+1;
        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break;//到最后一行退出
        }
    }
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < image_h; i++)
    {
        r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = image_h - 2;
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r[j][0] - 1;
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max   255*5//此参数可根据自己的需求调节
#define threshold_min   255*2//此参数可根据自己的需求调节
void image_filter(uint8(*bin_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
                + bin_image[i][j - 1] + bin_image[i][j + 1]
                + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


            if (num >= threshold_max && bin_image[i][j] == 0)
            {

                bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改

            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {

                bin_image[i][j] = 0;//黑

            }

        }
    }

}


/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[image_w] 图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        //image[image_h-1][i] = 0;

    }
}

/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
 */
void image_process(void)
{
uint16 i;
uint8 hightest = 0;//定义一个最高行，tip：这里的最高指的是y值的最小
uint8 hightest_local = 0;
/*这是离线调试用的*/
Get_image(mt9v03x_image);
turn_to_bin();

image_filter(bin_image);
image_draw_rectan(bin_image);

/*提取赛道边界*/
image_filter(bin_image);//滤波
image_draw_rectan(bin_image);//预处理
//清零
data_stastics_l = 0;
data_stastics_r = 0;
if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
{
    //printf("正在开始八领域\n");
    search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
    //printf("八邻域已结束\n");
    // 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
    get_left(data_stastics_l);
    get_right(data_stastics_r);
    //处理函数放这里，不要放到if外面去了，不要放到if外面去了，不要放到if外面去了，重要的事说三遍

    for (i = hightest_local; i < image_h - 1; i++) {
        center_line[i] = (l_border[i] + r_border[i]) >> 1;
    }

    ips200_displayimage03x(bin_image[0], MT9V03X_W, MT9V03X_H);
    for (i = 0; i < data_stastics_l; i++) {
        ips200_draw_point(limit_a_b(points_l[i][0] + 2, 0, MT9V03X_W - 1),
                         limit_a_b(points_l[i][1], 0, MT9V03X_H - 1), uesr_BLUE);
    }
    for (i = 0; i < data_stastics_r; i++) {
        ips200_draw_point(limit_a_b(points_r[i][0] - 2, 0, MT9V03X_W - 1),
                         limit_a_b(points_r[i][1], 0, MT9V03X_H - 1), uesr_RED);
    }
    for (i = hightest_local; i < image_h - 1; i++) {
        ips200_draw_point(center_line[i], i, uesr_GREEN);
    }

}


//显示图像   改成你自己的就行 等后期足够自信了，显示关掉，显示屏挺占资源的
//ips154_displayimage032_zoom(bin_image[0], image_w, image_h, image_w, image_h,0,0);
ips200_displayimage03x(bin_image[0], MT9V03X_W, MT9V03X_H);
    //根据最终循环次数画出边界点
    for (i = 0; i < data_stastics_l; i++)
    {
        ips200_draw_point(limit_a_b(points_l[i][0]+2, 0, MT9V03X_W - 1), limit_a_b(points_l[i][1], 0, MT9V03X_H - 1), uesr_BLUE);//显示起点
    }
    for (i = 0; i < data_stastics_r; i++)
    {
        ips200_draw_point(limit_a_b(points_r[i][0]-2, 0 , MT9V03X_W - 1), limit_a_b(points_r[i][1], 0, MT9V03X_H - 1), uesr_RED);//显示起点
    }

    for (i = hightest; i < image_h-1; i++)
    {
        center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
        //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
        ips200_draw_point(center_line[i], i, uesr_GREEN);//显示起点 显示中线
//        ips200_draw_point(l_border[i], i, uesr_GREEN);//显示起点 显示左边线
//        ips200_draw_point(r_border[i], i, uesr_GREEN);//显示起点 显示右边线
    }


}


int get_curve_info(uint8_t* center_line, int start_row, int end_row, CurveInfo* info) {
    if (end_row - start_row < 3) return -1;

    double coeff[3];
    int count = end_row - start_row + 1;
    double S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
    double T0 = 0, T1 = 0, T2 = 0;

    for (int y = start_row; y <= end_row; y++) {
        int x = center_line[y];
        S0 += 1;
        S1 += y;
        S2 += y * y;
        S3 += y * y * y;
        S4 += y * y * y * y;
        T0 += x;
        T1 += x * y;
        T2 += x * y * y;
    }

    double A[3][3] = {{S4, S3, S2}, {S3, S2, S1}, {S2, S1, S0}};
    double B[3] = {T2, T1, T0};
    int n = 3;

    for (int i = 0; i < n; i++) {
        int pivot = i;
        double max_val = fabs(A[i][i]);
        for (int r = i + 1; r < n; r++) {
            if (fabs(A[r][i]) > max_val) {
                max_val = fabs(A[r][i]);
                pivot = r;
            }
        }
        if (fabs(A[pivot][i]) < 1e-10) return -1;
        if (pivot != i) {
            for (int j = i; j < n; j++) {
                double temp = A[i][j];
                A[i][j] = A[pivot][j];
                A[pivot][j] = temp;
            }
            double temp = B[i];
            B[i] = B[pivot];
            B[pivot] = temp;
        }
        for (int r = i + 1; r < n; r++) {
            double factor = A[r][i] / A[i][i];
            for (int j = i; j < n; j++) {
                A[r][j] -= factor * A[i][j];
            }
            B[r] -= factor * B[i];
        }
    }
    for (int i = n - 1; i >= 0; i--) {
        coeff[i] = B[i];
        for (int j = i + 1; j < n; j++) {
            coeff[i] -= A[i][j] * coeff[j];
        }
        coeff[i] /= A[i][i];
    }

    info->a2 = coeff[0];
    info->a1 = coeff[1];
    info->a0 = coeff[2];

    double slope = 2 * info->a2 * LOOK_AHEAD_Y + info->a1;
    info->curvature = fabs(2 * info->a2) / pow(1 + slope * slope, 1.5);
    return 0;
}

// 道路宽度趋势分析
int analyze_width_trend(int* trend_score) {
    *trend_score = 0;
    int valid_samples = 0;
    const uint8_t sample_interval = 5;

    for (uint8_t y = MT9V03X_H - 1; y > hightest + sample_interval; y -= sample_interval) {
        int width1 = r_border[y] - l_border[y];
        int width2 = r_border[y - sample_interval] - l_border[y - sample_interval];
        int diff = width2 - width1;
        int std_diff = abs(width1 - Standard_Road_Wide[y]);

        if (std_diff < WIDTH_VARIATION_THRESH) continue;
        *trend_score += diff;
        valid_samples++;
    }
    if (valid_samples > 0) {
        *trend_score /= valid_samples;
        return valid_samples;
    }
    return 0;
}

// 边界跳变检测
int detect_border_jump(int* direction) {
    *direction = 0;
    int left_jump = 0, right_jump = 0;
    const uint8_t jump_threshold = 25;

    for (uint8_t y = MT9V03X_H - 30; y > hightest; y -= 5) {
        if (y + 5 >= MT9V03X_H) continue;
        int left_diff = abs(l_border[y] - l_border[y + 5]);
        int right_diff = abs(r_border[y] - r_border[y + 5]);
        if (left_diff > jump_threshold) left_jump++;
        if (right_diff > jump_threshold) right_jump++;
    }

    if (left_jump > 2 && right_jump <= 1) {
        *direction = -1;
        return 1;
    }
    if (right_jump > 2 && left_jump <= 1) {
        *direction = 1;
        return 1;
    }
    return 0;
}


RoadType recognize_road_type(void) {
    if (data_stastics_l < MIN_VALID_ROWS || data_stastics_r < MIN_VALID_ROWS) {
        return current_road_type;
    }

    CurveInfo curve;
    if (get_curve_info(center_line, hightest, MT9V03X_H - 1, &curve) != 0) {
        return current_road_type;
    }

    int trend_score;
    int valid_samples = analyze_width_trend(&trend_score);

    int jump_direction;
    int has_jump = detect_border_jump(&jump_direction);

    if (curve.curvature < CURVATURE_THRESHOLD &&
        abs(trend_score) < 15 &&
        !has_jump) {
        if (++straight_counter > STRAIGHT_CONFIRM_COUNT) {
            current_road_type = STRAIGHT;
        }
    } else {
        straight_counter = 0;
        if (curve.curvature > CURVATURE_THRESHOLD || valid_samples > 3 || has_jump) {
            if (curve.a2 > 0 || trend_score < -15 || jump_direction == -1) {
                current_road_type = LEFT_CURVE;
            } else if (curve.a2 < 0 || trend_score > 15 || jump_direction == 1) {
                current_road_type = RIGHT_CURVE;
            } else {
                current_road_type = UNKNOWN;
            }
        }
    }
//    printf("Type: %d, Curvature: %f\n", current_road_type,curve.curvature);
    return current_road_type;
}



/*

这里是起点（0.0）***************——>*************x值最大
************************************************************
************************************************************
************************************************************
************************************************************
******************假如这是一副图像*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
y值最大*******************************************(188.120)

*/

