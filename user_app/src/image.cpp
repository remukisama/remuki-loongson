#include "image.hpp"
#include "user_app.hpp"
#include "car_config.hpp"
#include <atomic>
#include <iostream>
#include <cstdlib>


int img1[60][80];//图像二值化之后的数组
int imgdisplay[60][80];//打印到中端图像数组

// 定义像素类型（方便后续上位机识别）
#define PIXEL_BLACK     0   // 黑色（赛道）
#define PIXEL_WHITE     1   // 白色（背景）
#define PIXEL_LEFT      2   // 左边界
#define PIXEL_RIGHT     3   // 右边界
#define PIXEL_CENTER    4   // 中线

uint8 Garage_Location_Flag=0;//斑马线累积初始次数
bool g_enable_element_process = true; // 默认启用元素判断与元素处理
uint8 Garage_num=2;//斑马线设定次数
int Zebra_num1=10000;//斑马线编码器计数设定值
int Zebra_num2=1000;//停车斑马线编码器计数设定值

static int IntervalLow = 0, IntervalHigh = 0;      //定义高低扫描区间
static int *PicTemp;
static int Ysite = 0, Xsite = 0;   
ImageDealDatatypedef ImageDeal[60]; //最终图像
uint8 ExtenLFlag = 0;  //是否左延长标志
uint8 ExtenRFlag = 0;  //是否右延长标志
int ImageScanInterval_Cross;                   //270°的弯道后十字的扫线范围
int ImageScanInterval;                         //扫边范围    上一行的边界+-ImageScanInterval
ImageStatustypedef ImageStatus;  //图像的全局变量
SystemDatatypedef SystemData;
ImageParametertypedef ImageParameter;

static int TFSite = 0, FTSite = 0;                 //存放行
static float DetR = 0, DetL = 0;                   //存放斜率
static int ytemp = 0;                              //存放行

uint8 Circle[5];
uint8  TP = 25, TP_O1 = 25 ,TP_O2 = 25;
uint32  circle_count_flag = 0;

float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77,0.71, 0.65, 0.59, 0.53, 0.47};//10行权重参数，随意更改，基本不影响，大致按照正态分布即可

float variance, variance_acc;  //方差--直线检测用

uint8 circle_num = 0;
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;   //左圆环判断的两点纵坐标
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite; //右圆环判断的两点纵坐标
uint8 Ring_Help_Flag = 0;                      //进环辅助标志
static uint8 left_ring_confirm_frames = 0;     // 左圆环连续确认帧数
static uint8 right_ring_confirm_frames = 0;    // 右圆环连续确认帧数
ImageFlagtypedef ImageFlag;
int Point_Xsite,Point_Ysite;                   //拐点横纵坐标
int Repair_Point_Xsite,Repair_Point_Ysite;     //补线点横纵坐标
uint8 Half_Road_Wide[60] =                      //直到赛道半宽
{  4, 5, 5, 6, 6, 6, 7, 7, 8, 8,
        9, 9,10,10,10,11,12,12,13,13,
       13,14,14,15,15,16,16,17,17,17,
       18,18,19,19,20,20,20,21,21,22,
       23,23,23,24,24,25,25,25,26,26,
       27,28,28,28,29,30,31,31,31,32,
};

static int clamp_image_x(int value)
{
  if (value < 1) return 1;
  if (value > 78) return 78;
  return value;
}

static int expected_road_width(int row)
{
  if (row < 0) row = 0;
  if (row > 59) row = 59;
  return static_cast<int>(Half_Road_Wide[row]) * 2;
}

static int center_from_right_border(int row, int right_border, int bias)
{
  return clamp_image_x(right_border - Half_Road_Wide[row] + bias);
}

static int center_from_left_border(int row, int left_border, int bias)
{
  return clamp_image_x(left_border + Half_Road_Wide[row] + bias);
}

static bool is_border_found(uint8 flag)
{
  return flag == 'T' || flag == 'H';
}

static bool is_road_width_valid(int row, int width)
{
  if (width < car_config::kCurveCenterMinValidWidth ||
      width > car_config::kCurveCenterMaxValidWidth) {
    return false;
  }
  return std::abs(width - expected_road_width(row)) <= car_config::kCurveCenterWidthTolerance;
}

static int limit_step(int value, int reference, int max_step)
{
  if (max_step <= 0) return clamp_image_x(value);
  if (value > reference + max_step) return clamp_image_x(reference + max_step);
  if (value < reference - max_step) return clamp_image_x(reference - max_step);
  return clamp_image_x(value);
}

static int g_curve_center_last[60] = {0};
static bool g_curve_center_has_last = false;

static void ResetCurveCenterHistory(void)
{
  g_curve_center_has_last = false;
}

static void ApplyCurveCenterCompensation(void)
{
  if (!car_config::kCurveCenterCompEnable) {
    ResetCurveCenterHistory();
    return;
  }
  if (ImageStatus.OFFLine >= 45 ||
      (ImageStatus.Left_Line >= car_config::kCurveCenterOppositeLossMax &&
       ImageStatus.Right_Line >= car_config::kCurveCenterOppositeLossMax)) {
    ResetCurveCenterHistory();
    return;
  }

  const bool left_loss_scene =
      ImageStatus.Left_Line >= car_config::kCurveCenterLossStart &&
      ImageStatus.Right_Line <= car_config::kCurveCenterOppositeLossMax;
  const bool right_loss_scene =
      ImageStatus.Right_Line >= car_config::kCurveCenterLossStart &&
      ImageStatus.Left_Line <= car_config::kCurveCenterOppositeLossMax;

  const int start_row = 58;
  const int end_row = ImageStatus.OFFLine;

  for (int row = start_row; row >= end_row; --row) {
    const int width = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
    const bool left_ok = is_border_found(ImageDeal[row].IsLeftFind);
    const bool right_ok = is_border_found(ImageDeal[row].IsRightFind);
    const bool both_ok = left_ok && right_ok && is_road_width_valid(row, width);

    int candidate = clamp_image_x(ImageDeal[row].Center);
    bool single_side_candidate = false;
    bool weak_candidate = false;

    if (both_ok) {
      candidate = clamp_image_x((ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2);
    } else if (!left_ok && right_ok && left_loss_scene &&
               ImageStatus.Right_Line <= car_config::kCurveCenterOppositeLossMax) {
      candidate = center_from_right_border(
          row, ImageDeal[row].RightBorder, -car_config::kCurveCenterSingleSideBias);
      single_side_candidate = true;
    } else if (!right_ok && left_ok && right_loss_scene &&
               ImageStatus.Left_Line <= car_config::kCurveCenterOppositeLossMax) {
      candidate = center_from_left_border(
          row, ImageDeal[row].LeftBorder, car_config::kCurveCenterSingleSideBias);
      single_side_candidate = true;
    } else {
      weak_candidate = true;
      if (row < start_row) {
        candidate = ImageDeal[row + 1].Center;
      } else {
        candidate = ImageStatus.MiddleLine;
      }
    }

    if (row < start_row && (single_side_candidate || weak_candidate)) {
      candidate = limit_step(candidate, ImageDeal[row + 1].Center,
                             car_config::kCurveCenterMaxRowStep);
    }

    ImageDeal[row].Center = clamp_image_x(candidate);
    if (ImageDeal[row].LeftBorder < ImageDeal[row].Center &&
        ImageDeal[row].Center < ImageDeal[row].RightBorder) {
      ImageDeal[row].Wide = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;
    }
    g_curve_center_last[row] = ImageDeal[row].Center;
  }
  g_curve_center_has_last = true;
}

void Data_Settings(void)           //参数赋值
{

  //图像参数
  //adcsum = 0;
  ImageStatus.MiddleLine = car_config::kImageMiddleLine;//中线  39
  ImageStatus.TowPoint_Gain = 0.2;
  ImageStatus.TowPoint_Offset_Max = 5;
  ImageStatus.TowPoint_Offset_Min = -2;
  ImageStatus.TowPointAdjust_v = 160;
  ImageStatus.Det_all_k = 0.5;  //待定自动补线斜率
  ImageStatus.CirquePass = 'F';
  ImageStatus.IsCinqueOutIn = 'F';
  ImageStatus.CirqueOut = 'F';
  ImageStatus.CirqueOff = 'F';
  ImageStatus.Barn_Flag = 0;
  ImageStatus.straight_acc = 0;
  ImageStatus.Road_type= zero ;

  ImageStatus.TowPoint = car_config::kImageTowPoint;                //前瞻          前瞻29 速度 220
  ImageStatus.Threshold_static = car_config::kImageThresholdStatic;  //静态阈值  40-80
  ImageStatus.Threshold_detach = car_config::kImageThresholdDetach;  //阳光算法  亮斑分离140-220
  ImageScanInterval = car_config::kImageScanInterval;                //扫边范围    上一行的边界+-ImageScanInterval
  ImageScanInterval_Cross = car_config::kImageScanIntervalCross;     //十字扫线范围
 ImageStatus.variance = car_config::kImageVariance;                 //直道方差阈值
  ImageStatus.variance_acc = car_config::kImageVarianceAcc;          //直线加速检测
//  SystemData.outbent_acc  =  5;
  //SystemData.clrcle_num=0;
  ImageStatus.newblue_flag=0;
  //SystemData.Stop = 2;                 //启动标志位
  //BlueTooth_Flag=1;

  Circle[0]=TP_O1;
  Circle[1]=TP_O2;
  
//   SteerPIDdata.Dl = 21.07;
//   SteerPIDdata.Dh = 5.0;
//   Left_Speed_Co_one_minus =0.06; Right_Speed_Co_one_minus=0.06 ;   //0.06

  /**位置式pid参数**/
  // 重中之重： 电池电压低于12V 去充电 否则参数都是不对的
  // 方向环可以先调P 发现P已经转弯接近内切的时候，再去加D
//  SteerPIDdata.P = 1.5;  //16 原来2.2 5.4大了 3.2小了 4.2大了一点点 3.9      原来2.9   100hz参数      200hz参数:9.0 9.5 10.5 11.5 12.0 13.0
//  //  11.0  比较适合的值：10.5
//  SteerPIDdata.I = 0.0;
//  SteerPIDdata.D = 2.7;  //原来7.2                       9.0 10.0   目前最好9.5                               原来6.8 测试200hz参数:29.0
  //200hz D：32小了 35也有点小大 39大 37大了
}

static uint8 drawfirstline(void)
{
  PicTemp=img1[59];  //单行图像PicTemp                //原本59  改成了57
  if (*(PicTemp + ImageSensorMid) == 0)                 //如果底边图像中点为黑，异常情况
  {
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)    //找左右边线
    {
      if (*(PicTemp + ImageSensorMid - Xsite) != 0)     //一旦找到左或右赛道到中心距离，就break
      break;                                            //并且记录Xsite
      if (*(PicTemp + ImageSensorMid + Xsite) != 0)
      break;
    }
    if (*(PicTemp + ImageSensorMid - Xsite) != 0)       //赛道如果在左边的话
    {
      BottomBorderRight = ImageSensorMid - Xsite + 1;   // 59行右边线有啦
      for (Xsite = BottomBorderRight; Xsite > 0; Xsite--)//开始找59行左边线
      {
        if (*(PicTemp + Xsite) == 0 &&*(PicTemp + Xsite - 1) == 0)//连续两个黑点，滤波
        {
          BottomBorderLeft = Xsite;                     //左边线找到
          break;
        }
        else if (Xsite == 1)
        {
        BottomBorderLeft = 0;                         //搜索到最后了，看不到左边线，左边线认为是0
        break;
        }
      }
    }
    else if (*(PicTemp + ImageSensorMid + Xsite) != 0)  //赛道如果在右边的话
    {
      BottomBorderLeft = ImageSensorMid + Xsite - 1;    // 59行右边线有啦
      for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++)  //开始找59行右边线
      {
        if (  *(PicTemp + Xsite) == 0&&*(PicTemp + Xsite + 1) == 0)              //连续两个黑点，滤波
        {
          BottomBorderRight = Xsite;                    //右边线找到
          break;
        }
          else if (Xsite == 78)
        {
          BottomBorderRight = 79;                       //搜索到最后了，看不到右边线，左边线认为是79
          break;
        }
      }
    }
  }
  else                                                 //左边线中点是白的，比较正常的情况
  {
    for (Xsite = 79; Xsite >ImageSensorMid; Xsite--)   //一个点一个点地搜索右边线
    {
      if (  *(PicTemp + Xsite) == 1&&*(PicTemp + Xsite - 1) == 1)                //连续两个白点，滤波
      {
        BottomBorderRight = Xsite;                      //找到就记录
        break;
      }
      else if (Xsite == 40)
      {
        BottomBorderRight = 39;                         //找不到认为79
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)    //一个点一个点地搜索左边线
    {
      if (  *(PicTemp + Xsite) == 1
          &&*(PicTemp + Xsite + 1) == 1)                //连续两个黑点，滤波
      {
        BottomBorderLeft = Xsite;                       //找到就记录
        break;
      }
      else if (Xsite == 38)
      {
        BottomBorderLeft = 39;                           //找不到认为0
        break;
      }
    }
  }
  BottomCenter =(BottomBorderLeft + BottomBorderRight) / 2;   // 59行中点直接取平均
  ImageDeal[59].LeftBorder = BottomBorderLeft;                //在数组里面记录一下信息，第一行特殊一点而已
  ImageDeal[59].RightBorder = BottomBorderRight;
  ImageDeal[59].Center = BottomCenter;                        //确定最底边
  ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;  //存储宽度信息
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';
  for (Ysite = 58; Ysite > 54; Ysite--)                       //由两边向中间确定底边五行
  {
    PicTemp = img1[Ysite];
    for (Xsite = 79; Xsite > ImageDeal[Ysite + 1].Center;Xsite--)                                             //和前面一样的搜索
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1)
      {
        ImageDeal[Ysite].RightBorder = Xsite;
        break;
      }
      else if (Xsite == (ImageDeal[Ysite + 1].Center+1))
      {
        ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageDeal[Ysite + 1].Center;Xsite++)                                             //和前面一样的搜索
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1)
      {
        ImageDeal[Ysite].LeftBorder = Xsite;
        break;
      }
      else if (Xsite == (ImageDeal[Ysite + 1].Center-1))
      {
        ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    ImageDeal[Ysite].IsLeftFind = 'T';                        //这些信息存储到数组里
    ImageDeal[Ysite].IsRightFind = 'T';
    ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /2; //存储中点
    ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;      //存储宽度
  }
  return 'T';
}

void GetJumpPointFromDet(int* p,uint8 type,int L,int H,JumpPointtypedef* Q)  //第一个参数是要查找的数组（80个点）
                                                                               //第二个扫左边线还是扫右边线
{                                                                              //三四是开始和结束点
  int i = 0;
  if (type == 'L')                              //扫描左边线
  {
    for (i = H; i >= L; i--)
    {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   //由黑变白
      {
        Q->point = i;                           //记录左边线
        Q->type = 'T';                          //正确跳变
        break;
      }
      else if (i == (L + 1))                  //若果扫到最后也没找到
      {
        if (*(p + (L + H) / 2) != 0)            //如果中间是白的
        {
          Q->point = (L + H) / 2;               //认为左边线是中点
          Q->type = 'W';                        //非正确跳变且中间为白，认为没有边
          break;
        }
        else                                  //非正确跳变且中间为黑
        {
          Q->point = H;                         //如果中间是黑的
          Q->type = 'H';                        //左边线直接最大值，认为是大跳变
          break;
        }
      }
    }
  }
  else if (type == 'R')                       //扫描右边线
  {
    for (i = L; i <= H; i++)                    //从右往左扫
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   //找由白到黑的跳变
      {
        Q->point = i;                           //记录
        Q->type = 'T';
        break;
      } else if (i == (H - 1))                  //若果扫到最后也没找到
      {
        if (*(p + (L + H) / 2) != 0)            //如果中间是白的
        {
          Q->point = (L + H) / 2;               //右边线是中点
          Q->type = 'W';
          break;
        } else                                  //如果中点是黑的
        {
          Q->point = L;                         //左边线直接最大值
          Q->type = 'H';
          break;
        }
      }
    }
  }
}

static void DrawLinesProcess(void)  //////不用更改
{
  uint8 L_Found_T = 'F';  //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_L_line = 'F';  //找到这一帧图像的基准左斜率
  uint8 R_Found_T = 'F';  //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_R_line = 'F';  //找到这一帧图像的基准右斜率
  float D_L = 0;           //延长线左边线斜率
  float D_R = 0;           //延长线右边线斜率
  int ytemp_W_L;           //记住首次左丢边行
  int ytemp_W_R;           //记住首次右丢边行
  ExtenRFlag = 0;          //延长线标志位清0
  ExtenLFlag = 0;
   ImageStatus.Left_Line = 0;
   ImageStatus.WhiteLine = 0;
   ImageStatus.Right_Line = 0;
   uint16_t end_1= 0;
  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--)            //前5行处理过了，下面从55行到（设定的不处理的行OFFLine）
  {                        //太远的图像不稳定，OFFLine以后的不处理
    PicTemp = img1[Ysite];
    JumpPointtypedef JumpPoint[2];                                          // 0左1右
    if (ImageStatus.Road_type != Cross_ture
           /* &&SystemData.SpeedData.Length*OX>500*/)
    {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval;//暂时为2             //从上一行右边线-Interval的点开始（确定扫描开始点）
      IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;           //到上一行右边线+Interval的点结束（确定扫描结束点）
    } else {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval_Cross;//暂时为2        //从上一行右边线-Interval_Cross的点开始（确定扫描开始点）
      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;    //到上一行右边线+Interval_Cross的点开始（确定扫描开始点）
    }

    LimitL(IntervalLow);   //确定左扫描区间并进行限制
    LimitH(IntervalHigh);  //确定右扫描区间并进行限制
    GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);     //扫右边线

    IntervalLow =ImageDeal[Ysite + 1].LeftBorder -ImageScanInterval;                //从上一行左边线-5的点开始（确定扫描开始点）
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;               //到上一行左边线+5的点结束（确定扫描结束点）

    LimitL(IntervalLow);   //确定左扫描区间并进行限制
    LimitH(IntervalHigh);  //确定右扫描区间并进行限制
    GetJumpPointFromDet(PicTemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);

    
    if (JumpPoint[0].type =='W')                                                    //如果本行左边线不正常跳变，即这10个点都是白的
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                 //本行左边线用上一行的数值
    }
    else                                                                          //左边线正常
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                             //记录下来啦
    }

    if (JumpPoint[1].type == 'W')                                                   //如果本行右边线不正常跳变
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;               //本行右边线用上一行的数值
    }
    else                                                                          //右边线正常
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                            //记录下来啦
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                 //记录本行是否找到边线，即边线类型
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

    //*************************添加右******************************** */
    if(ImageDeal[Ysite].IsRightFind =='W')
    {
      for (Xsite = (ImageDeal[Ysite].RightBorder - 1);
            Xsite >= (ImageDeal[Ysite].LeftBorder + 1);
            Xsite--)    
      {
        if ((*(PicTemp + Xsite) != 0) && (*(PicTemp + Xsite + 1) == 0))
        {
          ImageDeal[Ysite].RightBorder = Xsite;                                 //如果上一行左边线的右边有黑白跳变则为绝对边线直接取出
          ImageDeal[Ysite].IsRightFind = 'T';
          break;
        }
      }
    }

    //*************************添加左******************************** */
    if(ImageDeal[Ysite].IsLeftFind =='W')
    {
      for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);
            Xsite <= (ImageDeal[Ysite].RightBorder - 1);
            Xsite++)    
      {
        if ((*(PicTemp + Xsite) != 0) && (*(PicTemp + Xsite - 1) == 0))
        {
          ImageDeal[Ysite].LeftBorder = Xsite;                                 //如果上一行左边线的右边有黑白跳变则为绝对边线直接取出
          ImageDeal[Ysite].IsLeftFind = 'T';
          break;
        }
      }
    }


    

    //重新确定那些大跳变的边缘
    if (( ImageDeal[Ysite].IsLeftFind == 'H'||ImageDeal[Ysite].IsRightFind == 'H'))
    {
      if (ImageDeal[Ysite].IsLeftFind == 'H')                                   //如果左边线大跳变
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);
             Xsite <= (ImageDeal[Ysite].RightBorder - 1);
             Xsite++)                                                           //左右边线之间重新扫描
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0))
          {
            ImageDeal[Ysite].LeftBorder =Xsite;                                 //如果上一行左边线的右边有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
          else if (*(PicTemp + Xsite) != 0)                                   //一旦出现白点则直接跳出
            break;
          else if (Xsite ==(ImageDeal[Ysite].RightBorder - 1))
          {
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
        }
      if ((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <=7)                              //图像宽度限定
      {
        ImageStatus.OFFLine = Ysite + 1;  //如果这行比7小了后面直接不要了
        break;
      }
      if (ImageDeal[Ysite].IsRightFind == 'H')
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);
             Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--)
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0))
          {
            ImageDeal[Ysite].RightBorder =
                Xsite;                    //如果右边线的左边还有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
          else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1))
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
        }
    }

 /***********重新确定无边行************/
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;


    if(  ImageStatus.Road_type != Ramp)
    {
    if (    ImageDeal[Ysite].IsRightFind == 'W'
          &&Ysite > 10
          &&Ysite < 50
          )                     //最早出现的无边行
    {
      if (Get_R_line == 'F')    //这一帧图像没有跑过这个找基准线的代码段才运行
      {
        Get_R_line = 'T';       //找了  一帧图像只跑一次 置为T
        ytemp_W_R = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsRightFind =='T')  //往无边行下面搜索  一般都是有边的
            R_found_point++;
        }
        if (R_found_point >8)                      //找到基准斜率边  做延长线重新确定无边   当有边的点数大于8
        {
          D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder - ImageDeal[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));
                                                  //求下面这些点连起来的斜率
                                                  //好给无边行做延长线左个基准
          if (D_R > 0) {
            R_Found_T ='T';              //如果斜率大于0  那么找到了这个基准行  因为梯形畸变
                                                  //所以一般情况都是斜率大于0  小于0的情况也不用延长 没必要
          } else {
            R_Found_T = 'F';                      //没有找到这个基准行
            if (D_R < 0)
              ExtenRFlag = 'F';                   //这个标志位用于十字角点补线  防止图像误补用的
          }
        }
      }
      if (R_Found_T == 'T')
        ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  //如果找到了 那么以基准行做延长线

      LimitL(ImageDeal[Ysite].RightBorder);  //限幅
      LimitH(ImageDeal[Ysite].RightBorder);  //限幅
    }

    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50)    //下面同理  左边界
    {
      if (Get_L_line == 'F') {
        Get_L_line = 'T';
        ytemp_W_L = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsLeftFind == 'T')
            L_found_point++;
        }
        if (L_found_point > 8)              //找到基准斜率边  做延长线重新确定无边
        {
          D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -ImageDeal[Ysite + L_found_point].LeftBorder)) /((float)(L_found_point - 3));
          if (D_L > 0) {
            L_Found_T = 'T';

          } else {
            L_Found_T = 'F';
            if (D_L < 0)
              ExtenLFlag = 'F';
          }
        }
      }

      if (L_Found_T == 'T')
        ImageDeal[Ysite].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);

      LimitL(ImageDeal[Ysite].LeftBorder);  //限幅
      LimitH(ImageDeal[Ysite].LeftBorder);  //限幅
      end_1 = 1;
    }
}
    if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W')
         {
             ImageStatus.WhiteLine++;  //要是左右都无边，丢边数+1
         }
        if (ImageDeal[Ysite].IsLeftFind == 'W'&&Ysite<55)
        {
             ImageStatus.Left_Line++;
        }
        if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite<55)
        {
             ImageStatus.Right_Line++;
        }

      LimitL(ImageDeal[Ysite].LeftBorder);   //限幅
      LimitH(ImageDeal[Ysite].LeftBorder);   //限幅
      LimitL(ImageDeal[Ysite].RightBorder);  //限幅
      LimitH(ImageDeal[Ysite].RightBorder);  //限幅

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    if (ImageDeal[Ysite].Wide <= 7)         //重新确定可视距离
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }

    else if (  ImageDeal[Ysite].RightBorder <= 10||ImageDeal[Ysite].LeftBorder >= 70) 
            {
              ImageStatus.OFFLine = Ysite + 1;
              break;
            }                                        //当图像宽度小于0或者左右边达到一定的限制时，则终止巡边
  }


  return;
}

void Search_Bottom_Line_OTSU(int imageInput[CAMERA_H][CAMERA_W], uint8 Row, uint8 Col, uint8 Bottonline)
{

    //寻找左边边界
    for (int Xsite = Col / 2-2; Xsite > 1; Xsite--)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            ImageDeal[Bottonline].LeftBoundary = Xsite;//获取底边左边线
            break;
        }
    }
    for (int Xsite = Col / 2+2; Xsite < CAMERA_W-1; Xsite++)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            ImageDeal[Bottonline].RightBoundary = Xsite;//获取底边右边线
            break;
        }
    }

}

void Search_Left_and_Right_Lines(int imageInput[CAMERA_H][CAMERA_W], int Row, int Col, int Bottonline)
{
    //定义小人的当前行走状态位置为 上 左 下 右 一次要求 上：左边为黑色 左：上边为褐色 下：右边为色  右：下面有黑色
/*  前进方向定义：
                *   0
                * 3   1
                *   2
*/
/*寻左线坐标规则*/
    int Left_Rule[2][8] = {
                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )
                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}
    };
    /*寻右线坐标规则*/
    int Right_Rule[2][8] = {
                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},
                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}
    };
      int num=0;
    uint8 Left_Ysite = Bottonline;
    uint8 Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
    uint8 Left_Rirection = 0;//左边方向
    uint8 Pixel_Left_Ysite = Bottonline;
    uint8 Pixel_Left_Xsite = 0;

    uint8 Right_Ysite = Bottonline;
    uint8 Right_Xsite = ImageDeal[Bottonline].RightBoundary;
    uint8 Right_Rirection = 0;//右边方向
    uint8 Pixel_Right_Ysite = Bottonline;
    uint8 Pixel_Right_Xsite = 0;
    uint8 Ysite = Bottonline;
    ImageStatus.OFFLineBoundary = 5;
    while (1)
    {
            num++;
            if(num>400)
            {
                 ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
        {
            if (Ysite < ImageStatus.OFFLineBoundary)
            {
                ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
            else
            {
                Ysite--;
            }
        }
        /*********左边巡线*******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//前方是黑色
            {
                //顺时针旋转90
                if (Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else//前方是白色
            {
                /*计算左前方坐标*/
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0){
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                        ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    }
                }
                else//左前方为白色
                {
                    // 方向发生改变 Left_Rirection  逆时针90度
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0 )
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;
                    else
                        Left_Rirection--;
                }

            }
        }
        /*********右边巡线*******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//前方是黑色
            {
                //逆时针旋转90
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else//前方是白色
            {
                /*计算右前方坐标*/
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Right_Rirection
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79 )
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else//左前方为白色
                {
                    // 方向发生改变 Right_Rirection  逆时针90度
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                    if (Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }

            }
        }

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)//Ysite<80是为了放在底部是斑马线扫描结束  3 && Ysite < 30
        {
            ImageStatus.OFFLineBoundary = Ysite;
            break;
        }

    }
}

void Search_Border_OTSU(int imageInput[CAMERA_H][CAMERA_W], uint8 Row, uint8 Col, uint8 Bottonline)
{
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    //ImageStatus.OFFLine = 1;
    /*封上下边界处理*/
    for (int Xsite = 0; Xsite < CAMERA_W; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /*封左右边界处理*/
    for (int Ysite = 0; Ysite < CAMERA_H; Ysite++)
    {
            ImageDeal[Ysite].LeftBoundary_First = 0;
            ImageDeal[Ysite].RightBoundary_First = 79;

            imageInput[Ysite][0] = 0;
            imageInput[Ysite][CAMERA_W - 1] = 0;
    }
    /********获取底部边线*********/
    Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
    /********获取左右边线*********/
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);


    for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary < 3)
        {
            ImageStatus.WhiteLine_L++;
        }
        if (ImageDeal[Ysite].RightBoundary > CAMERA_W - 3)
        {
            ImageStatus.WhiteLine_R++;
        }
    }
}



static void DrawExtensionLine(void)        //绘制延长线并重新确定中线 ，把补线补成斜线
{
  TFSite = 55;
  FTSite = 0;
  DetL = 0.0f;
  DetR = 0.0f;

  if ((ImageStatus.Road_type != Ramp)
//        &&ImageStatus.pansancha_Lenth* OX==0
        &&ImageStatus.Road_type !=LeftCirque
        &&ImageStatus.Road_type !=RightCirque
        )                                  // g5.22  6.22调试注释  记得改回来
  {
    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
//    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == LeftCirque)
//      TFSite = 55;
    if (ExtenLFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)                       //从第五行开始网上扫扫到顶边下面两行   多段补线
                                          //不仅仅只有一段
       {
        // printf("延长%d\n",Ysite);
        PicTemp = img1[Ysite];           //存当前行
        if (ImageDeal[Ysite].IsLeftFind =='W')                          //如果本行左边界没扫到但扫到的是白色，说明本行没有左边界点
        {
          // printf("补线左\n");
          //**************************************************//**************************************************
          if (ImageDeal[Ysite + 1].LeftBorder >= 70)                    //如果左边界实在是太右边
          {
            // printf("左边届太靠右了\n");
            ImageStatus.OFFLine = Ysite + 1;
            break;                        //直接跳出（极端情况）
          }
          //************************************************//*************************************************

          while (Ysite >= (ImageStatus.OFFLine + 4))                    //此时还没扫到顶边
          {
            Ysite--;                      //继续往上扫
            if (  ImageDeal[Ysite].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 1].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 2].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 2].LeftBorder > 0
                &&ImageDeal[Ysite - 2].LeftBorder <70
                )                                                       //如果扫到本行出现了并且本行以上连续三行都有左边界点（左边界在空白上方）
	            {
	              // printf("找到斜\n");
	              FTSite = Ysite - 2;          //把本行上面的第二行存入FTsite
	              break;
	            }
	          }

	          if (FTSite > ImageStatus.OFFLine && FTSite != TFSite)
	          {
	            DetL =
	                ((float)(ImageDeal[FTSite].LeftBorder -
	                         ImageDeal[TFSite].LeftBorder)) /
	                ((float)(FTSite - TFSite));  //左边界的斜率：列的坐标差/行的坐标差
	            for (
	                ytemp = TFSite; ytemp >= FTSite; ytemp--)               //从第一次扫到的左边界的下面第二行的坐标开始往上扫直到空白上方的左边界的行坐标值
	            {
	              ImageDeal[ytemp].LeftBorder =
	                  (int)(DetL * ((float)(ytemp - TFSite))) +
	                  ImageDeal[TFSite]
	                      .LeftBorder;                                      //将这期间的空白处补线（补斜线），目的是方便图像处理
	            }
	          }
	        } else
	          TFSite = Ysite + 2;                                           //如果扫到了本行的左边界，该行存在这里面，（算斜率）
	      }

	    TFSite = 55;
	    FTSite = 0;
	    DetR = 0.0f;
	    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
	      TFSite = 55;
    // g5.22
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == RightCirque)
      TFSite = 55;
    if (ExtenRFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)               //从第五行开始网上扫扫到顶边下面两行
      {
        PicTemp = img1[Ysite];  //存当前行

        if (ImageDeal[Ysite].IsRightFind =='W')                       //如果本行右边界没扫到但扫到的是白色，说明本行没有右边界点，但是处于赛道内的
        {
          // printf("补线右\n");
          if (ImageDeal[Ysite + 1].RightBorder <= 10)                 //如果右边界实在是太左边
          {
            ImageStatus.OFFLine =Ysite + 1;                           //直接跳出，说明这种情况赛道就尼玛离谱
            break;
          }
          while (Ysite >= (ImageStatus.OFFLine + 4))                  //此时还没扫到顶边下面两行
          {
            Ysite--;
            if (  ImageDeal[Ysite].IsRightFind == 'T'
                &&ImageDeal[Ysite - 1].IsRightFind == 'T'
                &&ImageDeal[Ysite - 2].IsRightFind == 'T'
                &&ImageDeal[Ysite - 2].RightBorder < 70
                &&ImageDeal[Ysite - 2].RightBorder > 10
                )                                                      //如果扫到本行出现了并且本行以上连续三行都有左边界点（左边界在空白上方）
            {
	              FTSite = Ysite - 2;                                      // 把本行上面的第二行存入FTsite
	              break;
	            }
	          }

	          if (FTSite > ImageStatus.OFFLine && FTSite != TFSite)
	          {
	            DetR =((float)(ImageDeal[FTSite].RightBorder -ImageDeal[TFSite].RightBorder)) /((float)(FTSite - TFSite));         //右边界的斜率：列的坐标差/行的坐标差
	            for (ytemp = TFSite; ytemp >= FTSite;ytemp--)              //从第一次扫到的右边界的下面第二行的坐标开始往上扫直到空白上方的右边界的行坐标值
	            {
	              ImageDeal[ytemp].RightBorder =(int)(DetR * ((float)(ytemp - TFSite))) +ImageDeal[TFSite].RightBorder;          //将这期间的空白处补线（补斜线），目的是方便图像处理
	            }
	          }
	        } else
	          TFSite =Ysite +2;                                           //如果本行的右边界找到了，则把该行下面第二行坐标送个TFsite
      }
  }
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].Center =(ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /2;                                //扫描结束，把这一块经优化之后的中间值存入
    ImageDeal[Ysite].Wide =-ImageDeal[Ysite].LeftBorder +ImageDeal[Ysite].RightBorder;                                       //把优化之后的宽度存入
  }
}


static void RouteFilter(void) 
{
  for (Ysite = 57; Ysite >= (ImageStatus.OFFLine + 5);   //改为57，防止 Ysite+2 时发生ImageDeal[60]数组越界导致奔溃
       Ysite--)                                     //从开始位到停止位
  {
    if (   ImageDeal[Ysite].IsLeftFind == 'W'
         &&ImageDeal[Ysite].IsRightFind == 'W'
         &&Ysite <= 45
         &&ImageDeal[Ysite - 1].IsLeftFind == 'W'
         &&ImageDeal[Ysite - 1].IsRightFind =='W')  //当前行左右都无边，而且在前45行   滤波
    {
      ytemp = Ysite;
      while (ytemp >= (ImageStatus.OFFLine +5))     // 改改试试，-6效果好一些   原本+5
      {
        ytemp--;
        if (  ImageDeal[ytemp].IsLeftFind == 'T'
            &&ImageDeal[ytemp].IsRightFind == 'T')  //寻找两边都正常的，找到离本行最近的就不找了
        {
          DetR = (float)(ImageDeal[ytemp - 1].Center - ImageDeal[Ysite + 2].Center) /(float)(ytemp - 1 - Ysite - 2);          //算斜率
          int CenterTemp = ImageDeal[Ysite + 2].Center;
          int LineTemp = Ysite + 2;
          while (Ysite >= ytemp) {
            ImageDeal[Ysite].Center =(int)(CenterTemp +DetR * (float)(Ysite - LineTemp));                                     //用斜率补
            Ysite--;
          }
          break;
        }
      }
    }
    // ImageDeal[Ysite].Center =(ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) /3;                                  //求平均，应该会比较滑  本来是上下两点平均
    // 【重点】加强滤波，从3点平均改为5点滑动平均，减少抖动
    ImageDeal[Ysite].Center =(ImageDeal[Ysite - 2].Center + 2*ImageDeal[Ysite - 1].Center + 4*ImageDeal[Ysite].Center + 2*ImageDeal[Ysite + 1].Center + ImageDeal[Ysite + 2].Center) /10;
  }
  
}

void GetDet() 
{    
  float DetTemp = 0;
  int TowPoint = 0;
  float UnitAll = 0;

  //Speed_Control_Factor();

  /*固定圆环前瞻*/
  if (ImageStatus.image_element_rings_flag != 0) TowPoint =Circle[circle_count_flag];
//   else if(ImageStatus.Road_type ==Cross_ture)
//   {
//     TowPoint=29;
//   }
  else TowPoint = ImageStatus.TowPoint; //初始前瞻
  if (TowPoint < ImageStatus.OFFLine)   TowPoint = ImageStatus.OFFLine + 1;//前瞻限幅
  if (TowPoint >= 49)   TowPoint = 49;

  if ((TowPoint - 5) >= ImageStatus.OFFLine) //前瞻取设定前瞻还是可视距离  需要分情况讨论 正常前瞻（与截止行相差5行）
  {                                          
    for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) 
    {
      DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) 
    {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);

  } 
  else if (TowPoint > ImageStatus.OFFLine)  //正常前瞻与截止行不相差5行
  {
    for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) 
    {
      DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
         Ysite--) 
    {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
  } 
  else if (ImageStatus.OFFLine < 49)  //前瞻等于截至行
  {
    for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;Ysite--) 
    {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);

  } 
  else
  {
    if (ImageStatus.OFFLine >= car_config::kBadVisionCenterHoldOfflineThreshold) {
      DetTemp = car_config::kImageMiddleLine;
      if (!g_enable_element_process && ImageFlag.image_element_rings == 0) {
        ImageStatus.Road_type = Cross;
      }
    } else {
      DetTemp = ImageStatus.Det_True;                                                   // 可视距离很短但未全丢时，暂时保持上一帧偏差
    }
  }

  static bool det_history_valid = false;
  static float last_det_true = car_config::kImageMiddleLine;
  const bool det_bad_frame =
      ImageStatus.OFFLine >= 45 ||
      (ImageStatus.Left_Line >= car_config::kCurveCenterOppositeLossMax &&
       ImageStatus.Right_Line >= car_config::kCurveCenterOppositeLossMax);
  if (det_bad_frame) {
    det_history_valid = false;
  } else {
    if (det_history_valid) {
      const float max_det_step = 12.0f;
      if (DetTemp > last_det_true + max_det_step) {
        DetTemp = last_det_true + max_det_step;
      } else if (DetTemp < last_det_true - max_det_step) {
        DetTemp = last_det_true - max_det_step;
      }
    }
    last_det_true = DetTemp;
    det_history_valid = true;
  }

  ImageStatus.Det_True = DetTemp;                                                      //此时的解算出来的平均图像偏差

  ImageStatus.TowPoint_True = TowPoint;                                              //此时的前瞻

  // //前瞻限幅
  // if (TowPoint < ImageParameter.OFFLine)
  //     TowPoint = ImageParameter.OFFLine + 1;
  // if (TowPoint >= 49) TowPoint = 49;

  // if ((TowPoint - 5) >= ImageParameter.OFFLine) 
  // { //前瞻取设定前瞻还是可视距离  需要分情况讨论
  //     for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) 
  //     {
  //         DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
  //         UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
  //     }
  //     for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) 
  //     {
  //         DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
  //         UnitAll += Weighting[-TowPoint + Ysite - 1];
  //     }
  //     DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);
  // }
  // else if (TowPoint > ImageParameter.OFFLine) 
  // {
  //     for (Ysite = ImageParameter.OFFLine; Ysite < TowPoint; Ysite++) 
  //     {
  //         DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
  //         UnitAll += Weighting[TowPoint - Ysite - 1];
  //     }
  //     for (Ysite = (TowPoint + TowPoint - ImageParameter.OFFLine); Ysite > TowPoint;
  //         Ysite--) 
  //     {
  //         DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
  //         UnitAll += Weighting[-TowPoint + Ysite - 1];
  //     }
  //     DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
  // }
  // else if (ImageParameter.OFFLine < 49) 
  // {
  //     for (Ysite = (ImageParameter.OFFLine + 3); Ysite > ImageParameter.OFFLine;
  //         Ysite--) 
  //     {
  //         DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
  //         UnitAll += Weighting[-TowPoint + Ysite - 1];
  //     }
  //     DetTemp = (ImageDeal[ImageParameter.OFFLine].Center + DetTemp) / (UnitAll + 1);
  // }
  // else
  //     DetTemp = (float)ImageParameter.Det_True;//OFFLine>50

  // ImageParameter.Det_True = DetTemp;
  // ImageStatus.TowPoint_True = TowPoint;  
}

//void GetDet() 
// {
//   float DetTemp = 0;
//   int TowPoint = 0;
//   float UnitAll = 0;
  
//   if (ImageStatus.Road_type == RightCirque)
//   {
//     TowPoint = 30;
//   }
//   else if(ImageStatus.Road_type ==Cross_ture)
//   {
//     TowPoint=29;
//   }
//   TowPoint=18;
//   //if (TowPoint < ImageStatus.OFFLine)   TowPoint = ImageStatus.OFFLine + 1;//前瞻限幅
//   //if (TowPoint >= 49)   TowPoint = 49;

//   if ((TowPoint - 5) >= ImageStatus.OFFLine) //前瞻取设定前瞻还是可视距离  需要分情况讨论
//   {                                          
//     for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) 
//     {
//       DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
//       UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
//     }
//     for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) 
//     {
//       DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
//       UnitAll += Weighting[-TowPoint + Ysite - 1];
//     }
//     DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);

//   } 
//   else if (TowPoint > ImageStatus.OFFLine) 
//   {
//     for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) 
//     {
//       DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
//       UnitAll += Weighting[TowPoint - Ysite - 1];
//     }
//     for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
//          Ysite--) 
//     {
//       DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
//       UnitAll += Weighting[-TowPoint + Ysite - 1];
//     }
//     DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
//   } 
//   else if (ImageStatus.OFFLine < 49) 
//   {
//     for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;Ysite--) 
//     {
//       DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
//       UnitAll += Weighting[-TowPoint + Ysite - 1];
//     }
//     DetTemp = (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);

//   } 
//   else
//   {
//     DetTemp =ImageStatus.Det_True;                                                     //如果是出现OFFLine>50情况，保持上一次的偏差值
//   }

//   ImageStatus.Det_True = DetTemp;                                                      //此时的解算出来的平均图像偏差

//   ImageStatus.TowPoint_True = TowPoint;                                                //此时的前瞻
// }

void Element_Judgment_Left_Rings()
{
    // ==================== 原左圆环判断逻辑（保留注释，便于回退对比） ====================
    // if (       ImageStatus.Right_Line > 5
    //         || ImageStatus.Left_Line < 9
    //         || ImageStatus.OFFLine > 10
    //         || ImageDeal[58].RightBorder > 75
    //         || ImageDeal[57].RightBorder > 75
    //         || ImageDeal[56].RightBorder > 75
    //         || ImageDeal[55].RightBorder > 75
    //         || ImageDeal[54].RightBorder > 75
    //         || ImageDeal[52].IsLeftFind == 'W'
    //         || ImageDeal[53].IsLeftFind == 'W'
    //         || ImageDeal[54].IsLeftFind == 'W'
    //         || ImageDeal[55].IsLeftFind == 'W'
    //         || ImageDeal[56].IsLeftFind == 'W'
    //         || ImageDeal[57].IsLeftFind == 'W'
    //         || ImageDeal[58].IsLeftFind == 'W'
    //     )   return;
    //
    // int ring_ysite = 25;
    // Left_RingsFlag_Point1_Ysite = 0;
    // Left_RingsFlag_Point2_Ysite = 0;
    // for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    // {
    //     if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 6)
    //     {
    //         Left_RingsFlag_Point1_Ysite = Ysite;
    //         break;
    //     }
    // }
    // for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    // {
    //     if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 6)
    //     {
    //         Left_RingsFlag_Point2_Ysite = Ysite;
    //         break;
    //     }
    // }
    // for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    // {
    //     if (   (ImageDeal[Ysite + 6].LeftBorder) < ImageDeal[Ysite + 3].LeftBorder
    //         && (ImageDeal[Ysite + 5].LeftBorder) < ImageDeal[Ysite + 3].LeftBorder
    //         && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder
    //         && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder)
    //     {
    //         Ring_Help_Flag = 1;
    //         break;
    //     }
    // }
    // if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 2
    //         && Ring_Help_Flag == 1
    //         && ImageFlag.image_element_rings_flag == 0
    //         && ImageStatus.Left_Line > 12
    //         && Left_RingsFlag_Point1_Ysite != 0
    //         && Left_RingsFlag_Point2_Ysite < 35
    //         && Left_RingsFlag_Point2_Ysite - Left_RingsFlag_Point1_Ysite < 10)
    // {
    //     ImageFlag.image_element_rings = 1;
    //     ImageFlag.image_element_rings_flag = 1;
    //     ImageFlag.ring_big_small = 1;
    //     ImageStatus.Road_type = LeftCirque;
    // }
    // Ring_Help_Flag = 0;
    // ==================== 原逻辑结束 ====================

    if (ImageFlag.image_element_rings_flag != 0)
    {
        left_ring_confirm_frames = 0;
        return;
    }

    const bool base_blocked = (
               ImageStatus.Right_Line > 5
            || ImageStatus.Left_Line < 9
            || ImageStatus.OFFLine > 10
            || ImageDeal[58].RightBorder > 75
            || ImageDeal[57].RightBorder > 75
            || ImageDeal[56].RightBorder > 75
            || ImageDeal[55].RightBorder > 75
            || ImageDeal[54].RightBorder > 75
            || ImageDeal[52].IsLeftFind == 'W'
            || ImageDeal[53].IsLeftFind == 'W'
            || ImageDeal[54].IsLeftFind == 'W'
            || ImageDeal[55].IsLeftFind == 'W'
            || ImageDeal[56].IsLeftFind == 'W'
            || ImageDeal[57].IsLeftFind == 'W'
            || ImageDeal[58].IsLeftFind == 'W');

    if (base_blocked)
    {
        left_ring_confirm_frames = 0;
        return;
    }

    int ring_ysite = 25;
    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 6)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 6)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        if (   (ImageDeal[Ysite + 6].LeftBorder) < ImageDeal[Ysite + 3].LeftBorder
            && (ImageDeal[Ysite + 5].LeftBorder) < ImageDeal[Ysite + 3].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder)
        {
            Ring_Help_Flag = 1;
            break;
        }
    }

    const bool right_side_suspicious = (
           ImageStatus.Right_Line >= 10
        || ImageDeal[58].LeftBorder < 8
        || ImageDeal[57].LeftBorder < 8
        || ImageDeal[56].LeftBorder < 8);

    const bool left_candidate = (
           Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 2
        && Ring_Help_Flag == 1
        && ImageStatus.Left_Line > 12
        && Left_RingsFlag_Point1_Ysite != 0
        && Left_RingsFlag_Point2_Ysite < 35
        && Left_RingsFlag_Point2_Ysite - Left_RingsFlag_Point1_Ysite < 10);

    if (left_candidate && !right_side_suspicious)
    {
        if (left_ring_confirm_frames < 3)
            left_ring_confirm_frames++;
    }
    else
    {
        left_ring_confirm_frames = 0;
    }

    if (left_ring_confirm_frames >= 2)
    {
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = LeftCirque;
        left_ring_confirm_frames = 0;
        right_ring_confirm_frames = 0;
    }

    Ring_Help_Flag = 0;

    // 新增优化说明：
    // 1. 保留原有几何判据，不改变原始特征提取方式。
    // 2. 新增连续帧确认，避免单帧噪声、急弯抖动导致误入左圆环。
    // 3. 新增右侧排异条件，当对侧也明显异常时，不立即判左圆环。
    // 4. 这样做的目标是“降低误判优先”，如果后续发现真环触发偏慢，可把确认帧数从2改为1。
}

void Element_Judgment_Right_Rings()
{
    // ==================== 原右圆环判断逻辑（保留注释，便于回退对比） ====================
    // if (   ImageStatus.Left_Line > 5
    //     || ImageStatus.Right_Line < 10
    //     || ImageStatus.OFFLine > 10
    //     || ImageDeal[58].LeftBorder < 6
    //     || ImageDeal[57].LeftBorder < 6
    //     || ImageDeal[56].LeftBorder < 6
    //     || ImageDeal[55].LeftBorder < 6
    //     || ImageDeal[54].LeftBorder < 6
    //     || ImageDeal[52].IsRightFind == 'W'
    //     || ImageDeal[53].IsRightFind == 'W'
    //     || ImageDeal[54].IsRightFind == 'W'
    //     || ImageDeal[55].IsRightFind == 'W'
    //     || ImageDeal[56].IsRightFind == 'W'
    //     || ImageDeal[57].IsRightFind == 'W'
    //     || ImageDeal[58].IsRightFind == 'W')
    // {
    //     return;
    // }
    // int ring_ysite = 5;
    // Right_RingsFlag_Point1_Ysite = 0;
    // Right_RingsFlag_Point2_Ysite = 0;
    // for (int Ysite = 55; Ysite > ring_ysite; Ysite--)
    // {
    //     if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
    //     {
    //         Right_RingsFlag_Point1_Ysite = Ysite;
    //         break;
    //     }
    // }
    // for (int Ysite = 55; Ysite > ring_ysite; Ysite--)
    // {
    //     if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
    //     {
    //         Right_RingsFlag_Point2_Ysite = Ysite;
    //         break;
    //     }
    // }
    // for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    // {
    //     if (   ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder
    //         && ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder
    //         && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder
    //         && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder)
    //     {
    //         Ring_Help_Flag = 1;
    //         break;
    //     }
    // }
    // if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 1
    //     && Ring_Help_Flag == 1
    //     && ImageFlag.image_element_rings_flag == 0
    //     && ImageStatus.Right_Line > 13
    //     && Right_RingsFlag_Point1_Ysite != 0
    //     && Right_RingsFlag_Point2_Ysite > 25)
    // {
    //     ImageFlag.image_element_rings = 2;
    //     ImageFlag.image_element_rings_flag = 1;
    //     ImageFlag.ring_big_small = 1;
    //     ImageStatus.Road_type = RightCirque;
    // }
    // Ring_Help_Flag = 0;
    // ==================== 原逻辑结束 ====================

    if (ImageFlag.image_element_rings_flag != 0)
    {
        right_ring_confirm_frames = 0;
        return;
    }

    const bool base_blocked = (
           ImageStatus.Left_Line > 5
        || ImageStatus.Right_Line < 10
        || ImageStatus.OFFLine > 10
        || ImageDeal[58].LeftBorder < 6
        || ImageDeal[57].LeftBorder < 6
        || ImageDeal[56].LeftBorder < 6
        || ImageDeal[55].LeftBorder < 6
        || ImageDeal[54].LeftBorder < 6
        || ImageDeal[52].IsRightFind == 'W'
        || ImageDeal[53].IsRightFind == 'W'
        || ImageDeal[54].IsRightFind == 'W'
        || ImageDeal[55].IsRightFind == 'W'
        || ImageDeal[56].IsRightFind == 'W'
        || ImageDeal[57].IsRightFind == 'W'
        || ImageDeal[58].IsRightFind == 'W');

    if (base_blocked)
    {
        right_ring_confirm_frames = 0;
        return;
    }

    int ring_ysite = 5;
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 55; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 55; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        if (   ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder)
        {
            Ring_Help_Flag = 1;
            break;
        }
    }

    const bool left_side_suspicious = (
           ImageStatus.Left_Line >= 10
        || ImageDeal[58].RightBorder > 71
        || ImageDeal[57].RightBorder > 71
        || ImageDeal[56].RightBorder > 71);

    const bool right_candidate = (
           Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 1
        && Ring_Help_Flag == 1
        && ImageStatus.Right_Line > 13
        && Right_RingsFlag_Point1_Ysite != 0
        && Right_RingsFlag_Point2_Ysite > 25);

    if (right_candidate && !left_side_suspicious)
    {
        if (right_ring_confirm_frames < 3)
            right_ring_confirm_frames++;
    }
    else
    {
        right_ring_confirm_frames = 0;
    }

    if (right_ring_confirm_frames >= 2)
    {
        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = RightCirque;
        right_ring_confirm_frames = 0;
        left_ring_confirm_frames = 0;
    }

    Ring_Help_Flag = 0;

    // 新增优化说明：
    // 1. 原右圆环识别逻辑完整保留为注释，方便你随时做A/B对比。
    // 2. 新逻辑加入连续帧确认，避免右侧边界瞬时跳变导致误入右圆环。
    // 3. 新增左侧排异条件，用于过滤普通急弯或双边同时异常的情况。
    // 4. 当前策略偏稳健，如果后续实车发现真实右环识别偏慢，可适当降低确认门槛。
}

//左圆环判断
void Element_Handle_Left_Rings()
{   

    // // 左圆环处理，进环后替换原舵机中线逻辑，改为固定差速
    // if (ImageFlag.image_element_rings_flag == 6 || ImageFlag.image_element_rings_flag ==7)
    // {
    // // 左圆环：左轮低速，右轮高速，固定差速
    //   Speed_Goal = 40; // 圆环内基础速度大幅降低
    //   Speed_Goal_l = 25;
    //   Speed_Goal_r = 55;
    // // 跳过原seepd_dif()的自动差速，强制锁定轮速
    //   return;
    // }

    /***************************************判断**************************************/
    int num = 0;
    for (int Ysite = 55; Ysite > 40; Ysite--)
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W')
            num++;
        if(    ImageDeal[Ysite+3].IsLeftFind == 'W' && ImageDeal[Ysite+2].IsLeftFind == 'W'
            && ImageDeal[Ysite+1].IsLeftFind == 'W' && ImageDeal[Ysite].IsLeftFind == 'T')
            break;
    }
//    tft180_show_int(60,125,num,3);
//    int ring_ysite = 30;
//    for (int Ysite = 5; Ysite < ring_ysite; Ysite++)
//    {
//        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
//        {
//            Right_RingsFlag_Point1_Ysite = Ysite;
//            break;
//        }
//    }
//    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
//    {
//        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
//        {
//            Right_RingsFlag_Point2_Ysite = Ysite;
//            break;
//        }
//    }
        //准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num>10 )//&& ImageStatus.Left_Line >25
    {
        ImageFlag.image_element_rings_flag = 2;
        //wireless_uart_send_byte(2);
    }

//    if( ImageFlag.image_element_rings_flag == 2 )
//    {
//        if( SaiDaoKuanDu() > 62)
//        ImageFlag.image_element_rings_flag = 3;
//
//    }


    if (ImageFlag.image_element_rings_flag == 2 && num<9)//&& ImageStatus.Left_Line <15
    {
        ImageFlag.image_element_rings_flag = 5;
        //wireless_uart_send_byte(5);
    }
//    if(ImageFlag.image_element_rings_flag == 3 && ImageStatus.Left_Line >20)
//    {
//        ImageFlag.image_element_rings_flag = 5;
//    }

        //进环
    if(ImageFlag.image_element_rings_flag == 5 && /*num>15)*/ImageStatus.Right_Line>10)
    {
        ImageFlag.image_element_rings_flag = 6;
     //   ImageStatus.Road_type = LeftCirque;
        //wireless_uart_send_byte(6);
    }
        //进环小圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Right_Line<3)
    {
        //Stop = 1;
        ImageFlag.image_element_rings_flag = 7;
        //wireless_uart_send_byte(8);
    }
        //环内 大圆环判断
    if ( ImageFlag.image_element_rings_flag == 7)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLine+3 ; Ysite--)
        {
            if (    ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder
                     //ImageStatus.WhiteLine  > 8
//                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 4].RightBorder
//                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 4].RightBorder
               )
            {
                Point_Xsite = ImageDeal[Ysite].RightBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 15 && ImageStatus.Right_Line > 10)
        {
            ImageFlag.image_element_rings_flag = 8;
           // wireless_uart_send_byte(8);
            //Stop = 1;
        }
    }
//        //环内 小圆环判断
//    if (ImageFlag.image_element_rings_flag == 7 && ImageFlag.ring_big_small == 2)
//    {
//        Point_Ysite = 0;
//        Point_Xsite = 0;
//        for (int Ysite = 50; Ysite > ImageStatus.OFFLineBoundary + 3; Ysite--)
//        {
//            if (    ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite + 2].RightBoundary
//                 && ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite - 2].RightBoundary
//               )
//            {
//                Point_Xsite = ImageDeal[Ysite].RightBoundary;
//                Point_Ysite = Ysite;
//                break;
//            }
//        }
//        if (Point_Ysite > 20)
//          ImageFlag.image_element_rings_flag = 8;
//    }
    //出环后
        if (ImageFlag.image_element_rings_flag == 8)
        {
             if (
                     //Straight_Judge(2, ImageStatus.OFFLine+15, 50) < 1
                 ImageStatus.Right_Line < 10
                 &&  ImageStatus.OFFLine < 10

                 )    //右边为直线且截止行（前瞻值）很小
               {ImageFlag.image_element_rings_flag = 9;
                //wireless_uart_send_byte(9);
               }
        }

        //结束圆环进程
        if (ImageFlag.image_element_rings_flag == 9)
        {
            int num=0;
            for (int Ysite = 40; Ysite > 10; Ysite--)
            {
                if(ImageDeal[Ysite].IsLeftFind == 'W' )
                    num++;
            }
            if(num < 5)
            {
//                ImageStatus.Road_type = 0;   //出环处理完道路类型清0
                ImageFlag.image_element_rings_flag = 0;
                ImageFlag.image_element_rings = 0;
                ImageFlag.ring_big_small = 0;
                ImageStatus.Road_type = Normol;
                //wireless_uart_send_byte(0);
                circle_num ++;
            }
    }



    /***************************************处理**************************************/
        //准备进环  半宽处理
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
        //进环  补线
    if  ( ImageFlag.image_element_rings_flag == 5
        ||ImageFlag.image_element_rings_flag == 6
        )
    {
        int  flag_Xsite_1=0;
        int flag_Ysite_1=0;
        float Slope_Rings=0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)//下面弧点
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(  img1[Ysite][Xsite] == 1 && img1[Ysite][Xsite + 1] == 0)
                 {
                   flag_Ysite_1 = Ysite;
                   flag_Xsite_1 = Xsite;
                   Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                   break;
                 }
            }
            if(flag_Ysite_1 != 0)
            {
                break;
            }
        }
        if(flag_Ysite_1 == 0)
        {
            for(Ysite=ImageStatus.OFFLine+1;Ysite<30;Ysite++)
            {
                if(ImageDeal[Ysite].IsLeftFind=='T'&&ImageDeal[Ysite+1].IsLeftFind=='T'&&ImageDeal[Ysite+2].IsLeftFind=='W'
                    &&abs(ImageDeal[Ysite].LeftBorder-ImageDeal[Ysite+2].LeftBorder)>10
                  )
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=ImageDeal[flag_Ysite_1].LeftBorder;
                    ImageStatus.OFFLine=Ysite;
                    Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }

            }
        }
        //补线
        if(flag_Ysite_1 != 0)
        {
            for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
            {
                ImageDeal[Ysite].RightBorder=flag_Xsite_1+Slope_Rings*(Ysite-flag_Ysite_1);
                //if(ImageFlag.ring_big_small==1)//大圆环不减半宽
                    ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
                //else//小圆环减半宽
                //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                if(ImageDeal[Ysite].Center<4)
                    ImageDeal[Ysite].Center = 4;
            }
            ImageDeal[flag_Ysite_1].RightBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A点上方进行扫线
            {
                for(Xsite=ImageDeal[Ysite+1].RightBorder-10;Xsite<ImageDeal[Ysite+1].RightBorder+2;Xsite++)
                {
                    if(img1[Ysite][Xsite]==1 && img1[Ysite][Xsite+1]==0)
                    {
                        ImageDeal[Ysite].RightBorder=Xsite;
                        //if(ImageFlag.ring_big_small==1)//大圆环不减半宽
                            ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
                        //else//小圆环减半宽
                        //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                        if(ImageDeal[Ysite].Center<4)
                            ImageDeal[Ysite].Center = 4;
                        ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                        break;
                    }
                }

                if(ImageDeal[Ysite].Wide>8 &&ImageDeal[Ysite].RightBorder< ImageDeal[Ysite+2].RightBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }
        //环内 小环弯道减半宽 大环不减
    if (ImageFlag.image_element_rings_flag == 7)
    {

    }
        //大圆环出环 补线
    if (ImageFlag.image_element_rings_flag == 8)    //大圆环
    {
        Repair_Point_Xsite = 30;
        Repair_Point_Ysite = 7;
        for (int Ysite = 40; Ysite > 5; Ysite--)
        {
            if (img1[Ysite][15] == 1 && img1[Ysite-1][15] == 0)//28
            {
                Repair_Point_Xsite = 15;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
                break;
            }
        }

        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
        {
            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
//        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
//        {
////            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
////            ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
//            ImageDeal[Ysite].RightBorder =ImageDeal[Ysite].LeftBorder+Half_Road_Wide[Ysite]+8;
//            if(ImageDeal[Ysite].RightBorder>77) ImageDeal[Ysite].RightBorder=77;
//            ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
//        }
    }
//        //小圆环出环 补线
//    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 2)    //小圆环
//    {
//        Repair_Point_Xsite = 0;
//        Repair_Point_Ysite = 0;
//        for (int Ysite = 55; Ysite > 5; Ysite--)
//        {
//            if (Pixle[Ysite][15] == 1 && Pixle[Ysite-1][15] == 0)
//            {
//                Repair_Point_Xsite = 15;
//                Repair_Point_Ysite = Ysite-1;
//                ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
//                break;
//            }
//        }
//        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
//        {
//            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
//            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
//        }
//    }
        //已出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
}
//--------------------------------------------------------------
//  @name           Element_Handle_Right_Rings()
//  @brief          整个图像处理的子函数，用来处理右圆环类型.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Right_Rings();
//-------------------------------------------------------------
void Element_Handle_Right_Rings()
{   

    // // 右圆环同理
    // if (ImageFlag.image_element_rings_flag == 6 || ImageFlag.image_element_rings_flag ==7)
    // {
    //   Speed_Goal = 40;
    //   Speed_Goal_l = 55;
    //   Speed_Goal_r = 25;
    //   return;
    // }

    /****************判断*****************/
    int num =0 ;
    for (int Ysite = 55; Ysite > 20; Ysite--)
    {
        if(ImageDeal[Ysite].IsRightFind == 'W')
        {
            num++;
            // szr=num;
        }
        if(    ImageDeal[Ysite+3].IsRightFind == 'W' && ImageDeal[Ysite+2].IsRightFind == 'W'
            && ImageDeal[Ysite+1].IsRightFind == 'W' && ImageDeal[Ysite].IsRightFind == 'T' )
            break;
    }
        //准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num>10 )//&& ImageStatus.Right_Line >25
    {
        ImageFlag.image_element_rings_flag = 2;
    }



//    if( ImageFlag.image_element_rings_flag == 2 )
//    {
//        if( SaiDaoKuanDu() > 62)
//        ImageFlag.image_element_rings_flag = 3;
//
//    }




    if (ImageFlag.image_element_rings_flag == 2 && num<9 )//&& ImageStatus.Right_Line<15
    {
        ImageFlag.image_element_rings_flag = 5;
    }
//    if(ImageFlag.image_element_rings_flag = 3 && ImageStatus.Right_Line>25)//
//    {
//        ImageFlag.image_element_rings_flag = 5;
//    }
        //进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line>=20)
    {
        ImageFlag.image_element_rings_flag = 6;
       // ImageStatus.Road_type = RightCirque;
    }
        //进环小圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line<10)
    {
        ImageFlag.image_element_rings_flag = 7;
        //Stop=1;
    }
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--)
        {
            if (    //ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder
                    //&& ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder
                     ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder
                    && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder
                 // && ImageStatus.WhiteLine  > 8


              //   && ImageDeal[Ysite].LeftBorder > ImageDeal[Ysite - 4].LeftBorder
//                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder
//                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder
                )
            {
                        Point_Xsite = ImageDeal[Ysite].LeftBorder;
                        Point_Ysite = Ysite;
                        break;
            }
        }
        if (Point_Ysite > 35 && ImageStatus.Left_Line > 20)
        {
            ImageFlag.image_element_rings_flag = 8;
//            gpio_set_level(P33_10, 1);
        }
    }
    if (ImageFlag.image_element_rings_flag == 8)
    {
         if (   //Straight_Judge(1, ImageStatus.OFFLine+10, 45) < 1
                     ImageStatus.Left_Line < 20
                  && ImageStatus.OFFLine < 25

             )    //右边为直线且截止行（前瞻值）很小
            {ImageFlag.image_element_rings_flag = 9;}

    }
    if(ImageFlag.image_element_rings_flag == 9 )
    {
        int num=0;
        for (int Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsRightFind == 'W' )
            {
                num++;
            }
        }
        // szr=num;
        if(num < 10)
        {
           // ImageStatus.Road_type = 0;   //出环处理完道路类型清0
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;
//            Front_Ring_Continue_Count++;
            circle_num ++;
        }
    }
    /***************************************处理**************************************/
         //准备进环  半宽处理
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }

        //进环  补线
    if (   ImageFlag.image_element_rings_flag == 5
        || ImageFlag.image_element_rings_flag == 6
       )
    {
        int flag_Xsite_1=0;
        int  flag_Ysite_1=0;
        float Slope_Right_Rings = 0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(img1[Ysite][Xsite]==1 && img1[Ysite][Xsite+1]==0)
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=Xsite;
                    Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1!=0)
            {
              break;
            }
        }
        if(flag_Ysite_1==0)
        {
        for(Ysite=ImageStatus.OFFLine+5;Ysite<30;Ysite++)
        {
         if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite+1].IsRightFind=='T'&&ImageDeal[Ysite+2].IsRightFind=='W'
               &&abs(ImageDeal[Ysite].RightBorder-ImageDeal[Ysite+2].RightBorder)>10
         )
         {
             flag_Ysite_1=Ysite;
             flag_Xsite_1=ImageDeal[flag_Ysite_1].RightBorder;
             ImageStatus.OFFLine=Ysite;
             Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
             break;
         }

        }

        }
        //补线
        if(flag_Ysite_1!=0)
        {
            for(Ysite=flag_Ysite_1;Ysite<58;Ysite++)
            {
                ImageDeal[Ysite].LeftBorder=flag_Xsite_1+Slope_Right_Rings*(Ysite-flag_Ysite_1);
//                if(ImageFlag.ring_big_small==2)//小圆环加半宽
//                    ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
//                else//大圆环不加半宽
                    ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//板块
                if(ImageDeal[Ysite].Center>79)
                    ImageDeal[Ysite].Center=79;
            }
            ImageDeal[flag_Ysite_1].LeftBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A点上方进行扫线
            {
                for(Xsite=ImageDeal[Ysite+1].LeftBorder+8;Xsite>ImageDeal[Ysite+1].LeftBorder-4;Xsite--)
                {
                    if(img1[Ysite][Xsite]==1 && img1[Ysite][Xsite-1]==0)
                    {
                     ImageDeal[Ysite].LeftBorder=Xsite;
                     ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
//                     if(ImageFlag.ring_big_small==2)//小圆环加半宽
//                         ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
//                     else//大圆环不加半宽
                         ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//板块
                     if(ImageDeal[Ysite].Center>79)
                         ImageDeal[Ysite].Center=79;
                     if(ImageDeal[Ysite].Center<5)
                         ImageDeal[Ysite].Center=5;
                     break;
                    }
                }
                if(ImageDeal[Ysite].Wide>8 && ImageDeal[Ysite].LeftBorder>  ImageDeal[Ysite+2].LeftBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }
        //环内不处理
    if (ImageFlag.image_element_rings_flag == 7)
    {

    }
        //大圆环出环 补线
    if (ImageFlag.image_element_rings_flag == 8)  //大圆环
    {
        Repair_Point_Xsite = 30;
        Repair_Point_Ysite = 7;
        for (int Ysite = 40; Ysite > 8; Ysite--)
        {
            if (img1[Ysite][65] == 1 && img1[Ysite-1][65] == 0)
            {
                Repair_Point_Xsite = 65;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
                break;
            }
        }



        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
        {
            ImageDeal[Ysite].LeftBorder = ( Repair_Point_Xsite - ImageDeal[58].LeftBorder ) * (58 - Ysite) / (58 - Repair_Point_Ysite)  + ImageDeal[58].LeftBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    if (ImageFlag.image_element_rings_flag == 9)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
  }
}



void Element_Judgment_Zebra()//斑马线判断
{
    if(ImageFlag.Zebra_Flag) return;
    int  net = 0;

        for (int Ysite = 40; Ysite < 50; Ysite++)
        {
            net = 0;
            for (int Xsite =0; Xsite < 79; Xsite++)
            {
                if (img1[Ysite][Xsite] != img1[Ysite][Xsite + 1] && ImageStatus.OFFLine<10)
                {
                    net++;
                }
            }
            if (net > 8 && ImageFlag.Zebra_Flag == 0)
              {
                   ImageFlag.Zebra_Flag = 1;
                   Garage_Location_Flag++;
                   break;
              }
        }


}
void Element_Handle_Zebra()//斑马线处理
{
        if(Garage_Location_Flag < Garage_num)

        {
            if(ImageStatus.Barn_Lenth>Zebra_num1)   //第一次斑马线发车，当识别到编码器记录的路程大于设定的路程时，把车库标志位和编码器清零
            {
                ImageFlag.Zebra_Flag = 0;
                ImageStatus.Barn_Lenth=0;
                
            }
        }
        else if(Garage_Location_Flag >= Garage_num)
        {
            if(ImageStatus.Barn_Lenth > Zebra_num2)
             {
                ImageStatus.Road_type = Barn_in;
                ImageFlag.Zebra_Flag = 0;
                ImageStatus.Barn_Lenth=0;
             }
        }

}

//停车
void Stop_Test() 
{
  if (ImageStatus.Stop_lenth * OX > 50 && ImageFlag.image_element_rings == 0) 
  {
 
      //      if (ImageStatus.OFFLine >= 55)           //如果电磁很小并且可视距离基本没有就表示出界
      //           SystemData.Stop = 1;
      // else SystemData.Stop = 0;
      if (ImageStatus.OFFLine >= 55 )           //如果电磁很小并且可视距离基本没有就表示出界
      {      
            SystemData.Stop = 1;
            Speed_Goal_l = 0;
            Speed_Goal_r = 0;
      }
   }
 }

 //停车弱保护
 void Stop_Test2() {                            //弱保护
  if (  ImageStatus.OFFLine >= 55 && SystemData.Stop == 0
      &&SystemData.SpeedData.Length * OX > 150)
        SystemData.Stop = 2;

  if (      SystemData.Stop == 2
          &&ImageStatus.Stop_lenth * OX > 80
          &&ImageStatus.OFFLine >= 55)
    SystemData.Stop = 1;
  else if ( SystemData.Stop == 2
          &&ImageStatus.Stop_lenth * OX > 80
          &&ImageStatus.OFFLine < 55)
    SystemData.Stop = 0;
}

void Element_Handle() 
{
  if (ImageFlag.image_element_rings == 1 && ImageStatus.Road_type == LeftCirque)
      Element_Handle_Left_Rings();
  else if(ImageFlag.image_element_rings == 2 && ImageStatus.Road_type == RightCirque)
      Element_Handle_Right_Rings();
 
//      Element_Handle_Zebra(); //斑马线停车处理
}

//用于加速的直道检测
void Straightacc_Test(void) 
{
  int sum = 0;
  for (Ysite = 55; Ysite > ImageStatus.OFFLine + 1; Ysite--) 
  {
    sum += (ImageDeal[Ysite].Center - ImageStatus.MiddleLine) *(ImageDeal[Ysite].Center - ImageStatus.MiddleLine);
  }
  variance_acc = (float)sum / (54 - ImageStatus.OFFLine);
  //直线加速
  // if ( variance_acc < ImageStatus.variance_acc
  //   && ImageStatus.OFFLine <= 15
  //   && ImageStatus.Left_Line < 10
  //   && ImageStatus.Right_Line < 10 
  //   ) 
  // 直道判断：方差阈值更小，可视距离更短，门槛更高
  if ( variance_acc < 8  // 原阈值降低，更严格
    && ImageStatus.OFFLine <= 10 // 可视距离更近，更严格
    && ImageStatus.Left_Line < 5
    && ImageStatus.Right_Line < 5 
    )
    {
      ImageStatus.straight_acc = 1;
    } 
    else
    ImageStatus.straight_acc = 0;

    //判断直线
    // if ( variance_acc < ImageStatus.variance
    //   && ImageStatus.OFFLine <= 15
    //   && ImageStatus.Left_Line < 10
    //   && ImageStatus.Right_Line < 10 
    //   ) 
    // 直线判断同理提高门槛
    if ( variance_acc < car_config::kStraightVarianceAccThreshold
      && ImageStatus.OFFLine <= car_config::kStraightOfflineMax
      && ImageStatus.Left_Line < car_config::kStraightLeftLineMax
      && ImageStatus.Right_Line < car_config::kStraightRightLineMax 
      )
      {
        ImageStatus.straight_acc_flag=1;

        // 直道增大死区，防止抖动
        DEAD_BAND = car_config::kSteerDeadBandStraight;

      } 
      else
      {
        ImageStatus.straight_acc_flag=0;

        // 弯道恢复小死区，保证转向灵敏度
        DEAD_BAND = car_config::kSteerDeadBandCurve;

      }
      
      // 添加UI显示状态更新：当没有在特殊元素内时，实时反馈当前赛道是直道还是常规（弯道等）
      if (ImageFlag.image_element_rings == 0 && ImageStatus.Road_type != Barn_in) {
          if (ImageStatus.straight_acc_flag == 1) {
              ImageStatus.Road_type = Straight;
          } else {
              ImageStatus.Road_type = Normol; // 默认曲线/普通状态
          }
      }
    // printf("off_line:%d    ",ImageStatus.OFFLine);
    // printf("_acc:%.2f     ",variance_acc);
    // printf("l:%d     ",ImageStatus.Left_Line);
    // printf("r:%d     \n",ImageStatus.Right_Line);
}

//用于变参数的直道检测。短直线有BUG而且没必要
void Straight_Test_2(void) 
{
 float midd_k, sum;
 midd_k = (ImageDeal[55].Center - ImageDeal[ImageStatus.OFFLine + 1].Center) /(float)(55 - ImageStatus.OFFLine - 1);
 for (Ysite = 55; Ysite > ImageStatus.OFFLine + 1; Ysite--) 
 {
   ImageDeal[Ysite].mid_temp =ImageDeal[55].Center - midd_k * (55 - Ysite) + 0.5;
   sum += pow(ImageDeal[Ysite].Center - ImageDeal[Ysite].mid_temp, 2);
 }

 variance = sum / (54 - ImageStatus.OFFLine);
 if (variance < ImageStatus.variance && ImageStatus.OFFLine <= 9 ) 
 {
     ImageStatus.Road_type = Straight;
 }
}

void Element_Test()
{
//  if(ImageFlag.image_element_rings == 0)
//  {
//    Element_Judgment_Zebra();
//  }


  if (ImageStatus.Road_type != Ramp)/*&& circle_num < 2 */    //圆环检测//&& ImageStatus.Road_type != Ramp    
  {
    Element_Judgment_Left_Rings();           //左圆环检测
    Element_Judgment_Right_Rings();          //右圆环检测
  }

  
}


//要打印的数据
// static void get_imgdisplay()
// {
//     for(Ysite = 59; Ysite >= 0; Ysite--)
//     {
//       for(int x=0;x<=79;x++)
//       {
//         imgdisplay[Ysite][x]=0;
//       }
//         img3[Ysite][ImageDeal[Ysite].Center] =6;
//         img3[Ysite][ImageDeal[Ysite].LeftBorder]=7;
//         img3[Ysite][ImageDeal[Ysite].RightBorder]=8;
//     }
// }

static void get_imgdisplay()
{
    // 1. 先清空数组（或复制原始二值化图像）
    for (int Ysite = 0; Ysite < 60; Ysite++) {
        for (int Xsite = 0; Xsite < 80; Xsite++) {
            // 复制原始二值化图像（img1是二值化后的数组，0=黑，1=白）
            imgdisplay[Ysite][Xsite] = img1[Ysite][Xsite]; 
        }
    }

    // 2. 标记补线后的边界和中线（覆盖原始像素）
    for (int Ysite = 0; Ysite < 60; Ysite++) {
        // 标记左边界（确保不越界）
        int left = ImageDeal[Ysite].LeftBorder;
        if (left >= 0 && left < 80) {
            imgdisplay[Ysite][left] = PIXEL_LEFT; 
        }

        // 标记右边界
        int right = ImageDeal[Ysite].RightBorder;
        if (right >= 0 && right < 80) {
            imgdisplay[Ysite][right] = PIXEL_RIGHT; 
        }

        // 标记中线
        int center = ImageDeal[Ysite].Center;
        if (center >= 0 && center < 80) {
            imgdisplay[Ysite][center] = PIXEL_CENTER; 
        }
    }
}


void imageprocess(void)
{
    ImageStatus.OFFLine = 2;  //这个值根据真实距离得到，必须进行限制//图像顶边
    ImageStatus.WhiteLine = 0;//双边丢边数
    for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--)
    {
    ImageDeal[Ysite].IsLeftFind = 'F';//左边有边
    ImageDeal[Ysite].IsRightFind = 'F';//右边有边
    ImageDeal[Ysite].LeftBorder = 0;//边界
    ImageDeal[Ysite].RightBorder = 79;
    ImageDeal[Ysite].LeftTemp = 0;//临时边界
    ImageDeal[Ysite].RightTemp = 79;
    ImageDeal[Ysite].close_LeftBorder = 0;//靠边边界
    ImageDeal[Ysite].close_RightBorder = 79;
    }  

  drawfirstline();//绘制底边
  DrawLinesProcess();//获取基本边线
  Search_Border_OTSU(img1,CAMERA_H ,CAMERA_W, CAMERA_H - 2);//58行位底行//八邻域爬边
  //Straightacc_Test();//直线检测
  // if(ImageStatus.Road_type != LeftCirque&&ImageStatus.Road_type != RightCirque&&ImageStatus.Road_type != Ramp)
  // {Element_Test();}//元素检测
  if (g_enable_element_process) {
    Element_Test();
  }
  
  if(!g_enable_element_process || ImageFlag.image_element_rings == 0)
    {DrawExtensionLine(); } //绘制延长线

  RouteFilter();//中线平滑滤波
  ApplyCurveCenterCompensation(); // 最后用可靠边界重构中线，避免RouteFilter把坏邻点重新混入
  Straightacc_Test(); //直线加速
  if (g_enable_element_process) {
    Element_Handle();//元素处理
    Stop_Test();
  }
  GetDet();//前瞻获取，计算偏差
  if (g_enable_element_process) {
    Stop_Test2();
  }
  get_imgdisplay();//获取要打印的数据
  // lq_udp_img_trans_demo();
  // printf("stop:%.2f\n",SystemData.SpeedData.Length);
  // printf("stop_BZW:%d\n",SystemData.Stop);
  //   printf("圆环进程：%d\n",ImageFlag.image_element_rings_flag);
}
