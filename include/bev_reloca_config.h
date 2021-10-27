#ifndef BEV_RELOCA_CONFIG_H_
#define BEV_RELOCA_CONFIG_H_

#define BEV_RANGE_LENGTH 15  //m
#define BEV_RANGE_WIDTH 10     //m   

#define CAR_LENGTH 4.76              //m
#define CAR_WIDTH 1.86                 //m
#define CAR_REAR_TO_CENTER 1.01   //m
#define PI 3.1415926

#define SLOT_CORNER_Z 0
#define LANE_Z 10
#define LONG_WHITE_LINE_Z -10
#define ARROW_Z 20
#define ZEBRA_Z 30
#define SPEEDBUMP_Z 40


#define INTERVAL 0.1                        //矢量线段离散成点时的点距离间隔，单位m
#define LOCAL_FRAMES_NUM 5  //定位时连续选择的帧数

#define SLOT_CORNER_FUSE_DIS 0.3 //多帧叠加到一起时，角点距离小于30cm的认为不是同一个角点
#define LINE_FUSE_DIS 1.0 

//相机在车辆坐标系下的x、y坐标
static float CAM_CENTER[4][2] = {3.70651, 0.0152577,
                                                                    -1.00785, -0.0325327,
                                                                    2.07323, -1.03863,
                                                                    2.07537, 1.00252};

//多项式系数， a0*x^2+a1*x+a2，计算成像距离误差
//static float ERROR_COEF[3] = {0.0113, 0.0065, 0.0044};         
static float ERROR_COEF[3] = {0, 0, 0};                         

//perception noise
/* static float loss_rate_[5] = {0.2, 0.2, 0.2, 0.2, 0.2};  //元素检测不到的概率
static float false_rate_[5][5] = {0.0,  0.0,     0.0,     0.0,     0.0,  //车位角点的误检率以及被错分为其他类别的概率
                                                               0.0,  0.08,  0.7,     0.15,  0.15,          //lane
                                                               0.0,  0.7,     0.08,  0.12,   0.18,           //arrow
                                                               0.0,  0.7,     0.2,     0.1,      0.1,              //zebra
                                                               0.0,  0.45,  0.5,     0.05,   0.1};        //speedbump */ 

static float loss_rate_[5] = {0, 0, 0, 0, 0};  //元素检测不到的概率
static float false_rate_[5][5] = {0.0,  0.0,     0.0,     0.0,     0.0,  //车位角点的误检率以及被错分为其他类别的概率
                                                               0.0,  0.00,  0.7,     0.15,  0.15,          //lane
                                                               0.0,  0.7,     0.00,  0.12,   0.18,           //arrow
                                                               0.0,  0.7,     0.2,     0.0,      0.1,              //zebra
                                                               0.0,  0.45,  0.5,     0.05,   0.0};        //speedbump


#endif //#ifndef BEV_RELOCA_CONFIG_H_