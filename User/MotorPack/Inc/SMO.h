#ifndef  Sensorless_SMO_H
#define  Sensorless_SMO_H

#define Speed_est_DEFAULTS {0,0,0,0,0,0,0}                      // 初始化参数
#define SMO_Motor_DEFAULTS {0,0,0,0,0,4,0,0}                    // 初始化参数
#define Angle_SMO_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  // 初始化参数
#include "ch32v30x.h"

typedef struct {
    float  Valpha;       //二相静止坐标系alpha-轴电压
    float  Ealpha;       //二相静止坐标系alpha-轴反电动势
    float  Zalpha;       //alfa轴滑膜观测器的z平面
    float  EstIalpha;    //滑膜估算alpha-轴电流
    float  Vbeta;        //二相静止坐标系beta-轴电压
    float  Ebeta;  	     //二相静止坐标系beta-轴反电动势
    float  Zbeta;        //beta轴滑膜观测器的z平面
    float  EstIbeta;     //滑膜估算beta-轴电流
    float  Ialpha;  	 //二相静止坐标系alpha-轴电流
    float  IalphaError;  //二相静止坐标系beta-轴电流误差
    float  Kslide;       //滤波器系数
    float  Ibeta;  	     //二相静止坐标系beta-轴电流
    float  IbetaError;   //二相静止坐标系beta-轴电流误差
    float  Kslf;         //滤波器系数
    float  E0;	         //滑膜的电流误差的限幅值 0.5
    float  Fsmopos;      // 滑膜系数1
    float  Gsmopos;      // 滑膜系数2
} Angle_SMO, *p_Angle_SMO;



typedef struct {
    int16_t   JZElecTheta;  //速度电角度值（计算速度）
    int16_t   Oid_JZElecTheta;   // 电机历史电角度
    int16_t   Speed_ele_angleIQ;      // 电机电角度
    int16_t   Speed_ele_angleIQFitter;  //速度电角度值（计算速度）
    uint8_t   MXZ_State;
    int16_t   Speed_RPM;       	 //电机旋转速度
    float     speed_coeff;       //计算速度的系数
}Speed_est;



typedef struct{
    float  Rs; 			//电机的相电阻
    float  Ls;			//电机的相电感
    float  Ib; 			//电机控制器的基本相电流
    float  Vb;			//电机控制器的基本相电压
    float  Ts;			 //采样周期
    uint8_t  POLES; // 电机的极对数
    float  Fsmopos;		   //滑膜常数1
    float  Gsmopos;			 //滑膜常数2
}SMO_Motor;



extern  Angle_SMO   Angle_SMOPare ;
extern  Speed_est   Speed_estPare ;
extern  SMO_Motor   SMO_MotorPare ;

void  Angle_Correct(void);
void  Angle_Cale(p_Angle_SMO  pV);  //滑膜电机位置电角度计算
void  SMO_Pare_init (void );        //滑膜观测器的参数初始化
void  SMO_Speedcale(void);          //滑膜的角度计算速度函数
#endif /* Sensorless_SMO*/
