
#include "math.h"
#include "SMO.h"

float  Limit_Sat( float Uint,float U_max, float U_min) //限制赋值函数
{
    float Uout;
    if(Uint<= U_min)
        Uout= U_min;
    else if( Uint>=U_max)
        Uout=U_max;
    else
        Uout= Uint;
    return  Uout;
}

Angle_SMO   Angle_SMOPare   =Angle_SMO_DEFAULTS ;
Speed_est   Speed_estPare   =Speed_est_DEFAULTS ;
SMO_Motor   SMO_MotorPare   =SMO_Motor_DEFAULTS ;

float  Angle_CompCoff=0.15;   // 角度补偿系数


void  Angle_Cale(p_Angle_SMO pV)
{
    float IalphaError_Limit=0,IbetaError_Limit=0;

    /*	Sliding mode current observer	*/
    pV->EstIalpha = pV->EstIalpha*pV->Fsmopos + (pV->Valpha-pV->Ealpha-pV->Zalpha)*pV->Gsmopos;
    pV->EstIbeta  = pV->EstIbeta*pV->Fsmopos + (pV->Vbeta -pV->Ebeta-pV->Zbeta)*pV->Gsmopos ;

    /*	Current errors	*/
    pV->IalphaError = pV->EstIalpha - pV->Ialpha;
    pV->IbetaError  = pV->EstIbeta  - pV->Ibeta;

    /*  Sliding control calculator	*/
    /* pV->Zalpha=pV->IalphaError*pV->Kslide/pV->E0) where E0=0.5 here*/
    //限制赋值函数
    IalphaError_Limit =Limit_Sat(pV->IalphaError,pV->E0,-pV->E0);
    IbetaError_Limit =Limit_Sat(pV->IbetaError ,pV->E0,-pV->E0);

    pV->Zalpha = IalphaError_Limit*pV->Kslide;
    pV->Zbeta  = IbetaError_Limit*pV->Kslide ;

    /*	Sliding control filter -> back EMF calculator	*/
    pV->Ealpha = pV->Ealpha +(pV->Zalpha-pV->Ealpha)*pV->Kslf;
    pV->Ebeta  = pV->Ebeta +(pV->Zbeta -pV->Ebeta)*pV->Kslf;
}

void  SMO_Pare_init(void)  // 电机参数初始化
{
    SMO_MotorPare.Rs      = 0.10f;      // 电机电阻欧姆
    SMO_MotorPare.Ls      = 0.000022f;  // 电机电感H
    SMO_MotorPare.Ib      = 10.0f;      // 电机的额定相电流
    SMO_MotorPare.Vb      = 16.8f*0.57735f;      // 电机额定相电压  24*0.57735
    SMO_MotorPare.Ts      = 0.00004f;  // 代码运行周期，PWM中断周期 15KHZ
    SMO_MotorPare.POLES   = 7;          // 电机极对数
    SMO_MotorPare.Fsmopos = exp((-SMO_MotorPare.Rs/SMO_MotorPare.Ls)*(SMO_MotorPare.Ts));
    SMO_MotorPare.Gsmopos = (SMO_MotorPare.Vb/SMO_MotorPare.Ib)*(1/SMO_MotorPare.Rs)*(1-SMO_MotorPare.Fsmopos);

    // 计算滑膜 观测器系数， 根据实际电机调节滤波系数  Kslide和 Kslf
    Angle_SMOPare.Fsmopos = SMO_MotorPare.Fsmopos ;
    Angle_SMOPare.Gsmopos = SMO_MotorPare.Gsmopos ;
    Angle_SMOPare.Kslide  = 0.168f;    //
    Angle_SMOPare.Kslf    = 0.02f;    //
    Angle_SMOPare.E0      = 0.1f;    //  标幺值1的一半
    Speed_estPare.speed_coeff=(float)(500*60/(SMO_MotorPare.POLES*4096.0f));  //  1.831054463982609
}



void  Angle_Correct(void)  // 电角度校正
{
//    Angle_Comp=ADCSampPara.RP_speed_Voltage*Angle_CompCoff;   // 相当于根据给定转速乘一个补偿角度系数，
//    // 硬件和软件滤波导致的角度滞后补偿
//    IQAtan_Pare.IQAngle_JZ=IQAtan_Pare.IQAngle+Angle_Comp ;
//
//    if( IQAtan_Pare.IQAngle_JZ>4095)   // 调整电角度方位IQ12格式
//        IQAtan_Pare.IQAngle_JZ=IQAtan_Pare.IQAngle_JZ-4096;
//    else if( IQAtan_Pare.IQAngle_JZ<0)
//        IQAtan_Pare.IQAngle_JZ=IQAtan_Pare.IQAngle_JZ+4096;
}


// 电机在2ms时间计算角度变化量。即是公式：
// Speed_estPare.Speed_ele_angleIQ =Speed_estPare.ele_angleIQ -Speed_estPare.old_ele_angleIQ
// 防止超过65535和小于0，把差值一阶滤波，插值变化量乘系数就可以得到速度。
// 速度信号的计算可以简单根据转位置的步进角计算或者直接根据角度在一定周期内的变化量计算
// 其中移位16是把角度变化量归1的一个系数，变化角度/360度，在乘一个系数得到速度，
// 可以通过示波器测量一个霍尔周期时间来计算。
// 假设一个霍尔周期时间15ms，电机极对数为4，速度RPM=1/T*60/p=1000/15*60/4=1000rpm
// 然后看在线看角度变化量（或者通讯发出来），速度RPM=变化量/65535*K=1000，求得系数K。
// 系数 :Speed_estPare.speed_coeff

void SMO_Speedcale(void)  // 2ms执行一次
{
//    Speed_estPare.JZElecTheta=IQAtan_Pare.IQAngle_JZ;
//    if(Speed_estPare.JZElecTheta>1000)  // 1/4电角度，消除角度过零判断方向错误
//    {
//        if(Speed_estPare.JZElecTheta<3100)  // // 3/4电角度，消除角度过零判断方向错误
//        {
//            if( Speed_estPare.JZElecTheta> Speed_estPare.Oid_JZElecTheta)
//                Speed_estPare.MXZ_State=1; // 判断正转
//            else if(Speed_estPare.JZElecTheta<Speed_estPare.Oid_JZElecTheta)
//                Speed_estPare.MXZ_State=2;   // 判断反转
//            else
//                Speed_estPare.MXZ_State=0;   // 停止
//        }
//    }
//    if( Speed_estPare.MXZ_State==1) // 根据正反转
//    {
//        Speed_estPare.Speed_ele_angleIQ =Speed_estPare.JZElecTheta -Speed_estPare.Oid_JZElecTheta ;
//        if( Speed_estPare.Speed_ele_angleIQ <0)
//            Speed_estPare.Speed_ele_angleIQ+= 4095;
//    }
//    else  if( Speed_estPare.MXZ_State==2) // 负转速
//    {
//        Speed_estPare.Speed_ele_angleIQ =Speed_estPare.JZElecTheta-Speed_estPare.Oid_JZElecTheta;
//        if( Speed_estPare.Speed_ele_angleIQ>0)
//            Speed_estPare.Speed_ele_angleIQ-=4095;
//    }
//    else
//        Speed_estPare.Speed_ele_angleIQ=0;
//
//    Speed_estPare.Speed_ele_angleIQFitter= Speed_estPare.Speed_ele_angleIQFitter*0.9f+ Speed_estPare.Speed_ele_angleIQ*0.1f ;
//    Speed_estPare.Speed_RPM = Speed_estPare.Speed_ele_angleIQ*Speed_estPare.speed_coeff; // 最大角度 2pi是一圈 65536
//    if(Speed_estPare.Speed_RPM>3000)  // 限制异常反馈转速
//        Speed_estPare.Speed_RPM=3000;
//
//    Speed_estPare.Oid_JZElecTheta=Speed_estPare.JZElecTheta;
}


//===========================================================================
// No more.
//===========================================================================

