/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
// #include "struct_typedef.h"
#include "ros/ros.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;


typedef struct   //PID死区修改
{
    uint8_t mode;
    //PID 三参数
    float P;
    float I;
    float D;

    float I_LIMIT;  //最大输出
    float OUT_LIMIT; //最大积分输出

    float set;
    float get;

    float OUT;
    float err;
    float err_err;
	  float err_old;
    float P_OUT;
    float I_OUT;  //微分项 0最新 1上一次 2上上次
    float D_OUT; //误差项 0最新 1上一次 2上上次

} _PID;


/*带死区PID*/
 
 typedef struct
 {
   float setpoint;               /*设定值*/
   float kp;                     /*比例系数*/
   float ki;                     /*积分系数*/
   float kd;                     /*微分系数*/
   float lasterror;              /*前一拍偏差*/
   float preerror;               /*前两拍偏差*/
   float deadband;               /*死区*/
   float result;                 /*PID控制器计算结果*/
   float output;                 /*输出值0-100%*/
   float maximum;                /*输出值上限*/
   float minimum;                /*输出值下限*/
   float errorabsmax;            /*偏差绝对值最大值*/
   float errorabsmin;            /*偏差绝对值最小值*/
   float alpha;                  /*不完全微分系数*/
   float derivative;              /*微分项*/
   float integralValue;          /*积分累计量*/
 }CLASSICPID;
 
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern float PID_calc(pid_type_def *pid, float ref, float set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

extern float PID_Cal_Limt(_PID *PID, float limit, float get, float set);//PID死区修改
extern void PID_Cal_Limt_Init(_PID *PID, const float PID_l[3], float max_out, float max_iout);
//带死区、抗积分饱和、梯形积分、变积分算法以及不完全微分算法的增量型PID控制器
extern void PIDRegulator( CLASSICPID *vPID , float P , float I , float D , float set , float pv , float alp , float max_out , float min_out);

#endif




