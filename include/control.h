#ifndef CONTROL_H_
#define CONTROL_H_
#include "../Include/HardwareDriver/main.h"

#define PI 3.14159265							//PI圆周率
#define Control_Frequency  200.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  13.0 	//编码器精度 13线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm

#define Middle_angle 0
#define DIFFERENCE 100
    struct UprightRingPID
    {
       float P;
       float D;
    }UprightRingPIDs;
    struct VelocityRingPID
    {
       float P;
       float I;
       float D;
    }VelocityRingPIDs;

struct Gyroscope_Information  //陀螺仪信息
{
    float Roll;  //计算出横滚角  可能是平衡角
    float Pitch;// 计算出俯仰角
    float Yaw;  //计算出偏航角
    float Angle_Balance;  //平衡角
    float Gyro_Balance;  //更新平衡角速度
    float Gyro_Turn;  //更新转向角速度
    float Acceleration_Z;  //更新Z轴加速度计
} PoseData;

int Read_Encoder(uint8_t TIMX);
int Balance(float angle,float gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
void Set_Pwm(int motor_left,int motor_right);
void Key(void);
void Limit_Pwm(void);
int PWM_Limit(int IN,int max,int min);
uint8_t Turn_Off(float angle, int voltage);
void Get_Angle(uint8_t way);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
void Choose(int encoder_left,int encoder_right);
int SpeedDealRigit(int SPEED);
int SpeedDealLeft(int SPEED);




#endif