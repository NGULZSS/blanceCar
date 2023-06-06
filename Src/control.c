#include"control.h"
#include "../include/HardwareDriver/gpio.h"

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
    
//     if(GPIO_Pin==CO_Pin)
//     {
//         Red_LED_ON;
//         __HAL_GPIO_EXTI_CLEAR_IT(CO_Pin);		
//     }

// }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//电压测量相关变量
	static u8 Flag_Target;																//控制函数相关变量，提供10ms基准
	int Encoder_Left,Encoder_Right;             					//左右编码器的脉冲计数
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//平衡环PWM变量，速度环PWM变量，转向环PWM变
	int CoPwm;
    u8 Way_Angle=1;
	
	// if( GPIO_Pin == KEY3_Pin)//判断外部中断源
	// {
	// 	Way_Angle++;
	// 	if(Way_Angle>3)
	// 	{
	// 		Way_Angle=1;
	// 	}
	// }
  	if( GPIO_Pin == KEY2_Pin)//判断外部中断源
	{
          STBY_HIGH;
           Red_LED_ON;
           __HAL_GPIO_EXTI_CLEAR_IT(KEY2_Pin);					//清除中断标志位
	}
    	if( GPIO_Pin == KEY1_Pin)//判断外部中断源
	{
	}


if( GPIO_Pin == CO_Pin)//判断外部中断源
	{
    Flag_Target=!Flag_Target;
    //Get_Angle(3);
    Encoder_Left = SpeedDealLeft((int)(__HAL_TIM_GET_COUNTER(&htim3)));   // 读取左轮编码器的值，前进为正，后退为负
    Encoder_Right = SpeedDealRigit((int)(__HAL_TIM_GET_COUNTER(&htim4))); // 读取右轮编码器的值，前进为正，后退为负
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//编码器读数转速度（mm/s）
    Get_Angle(2);
	delay_flag=1;
       if(delay_flag==1)
		{
			if(++delay_50==10)	 
                delay_50=0,delay_flag=0;  //给主函数提供50ms的精准延时，示波器需要50ms高精度延时
		}
        Balance_Pwm=Balance(PoseData.Angle_Balance,PoseData.Gyro_Balance);

        Balance_Pwm=PWM_Limit(Balance_Pwm,1000,-1000);
         Set_Pwm(Balance_Pwm,Balance_Pwm);    


	 	__HAL_GPIO_EXTI_CLEAR_IT(CO_Pin);					//清除中断标志位

	}

}

int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}

void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	   { BN1_HIGH;			BN2_LOW;} //前进
  else           		{BN1_LOW;			BN2_HIGH; } //后退
   __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, myabs(motor_left));
  if(motor_right>0)			{AN1_LOW;			AN2_HIGH;}	//前进
  else 	        			{AN2_LOW;			AN1_HIGH;}   //后退
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, myabs(motor_right));
}

void Get_Angle(uint8_t way)
{
    float Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
    uint8_t Rec_Data[14];
    int16_t temp;
    if (way == 1) // DMP的读取在数据采集中断读取，严格遵循时序要求
    {
        mpu_dmp_get_data(&PoseData.Pitch, &PoseData.Roll, &PoseData.Yaw, &PoseData.Gyro_Balance, &PoseData.Gyro_Turn, &PoseData.Acceleration_Z); // 读取加速度、角速度、倾角
        PoseData.Angle_Balance = PoseData.Pitch;
		
    }
    else
    {
        HAL_I2C_Mem_Read(&MPU6050_I2C_Handle, MPU_READ, MPU_ACCEL_XOUTH_REG, 1, Rec_Data, 14, 100);

        Accel_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        Accel_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        Accel_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
        Gyro_X = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
        Gyro_Y = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
        Gyro_Z = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
        if (Gyro_X > 32768)
            Gyro_X -= 65536; // 数据类型转换  也可通过short强制类型转换
        if (Gyro_Y > 32768)
            Gyro_Y -= 65536; // 数据类型转换  也可通过short强制类型转换
        if (Gyro_Z > 32768)
            Gyro_Z -= 65536; // 数据类型转换
        if (Accel_X > 32768)
            Accel_X -= 65536; // 数据类型转换
        if (Accel_Y > 32768)
            Accel_Y -= 65536; // 数据类型转换
        if (Accel_Z > 32768)
            Accel_Z -= 65536;                                 // 数据类型转换
       
        Accel_Angle_x = atan2(Accel_Y, Accel_Z) * 180 / PI; // 计算倾角，转换单位为度
        Accel_Angle_y = atan2(Accel_X, Accel_Z) * 180 / PI; // 计算倾角，转换单位为度
        Gyro_X = Gyro_X / 16.4;                               // 陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册
        Gyro_Y = Gyro_Y / 16.4;                               // 陀螺仪量程转换
        if (way == 2)
        {
            PoseData.Pitch = -Kalman_Filter_x(Accel_Angle_x, Gyro_X); // 卡尔曼滤波
            PoseData.Roll = -Kalman_Filter_y(Accel_Angle_y, Gyro_Y);
        }
        else if (way == 3)
        {
            PoseData.Pitch = -Complementary_Filter_x(Accel_Angle_x, Gyro_X); // 互补滤波
            PoseData.Roll = -Complementary_Filter_y(Accel_Angle_y, Gyro_Y);
        }
        PoseData.Gyro_Balance = -Gyro_X;                      // 更新平衡角速度
        PoseData.Angle_Balance = PoseData.Pitch; // 更新平衡倾角
        PoseData.Gyro_Turn = Gyro_Z;             // 更新转向角速度
        PoseData.Acceleration_Z = Accel_Z;       // 更新Z轴加速度计
        ShowDataCo.AC_Angle_Balance=-Gyro_X;
        ShowDataCo.AX_Angle_Balance=Gyro_Y;
    }
    
    ShowDataCo.Angle_Balance=PoseData.Angle_Balance;

}
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}

void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{
	float Rotation_Speed_L,Rotation_Speed_R;						//电机转速  转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	ShowDataCo.LeftWheel_Velocity = Rotation_Speed_L*PI*Diameter_67;		//求出编码器速度=转速*周长
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	ShowDataCo.RightWheel_Velocity = Rotation_Speed_R*PI*Diameter_67;		//求出编码器速度=转速*周长
}

int SpeedDealRigit(int SPEED)
{
    static int SPEEDlast = 0;
    static int ShaftFlag = 0;
    int speedActual;
    if (SPEED - SPEEDlast > 30000)
    {
        ShaftFlag = 1;
    }
    else if (SPEED - SPEEDlast < -30000)
    {
        ShaftFlag = 2;
    }
    if (ShaftFlag == 1)
    {
        speedActual = SPEED - 65535;
    }
    else
    {
        speedActual = SPEED;
    }
    SPEEDlast = SPEED;
    return speedActual;
}
int SpeedDealLeft(int SPEED)
{
    static int SPEEDlast = 0;
    static int ShaftFlag = 0;
    int speedActual;
    int as;
    if (SPEED - SPEEDlast > 30000)
    {
        ShaftFlag = 1;
    }
    else if (SPEED - SPEEDlast < -30000)
    {
        ShaftFlag = 2;
    }
    if (ShaftFlag == 1)
    {
        speedActual = SPEED - 65535;
    }
    else
    {
        speedActual = SPEED;
    }
    SPEEDlast = SPEED;
    return speedActual;
}

int Balance(float Angle,float Gyro)
{
   float Angle_bias,Gyro_bias;
   UprightRingPIDs.P=80;
   UprightRingPIDs.D=1;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//求出平衡的角度中值 和机械相关
	 Gyro_bias=0-Gyro;
	 balance=-UprightRingPIDs.P*Angle_bias-Gyro_bias*UprightRingPIDs.D; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
	 return balance;
}



int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;
   switch(TIMX)
	 {
	   case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
	   case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;
	   case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;
	   default: Encoder_TIM=0;
	 }
		return Encoder_TIM;
}