/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __motor_H
#define __motor_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"	 
	 
/* USER CODE BEGIN Includes */
#include "gpio.h"	 
#include "datacheck.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define FRONT_LEFT 	  1  //LF
#define FRONT_RIGHT   2	 //RF
#define BACK_RIGHT 	  3  //RB
#define BACK_LEFT  	  4  //LB

#define FL_Clip		  5          //
#define FR_Clip		  6          //
#define BR_Clip		  7          //
#define BL_Clip		  8          //



#define Clip_MOTOR_SPEED_S      600        //快到时抱夹速度 快 //20220909
#define Clip_MOTOR_LOW_SPEED_S  500        //快到时抱夹速度 慢 //20220909

#define Clip_MOTOR_SPEED     1000         //抱夹速度 1000 
#define Clip_MIDDLE_POSITION 0         //抱夹中间位置
#define Clip_MOTOR_LOW_SPEED 800        //抱夹速度 慢 
#define Clip_DISTANCE        30         //两个夹臂移动时，最大差距距离

#define Clip_DISTANCE_ERR        100         //两个夹臂移动时，最大差距距离,超过这个距离停止并报警

#define Clip_SPEED_LOW      800
#define Clip_SPEED_MID      1200
#define Clip_SPEED_HIGH     2400
#define Clip_SPEED_OFFSET   50


#define EncoderMiddleValue   2099200     //编码器中间初始值
#define ZeroAbout            20        //(40/14336)*360=1.0044度
#define ZeroAngleAbout       0.5        //0.5 度
#define ZeroAbout_MAX        200       //允许最大偏差±5度 20220926
#define ZeroAboutRot         50         //旋转时 
#define OneCircleDistance    213.52      //编码器2转一圈轴距移动的距离106.76
#define WheelbaseUpperLimit  3200         //轴距调整上限
#define WheelbaseLowerLimit  2400         //轴距调整下限
#define AdjustSpeed          200         //调整时的电机速度100*4
#define AdjustSpeed1         30          //调整时的电机速度 旋转

#define WHEEL_ROT_SPEED_HIGH		600//300					//前后轮旋转速度-高速
#define WHEEL_ROT_SPEED_MID		    300//300					//前后轮旋转速度-高速
#define WHEEL_ROT_SPEED_LOW			100//50					//前后轮旋转速度-低速

#define Adjust_Moving_Speed     30          //调整时的移动速度 20220914
#define AccelerateSpeedFactor  1.1         //调整后直行过程中的加速度调整系数
#define AccelerateScaleFactor  40      //加速系数
#define ANGLE_ADJUST_SCALE		398		//默认角度纠偏在10°以内，10° = 398 (10/360*14336)编码器的值
#define RotSpeed_LIMIT			10*PI/180			//车体旋转限速10°/s
#define RPM_LIMIT		2822//1000			//500mm/s
#define SPEED_LIMIT_MM  1000
#define ROTSPEED_LIMIT_DEG 20              //旋转角度限制 10度/s
#define SetCount             1           //set速度发送次数
#define DISTAN_PER_ROUND 		21.26	 //行走电机一圈实际行进距离，mm
#define ADJUSTMODE_NONE      0           //轴距调整状态位=0  不调整状态
#define ADJUSTMODE_START     1           //轴距调整状态位=1  开始调整
#define ADJUSTMODE_STOP      2           //轴距调整状态位=1  停止调整
#define	WHEEL_DISTANCE		364			 //轮距，mm
#define READY_START         1            //启动前的准备
#define READY_STOP          0            //完成后的准备
#define MOVEREADY_OK       1             //移动前准备(车轮转到固定方向)就绪
#define MOVEREADY_NO       0             //移动前准备未就绪
//移动过程中轮子编码器与基准值最大偏差值  10°
#define MoveEcOffsetLimit  400
#define TurnEcOffsetLimit  200
#define RPM_TO_PULSE   2730.667   //转速转脉冲值 512 * 10000 / 1875
#define PULSE_TO_RPM   0.00036621    //脉冲值转转速  / 512 / 10000 * 1875

//extern float a, f;                        //全局变量，轴距和2800+轴距


enum ROTDIR
{
	ROTDIR_CW = 0,   //顺时针旋转
	ROTDIR_CCW = 1   //逆时针旋转
};



typedef struct
{
		int16_t TARGET_SPEED; //0.1mm/s
		int16_t TARGET_ANGLE; //0.1°
		float TARGET_RPS; //rad/s

		uint16_t RT_SPEED;
		uint16_t RT_ANGLE;
		float RT_RPS;	
		int16_t RT_VY;
		int16_t RT_VX;	
		uint16_t ODOMETER_RT_SPEED;
		uint16_t ODOMETER_RT_ANGLE;
		float ODOMETER_RT_RPS;	
		int16_t ODOMETER_RT_VY;
		int16_t ODOMETER_RT_VX;	
	
		float ODOMETER_X;
		float ODOMETER_Y;	
		float ODOMETER_Z;	
		float PC_ODOMETER_Z;	
}CAR_CTRL_STA;


typedef struct
{
	float RT_RPS;	
	float RT_VY;
	float RT_VX;	
	float ODOMETER_X;
	float ODOMETER_Y;
	float ODOMETER_Z;
	float PC_ODOMETER_Z;	
}CAR_Odometer_Integral;//抱夹AGV里程计积分
 
extern CAR_CTRL_STA INNER;
//extern uint8_t limit;
//extern Clip_CTRL_STA Clip_STA;
extern CAR_Odometer_Integral CAR_Odometer;
extern float Af,Ar;
extern float Vf,Vr;
typedef struct
{
	int16_t TARGET_VELOCITY;  //目标线速度 0.1mm/s
	int32_t ACTUAL_RPM;    //输出轮子转速（每分钟多少转）     抱闸的步科电机是32位的	
	uint16_t alarm;            //告警信号
	uint16_t update;           //数据更新次数
}MOTOR_MOVE_CTRL;

typedef struct
{
	int32_t POSITION;         //输出轮子的实际位置  
	int32_t clipPos_open;     //包夹打开限位值
	int32_t clipPos_close;    //包夹关闭限位值
	int32_t clip_distance;    //包夹距离
	uint16_t alarm;           //告警信号
	uint16_t update;          //数据更新次数
}MOTOR_CLIP_CTRL;



typedef struct
{
	int32_t clip_pos_LF;
	int32_t clip_pos_RF;
	int32_t clip_pos_LB;
	int32_t clip_pos_RB;
	uint16_t stopCount;

}ClipPosion;



typedef struct
{
    float target_val;           //目标值
    float actual_val;        		//实际值
    float err;             			//定义偏差值
    float err_last;          		//定义上一个偏差值
	float err_last_last;        //上上次偏差值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float integral;          		//定义积分值
	float Limit;                    //限幅
}PID;


void  PID_Init(PID* pid,float kp,float ki,float kd,float limit);

float PID_Pos_Output(PID* pid,float temp_val);

float PID_Inc_Output(PID* pid,float temp_val);

float PID_Inc_ErrOutput(PID* pid, float err);


extern float Wheelbase_Data;//轴距相关结构体 20221004

extern MOTOR_MOVE_CTRL MotorFL;
extern MOTOR_MOVE_CTRL MotorFR;
extern MOTOR_MOVE_CTRL MotorBL;
extern MOTOR_MOVE_CTRL MotorBR;
extern MOTOR_CLIP_CTRL MotorCFL;
extern MOTOR_CLIP_CTRL MotorCFR;
extern MOTOR_CLIP_CTRL MotorCBL;
extern MOTOR_CLIP_CTRL MotorCBR;

//extern  PID   pcMovPID_F;
//extern  PID   pcMovPID_B;
//extern  PID   RpmPID;      //速度控制控制PID

extern int16_t FrontReadyRpm ;   //前轮调整方向的速度
extern int16_t BackReadyRpm ;    //后轮调整方向速度

extern int16_t FClipRpm ;    //前包夹速度
extern int16_t BClipRpm ;    //后包夹速度
extern int16_t MaxClipRpm;  //当前包夹最大运动速度
extern ClipPosion ClipPos;


void Set_Can_RPM(uint8_t can_channel,int16_t rpm,int8_t count);
void Set_Can_Motor_Enable(uint8_t can_channel,int8_t count);
void Set_Can_RPM_RPDO(uint8_t can_channel, int16_t rpm, int8_t count);

void SetMotorEnable(void);

void SetMotorDisable(void);//抱夹时，行走电机关使能
void Set_Can_Motor_Disable(uint8_t can_channel,int8_t count);//抱夹时，行走电机关使能

//轴距动作函数 20220920
//void Front_Positive_Behind_Negative(uint16_t Adjustment_Value);        //前差动单元正转，后差动单元反转
//void Front_Negative_Behind_Positive(uint16_t Adjustment_Value);        //前差动单元反转，后差动单元正转
//void Rear_Wheel_Negative(uint16_t Adjustment_Value);                   //后差动单元后退
//void Rear_Wheel_Positive(uint16_t Adjustment_Value);                    //后差动单元前进  20221005
void Wheelbase_Calculation(void);               //轴距计算程序
//前轮直线移动  正为前进 负为后退
void FrontStraightMove(int16_t rpm);
//后轮直线移动 正为前进 负为后退
void BackStraightMove(int16_t rpm);

int FindCarWheel(void);            //找汽车轮胎
int AjustCarWheelDis(void);        //调整汽车前后轮间距距
int AjustDisReady(int readyMode); //调整轴距准备

/*********************************************************************/

void Front_Unit_Rotation(int16_t rpm);        //前差动单元旋转/顺时针为正，逆时针为负
void Behind_Unit_Rotation(int16_t rpm);       //后差动单元旋转/顺时针为正，逆时针为负
void Stop_Moving(void);                           //电机停止移动
void Stop_Back_Wheel(void);                       //后差动单元停止
void Stop_Front_Wheel(void);                      //前差动单元停止


float RpmUpdate(float acc, float  targetSpd, float curSpd); //速度更新计算
float RpmUpdateWithDec(float acc, float dec, float  targetSpd, float curSpd);//带减速的速度更新计算

void Odometer_Solution(void);//里程计解算

int Front_Differential_Unit_Adjustment(int maxRpm);  //前差动单元调整(任意角度) 0 调整到位 1 没有调整到位
int Back_Differential_Unit_Adjustment(int maxRpm);   //后差动单元调整(任意角度) 0 调整到位 1 没有调整到位
                    
//获取包夹状态
int GetClipState(void);
//抱夹打开
int ClipOpen(void);
//包夹关闭
int ClipClose(void);
//包夹停止
void ClipStop(void);
//包夹准备 1 准备完成 0位完成
int  ClipReady(int readyMode);  
//包夹运动是否停止
int IsClipStop(void);
/*********************************************************************/


/* USER CODE END Prototypes */

//是否移动静止
int IsMoveStoped(void);

//前轮是否静止
int IsFWheelStoped(void);

//后轮是否静止
int IsBWheelStoped(void);

//移动前的准备
int MoveReady(void);

//自动移动
void AutoMove(int16_t speed,float fAngleDelta, float bAngleDelta, uint8_t slowdown);

//保持移动
void KeepMove(float rpm, float fAngleOffset_Deg, float bAngleOffset_Deg);

//正为逆时针旋转，负为顺时针旋转  Rot_spd 旋转角速度   slowdown 是否减速停止 0 否 1减速停止
void AutoRotation(float Rot_spd,uint8_t slowdown);

//限制最大值
float LimitMax(float val, float limit);
//限制最小值
float LimitMin(float val, float limit);


//保持旋转 rpm：转速   out_in_rpmScale:内外轮速度比例 fAngleDelta 前轮偏转角(度)  bAngleDelta 后轮偏转角
void KeepRotation(float rpm, float out_in_rpmScale,float fAngleDelta,float bAngleDelta);


//检测前后编码器是否超限
int CheckECOverLimit(float f_ecKeepAngle, float b_ecKeepAngle, float limit);

//检测是否轮子到位，如果到位返回1，否则返回0
int CheckWheel();

#ifdef __cplusplus
}
#endif
#endif 

/**
  * @}
  */

/**
  * @}
  */
