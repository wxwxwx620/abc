#include "parameter.h"


uint8_t INTRGRAL_TIME;//速度积分时间
uint8_t ACC_COEFFICIENT;//加速度系数 
float ODOMETER_Y_CORRECTION;//修正值
float ODOMETER_X_CORRECTION;
float ODOMETER_Z_CORRECTION;
float ODOMETER_Y_COEFFICIENT;//里程计系数
float ODOMETER_X_COEFFICIENT;
float ODOMETER_Z_COEFFICIENT;

/*0.1mm*/
uint16_t IX_CENTER_DIST;
uint16_t IY_CENTER_DIST;
uint16_t OX_CENTER_DIST;
uint16_t OY_CENTER_DIST;
uint16_t IL_COEFFICIENT;//内轮位置系数
uint16_t OL_COEFFICIENT;//外轮位置系数

//uint16_t PULSES_PER_CIRCLE;//单圈脉冲数
//uint8_t REDUCTION_RATIO;//减速比
float REDUCTION_RATIO;//减速比
//uint16_t WHEEL_RADIUS;//车轮半径
float WHEEL_RADIUS;//车轮半径

float VMM_TO_RPM;//输出频率系数,每行进1mm距离脉冲数量
float RPM_TO_VMM;
uint8_t AutomatiCaccess_control;//控制权标志 0：无；12：旧版有线遥控；10：有线遥控；8：无线遥控；6：app遥控；4：上位机；2自动；
uint8_t alarm__control;//yao 0：无急停 1：车体急停；2遥控器急停；4：上位机急停
uint8_t control_mode;
uint8_t control_mode_1;
float DEG_TO_RAD=0.017453;   //角度转换为弧度
float RAD_TO_DEG=57.2958;    //弧度转角度

void Parameter_Init(void)
{
	INTRGRAL_TIME = 10;//10ms
	ACC_COEFFICIENT = 50;//1*200=200mm/s
	ODOMETER_Y_CORRECTION = 1.0f;//0.980f;
	ODOMETER_X_CORRECTION = 1.0f;//0.954f;
	ODOMETER_Z_CORRECTION = 1.0f;//0.966f;

	ODOMETER_Y_COEFFICIENT = 	ODOMETER_Y_CORRECTION*INTRGRAL_TIME/1000;
	ODOMETER_X_COEFFICIENT = 	ODOMETER_X_CORRECTION*INTRGRAL_TIME/1000;
	ODOMETER_Z_COEFFICIENT = 	ODOMETER_Z_CORRECTION*INTRGRAL_TIME/1000;	
	
	IX_CENTER_DIST = 6725;//6720;
	IY_CENTER_DIST = 8800;//8750;
	OX_CENTER_DIST = 6725;//6720;
	OY_CENTER_DIST = 14900;//12950;
	IL_COEFFICIENT = IX_CENTER_DIST+IY_CENTER_DIST;
	OL_COEFFICIENT = OX_CENTER_DIST+OY_CENTER_DIST;	
	
	//PULSES_PER_CIRCLE = 5000;
	REDUCTION_RATIO = 10*24/13; //10为齿轮减速器减速比，24/13为链条结构减速比
	WHEEL_RADIUS = 125/2;	//直径为125mm
	VMM_TO_RPM = (1*60*REDUCTION_RATIO)/(WHEEL_RADIUS*2*3.1415926f); // 1mm/S线速度等效的轮子的转速
	RPM_TO_VMM = 1 / VMM_TO_RPM;
	//yao start
	MOTORPOWER=1;//关驱动器电源
	HOIST_DOWN=1;//缓上电关
	BRAKE_EN=0;//使能/抱闸关

	LR_AK = 0;
	control_mode=0x01;
	control_mode_1=0x00;
	AutomatiCaccess_control = 8;

	//yao end

//	PFM_COEFFICIENT//
	
//Motor acceleration 

}



