#include "main.h"

#define RC_Manual 1       //手动全向控制
#define RC_Manual_Dir 2   //手动定向控制
#define RC_Auto 4         //遥控器控制
#define RC_Online 1         //遥控器控制
#define RC_Offline 4         //遥控器控制
#define RC_SPEED_LOW      4   //遥控器低速
#define RC_SPEED_MID      2  //遥控器中速
#define RC_SPEED_HIGH     1  //遥控器高速
#define RC_CLAMP_OPEN     4   //遥控抱夹松开
#define RC_CLAMP_CLOSE    1   //遥控包夹
#define RC_AJUST_OPEN    1    //遥控器轴距扩大
#define RC_AJUST_CLOSE    4  //遥控器轴距缩小


#define PC_OnLine      1       //在线
#define PC_Offline     0       //离线

#define PC_Manual      0
#define PC_Auto        1

#define AgvOwner_PC 4      //上位机拥有agv权限
#define AgvOwner_RC 8      //遥控器拥有agv权限
#define AgvOwner_None 0    //无人拥有agv权限

/* ----------------------- Data Struct ------------------------------------- */

typedef __packed struct
{
	int8_t  EmergencyStop;
	int8_t  Clip;                          //抱夹开关     抱夹关01    抱夹停止02    抱夹开04
	int8_t  SpeedGear;
	int8_t  OperationMode;
	int8_t  OnlineOperation;
	int8_t  WheelBaseAdjust;							   //  轴距调整   轴距前进：04   轴距停止：02    轴距后退：01  
	int16_t MovingDirection;                //移动方向 0-360
	int16_t MovingSpeed;                    //移动速度  mm/s
	int16_t RotationSpeed;                  //旋转角速度  度/s 正逆时针，负顺时针
	int16_t RADv;                           //旋转弧度角速度

}RCCtrl;

typedef __packed struct
{
	uint8_t     cmd;                //当前运行指令 0无任务   1 移动  2旋转   3包夹  4松夹 	                                //5 调定轴距   6 自动找轴距   7充电  8停止充电
	uint16_t	speed;			    //目标方向设定速度,mm/s
	int16_t	    deg;				//目标方向与当前车体夹角 0.1度
	int16_t     frontAngle;			//前进方向前轮调整角度 0.1度
	int16_t     backAngle;			//前进方向后轮调整角度 0.1度
	uint16_t    wheelBaseLength;    //轴距值 mm
	int16_t     rot_spd;			//车体旋转角速度 /原地旋转的角速度，有正负 0.1度
	uint8_t		voc_sta;		    //语音状态码
	uint8_t		em_stop;		    //急停
	uint8_t		alarm;			    //报警
	uint8_t		charg;			    //充电回路
	uint8_t		up;					//抱夹
	uint8_t		WheelAdjustState;	//轴距调整		
	uint8_t		numb;				//车号
	int8_t  	Period;			    //命令间隔		单位：毫秒

}PCCtrl;

#define Task_None 0
#define Task_Move 1
#define Task_Turn 2
#define Task_ClampClose 3
#define Task_ClampOpen 4
#define Task_AjustDis 5
#define Task_FindDis 6
#define Task_Charge 7
#define Task_StopCharge 8
#define Task_Reset  9
#define Task_MotorOff 10
#define Task_RestartMotor 11

#define TaskState_Init 0
#define TaskState_Start 1
#define TaskState_Run 2
#define TaskState_Stop 3
#define TaskState_Finish 4

#define ClipState_Opened    0
#define ClipState_Closed    1
#define ClipState_Running   2
#define ClipState_Stoped    3

#define AjustState_None     0
#define AjustState_OK       1
#define AjustState_Running  2
#define AjustState_Error    3


#define Err_MotorFL_Alarm  1
#define Err_MotorFL_Timeout 2
#define Err_MotorFR_Alarm  3
#define Err_MotorFR_Timeout  4
#define Err_MotorBL_Alarm  5
#define Err_MotorBL_Timeout  6
#define Err_MotorBR_Alarm  7
#define Err_MotorBR_Timeout  8

#define Err_MotorCFL_Alarm  9
#define Err_MotorCFL_Timeout  10
#define Err_MotorCFR_Alarm  11
#define Err_MotorCFR_Timeout  12
#define Err_MotorCBL_Alarm  13
#define Err_MotorCBL_Timeout  14
#define Err_MotorCBR_Alarm  15
#define Err_MotorCBR_Timeout  16


#define Err_FEC_Timeout    17
#define Err_FEC_OverLimit  18
#define Err_BEC_Timeout    19
#define Err_BEC_OverLimit  20
#define Err_DisEC_Timeout    21
#define Err_DisEC_OverLimit  22

#define Err_MBS_Timeout    23
#define Err_Wheel_NotFind    50   //未找到轮子
#define Err_Clip_NotOpened    51  //包夹未打开

//AGV任务
typedef struct
{
	uint8_t     cmd;                //当前运行指令 0无任务   1 移动  2旋转   3包夹  4松夹 	            
									//5 调定轴距   6 自动找轴距   7充电  8停止充电
	uint16_t	speed;			    //目标方向设定速度,mm/s
	uint16_t	deg;				//目标方向与当前车体夹角 0.1度 0-3600
	int16_t     rotSpeed;           //旋转角速度 0.1度 正为逆时针，负为顺时针
	int16_t		frontAngle;			//前进方向前轮调整角度 0.1度
	int16_t		backAngle;			//前进方向后轮调整角度 0.1度
	uint16_t    wheelBaseLength;    //轴距值 mm
	uint8_t     state;              //任务运行状态
}AGVTASK;




//AGV状态
typedef struct
{
	uint8_t     clipState;          //包夹状态      0 降到位  1升到位  2运行中 3 中间位
	uint8_t     chargeState;        //充电状态      0 未充电 1充电中
	uint8_t     ajustDisState;      //轴距调整状态  0未调整  1调整完成 2 调整中  3调整出错
	uint8_t     carState;           //车辆状态      0 无车辆 1有车辆
	uint8_t     onLineState;        //上线状态      0  下线  1上线
	uint8_t     ctrlMode;           //控制模式      0  手动  1自动
	uint8_t     owner;              //权限拥有者
	uint8_t     stopButton;         //急停按钮状态      0未急停   1急停 
	uint8_t     stopFlag;           //急停标记，包括上位，遥控，和急停按钮  0未急停 1急停
	uint8_t     dirReady;           //轮子方向调整就绪  0未就绪  1就绪
	uint8_t     errorCode;          //错误码
	uint16_t    errorInfo;          //错误附加信息
	uint16_t    wheelBaseLength;    //轴距值 mm
	uint8_t     power;              //电量
	int16_t     odoX;               //里程计X mm
	int16_t     odoY;               //里程计Y mm
	int16_t     odoZ;               //里程计角度 0.1
	int32_t    ecKeepAngle;        //编码器保持的角度
	float       outputRpm;          //输出转速

}AGVSTATUS;


extern AGVTASK agvTask;
extern AGVSTATUS agvStatus;

typedef __packed struct                   //状态
{
	__packed struct
	{
		int8_t AutomatiCaccesscontrol;       //  =8(PC无权限，遥控器有权限)     =4(PC有控制权)      =0(遥控器释放控制权)然后在sendtoPC里，变成4
		int8_t Clipmode;                    //抱夹关01    抱夹停止02    抱夹开04  ？？与协议不一致！！缺少状态：抱夹动作中		//2022.7.9， ds
		int8_t scram_alarm_filter;
		int8_t OperationMode;                //  =0 遥控器控制(手动)             =1 PC控制(自动) 
		int8_t OnlineOperation;
		int8_t WheelbaseAdjust_mode;              //轴距调整状态	 =0轴距调整停止状态     =1轴距调整
		int8_t WheelbaseAdjust_mode_pc;           //上位机控制 轴距调整状态	 =0轴距调整停止状态     =1轴距调整					
		int8_t AdjustMode;                      //AGV调整的状态  =0 不调整状态			=1 AGV调整开始   =2 AGV调整结束		=3AGV调整中
		int8_t AdjustMode_Wheelbase;            //轴距调整任务时的AGV调整的状态  =0 不调整状态			=1 AGV调整开始   =2 AGV调整结束		=3AGV调整中					
	} state;
	__packed struct
	{
		int16_t MovingDirection;		//与当前方向的夹角
		int16_t MovingSpeed;				//调整后目标速度
		int16_t RotationSpeed;		//原地旋转角速度
		int16_t RADv;		//车体旋转速度
		int16_t ACC_COEFFICIENT;		//老八轮加速度系数
	} move;
}ctrl_mode_t;                                 //状态


typedef __packed struct                 //can反馈的电机状态   抱闸车上的电机
{
	int8_t Motor_Alarm_FL;
	int8_t Motor_Alarm_FR;
	int8_t Motor_Alarm_BR;
	int8_t Motor_Alarm_BL;
	int8_t Motor_Alarm_Clip_FL;
	int8_t Motor_Alarm_Clip_FR;
	int8_t Motor_Alarm_Clip_BR;
	int8_t Motor_Alarm_Clip_BL;
}can_motor_status;



typedef __packed struct
{
	int8_t NetworkingStatus;   //联网状态
	int8_t RunningState;       //运行状态
	int8_t ControlModel;       //控制状态
	int8_t CommunicationAlarm; //通信报警
}ui_mode_t;

extern RCCtrl rc_ctrl;
extern PCCtrl pc_ctrl;
extern ctrl_mode_t ctrl_mode;
extern can_motor_status can_status;
extern ui_mode_t ui_t;

extern uint8_t Sensor_Flag; //20221005

extern uint8_t PowerReady;

extern void srcdata_to_rc(volatile uint8_t *srcdata);
extern void srcdata_to_pc(volatile uint8_t *srcdata);

void motor_move_rc(void);
void motor_move_pc(void);

void RemoteInit(void);                 //初始化
int  HandleError(int isInit);      //错误处理
void DoAgvCtrl(void);                  //AGV控制
void SwitchTask(AGVTASK *pTask);   //切换任务
void HandleMove(void);                 //处理移动
void HandleTurn(void);                 //处理旋转
void HandleClampOpen(void);            //处理松夹
void HandleClampClose(void);           //处理抱夹
void HandleCharge(void);               //处理充电
void HandleStopCharge(void);           //处理停止充电
void HandleAjustDis(void);             //处理调轴距
void HandleFindDis(void);              //处理找轴距
void HandleNone(void);                 //无任务时处理
void UpdateAgvStatus(void);            //更新agv状态
void HandleTask(void);                 //处理任务
void HandleReset(void);                //处理复位
void HandleRestartMotor(void);         //处理重启电机
void HandleMotorPowerOff(void);        //处理电机断电

