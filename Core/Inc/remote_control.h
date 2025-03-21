#include "main.h"

#define RC_Manual 1       //�ֶ�ȫ�����
#define RC_Manual_Dir 2   //�ֶ��������
#define RC_Auto 4         //ң��������
#define RC_Online 1         //ң��������
#define RC_Offline 4         //ң��������
#define RC_SPEED_LOW      4   //ң��������
#define RC_SPEED_MID      2  //ң��������
#define RC_SPEED_HIGH     1  //ң��������
#define RC_CLAMP_OPEN     4   //ң�ر����ɿ�
#define RC_CLAMP_CLOSE    1   //ң�ذ���
#define RC_AJUST_OPEN    1    //ң�����������
#define RC_AJUST_CLOSE    4  //ң���������С


#define PC_OnLine      1       //����
#define PC_Offline     0       //����

#define PC_Manual      0
#define PC_Auto        1

#define AgvOwner_PC 4      //��λ��ӵ��agvȨ��
#define AgvOwner_RC 8      //ң����ӵ��agvȨ��
#define AgvOwner_None 0    //����ӵ��agvȨ��

/* ----------------------- Data Struct ------------------------------------- */

typedef __packed struct
{
	int8_t  EmergencyStop;
	int8_t  Clip;                          //���п���     ���й�01    ����ֹͣ02    ���п�04
	int8_t  SpeedGear;
	int8_t  OperationMode;
	int8_t  OnlineOperation;
	int8_t  WheelBaseAdjust;							   //  ������   ���ǰ����04   ���ֹͣ��02    �����ˣ�01  
	int16_t MovingDirection;                //�ƶ����� 0-360
	int16_t MovingSpeed;                    //�ƶ��ٶ�  mm/s
	int16_t RotationSpeed;                  //��ת���ٶ�  ��/s ����ʱ�룬��˳ʱ��
	int16_t RADv;                           //��ת���Ƚ��ٶ�

}RCCtrl;

typedef __packed struct
{
	uint8_t     cmd;                //��ǰ����ָ�� 0������   1 �ƶ�  2��ת   3����  4�ɼ� 	                                //5 �������   6 �Զ������   7���  8ֹͣ���
	uint16_t	speed;			    //Ŀ�귽���趨�ٶ�,mm/s
	int16_t	    deg;				//Ŀ�귽���뵱ǰ����н� 0.1��
	int16_t     frontAngle;			//ǰ������ǰ�ֵ����Ƕ� 0.1��
	int16_t     backAngle;			//ǰ��������ֵ����Ƕ� 0.1��
	uint16_t    wheelBaseLength;    //���ֵ mm
	int16_t     rot_spd;			//������ת���ٶ� /ԭ����ת�Ľ��ٶȣ������� 0.1��
	uint8_t		voc_sta;		    //����״̬��
	uint8_t		em_stop;		    //��ͣ
	uint8_t		alarm;			    //����
	uint8_t		charg;			    //����·
	uint8_t		up;					//����
	uint8_t		WheelAdjustState;	//������		
	uint8_t		numb;				//����
	int8_t  	Period;			    //������		��λ������

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
#define Err_Wheel_NotFind    50   //δ�ҵ�����
#define Err_Clip_NotOpened    51  //����δ��

//AGV����
typedef struct
{
	uint8_t     cmd;                //��ǰ����ָ�� 0������   1 �ƶ�  2��ת   3����  4�ɼ� 	            
									//5 �������   6 �Զ������   7���  8ֹͣ���
	uint16_t	speed;			    //Ŀ�귽���趨�ٶ�,mm/s
	uint16_t	deg;				//Ŀ�귽���뵱ǰ����н� 0.1�� 0-3600
	int16_t     rotSpeed;           //��ת���ٶ� 0.1�� ��Ϊ��ʱ�룬��Ϊ˳ʱ��
	int16_t		frontAngle;			//ǰ������ǰ�ֵ����Ƕ� 0.1��
	int16_t		backAngle;			//ǰ��������ֵ����Ƕ� 0.1��
	uint16_t    wheelBaseLength;    //���ֵ mm
	uint8_t     state;              //��������״̬
}AGVTASK;




//AGV״̬
typedef struct
{
	uint8_t     clipState;          //����״̬      0 ����λ  1����λ  2������ 3 �м�λ
	uint8_t     chargeState;        //���״̬      0 δ��� 1�����
	uint8_t     ajustDisState;      //������״̬  0δ����  1������� 2 ������  3��������
	uint8_t     carState;           //����״̬      0 �޳��� 1�г���
	uint8_t     onLineState;        //����״̬      0  ����  1����
	uint8_t     ctrlMode;           //����ģʽ      0  �ֶ�  1�Զ�
	uint8_t     owner;              //Ȩ��ӵ����
	uint8_t     stopButton;         //��ͣ��ť״̬      0δ��ͣ   1��ͣ 
	uint8_t     stopFlag;           //��ͣ��ǣ�������λ��ң�أ��ͼ�ͣ��ť  0δ��ͣ 1��ͣ
	uint8_t     dirReady;           //���ӷ����������  0δ����  1����
	uint8_t     errorCode;          //������
	uint16_t    errorInfo;          //���󸽼���Ϣ
	uint16_t    wheelBaseLength;    //���ֵ mm
	uint8_t     power;              //����
	int16_t     odoX;               //��̼�X mm
	int16_t     odoY;               //��̼�Y mm
	int16_t     odoZ;               //��̼ƽǶ� 0.1
	int32_t    ecKeepAngle;        //���������ֵĽǶ�
	float       outputRpm;          //���ת��

}AGVSTATUS;


extern AGVTASK agvTask;
extern AGVSTATUS agvStatus;

typedef __packed struct                   //״̬
{
	__packed struct
	{
		int8_t AutomatiCaccesscontrol;       //  =8(PC��Ȩ�ޣ�ң������Ȩ��)     =4(PC�п���Ȩ)      =0(ң�����ͷſ���Ȩ)Ȼ����sendtoPC����4
		int8_t Clipmode;                    //���й�01    ����ֹͣ02    ���п�04  ������Э�鲻һ�£���ȱ��״̬�����ж�����		//2022.7.9�� ds
		int8_t scram_alarm_filter;
		int8_t OperationMode;                //  =0 ң��������(�ֶ�)             =1 PC����(�Զ�) 
		int8_t OnlineOperation;
		int8_t WheelbaseAdjust_mode;              //������״̬	 =0������ֹͣ״̬     =1������
		int8_t WheelbaseAdjust_mode_pc;           //��λ������ ������״̬	 =0������ֹͣ״̬     =1������					
		int8_t AdjustMode;                      //AGV������״̬  =0 ������״̬			=1 AGV������ʼ   =2 AGV��������		=3AGV������
		int8_t AdjustMode_Wheelbase;            //����������ʱ��AGV������״̬  =0 ������״̬			=1 AGV������ʼ   =2 AGV��������		=3AGV������					
	} state;
	__packed struct
	{
		int16_t MovingDirection;		//�뵱ǰ����ļн�
		int16_t MovingSpeed;				//������Ŀ���ٶ�
		int16_t RotationSpeed;		//ԭ����ת���ٶ�
		int16_t RADv;		//������ת�ٶ�
		int16_t ACC_COEFFICIENT;		//�ϰ��ּ��ٶ�ϵ��
	} move;
}ctrl_mode_t;                                 //״̬


typedef __packed struct                 //can�����ĵ��״̬   ��բ���ϵĵ��
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
	int8_t NetworkingStatus;   //����״̬
	int8_t RunningState;       //����״̬
	int8_t ControlModel;       //����״̬
	int8_t CommunicationAlarm; //ͨ�ű���
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

void RemoteInit(void);                 //��ʼ��
int  HandleError(int isInit);      //������
void DoAgvCtrl(void);                  //AGV����
void SwitchTask(AGVTASK *pTask);   //�л�����
void HandleMove(void);                 //�����ƶ�
void HandleTurn(void);                 //������ת
void HandleClampOpen(void);            //�����ɼ�
void HandleClampClose(void);           //������
void HandleCharge(void);               //������
void HandleStopCharge(void);           //����ֹͣ���
void HandleAjustDis(void);             //��������
void HandleFindDis(void);              //���������
void HandleNone(void);                 //������ʱ����
void UpdateAgvStatus(void);            //����agv״̬
void HandleTask(void);                 //��������
void HandleReset(void);                //����λ
void HandleRestartMotor(void);         //�����������
void HandleMotorPowerOff(void);        //�������ϵ�

