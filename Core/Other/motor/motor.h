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



#define Clip_MOTOR_SPEED_S      600        //�쵽ʱ�����ٶ� �� //20220909
#define Clip_MOTOR_LOW_SPEED_S  500        //�쵽ʱ�����ٶ� �� //20220909

#define Clip_MOTOR_SPEED     1000         //�����ٶ� 1000 
#define Clip_MIDDLE_POSITION 0         //�����м�λ��
#define Clip_MOTOR_LOW_SPEED 800        //�����ٶ� �� 
#define Clip_DISTANCE        30         //�����б��ƶ�ʱ����������

#define Clip_DISTANCE_ERR        100         //�����б��ƶ�ʱ����������,�����������ֹͣ������

#define Clip_SPEED_LOW      800
#define Clip_SPEED_MID      1200
#define Clip_SPEED_HIGH     2400
#define Clip_SPEED_OFFSET   50


#define EncoderMiddleValue   2099200     //�������м��ʼֵ
#define ZeroAbout            20        //(40/14336)*360=1.0044��
#define ZeroAngleAbout       0.5        //0.5 ��
#define ZeroAbout_MAX        200       //�������ƫ���5�� 20220926
#define ZeroAboutRot         50         //��תʱ 
#define OneCircleDistance    213.52      //������2תһȦ����ƶ��ľ���106.76
#define WheelbaseUpperLimit  3200         //����������
#define WheelbaseLowerLimit  2400         //����������
#define AdjustSpeed          200         //����ʱ�ĵ���ٶ�100*4
#define AdjustSpeed1         30          //����ʱ�ĵ���ٶ� ��ת

#define WHEEL_ROT_SPEED_HIGH		600//300					//ǰ������ת�ٶ�-����
#define WHEEL_ROT_SPEED_MID		    300//300					//ǰ������ת�ٶ�-����
#define WHEEL_ROT_SPEED_LOW			100//50					//ǰ������ת�ٶ�-����

#define Adjust_Moving_Speed     30          //����ʱ���ƶ��ٶ� 20220914
#define AccelerateSpeedFactor  1.1         //������ֱ�й����еļ��ٶȵ���ϵ��
#define AccelerateScaleFactor  40      //����ϵ��
#define ANGLE_ADJUST_SCALE		398		//Ĭ�ϽǶȾ�ƫ��10�����ڣ�10�� = 398 (10/360*14336)��������ֵ
#define RotSpeed_LIMIT			10*PI/180			//������ת����10��/s
#define RPM_LIMIT		2822//1000			//500mm/s
#define SPEED_LIMIT_MM  1000
#define ROTSPEED_LIMIT_DEG 20              //��ת�Ƕ����� 10��/s
#define SetCount             1           //set�ٶȷ��ʹ���
#define DISTAN_PER_ROUND 		21.26	 //���ߵ��һȦʵ���н����룬mm
#define ADJUSTMODE_NONE      0           //������״̬λ=0  ������״̬
#define ADJUSTMODE_START     1           //������״̬λ=1  ��ʼ����
#define ADJUSTMODE_STOP      2           //������״̬λ=1  ֹͣ����
#define	WHEEL_DISTANCE		364			 //�־࣬mm
#define READY_START         1            //����ǰ��׼��
#define READY_STOP          0            //��ɺ��׼��
#define MOVEREADY_OK       1             //�ƶ�ǰ׼��(����ת���̶�����)����
#define MOVEREADY_NO       0             //�ƶ�ǰ׼��δ����
//�ƶ����������ӱ��������׼ֵ���ƫ��ֵ  10��
#define MoveEcOffsetLimit  400
#define TurnEcOffsetLimit  200
#define RPM_TO_PULSE   2730.667   //ת��ת����ֵ 512 * 10000 / 1875
#define PULSE_TO_RPM   0.00036621    //����ֵתת��  / 512 / 10000 * 1875

//extern float a, f;                        //ȫ�ֱ���������2800+���


enum ROTDIR
{
	ROTDIR_CW = 0,   //˳ʱ����ת
	ROTDIR_CCW = 1   //��ʱ����ת
};



typedef struct
{
		int16_t TARGET_SPEED; //0.1mm/s
		int16_t TARGET_ANGLE; //0.1��
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
}CAR_Odometer_Integral;//����AGV��̼ƻ���
 
extern CAR_CTRL_STA INNER;
//extern uint8_t limit;
//extern Clip_CTRL_STA Clip_STA;
extern CAR_Odometer_Integral CAR_Odometer;
extern float Af,Ar;
extern float Vf,Vr;
typedef struct
{
	int16_t TARGET_VELOCITY;  //Ŀ�����ٶ� 0.1mm/s
	int32_t ACTUAL_RPM;    //�������ת�٣�ÿ���Ӷ���ת��     ��բ�Ĳ��Ƶ����32λ��	
	uint16_t alarm;            //�澯�ź�
	uint16_t update;           //���ݸ��´���
}MOTOR_MOVE_CTRL;

typedef struct
{
	int32_t POSITION;         //������ӵ�ʵ��λ��  
	int32_t clipPos_open;     //���д���λֵ
	int32_t clipPos_close;    //���йر���λֵ
	int32_t clip_distance;    //���о���
	uint16_t alarm;           //�澯�ź�
	uint16_t update;          //���ݸ��´���
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
    float target_val;           //Ŀ��ֵ
    float actual_val;        		//ʵ��ֵ
    float err;             			//����ƫ��ֵ
    float err_last;          		//������һ��ƫ��ֵ
	float err_last_last;        //���ϴ�ƫ��ֵ
    float Kp,Ki,Kd;          		//������������֡�΢��ϵ��
    float integral;          		//�������ֵ
	float Limit;                    //�޷�
}PID;


void  PID_Init(PID* pid,float kp,float ki,float kd,float limit);

float PID_Pos_Output(PID* pid,float temp_val);

float PID_Inc_Output(PID* pid,float temp_val);

float PID_Inc_ErrOutput(PID* pid, float err);


extern float Wheelbase_Data;//�����ؽṹ�� 20221004

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
//extern  PID   RpmPID;      //�ٶȿ��ƿ���PID

extern int16_t FrontReadyRpm ;   //ǰ�ֵ���������ٶ�
extern int16_t BackReadyRpm ;    //���ֵ��������ٶ�

extern int16_t FClipRpm ;    //ǰ�����ٶ�
extern int16_t BClipRpm ;    //������ٶ�
extern int16_t MaxClipRpm;  //��ǰ��������˶��ٶ�
extern ClipPosion ClipPos;


void Set_Can_RPM(uint8_t can_channel,int16_t rpm,int8_t count);
void Set_Can_Motor_Enable(uint8_t can_channel,int8_t count);
void Set_Can_RPM_RPDO(uint8_t can_channel, int16_t rpm, int8_t count);

void SetMotorEnable(void);

void SetMotorDisable(void);//����ʱ�����ߵ����ʹ��
void Set_Can_Motor_Disable(uint8_t can_channel,int8_t count);//����ʱ�����ߵ����ʹ��

//��ද������ 20220920
//void Front_Positive_Behind_Negative(uint16_t Adjustment_Value);        //ǰ���Ԫ��ת������Ԫ��ת
//void Front_Negative_Behind_Positive(uint16_t Adjustment_Value);        //ǰ���Ԫ��ת������Ԫ��ת
//void Rear_Wheel_Negative(uint16_t Adjustment_Value);                   //����Ԫ����
//void Rear_Wheel_Positive(uint16_t Adjustment_Value);                    //����Ԫǰ��  20221005
void Wheelbase_Calculation(void);               //���������
//ǰ��ֱ���ƶ�  ��Ϊǰ�� ��Ϊ����
void FrontStraightMove(int16_t rpm);
//����ֱ���ƶ� ��Ϊǰ�� ��Ϊ����
void BackStraightMove(int16_t rpm);

int FindCarWheel(void);            //��������̥
int AjustCarWheelDis(void);        //��������ǰ���ּ���
int AjustDisReady(int readyMode); //�������׼��

/*********************************************************************/

void Front_Unit_Rotation(int16_t rpm);        //ǰ���Ԫ��ת/˳ʱ��Ϊ������ʱ��Ϊ��
void Behind_Unit_Rotation(int16_t rpm);       //����Ԫ��ת/˳ʱ��Ϊ������ʱ��Ϊ��
void Stop_Moving(void);                           //���ֹͣ�ƶ�
void Stop_Back_Wheel(void);                       //����Ԫֹͣ
void Stop_Front_Wheel(void);                      //ǰ���Ԫֹͣ


float RpmUpdate(float acc, float  targetSpd, float curSpd); //�ٶȸ��¼���
float RpmUpdateWithDec(float acc, float dec, float  targetSpd, float curSpd);//�����ٵ��ٶȸ��¼���

void Odometer_Solution(void);//��̼ƽ���

int Front_Differential_Unit_Adjustment(int maxRpm);  //ǰ���Ԫ����(����Ƕ�) 0 ������λ 1 û�е�����λ
int Back_Differential_Unit_Adjustment(int maxRpm);   //����Ԫ����(����Ƕ�) 0 ������λ 1 û�е�����λ
                    
//��ȡ����״̬
int GetClipState(void);
//���д�
int ClipOpen(void);
//���йر�
int ClipClose(void);
//����ֹͣ
void ClipStop(void);
//����׼�� 1 ׼����� 0λ���
int  ClipReady(int readyMode);  
//�����˶��Ƿ�ֹͣ
int IsClipStop(void);
/*********************************************************************/


/* USER CODE END Prototypes */

//�Ƿ��ƶ���ֹ
int IsMoveStoped(void);

//ǰ���Ƿ�ֹ
int IsFWheelStoped(void);

//�����Ƿ�ֹ
int IsBWheelStoped(void);

//�ƶ�ǰ��׼��
int MoveReady(void);

//�Զ��ƶ�
void AutoMove(int16_t speed,float fAngleDelta, float bAngleDelta, uint8_t slowdown);

//�����ƶ�
void KeepMove(float rpm, float fAngleOffset_Deg, float bAngleOffset_Deg);

//��Ϊ��ʱ����ת����Ϊ˳ʱ����ת  Rot_spd ��ת���ٶ�   slowdown �Ƿ����ֹͣ 0 �� 1����ֹͣ
void AutoRotation(float Rot_spd,uint8_t slowdown);

//�������ֵ
float LimitMax(float val, float limit);
//������Сֵ
float LimitMin(float val, float limit);


//������ת rpm��ת��   out_in_rpmScale:�������ٶȱ��� fAngleDelta ǰ��ƫת��(��)  bAngleDelta ����ƫת��
void KeepRotation(float rpm, float out_in_rpmScale,float fAngleDelta,float bAngleDelta);


//���ǰ��������Ƿ���
int CheckECOverLimit(float f_ecKeepAngle, float b_ecKeepAngle, float limit);

//����Ƿ����ӵ�λ�������λ����1�����򷵻�0
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
