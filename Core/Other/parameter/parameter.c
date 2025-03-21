#include "parameter.h"


uint8_t INTRGRAL_TIME;//�ٶȻ���ʱ��
uint8_t ACC_COEFFICIENT;//���ٶ�ϵ�� 
float ODOMETER_Y_CORRECTION;//����ֵ
float ODOMETER_X_CORRECTION;
float ODOMETER_Z_CORRECTION;
float ODOMETER_Y_COEFFICIENT;//��̼�ϵ��
float ODOMETER_X_COEFFICIENT;
float ODOMETER_Z_COEFFICIENT;

/*0.1mm*/
uint16_t IX_CENTER_DIST;
uint16_t IY_CENTER_DIST;
uint16_t OX_CENTER_DIST;
uint16_t OY_CENTER_DIST;
uint16_t IL_COEFFICIENT;//����λ��ϵ��
uint16_t OL_COEFFICIENT;//����λ��ϵ��

//uint16_t PULSES_PER_CIRCLE;//��Ȧ������
//uint8_t REDUCTION_RATIO;//���ٱ�
float REDUCTION_RATIO;//���ٱ�
//uint16_t WHEEL_RADIUS;//���ְ뾶
float WHEEL_RADIUS;//���ְ뾶

float VMM_TO_RPM;//���Ƶ��ϵ��,ÿ�н�1mm������������
float RPM_TO_VMM;
uint8_t AutomatiCaccess_control;//����Ȩ��־ 0���ޣ�12���ɰ�����ң�أ�10������ң�أ�8������ң�أ�6��appң�أ�4����λ����2�Զ���
uint8_t alarm__control;//yao 0���޼�ͣ 1�����弱ͣ��2ң������ͣ��4����λ����ͣ
uint8_t control_mode;
uint8_t control_mode_1;
float DEG_TO_RAD=0.017453;   //�Ƕ�ת��Ϊ����
float RAD_TO_DEG=57.2958;    //����ת�Ƕ�

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
	REDUCTION_RATIO = 10*24/13; //10Ϊ���ּ��������ٱȣ�24/13Ϊ�����ṹ���ٱ�
	WHEEL_RADIUS = 125/2;	//ֱ��Ϊ125mm
	VMM_TO_RPM = (1*60*REDUCTION_RATIO)/(WHEEL_RADIUS*2*3.1415926f); // 1mm/S���ٶȵ�Ч�����ӵ�ת��
	RPM_TO_VMM = 1 / VMM_TO_RPM;
	//yao start
	MOTORPOWER=1;//����������Դ
	HOIST_DOWN=1;//���ϵ��
	BRAKE_EN=0;//ʹ��/��բ��

	LR_AK = 0;
	control_mode=0x01;
	control_mode_1=0x00;
	AutomatiCaccess_control = 8;

	//yao end

//	PFM_COEFFICIENT//
	
//Motor acceleration 

}



