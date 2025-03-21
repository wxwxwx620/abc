#include "usart.h"	 
#include "datacheck.h"

#define EncoderReadMin  2084864   //��������ȡ��Сֵ -360 360��Ӧ14336
#define EncoderReadMax  2113536   //��������ȡ���ֵ +360

#define EncoderRunMin  2093824   //���������������λ
#define EncoderRunMax  2104576   //���������������λ

#define EncoderZero 2099200            //���������ֵ
#define EncoderAngle90 2102784         //������90��ֵ
#define Angle_To_Encoder 39.8222       //ÿ�ȴ������ֵ  360�� 14336
#define TenthAngle_To_Encoder 3.98222  //ÿ0.1�ȴ������ֵ
#define Encoder_To_Angle 0.025111      //������ֵת��Ϊ����   


typedef  struct
{
	int32_t	val;        //������ֵ       ˳ʱ��ӣ���ʱ�Ӽ�    ��
	uint8_t		dir;        //��ת���� 01˳ʱ��02��ʱ��
	uint16_t	rpm;        //RPM
	uint16_t  update;     //���ݸ��´���
	int16_t   zeroDelt;   //0��ƫת����ֵ
}Encoder;

extern Encoder Encoder_Front;   //ǰ�ֱ�����
extern Encoder Encoder_Back;    //���ֱ�����
extern Encoder Encoder_Axle;    //��������


extern uint32_t M,N;
void Encoder_Send(void);
void EncoderSendWithCRC(void);
void Usart3Receive_encoder_IDLE(void);
void InitEncoder(Encoder * ec, int16_t zeroDelt);










