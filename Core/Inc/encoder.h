#include "usart.h"	 
#include "datacheck.h"

#define EncoderReadMin  2084864   //编码器读取最小值 -360 360对应14336
#define EncoderReadMax  2113536   //编码器读取最大值 +360

#define EncoderRunMin  2093824   //编码器允许最大限位
#define EncoderRunMax  2104576   //编码器允许最大限位

#define EncoderZero 2099200            //编码器零度值
#define EncoderAngle90 2102784         //编码器90度值
#define Angle_To_Encoder 39.8222       //每度代表多少值  360度 14336
#define TenthAngle_To_Encoder 3.98222  //每0.1度代表多少值
#define Encoder_To_Angle 0.025111      //编码器值转换为度数   


typedef  struct
{
	int32_t	val;        //编码器值       顺时针加，逆时钟减    后
	uint8_t		dir;        //旋转方向 01顺时针02逆时针
	uint16_t	rpm;        //RPM
	uint16_t  update;     //数据更新次数
	int16_t   zeroDelt;   //0度偏转矫正值
}Encoder;

extern Encoder Encoder_Front;   //前轮编码器
extern Encoder Encoder_Back;    //后轮编码器
extern Encoder Encoder_Axle;    //轴距编码器


extern uint32_t M,N;
void Encoder_Send(void);
void EncoderSendWithCRC(void);
void Usart3Receive_encoder_IDLE(void);
void InitEncoder(Encoder * ec, int16_t zeroDelt);










