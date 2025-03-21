#ifndef BMS_H
#define BMS_H

#include "datacheck.h"


typedef __packed struct
{
	uint16_t	voltage;//��ѹ  
	int16_t current;//����:����Ϊ����ʾ�ŵ磬����Ϊ����ʾ���
    uint16_t capacity_per;//�����ٷֱ� SOC
	uint16_t Max_unit_voltage;//��ߵ����ѹ
	uint16_t Min_unit_voltage;//��͵����ѹ
	int16_t Max_unit_Temperature;//��ߵ����¶�
	int16_t Min_unit_Temperature;//��͵����¶�
	uint16_t Alarm;//��ر���:��ظ��±��������SOC�ͱ���
				
}battery_value_t;

typedef __packed struct
{
	uint8_t SOC;//ʣ������ٷֱ�
	uint8_t SOH;//��ؽ���״̬�ٷֱ�
	uint8_t sysVol;//�ܵ�ѹ
	uint8_t sysCurt;//�ܵ���
	uint32_t TIME1;//ʣ��ŵ�ʱ��
	uint32_t TIME2;//ʣ����ʱ��
	uint32_t curtState;//����״̬�жϳ�ŵ�
	uint32_t chrgPlugIn;//���������״̬
	uint32_t state0;//�ŵ翪��״̬
	uint32_t state1;//��翪��״̬
	uint32_t state2;//���������״̬
	uint32_t alarm1;//�ܵ�ѹ��ѹ����
	uint32_t alarm2;//�ܵ�ѹǷѹ����
	uint32_t alarm3;//soc�;���
	uint32_t alarm4;//ͨ��״̬����
	  //	uint8_t alarm4;//
  //	uint8_t alarm6;//�ŵ��������
  //	uint8_t alarm7;//����������
  //	uint8_t alarm12;//MOS��·�¶ȹ��߾���
  //	uint8_t CC1;//�ŵ��·״̬0���Ͽ�1������
  //	uint8_t CC2;//Ԥ���·״̬
  //	uint8_t CC3;//����·״̬




}battery_value_t1;

//		uint16_t voltage;//��ѹ
//		int16_t current;//����	
//		uint16_t capacity_per;//�����ٷֱ�
//		uint16_t Total_hours;//������ʱ��
//		uint16_t cycle_count;//ѭ������
//		uint16_t Max_unit_voltage;//��ߵ����ѹ
//		uint16_t MaxV_Location2_1;//λ��2 λ��1
//		uint16_t Min_unit_voltage;//��͵����ѹ
//		uint16_t MinV_Location2_1;//λ��2 λ��1
//		int16_t Max_unit_Temperature;//��ߵ����¶�
//		uint16_t MaxT_location2_1;//λ��2 λ��1 
//		int16_t Min_unit_Temperature;//��͵����¶�
//		uint16_t MinT_Location2_1;//λ��2 λ��1
//		uint16_t Ta_Det;//������
//		uint16_t Ta_Present;//�������
//		uint16_t Cha_Discha_Pro_Sta;//��籣��״̬ �ŵ㱣��״̬
//		uint8_t Cha_Relay_Sta;//���̵���״̬ 
//		uint8_t Discha_Relay_Sta;//�ŵ�̵���״̬
//		uint16_t ALRN;//�澯��Ϣ����
//		uint16_t Alarm_Type;//�澯����	
//		uint8_t PC_Cha_Command;//�澯����	
//		uint8_t PC_Cha_Flag;//�澯����	


extern battery_value_t battery_t;
 
void UART4_Send(uint8_t c);
void BMSSendWithCRC(void);
void Usart4Receive_BMS_IDLE(void);
long Hex2Dec_32(uint32_t temp);
int Read_parameter1(void);
int Read_parameter2(void);
int Read_parameter3(void);

#endif

