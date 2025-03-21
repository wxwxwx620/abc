#include "chassis.h"
#include "main.h"
#include "can.h"
#include "math.h"
#include "cmsis_os.h"


#define PC_PROTOCOL_TX_LENGTH 29
#define PROTOCOL_RX_LENGTH 18
#define LIFT_MOTOR_SPEED (1500)

uint8_t protocol_send_buffer[PC_PROTOCOL_TX_LENGTH];

int16_t motor_target_speed[13];
int16_t motor_actual_speed_rpm[9];
int16_t test = 0;

uint16_t remoter_time_out_cnt = 20;
uint8_t remoter_time_out_flag = 0;            //1

uint8_t lift_up_flag,lift_down_flag,lift_stop_flag;

float target_direction = 90;
float target_velocity = 200;
float target_palstance = 0;

uint8_t body_stop_flag;

uint8_t remotor_stop_flag;


void MotorEnable(void)
{
	int i;
	static CAN_TxHeaderTypeDef 	TxMessage;
	static uint8_t      		can_send_data[8];
	uint32_t 					send_mail_box;
	for(i=1;i<13;i++)
	{
		TxMessage.StdId = i;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		can_send_data[0] = 0x00;
		can_send_data[1] = 0x00;
		can_send_data[2] = 0x0a;
		can_send_data[3] = 0x00;
		can_send_data[4] = 0x00;
		can_send_data[5] = 0x00;
		can_send_data[6] = 0x00;
		can_send_data[7] = 0x00;
		
		HAL_CAN_AddTxMessage(&hcan1, &TxMessage, can_send_data, &send_mail_box);
		
		osDelay(1);
	}
	osDelay(2);
}

void MotorSpeedSet(void)
{
	int i;
	static CAN_TxHeaderTypeDef 	TxMessage;
	static uint8_t      		can_send_data[8];
	uint32_t 					send_mail_box;

	for(i=1;i<4;i++)
	{
		TxMessage.StdId = i;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		can_send_data[0] = 0x03;
		can_send_data[1] = motor_target_speed[i] >> 8;
		can_send_data[2] = motor_target_speed[i];
		can_send_data[3] = 0x00;
		can_send_data[4] = 0x00;
		can_send_data[5] = 0x00;
		can_send_data[6] = 0x00;
		can_send_data[7] = 0x00;
		HAL_CAN_AddTxMessage(&hcan1, &TxMessage, can_send_data, &send_mail_box);
	}
	osDelay(2);

	for(i=4;i<7;i++)
	{
		TxMessage.StdId = i;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		can_send_data[0] = 0x03;
		can_send_data[1] = motor_target_speed[i] >> 8;
    can_send_data[2] = motor_target_speed[i];
		can_send_data[3] = 0x00;
		can_send_data[4] = 0x00;
		can_send_data[5] = 0x00;
		can_send_data[6] = 0x00;
		can_send_data[7] = 0x00;
		HAL_CAN_AddTxMessage(&hcan1, &TxMessage, can_send_data, &send_mail_box);
		osDelay(2);
	}
	for(i=7;i<9;i++)
	{
		TxMessage.StdId = i;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		can_send_data[0] = 0x03;
		can_send_data[1] = motor_target_speed[i] >> 8;
		can_send_data[2] = motor_target_speed[i];
		can_send_data[3] = 0x00;
		can_send_data[4] = 0x00;
		can_send_data[5] = 0x00;
		can_send_data[6] = 0x00;
		can_send_data[7] = 0x00;
		HAL_CAN_AddTxMessage(&hcan1, &TxMessage, can_send_data, &send_mail_box);
		osDelay(2);
	}
	for(i=9;i<13;i++)
	{
		TxMessage.StdId = i;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		can_send_data[0] = 0x03;
		can_send_data[1] = motor_target_speed[i] >> 8;
		can_send_data[2] = motor_target_speed[i];
		can_send_data[3] = 0x00;
		can_send_data[4] = 0x00;
		can_send_data[5] = 0x00;
		can_send_data[6] = 0x00;
		can_send_data[7] = 0x00;
		HAL_CAN_AddTxMessage(&hcan1, &TxMessage, can_send_data, &send_mail_box);
		osDelay(2);
	}
}

void MotorControl(void)
{
	double	target_linear_velocity[9];
	double	direction,velocity,palstance,vx,vy;

	direction = target_direction;
  velocity  = target_velocity;
	palstance = target_palstance;

	direction = direction * 3.1415926/180.0;
	palstance = palstance * 3.1415926/180.0;
	vx = cos(direction) * velocity;
	vy = sin(direction) * velocity;

	target_linear_velocity[3] = (vy - vx + palstance * 1553.5);
	target_linear_velocity[1] =  vy + vx - palstance * 1553.5;
	target_linear_velocity[2] =  vy - vx - palstance * 1553.5;
	target_linear_velocity[4] = (vy + vx + palstance * 1553.5);
	target_linear_velocity[7] = (vy - vx + palstance * 2163.5);
	target_linear_velocity[5] =  vy + vx - palstance * 2163.5;
	target_linear_velocity[6] =  vy - vx - palstance * 2163.5;
	target_linear_velocity[8] = (vy + vx + palstance * 2163.5);

	if((body_stop_flag + remotor_stop_flag + remoter_time_out_flag) == 0)
	{
		motor_target_speed[1] = target_linear_velocity[1]   / 0.5233;
		motor_target_speed[2] = target_linear_velocity[2]   / 0.5233;
		motor_target_speed[3] = -(target_linear_velocity[3] / 0.5233);
		motor_target_speed[4] = -(target_linear_velocity[4] / 0.5233);
		motor_target_speed[5] = target_linear_velocity[5] 	/ 0.5233;
		motor_target_speed[6] = target_linear_velocity[6] 	/ 0.5233;
		motor_target_speed[7] = -(target_linear_velocity[7] / 0.5233);
		motor_target_speed[8] = -(target_linear_velocity[8] / 0.5233);
	}
	else
	{
		motor_target_speed[1] = motor_target_speed[2] = 
		motor_target_speed[3] = motor_target_speed[4] = 
		motor_target_speed[5] = motor_target_speed[6] = 
		motor_target_speed[7] = motor_target_speed[8] = 0;

		motor_target_speed[9] = motor_target_speed[10] = 
		motor_target_speed[11] = motor_target_speed[12] = 0;
	}
}


