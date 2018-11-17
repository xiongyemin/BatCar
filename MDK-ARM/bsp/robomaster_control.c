/**
  *@file robomaster_control.c
  *@date 2018-10-5
  *@author izh20
  *@brief 
  */


#include "robomaster_control.h"
#include "robomaster_common.h"


void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty)
	{
	
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = (10000*duty) - 1;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = (10000*duty) - 1;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = (10000*duty) - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = (10000*duty) - 1;break;
	}
	
}

void shoot_control()
{
	if(remote_control.switch_left!=3)
		{
				if(remote_control.switch_right==3)
				{
				  motor_pid[6].target=4000;
					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.14);
				}
				if(remote_control.switch_right==2)
				{	
					motor_pid[6].target=7000;
					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					
					set_yuntai_current(&hcan1,0,0,motor_pid[6].output,0);
				}
				if(remote_control.switch_right==1)
				{
					set_yuntai_current(&hcan1,0,0,0,0);
					init_TIM5_PWM();
				}

		}else
		{
			set_yuntai_current(&hcan1,0,0,0,0);
			init_TIM5_PWM();
		}
}


extern int16_t moto_ctr[6];
int32_t set_spd = 0;//�ٶȲ���
int32_t turn=0;     //ת��
extern int cnt1;
extern int cnt2;
void chassis_control()
{
	if(remote_control.switch_left!=3)
	{
	if(cnt1==100)//0.5s����һ�Σ�ʹ��4��led��2HZƵ����˸���жϵ��̳�����������
		{
				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_4);
				cnt1=0;
		}
		DBUS_Deal();//��ȡң���������ݲ������ݸ�ֵ�������Ŀ��ת��
	motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //�����趨ֵ����PID���㡣
	motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //�����趨ֵ����PID���㡣
	motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //�����趨ֵ����PID���㡣
	motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //�����趨ֵ����PID���㡣
	
	set_moto_current(&hcan1,motor_pid[0].output,   //��PID�ļ�����ͨ��CAN���͵����
														motor_pid[1].output,
														motor_pid[2].output,
														motor_pid[3].output);
	
	
	}
		else{
		if(cnt2<=200){
			motor_pid[0].target=1000;
			motor_pid[1].target=1000;
			motor_pid[2].target=1000;
			motor_pid[3].target=1000;
			cnt2++;
		}
		else{
			motor_pid[0].target=-1000;
			motor_pid[1].target=-1000;
			motor_pid[2].target=-1000;
			motor_pid[3].target=-1000;
			cnt2++;
		}
		if(cnt2>=400)cnt2=0;
		motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);  
		motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);   
		motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);   
		motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);   
		//set_moto_current(&hcan1,0,0,0,0);
	}
}
/******************************************************
                      ���̵������2017(��һ��)

1, �����������̬�����
2��������̬�ĵ���
3��׼��CAN���������
4����ʼ�������
__________________________________________________________________
| ���λ�� |�����| ���\���� | ǰ | �� | �� | �� | ˳ | �� | ͣ | 													ǰ			��
``````````````````````````````````````````````````````````````````
|   ��ǰ   |0X201 |moto_ctr[0]| >0 | <0 | <0 | >0 | >0 | <0 | =0 |      									>0	|		<0
``````````````````````````````````````````````````````````````````	
|   ��ǰ   |0X202 |moto_ctr[1]| <0 | >0 | <0 | >0 | >0 | <0 | =0 |												<0	|		>0
``````````````````````````````````````````````````````````````````   //2018.10.6 �޸� 												2018.10.7�������ԣ�ԭ��û���⣩
|   ���   |0X203 |moto_ctr[2]| <0 | >0 | >0 | <0 | >0 | <0 | =0 |												>0  |		<0
``````````````````````````````````````````````````````````````````	
|   �Һ�   |0X204 |moto_ctr[3]| >0 | <0 | >0 | <0 | >0 | <0 | =0 |												<0	|		>0
``````````````````````````````````````````````````````````````````
|  ��ע:|moto_ctr[0]|=|moto_ctr[1]|=|moto_ctr[2]|=|moto_ctr[3]|  |
``````````````````````````````````````````````````````````````````											
*************************** ***************************/
static float record=0.0;
void walk_straight(void){
	if(moto_ctr[0]>0&&moto_ctr[1]>0&&moto_ctr[2]>0&&moto_ctr[3]>0){
			record=imu.yaw;
	}else if(moto_ctr[0]<0&&moto_ctr[1]<0&&moto_ctr[2]<0&&moto_ctr[3]<0){
	}
}