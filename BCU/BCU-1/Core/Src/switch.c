#include <switch.H>
#include "stm32l4xx_hal.h"
extern TIM_HandleTypeDef htim2;
extern uint8_t vehicle_state;
//#include "J1939.H"
//#include "J1939_Config.H"



void switches_call(void)
{

	/* BEGIN KEY INFO*/

	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11))==0)// changing c11 to pa1 change later

	{
		vehicle_state=1;
		HAL_TIM_Base_Start_IT(&htim2);

		count1++;//odd num on even off the key

		if(count1%2==1)
		{
			count=count1;

			Key_Infovar.Key_In=1;
			Key_Infovar.Key_Off=0;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Key_Infovar);
			_info1=*info;
			Message.Mxe.Data[0]=_info1;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,1);


			//Message.Mxe.Data[0]=0x02;
			//__HAL_TIM_ENABLE(&htim2);
			//HAL_TIM_Base_Start_IT(&htim2);
			//_info=*info=0;

		}
		else//((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))==1)

		{
			Key_Infovar.Key_In=0;
			Key_Infovar.Key_Off=1;

			j1939_uint8_t *info=(j1939_uint8_t*)(&Key_Infovar);
			_info1 = *info;
			Message.Mxe.Data[0]=_info1;
			//_info1=*info=0;
			//Message.Mxe.Data[0]=0x01;
			//sendMsg_KEYINFO(&Message);
			memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

			/* BMS STATUS IS OFF */
			BMS_Statusvar.BMS_Status=0;
			 *info=(j1939_uint8_t*)(&BMS_Statusvar);
			_info5 = *info;
			Message.Mxe.Data[0]=0;
			Message.Mxe.Data[1]=_info5;
			sendMsg_BMSSTATUS(&Message);
			memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

			/* VCU STATE IS OFF */
			VCU_Statusvar.VCU_State=0;
			*info=(j1939_uint8_t*)(&VCU_Statusvar);
			_info6 = *info;
			Message.Mxe.Data[0]=_info6;
			sendMsg_VCUSTATUS(&Message);
			memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

			/* MC STATE IS OFF */

			Motorcontroller_Statusvar.motorcontroller_state=0;
			*info=(j1939_uint8_t*)(&Motorcontroller_Statusvar);
			_info7 = *info;
			Message.Mxe.Data[0]=_info7;
			sendMsg_MOTORCONTROLLERSTATUS(&Message);
			memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,0);


		}
		if(count1>=254)
			count1=0;

	}
}

	/* END OF KEY INFO*/

	/* BEGIN KILL SWITCH */
//
//	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10))==1)
//	{
//		count2++;
//
//		//kill switch
//		//with key on condition rest switches work
//		if((_info1==2)&&(count2%2==1))
//		{
//			Switch_Infovar.killswitch=1;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info=*info;
//			Message.Mxe.Data[0]=_info;
//			//Message.Mxe.Data[0]=0x04;
//			sendMsg_SWITCHINFO(&Message);
//			_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//		else
//		{
//			Switch_Infovar.killswitch=0;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info=*info;
//			Message.Mxe.Data[0]=_info;
//			//Message.Mxe.Data[0]=0x00;
//			sendMsg_SWITCHINFO(&Message);
//			_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//
//		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		if(count2>=254)
//			count2=0;
//
//	}
//
//	/* END OF KILL SWITCH */
//
//	/* BEGIN OF IGNITION */
//
//	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))==1)
//	{
//		//ignition
//		count3++;
//		if((_info1==2)&&(count3%2==1))
//		{
//			Motorcontroller_commandvar.ignition_command=1;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Motorcontroller_commandvar);
//			_info2=*info;
//			Message.Mxe.Data[0]=_info2;
//			//Message.Mxe.Data[0]=0x01;
//			sendMsg_MOTORCONTROLLERCOMMANDS(&Message);
//			//_info=*info=0;
//			//	memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//		else
//		{
//			Motorcontroller_commandvar.ignition_command=0;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Motorcontroller_commandvar);
//			_info2=*info;
//			Message.Mxe.Data[0]=_info2;
//			//Message.Mxe.Data[0]=0x00;
//			sendMsg_MOTORCONTROLLERCOMMANDS(&Message);
//			//				_info=*info=0;
//			//	memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//
//		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		if(count3>=254)
//			count3=0;
//
//	}
//
//	/* END OF IGNITION */
//
//	/* BEGIN OF NAVIGATION_5 */
//
//	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8))==1)
//	{
//		//navigation_5  (centre 101=5)
//
//		count9++;
//
//		if((_info1==2)&&(count9%2==1))
//		{
//			Switch_Infovar.navigation=5;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x04;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//		else
//		{
//			Switch_Infovar.navigation=0;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x00;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//
//		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		if(count8>=254)
//			count8=0;
//
//	}
//	/* END OF NAVIGATION_5 */
//
//	/* BEGIN OF BRAKE LIGHT */
//
//	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7))==1)
//	{
//		//brake_light op
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
//		HAL_Delay(100);
//
//	}
//
//	/* END OF BRAKE LIGHT */
//
//
//	/* BEGIN OF NAVIGATION_4 */
//	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))==1)
//	{
//		//navigation_4  (bottom 100=4)
//		count8++;
//
//		if((_info1==2)&&(count8%2==1))
//		{
//			Switch_Infovar.navigation=4;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x04;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//		else
//		{
//			Switch_Infovar.navigation=0;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x00;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//
//		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		if(count8>=254)
//			count8=0;
//
//	}
//
//	/* END OF NAVIGATION_4 */
//
//	/* BEGIN OF NAVIGATION_3 */
//	if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))==1)
//	{
//		//navigation_3  (top 011)
//		count7++;
//
//		if((_info1==2)&&(count7%2==1))
//		{
//			Switch_Infovar.navigation=3;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x04;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//		else
//		{
//			Switch_Infovar.navigation=0;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x00;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//
//		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		if(count7>=254)
//			count7=0;
//
//	}
//
//	/* END OF NAVIGATION_3 */
//
//	/* BEGIN OF NAVIGATION_2 */
//	if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))==1)
//	{
//		//navigation_2  (010 - RIGHT)
//		count6++;
//
//		if((_info1==2)&&(count6%2==1))
//		{
//			Switch_Infovar.navigation=2;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x04;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		}
//		else
//		{
//			Switch_Infovar.navigation=0;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x00;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//
//		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		if(count6>=254)
//			count6=0;
//	}
//
//	/* END OF NAVIGATION_2 */
//
//	/* BEGIN OF NAVIGATION_1 */
//	if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))==1)
//	{
//		//navigation_1  (left 001)
//		count5++;
//		//with key on condition rest switches work
//		if((_info1==2)&&(count5%2==1))
//		{
//			Switch_Infovar.navigation=1;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x04;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//				memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//		else
//		{
//			Switch_Infovar.navigation=0;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//			_info4 = *info;
//			Message.Mxe.Data[0]=_info4;
//			//Message.Mxe.Data[0]=0x00;
//			sendMsg_SWITCHINFO(&Message);
//			//_info=*info=0;
//			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		}
//
//		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		if(count5>=254)
//			count5=0;
//
//	}
//
//	/* END OF NAVIGATION_1 */
//
//	/* BEGIN OF RIGHT INDICATOR */
//	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))==1)
//	{
//		//right_indicator
//		count10++;
//				//with key on condition rest switches work
//				if((_info1==2)&&(count10%2==1))
//				{
//					Switch_Infovar.indicator=2;
//					j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//					_info4 = *info;
//					Message.Mxe.Data[0]=_info4;
//					//Message.Mxe.Data[0]=0x04;
//					sendMsg_SWITCHINFO(&Message);
//					//_info=*info=0;
//					//				memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,1);
//				}
//				else
//				{
//					Switch_Infovar.indicator=0;
//					j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//					_info4 = *info;
//					Message.Mxe.Data[0]=_info4;
//					//Message.Mxe.Data[0]=0x00;
//					sendMsg_SWITCHINFO(&Message);
//					//_info=*info=0;
//					//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
//				}
//
//				memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//				if(count10>=254)
//					count10=0;
//
//
//
//	}
//
//	/* END OF RIGHT INDICATOR */
//
//	/* BEGIN OF INDICATOR MIDDLE */
//	if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))==1)
//	{
//		//indicator_middle
//		count11++;
//						//with key on condition rest switches work
//						if((_info1==2)&&(count11%2==1))
//						{
//							Switch_Infovar.indicator=3;
//							j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//							_info4 = *info;
//							Message.Mxe.Data[0]=_info4;
//							//Message.Mxe.Data[0]=0x04;
//							sendMsg_SWITCHINFO(&Message);
//							//_info=*info=0;
//							//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,1);
//						}
//						else
//						{
//							Switch_Infovar.indicator=0;
//							j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//							_info4 = *info;
//							Message.Mxe.Data[0]=_info4;
//							//Message.Mxe.Data[0]=0x00;
//							sendMsg_SWITCHINFO(&Message);
//							//_info=*info=0;
//							//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,0);//PA0
//						}
//
//						memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//						if(count11>=254)
//							count11=0;
//
//	}
//	/* END OF INDICATOR MIDDLE */
//
//	/* BEGIN OF FRONT BRAKE */
//	if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))==1)
//	{
//		//brake_front
//		count4++;
//		if((_info1==2)&&(count4%2==1))
//		{
//			Motorcontroller_commandvar.brake=1;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Motorcontroller_commandvar);
//			_info2=*info;
//			Message.Mxe.Data[0]=_info2;
//			//Message.Mxe.Data[0]=0x01;
//			sendMsg_MOTORCONTROLLERCOMMANDS(&Message);
//			//_info=*info=0;
//			//	memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,1);
//		}
//		else
//		{
//			Motorcontroller_commandvar.brake=0;
//			j1939_uint8_t *info=(j1939_uint8_t*)(&Motorcontroller_commandvar);
//			_info2=*info;
//			Message.Mxe.Data[0]=_info2;
//			//Message.Mxe.Data[0]=0x00;
//			sendMsg_MOTORCONTROLLERCOMMANDS(&Message);
//			//				_info=*info=0;
//			//	memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,0);
//
//		}
//		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		if(count4>=254)
//			count4=0;
//	}
//
//	/* END OF FRONT BRAKE */
//
//	/* BEGIN OF HIGH BEAM */
//	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12))==1)
//	{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,1);
//
//	}
//
//	/* END OF HIGH BEAM */
///* BEGIN OF LEFT INDICATOR */
//	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))==1)
//	{
//		count11++;
//
//						if((_info1==2)&&(count11%2==1))
//						{
//							Switch_Infovar.indicator=1;
//							j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//							_info4 = *info;
//							Message.Mxe.Data[0]=_info4;
//							//Message.Mxe.Data[0]=0x04;
//							sendMsg_SWITCHINFO(&Message);
//							//_info=*info=0;
//							//				memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,1);
//						}
//						else
//						{
//							Switch_Infovar.indicator=0;
//							j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
//							_info4 = *info;
//							Message.Mxe.Data[0]=_info4;
//							//Message.Mxe.Data[0]=0x00;
//							sendMsg_SWITCHINFO(&Message);
//							//_info=*info=0;
//							//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,1);//PD2 OP
//						}
//
//						memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//						if(count11>=254)
//							count11=0;
//
//	}
//
//	/* END OF LEFT INDICATOR */
//
//
//	/* BEGIN OF HIGH  BEAM OP */
//
//
//
//
//	HAL_Delay(100);
//	J1939_Poll();
//
//
//}
