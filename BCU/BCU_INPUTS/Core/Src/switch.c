#include <switch.H>
#include "stm32l4xx_hal.h"
extern TIM_HandleTypeDef htim3;
extern uint8_t vehicle_state;
extern uint8_t countvar;

void sendMsg_KEYINFO(J1939_MESSAGE *Message)
{
	J1939_MESSAGE _msgKEYINFO;
	_msgKEYINFO=*Message;
	_msgKEYINFO.Mxe.DataPage = 0;
	_msgKEYINFO.Mxe.Priority = 0x01;
	_msgKEYINFO.Mxe.DestinationAddress = 0x33;//changed from 33 to 31//destination is all
	_msgKEYINFO.Mxe.DataLength = 8;
	_msgKEYINFO.Mxe.PDUFormat = 0xf1;
	_msgKEYINFO.Mxe.SourceAddress = 0x33;
	while (J1939_Send_Message(&_msgKEYINFO)!= RC_SUCCESS);
//	J1939_Poll( );


}


void sendMsg_MOTORCONTROLLERCOMMANDS(J1939_MESSAGE *Message)
{
	J1939_MESSAGE _msgMOTORCONTROLLER_COMMANDS;
	_msgMOTORCONTROLLER_COMMANDS=*Message;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.DataPage = 0;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.Priority = 0x03;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.DestinationAddress = 0x11;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.DataLength = 8;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.PDUFormat = 0x17;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.SourceAddress = 0x33;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.PGN=0x1700;
	while (J1939_Send_Message( &_msgMOTORCONTROLLER_COMMANDS) != RC_SUCCESS);
//	J1939_Poll( );
//	HAL_Delay(50);

}

void sendMsg_SWITCHINFO(J1939_MESSAGE *Message)
{

	J1939_MESSAGE _msgSWITCHINFO;
	_msgSWITCHINFO=*Message;
	_msgSWITCHINFO.Mxe.DataPage = 0;
	_msgSWITCHINFO.Mxe.Priority = 0x03;
	_msgSWITCHINFO.Mxe.DestinationAddress = 0x22;
	_msgSWITCHINFO.Mxe.DataLength = 8;
	_msgSWITCHINFO.Mxe.PDUFormat = 0x15;
	_msgSWITCHINFO.Mxe.SourceAddress = 0x33;
	while (J1939_Send_Message( &_msgSWITCHINFO) != RC_SUCCESS);
//	J1939_Poll( );
//	HAL_Delay(50);
}

void sendMsg_BMSSTATUS(J1939_MESSAGE *Message)
{
	J1939_MESSAGE _msgBMSSTATUS;
	_msgBMSSTATUS=*Message;
	_msgBMSSTATUS.Mxe.DataPage = 0;
	_msgBMSSTATUS.Mxe.Priority = 0x03;
	_msgBMSSTATUS.Mxe.DestinationAddress = 0x22;
	_msgBMSSTATUS.Mxe.DataLength = 8;
	_msgBMSSTATUS.Mxe.PDUFormat = 0xF4;
	_msgBMSSTATUS.Mxe.SourceAddress = 0x44;
	while (J1939_Send_Message( &_msgBMSSTATUS) != RC_SUCCESS);
//	J1939_Poll( );
//	HAL_Delay(50);

}

void sendMsg_MOTORCONTROLLERSTATUS(J1939_MESSAGE *Message)
{
	J1939_MESSAGE _msgMOTORCONTROLLERSTATUS;
	_msgMOTORCONTROLLERSTATUS =*Message;

	_msgMOTORCONTROLLERSTATUS.Mxe.DataPage = 0;
	_msgMOTORCONTROLLERSTATUS.Mxe.Priority = 0x03;
	_msgMOTORCONTROLLERSTATUS.Mxe.DestinationAddress = 0x33;
	_msgMOTORCONTROLLERSTATUS.Mxe.DataLength = 1;
	_msgMOTORCONTROLLERSTATUS.Mxe.PDUFormat = 0xF2;
	_msgMOTORCONTROLLERSTATUS.Mxe.SourceAddress = 0x11;

	while (J1939_Send_Message( &_msgMOTORCONTROLLERSTATUS) != RC_SUCCESS);
//	J1939_Poll( );
//	HAL_Delay(50);


}







void switches_call(void)
{

	/* BEGIN KILL SWITCH */

	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10))==0)
	{

		if(vehicle_state==1)

		{
			Switch_Infovar.killswitch=1;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
			_info4=*info;
			Message.Mxe.Data[0]=_info;

			sendMsg_SWITCHINFO(&Message);
			_info4=*info=0;

		}
	}
	else
	{
		Switch_Infovar.killswitch=0;
		j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
		_info4=*info;
		Message.Mxe.Data[0]=_info;

		sendMsg_SWITCHINFO(&Message);
		_info4=*info=0;



		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

	}

	/* END OF KILL SWITCH */

	/* BEGIN OF IGNITION */

	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))==0)
	{

		if(vehicle_state==1)
		{
			Motorcontroller_commandvar.ignition_command=1;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Motorcontroller_commandvar);
			_info2=*info;
			Message.Mxe.Data[0]=_info2;

			sendMsg_MOTORCONTROLLERCOMMANDS(&Message);

		}
	}
	else
	{
		Motorcontroller_commandvar.ignition_command=0;
		j1939_uint8_t *info=(j1939_uint8_t*)(&Motorcontroller_commandvar);
		_info2=*info;
		Message.Mxe.Data[0]=_info2;

		sendMsg_MOTORCONTROLLERCOMMANDS(&Message);



		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

	}

	/* END OF IGNITION */

	/* BEGIN OF NAVIGATION_5 */

	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8))==0)
	{
		//navigation_5  (centre 101=5)

		//		count9++;

		//		if((_info1==2)&&(count9%2==1))
		if(vehicle_state==1)
		{
			Switch_Infovar.navmid=1;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
			_info4 = *info;
			Message.Mxe.Data[0]=_info4;
			//Message.Mxe.Data[0]=0x04;
			sendMsg_SWITCHINFO(&Message);
			//_info=*info=0;
			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
		}
	}
	else
	{
		Switch_Infovar.navmid=0;
		j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
		_info4 = *info;
		Message.Mxe.Data[0]=_info4;
		//Message.Mxe.Data[0]=0x00;
		sendMsg_SWITCHINFO(&Message);
		//_info=*info=0;
		//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));


		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		//		if(count8>=254)
		//			count8=0;

	}
//	/* END OF NAVIGATION_5 */
//
//	/* BEGIN OF BRAKE LIGHT */
//
//	//	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7))==0)
//	//	{
//	//		//brake_light op
//	//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
//	//		HAL_Delay(100);
//	//
//	//	}
//
//	/* END OF BRAKE LIGHT */
//
//
	/* BEGIN OF NAVIGATION_4 *///bottom
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))==0)
	{
		//navigation_4  (bottom 100=4)
		//		count8++;

		//		if((_info1==2)&&(count8%2==1))
		if(vehicle_state==1)
		{
			Switch_Infovar.navtop=1;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
			_info4 = *info;
			Message.Mxe.Data[0]=_info4;
			//Message.Mxe.Data[0]=0x04;
			sendMsg_SWITCHINFO(&Message);
			//_info=*info=0;
			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
		}
	}
	else
	{
		Switch_Infovar.navtop=0;
		j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
		_info4 = *info;
		Message.Mxe.Data[0]=_info4;
		//Message.Mxe.Data[0]=0x00;
		sendMsg_SWITCHINFO(&Message);
		//_info=*info=0;
		//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));


		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		//		if(count8>=254)
		//			count8=0;

	}

	/* END OF NAVIGATION_4 */

	/* BEGIN OF NAVIGATION_3 *///left
	if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))==0)
	{
		//navigation_3  (top 011)
		//		count7++;

		//		if((_info1==2)&&(count7%2==1))
		if(vehicle_state==1)
		{
			Switch_Infovar.navright=1;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
			_info4 = *info;
			Message.Mxe.Data[0]=_info4;
			//Message.Mxe.Data[0]=0x04;
			sendMsg_SWITCHINFO(&Message);
			//_info=*info=0;
			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
		}
	}
	else
	{
		Switch_Infovar.navright=0;
		j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
		_info4 = *info;
		Message.Mxe.Data[0]=_info4;
		//Message.Mxe.Data[0]=0x00;
		//sendMsg_SWITCHINFO(&Message);
		//_info=*info=0;
		//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));


		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		//		if(count7>=254)
		//			count7=0;

	}

	/* END OF NAVIGATION_3 */
//
//	/* BEGIN OF NAVIGATION_2 */
	if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))==0)
	{
		//navigation_2  (010 - RIGHT)
		//		count6++;

		//		if((_info1==2)&&(count6%2==1))
		if(vehicle_state==1)
		{
			Switch_Infovar.navbot=1;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
			_info4 = *info;
			Message.Mxe.Data[0]=_info4;
			//Message.Mxe.Data[0]=0x04;
			sendMsg_SWITCHINFO(&Message);
			//_info=*info=0;
			//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		}
	}
	else
	{
		Switch_Infovar.navbot=0;
		j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
		_info4 = *info;
		Message.Mxe.Data[0]=_info4;
		//Message.Mxe.Data[0]=0x00;
		sendMsg_SWITCHINFO(&Message);
		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		//		if(count6>=254)
		//			count6=0;
	}

	/* END OF NAVIGATION_2 */
//
//	/* BEGIN OF NAVIGATION_1 */
	if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))==0)
	{
		//navigation_1  (left 001)
		//		count5++;
		//with key on condition rest switches work
		//		if((_info1==2)&&(count5%2==1))
		if(vehicle_state==1)
		{
			Switch_Infovar.navleft=1;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
			_info4 = *info;
			Message.Mxe.Data[0]=_info4;
			//Message.Mxe.Data[0]=0x04;
			sendMsg_SWITCHINFO(&Message);
			//_info=*info=0;
			//				memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
		}
	}
	else
	{
		Switch_Infovar.navleft=0;
		j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
		_info4 = *info;
		Message.Mxe.Data[0]=_info4;
		//Message.Mxe.Data[0]=0x00;
		sendMsg_SWITCHINFO(&Message);
		//_info=*info=0;
		//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));


		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));


	}

//	/* END OF NAVIGATION_1 */
//
//	/* BEGIN OF RIGHT INDICATOR */
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))==0)
	{
		//right_indicator
		//		count10++;
		//with key on condition rest switches work
		//				if((_info1==2)&&(count10%2==1))
		if(vehicle_state==1)
		{
			Switch_Infovar.rightindicator=1;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
			_info4 = *info;
			Message.Mxe.Data[0]=_info4;
			//Message.Mxe.Data[0]=0x04;
			sendMsg_SWITCHINFO(&Message);
			//_info=*info=0;
			//				memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,1);

		}
	}
	else
	{
		Switch_Infovar.rightindicator=0;
		j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
		_info4 = *info;
		Message.Mxe.Data[0]=_info4;
		//Message.Mxe.Data[0]=0x00;
		sendMsg_SWITCHINFO(&Message);
		//_info=*info=0;
		//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);


		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		//				if(count10>=254)
		//					count10=0;



	}

//	/* END OF RIGHT INDICATOR */

//
//	/* BEGIN OF FRONT BRAKE */
//	if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))==0)
//	{
//		//brake_front
//		//		count4++;
//		//		if((_info1==2)&&(count4%2==1))
//		if(vehicle_state==1)
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
//	}
//	if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))==1)
//	{
//		Motorcontroller_commandvar.brake=0;
//		j1939_uint8_t *info=(j1939_uint8_t*)(&Motorcontroller_commandvar);
//		_info2=*info;
//		Message.Mxe.Data[0]=_info2;
//		//Message.Mxe.Data[0]=0x00;
//		sendMsg_MOTORCONTROLLERCOMMANDS(&Message);
//		//				_info=*info=0;
//		//	memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,0);
//
//
//		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
//
//		//		if(count4>=254)
//		//			count4=0;
//	}
//
//	/* END OF FRONT BRAKE */
//
//	/* BEGIN OF HIGH BEAM */
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12))==0)
	{

		//send information
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,1);
		HAL_Delay(3000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,0);//remove this later and make use of count

	}

//	/* END OF HIGH BEAM */
//	/* BEGIN OF LEFT INDICATOR */
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))==0)
	{
		//		count11++;

		//						if((_info1==2)&&(count11%2==1))
		if(vehicle_state==1)
		{
			Switch_Infovar.leftindicator=1;
			j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
			_info4 = *info;
			Message.Mxe.Data[0]=_info4;
			//Message.Mxe.Data[0]=0x04;
			sendMsg_SWITCHINFO(&Message);
			//_info=*info=0;
			//				memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,1);
		}
	}
	else
	{
		Switch_Infovar.leftindicator=0;
		j1939_uint8_t *info=(j1939_uint8_t*)(&Switch_Infovar);
		_info4 = *info;
		Message.Mxe.Data[0]=_info4;
		//Message.Mxe.Data[0]=0x00;
		sendMsg_SWITCHINFO(&Message);
		//_info=*info=0;
		//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,0);//PD2 OP


		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		//						if(count11>=254)
		//							count11=0;

	}

//	/* END OF LEFT INDICATOR */

}


