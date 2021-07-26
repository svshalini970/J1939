#include "fun.h"

#include "J1939.H"

#include "J1939_Config.H"


void can_fil_config()
{
		fil.FilterIdHigh=0x0000;
	  	fil.FilterIdLow=0x0000;
	  	fil.FilterMaskIdHigh=0x0000;
	  	fil.FilterMaskIdLow=0x0000;
	  	fil.FilterFIFOAssignment=0;
	  	fil.FilterBank=0;
	  	fil.FilterMode=CAN_FILTERMODE_IDMASK;
	  	fil.FilterScale=CAN_FILTERSCALE_32BIT;
	  	fil.FilterActivation=CAN_FILTER_ENABLE;
	  	fil.SlaveStartFilterBank=0;

	  	if (HAL_CAN_ConfigFilter(&hcan1, &fil) != HAL_OK)
	  		    {
	  		        /* Filter configuration Error */
	  		        HAL_UART_Transmit(&huart3,"error in configfil\n\r", 20, 1000);
	  		        Error_Handler();
	  		    }
}

/* WHEN IT ENABLED */
/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&rxhead, arr) != HAL_OK)
    {

    	HAL_UART_Transmit(&huart3,"error in reception\n\r", 20, 1000);
    	Error_Handler();
    }
	J1939_ReceiveMessages();
	 //readMsg();

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t mssg[25];

			HAL_UART_Transmit(&huart3, "callbck_error\n\r", 25, 1000);
			memset(mssg,0,25);

}
*/





