# ifndef __switch_H
#define __switch_H

#include "J1939.H"
#include "J1939_Config.H"
#include <string.h>


void switches_call(void);
void sendMsg_KEYINFO(J1939_MESSAGE *Message);
void sendMsg_SWITCHINFO(J1939_MESSAGE *Message);
void sendMsg_MOTORCONTROLLERCOMMANDS(J1939_MESSAGE *Message);
void sendMsg_BMSSTATUS(J1939_MESSAGE *Message);
void sendMsg_MOTORCONTROLLERSTATUS(J1939_MESSAGE *Message);



struct Key_Info
{

	j1939_uint8_t Key_Off:1;
	j1939_uint8_t Key_In:1;
}Key_Infovar;

 struct Motorcontroller_command
{
	j1939_uint8_t ignition_command:2;
	j1939_uint8_t brake:1;

}Motorcontroller_commandvar;

 struct Switch_Info
{

	j1939_uint8_t brake:1;
	j1939_uint8_t highbeam:1;
	j1939_uint8_t killswitch:1;
	j1939_uint8_t leftindicator:1;
	j1939_uint8_t rightindicator:1;
	j1939_uint8_t midindicator:1;
	j1939_uint8_t ignition:1;
	j1939_uint8_t navbot:1;
	j1939_uint8_t navleft:1;
	j1939_uint8_t navmid:1;
	j1939_uint8_t navright:1;
	j1939_uint8_t navtop:1;

}Switch_Infovar;

struct BMS_Status
{
	j1939_uint8_t BMS_Status:1;
	j1939_uint8_t SOC;

}BMS_Statusvar;

 struct Motorcontroller_Status
{
	j1939_uint8_t inverter_temperature;
	j1939_uint8_t motor_temperature;
	j1939_uint8_t motorcontroller_state:1;
	j1939_uint8_t ridemode_status:4;
	j1939_uint16_t speed;
	j1939_uint8_t startstop_status:1;

}Motorcontroller_Statusvar;

 struct Ride_Command
{

	j1939_uint8_t Ridemode_command:4;

}Ride_Commandvar;

 struct VCU_Status
{
	j1939_uint8_t VCU_State:1;


}VCU_Statusvar;



struct Key_Info Key_Infovar;
struct Motorcontroller_Status Motorcontroller_Statusvar;
struct Switch_Info Switch_Infovar;
struct Motorcontroller_command Motorcontroller_commandvar;
j1939_uint8_t _info;
j1939_uint8_t _info1;
j1939_uint8_t _info2;
j1939_uint8_t _info3;
j1939_uint8_t _info4;//used for switch information
j1939_uint8_t _info5;//bms
j1939_uint8_t _info6;//vcu
j1939_uint8_t _info7;//mc





J1939_MESSAGE Message;


# endif
