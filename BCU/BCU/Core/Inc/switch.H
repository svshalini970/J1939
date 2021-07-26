# ifndef __switch_H
#define __switch_H

#include "J1939.H"
#include "J1939_Config.H"
#include <string.h>


void switches_call(void);
void sendMsg_KEYINFO(J1939_MESSAGE *Message);
void sendMsg_SWITCHINFO(J1939_MESSAGE *Message);
void sendMsg_MOTORCONTROLLERCOMMANDS(J1939_MESSAGE *Message);


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



uint8_t flag1;
uint8_t flag2;
uint8_t cnt1;
uint8_t count;
uint8_t count1;
uint8_t count2;
uint8_t count3;
uint8_t count4;
uint8_t count5;
uint8_t count6;
uint8_t count7;
uint8_t count8;
uint8_t count9;
uint8_t count10;
uint8_t count11;
J1939_MESSAGE Message;


# endif
