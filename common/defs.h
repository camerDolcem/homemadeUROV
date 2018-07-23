/***********************************************************************************
 Name:
     defs.h
 Description:
     Misc definitions
 Version:
     01
 Created:
	2017
 By:
	Jakub Kurkowski
***********************************************************************************/

//type definitions
typedef boolean					bool;
typedef	unsigned int			uint16;
typedef unsigned long			uint32;

//general definitions
#define TRUE					true
#define FALSE					false

//physical constants
#define RHO 					1023.6f 	//density of sea water [kg/m^3]
#define G 						9.81f		//Gravitational acceleration [m/s^2]

//serial comms bitrate for both microcontrollers
#define BITRATE 				250000
#define SERIAL_SETTINGS			SERIAL_8E1	//SERIAL_8N1 - default

//update intervals
#define RUNTIME_UPDATE			1000	//1sec
#define TIME_UPDATE				15000	//15sec
#define MSRMTS_TOPSIDE_UPDATE	2500
#define MSRMTS_SUBSEA_UPDATE	2500
#define SERVO_STOP				500
//the below should not overlap
#define SEND_MSRMTS_INTERVAL	1300
#define SEND_CMDS_INTERVAL		4005

//watchdog time constants
#define ZERO_DATA_DELAY			10000	//10sec
#define ZERO_DATA_BEEP			3000	//3sec
#define RECOVERY_DELAY			30000	//30sec
#define STOP_DELAY				10000	//10sec

//joystick default position mapped readings
#define FRWRD_BCKWRD_DEFAULT 	124
#define LEFT_RIGHT_DEFAULT 		131
#define FLUCTUATION_ZERO	 	10
#define FLUCTUATION_NONZERO		2

//default motors and servo commands
#define PWM_CMD_DEFAULT			1385	//motors stopped (ESC middle range)
#define SERVO_CMD_DEFAULT		80		//deg
