/***********************************************************************************
	Name:
		defs.h
	Description:
		Types and constants definitions.
	Version:
		1.0
	Created:
		Jul 2018
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

//cursor positions on the display			
#define SECTIONS_X				0				//the above two			
#define TOPSIDE_Y				10				//e.g. 'surface:'		
#define SUBSEA_Y				TOPSIDE_Y + 52	//e.g. 'UROV:'
#define MAX_Y					TOPSIDE_Y + 122	//'Max:'

#define MSRMNTS_X				12				//air and water measurements
#define AIR_TEMP_Y				TOPSIDE_Y + 12 		
#define AIR_PRESS_Y				AIR_TEMP_Y + 21	

#define WATER_TEMP_Y			SUBSEA_Y + 12
#define WATER_PRESS_Y			WATER_TEMP_Y + 21
#define DEPTH_Y					WATER_PRESS_Y + 21

#define MAX_DEPTH_Y				MAX_Y + 10

#define TIME_X					12
#define TIME_Y					0

#define DATE_X					TIME_X + 46
#define DATE_Y					TIME_Y

#define RUNTIME_X				12				//e.g. 'Runtime:'
#define RUNTIME_Y				153				

#define DEG_AIR_X				MSRMNTS_X + 74	//'o' for air temp		
#define DEG_AIR_Y				AIR_TEMP_Y - 2
#define DEG_WATER_X				DEG_AIR_X		//'o' for water temp
#define DEG_WATER_Y				WATER_TEMP_Y - 2
#define HPA_X					DEG_AIR_X		//'hPa'
#define HPA_Y					AIR_PRESS_Y
#define	BAR_X					DEG_AIR_X		//'bar'
#define BAR_Y					WATER_PRESS_Y
#define	METER_X					DEG_AIR_X		//'m'
#define	METER_Y					DEPTH_Y
#define	MAX_METER_X				52				//'m'
#define	MAX_METER_Y				MAX_DEPTH_Y