/***********************************************************************************
	Name:
		defs.h
	Description:
		Types and constants definitions.
	Version:
		1.0
	Created:
		Aug 2018
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
#define FLUCTUATION_ZERO	 	7
#define FLUCTUATION_NONZERO		2

//default motors and servo commands
#define PWM_CMD_DEFAULT			1385	//motors stopped (ESC middle range)
#define SERVO_CMD_DEFAULT		80		//deg

//cursor positions on the display	
//strings
#define TITLE_TXT_X				0				//main titles x position

#define TOPSIDE_TXT_X			TITLE_TXT_X		//e.g. 'surface:'		
#define TOPSIDE_TXT_Y			14	

#define SUBSEA_TXT_X			TITLE_TXT_X		//e.g. 'UROV:'
#define SUBSEA_TXT_Y			TOPSIDE_TXT_Y + 52	

#define MAXDEPTH_TXT_X			TITLE_TXT_X		//e.g. 'Max depth:'
#define MAXDEPTH_TXT_Y			TOPSIDE_TXT_Y + 126

#define RUNTIME_TXT_X			TITLE_TXT_X		//e.g. 'Runtime:'
#define RUNTIME_TXT_Y			153	

#define ALARM_TXT_X				TITLE_TXT_X + 55//leak alarm text
#define ALARM_TXT_Y				TOPSIDE_TXT_Y

//numbers
#define MSRMNT_VAL_X			12				//air and water measurements

#define AIRTEMP_VAL_X			MSRMNT_VAL_X
#define AIRTEMP_VAL_Y			TOPSIDE_TXT_Y + 12 	

#define AIRPRESS_VAL_X			MSRMNT_VAL_X
#define AIRPRESS_VAL_Y			AIRTEMP_VAL_Y + 21	

#define WATERTEMP_VAL_X			MSRMNT_VAL_X
#define WATERTEMP_VAL_Y			SUBSEA_TXT_Y + 12

#define WATERPRESS_VAL_X		MSRMNT_VAL_X
#define WATERPRESS_VAL_Y		WATERTEMP_VAL_Y + 21

#define DEPTH_VAL_X				MSRMNT_VAL_X
#define DEPTH_VAL_Y				WATERPRESS_VAL_Y + 21

#define MAXDEPTH_VAL_X			58
#define MAXDEPTH_VAL_Y			MAXDEPTH_TXT_Y 

#define TIME_VAL_X				12				//current time
#define TIME_VAL_Y				0

#define DATE_VAL_X				TIME_VAL_X + 46	//date
#define DATE_VAL_Y				TIME_VAL_Y

#define	RUNTIME_VAL_X			MAXDEPTH_VAL_X	//runtime [hh:][mm:]ss
#define	RUNTIME_VAL_Y			RUNTIME_TXT_Y

//units
#define AIRTEMP_UN_X			MSRMNT_VAL_X + 76//degC		
#define AIRTEMP_UN_Y			AIRTEMP_VAL_Y - 2

#define AIRPRESS_UN_X			AIRTEMP_UN_X	//hPa
#define AIRPRESS_UN_Y			AIRPRESS_VAL_Y

#define WATERTEMP_UN_X			AIRTEMP_UN_X	//degC
#define WATERTEMP_UN_Y			WATERTEMP_VAL_Y - 2

#define	WATERPRESS_UN_X			AIRTEMP_UN_X	//bar
#define WATERPRESS_UN_Y			WATERPRESS_VAL_Y

#define	DEPTH_UN_X				AIRTEMP_UN_X	//m for depth
#define	DEPTH_UN_Y				DEPTH_VAL_Y

#define	MAXDEPTH_UN_X			AIRTEMP_UN_X	//m for max depth
#define	MAXDEPTH_UN_Y			MAXDEPTH_VAL_Y