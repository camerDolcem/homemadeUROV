/***********************************************************************************
 Name:
     msgID.h
 Description:
     List of delimiters indicating the beginning of serial message
 Version:
     01
 Created:
	2017
 By:
	Jakub Kurkowski
***********************************************************************************/

enum ListOfMsgIDs
{
	//Subsea -> Topside messages delimiters
	START_WATERTEMP_MSG_ID =	1,
	STOP_WATERTEMP_MSG_ID =		2,

	START_WATERPRESS_MSG_ID =	3,
	STOP_WATERPRESS_MSG_ID =	4,

	START_WATERING_MSG_ID =		5,
	STOP_WATERING_MSG_ID =		6,

	//Topside -> Subsea messages delimiters
	START_X_MSG_ID =			11,
	STOP_X_MSG_ID =				12,

	START_Y_MSG_ID =			13,
	STOP_Y_MSG_ID =				14,

	START_Z_MSG_ID =			15,
	STOP_Z_MSG_ID =				16,

	START_LIGHTS_MSG_ID =		17,
	STOP_LIGHTS_MSG_ID =		18,

	START_SERVO_MSG_ID =		19,
	STOP_SERVO_MSG_ID =			20,

	START_WATCHDOG_MSG_ID =		21,
	STOP_WATCHDOG_MSG_ID =		22,
	WATCHDOG_MSG_ID =			100,
};