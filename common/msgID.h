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
	START_MSG_ID =			10,

	WATER_TEMPERATURE_ID =	50,
	WATER_PRESSURE_ID =		75,
	WATER_INGRESS_ID =		100,

	START_CTRL_ID =			20,

	X_MSG_ID =				125,
	Y_MSG_ID =				150,
	Z1_MSG_ID =				175,
	Z2_MSG_ID =				200,
};