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

//general definitions
#define TRUE				true
#define FALSE				false

//type definitions
typedef boolean				bool;			
typedef	unsigned int		uint16;
typedef unsigned long		uint32;

//physical constants
const float RHO = 1023.6; 	//density of sea water [kg/m^3]
const float G = 9.81;		//Gravitational acceleration [m/s^2]

//serial comms bitrate for both microcontrollers
const uint32 BITRATE = 19200;

//joystick default position mapped readings
const byte FORWARD_BACKWARD_DEFAULT = 124;
const byte LEFT_RIGHT_DEFAULT = 131;
const byte FLUCTUATION_DEFAULT = 5;