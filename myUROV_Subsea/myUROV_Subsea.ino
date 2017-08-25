/***********************************************************************************
 Name:
     myUROV_Subsea.ino
 Description:
     Subsea controller
 Version:
     01
 Created:
	2017
 By:
	Jakub Kurkowski
***********************************************************************************/

#include "defs.h""
#include "msgID.h"
#include <DallasTemperature.h>		//ds18b20 temp sensor library
#include <Servo.h>

//pins definition for water detection sensor
#define PIN_WATER_INGRESS	3

//pins definition for pressure transducer
#define PIN_WATER_PRESSURE	A0

//pins definition for 1-Wire bus
#define PIN_LIGHTS_SWITCH	4

//pins definition for 1-Wire bus
#define PIN_ONE_WIRE_BUS	6

//pins definition for RS485 serial comms
#define PIN_RS485_MODE		13

//tilt camera servo obj
Servo TiltServo;

//watchdog data
uint32 Wdog_Timestamp = 0;


/////////////////////////////////////////////////////////////////////////////////////////////////
//retrieves water temperature measurement via 1-Wire protocol in deg C and returns it in deg C
float getWaterTemperature()
{
	OneWire OneWire(PIN_ONE_WIRE_BUS);						//setup 1-Wire bus
	DallasTemperature Sensors(&OneWire);					//setup temperature sensor with 1-Wire params
	Sensors.begin();										//locate devices on the bus
	Sensors.requestTemperaturesByIndex(0);					//start conversion in ds18b20 sensor (the only device on the bus)
	float water_temperature = Sensors.getTempCByIndex(0);	//assign temp (in deg C)
											
	return water_temperature;								//in degrees Celsius
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//retrieves water ingress level measurement as an analogue value and returns as mm value
byte getWaterIngress()
{
	uint16 water_ingress_read = analogRead(PIN_WATER_INGRESS);
	
	byte water_ingress = 0;											//range 0 - 40mm

	//scaling of the sensor
	if (water_ingress_read <= 20)
		water_ingress = 0;											//values reported for 0mm
	else if (water_ingress_read > 20 && water_ingress_read < 200)
		water_ingress = map(water_ingress_read, 20, 200, 1, 5);		//in range of 1 to 5mm
	else
		water_ingress = map(water_ingress_read, 200, 500, 5, 40);	//in range of 5 to 40mm 

	//Serial.print("water ingress in mm: "); Serial.println(water_ingress);//debug

	return water_ingress;											//in mm
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//retrieves water pressure measurement as an analogue value and returns as Pa value
float getWaterPressure()
{
	static const byte samples = 15;
	uint16 water_pressure_read[samples];
	float water_pressure = 0.0;

	//averaging over number of samples
	for (byte i = 1; i <= samples; i++)										
		{
			water_pressure_read[i] = analogRead(PIN_WATER_PRESSURE);
			
			//Serial.println(water_pressure_read[i]); //debug
			
			if (water_pressure_read[i] < 116)
			{
				//Serial.println(water_pressure_read[i]);//debug

				water_pressure_read[i] = 116;
			}


			water_pressure += water_pressure_read[i];
		}
	
	water_pressure = 1490.68323 * (water_pressure / samples) - 172919.25466;//max measured water pressure is 12 000hPa or 1 200 000Pa according to sensor spec
	
	//Serial.print("water pressure result: "); Serial.println(water_pressure / 100000.0, 2);//debug
	
	return water_pressure;													//in Pa
}


//define packet for water temperature value storage and type substitution
union WaterTemperatureImplicitCast
{
	float As_Float;
	byte As_Bytes[4];
} WaterTemperaturePacket;


//define packet for water pressure value storage and type substitution
union WaterPressureImplicitCast
{
	float As_Float;
	byte As_Bytes[4];
} WaterPressurePacket;

//define variable for water ingress level value storage
byte Water_Ingress_Storage;

const byte Float_Size_In_Bytes = 4;						//size of byte array to store float, constant value


/////////////////////////////////////////////////////////////////////////////////////////////////
//send measurements to Topside
void sendPacket(byte waterTempArg[], byte waterPressArg[], byte& waterIngressArg)
{
	digitalWrite(PIN_RS485_MODE, HIGH);			//DE=RE=high transmit enabled
	delay(1);

	//send water temperature value
	Serial.write(START_MSG_ID);
	Serial.write(WATER_TEMPERATURE_ID);
	for (byte i = 0; i < Float_Size_In_Bytes; i++)
		Serial.write(waterTempArg[i]);

	//send water pressure value
	Serial.write(START_MSG_ID);
	Serial.write(WATER_PRESSURE_ID);
	for (byte i = 0; i < Float_Size_In_Bytes; i++)
		Serial.write(waterPressArg[i]);
	
	//send water ingress level value
	Serial.write(START_MSG_ID);
	Serial.write(WATER_INGRESS_ID);
	Serial.write(waterIngressArg);

	delay(7);									//time needed to clock out last three bytes for bitrate >= 28800
	digitalWrite(PIN_RS485_MODE, LOW);			//DE=RE=low transmit disabled
}


//define struct for received control messages 
struct
{
	byte X_MVMT = 131;			//0 - 128 -> left, 131 - center, 133 - 255 -> right
	byte Y_MVMT = 124;			//0 - 122 -> bwd, 124 - center, 126 - 255 -> right
	byte Z1_MVMT = 1;			//0 in order to add to surfacing speed
 	byte Z2_MVMT = 1;			//0 in order to add to diving speed
	byte LIGHTS = 1;			//0 in order to turn the lights on or off
} controls;

//define struct for mapped control messages into motion commands
struct
{
	uint16 X_PWM_CMD = 1500;		//1000-1500us to turn left, 1500-2000us to turn right
	uint16 Y_PWM_CMD = 1500;		//1000-1500us to go bwd, 1500-2000us to go fwd
	uint16 Z_PWM_CMD = 1500;		//1000-1500us to rise, 1500-2000us to dive
	bool LIGHTS_CMD = FALSE;		//TRUE to turn the lights on, FALSE to turn them off
} commands;

//define byte to store single byte from the message
byte Incoming_Byte = 0;

//define the flag for sending the commands
bool SendCmd_Flag = FALSE;


/////////////////////////////////////////////////////////////////////////////////////////////////
//receive left/right, fwd/bwd,  up and down info
void receiveTopsideJoystickData()
{
	digitalWrite(PIN_RS485_MODE, LOW);				//DE=RE=low receive enabled	{
	if (Serial.available() >= 15)					//if full packet is in the buffer - to prevent light flickering
	{
	
		while (Serial.available() >= 3)		//3 bytes is the biggest full message
		{
			//there is data, update watchdog and set send cmd flag
			Wdog_Timestamp = millis();

			//data is updated so send it afterwards
			SendCmd_Flag = TRUE;

			//process Rx buffer
			Incoming_Byte = Serial.read();

			if (Incoming_Byte == START_CTRL_ID)
			{
				Incoming_Byte = Serial.read();

				switch (Incoming_Byte)
				{
					case X_MSG_ID:
						controls.X_MVMT = Serial.read();
						break;

					case Y_MSG_ID:
						controls.Y_MVMT = Serial.read();
						break;

					case Z1_MSG_ID:
						controls.Z1_MVMT = Serial.read();
						break;

					case Z2_MSG_ID:
						controls.Z2_MVMT = Serial.read();
						break;

					case L_MSG_ID:
						controls.LIGHTS = Serial.read();
						break;

					default:						//corrupted packet								
						break;
				}//switch
			}//if
		}//while
	}//if
}

//create delay on a light switch to prevent flickering
uint32 Lights_Counter = 0;
//and a toggle helping flag
bool Reset_Flag = FALSE; 


/////////////////////////////////////////////////////////////////////////////////////////////////
//set lights to ON or OFF
void processControls()
{
	//convert ctrls to cmds for 3 pair of engines and lights 

	//lights
	switch (controls.LIGHTS)
	{
		case 1:
		{
				Lights_Counter = 0;
				Reset_Flag = TRUE;
				break;
		}

		case 0:
		{
			Lights_Counter++;
			if (Lights_Counter > 10)
			{
				if (commands.LIGHTS_CMD == TRUE && Reset_Flag == TRUE)			//if already ON but button was released before, turn them off
				{
					commands.LIGHTS_CMD = FALSE;
					TiltServo.write(400);
					Reset_Flag = FALSE;
				}

				else if (commands.LIGHTS_CMD == FALSE && Reset_Flag == TRUE)	//if OFF but button was released before, turn them on
				{
					commands.LIGHTS_CMD = TRUE;
					TiltServo.write(80);
					Reset_Flag = FALSE;
				}
			}
			break;
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//set lights to ON or OFF
void sendCommands()
{
	//send each commands on 3 pair of engines and lights

	//lights
	if (commands.LIGHTS_CMD)
		digitalWrite(PIN_LIGHTS_SWITCH, HIGH);
	else
		digitalWrite(PIN_LIGHTS_SWITCH, LOW);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//checks when the last comms took place, if more than 10mins, starts surfacing
void watchdog()
{
	if (millis() - Wdog_Timestamp > 600000)	//10min lack of comms for Auto-Recovery to kick in
		//do not send command
		autoRecovery();

	if (millis() - Wdog_Timestamp > 3000)	//3sec lack of comms for Auto-Stop to kick in
		autoStop();
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//Auto-Recovery Subsea Safety System
void autoRecovery()
{	//turn the lights off and on 3 times 

	//set z to some 1600
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//checks when the last comms took place, if more than 10mins, starts surfacing
void autoStop()
{
	//set x to 1500
	//set y to 1500
	//set z to 1500
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//setup
void setup()
{	
	//set pins to input/output
	pinMode(PIN_WATER_INGRESS, INPUT);					//analog input
	pinMode(PIN_WATER_PRESSURE, INPUT);					//analog input
	pinMode(PIN_LIGHTS_SWITCH, OUTPUT);					//lights switch
	pinMode(PIN_RS485_MODE, OUTPUT);					//DE/RE Data Enable/Receive Enable - transmit/receive pin set to output
	
	//lights off on start-up
	digitalWrite(PIN_LIGHTS_SWITCH, LOW);

	//set camera to horizontal position
	TiltServo.attach(5);
	digitalWrite(5, LOW);

	Serial.begin(BITRATE, SERIAL_8E1);					//open Serial hardware port for RS485 comms
} //end of setup


//global data defining how often to retrieve and send data
uint32 Measurements_Timestamp = 0;
bool Measurements_Flag = TRUE;

uint32 Send_Timestamp = 0;
bool SendPacket_Flag = TRUE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//main program
void loop()
{
	//set flag for measurements update (in ms)
	if (millis() - Measurements_Timestamp >= 750)
		Measurements_Flag = TRUE;

	//set flag for sending telemetry data to Topside (in ms)
	if (millis() - Send_Timestamp >= 1300)
		SendPacket_Flag = TRUE;

	//retrieve the measurements
if (Measurements_Flag)
	{
		//update water temperature, pressure and ingress level
		WaterTemperaturePacket.As_Float = getWaterTemperature();
		WaterPressurePacket.As_Float = getWaterPressure();
		Water_Ingress_Storage = getWaterIngress();

		Measurements_Timestamp = millis();
		Measurements_Flag = FALSE;
	}

	if (SendPacket_Flag) //ustaw ta flage jezeli otrzymasz flage Request z topside, ktora oznacza, ze nic nie jest wysylane z joysticka (np, gdy wszystko jest na zero albo rowne old value)
	{
		sendPacket(WaterTemperaturePacket.As_Bytes, WaterPressurePacket.As_Bytes, Water_Ingress_Storage);

		Send_Timestamp = millis();
		SendPacket_Flag = FALSE;
	}
	
	receiveTopsideJoystickData();

	if (SendCmd_Flag)
	{
		processControls();
		sendCommands();
		SendCmd_Flag = FALSE;
	}


	//joystick stuff//////////////

	//Serial.print("UP = ");  Serial.print(controls.Z1_MVMT); Serial.print("\t");
	//Serial.print("DOWN = "); Serial.println(controls.Z2_MVMT);
	//Serial.print("X = "); Serial.print(controls.X_MVMT);  Serial.print("\t"); 
	//Serial.print("Y = "); Serial.println(controls.Y_MVMT); 
	//Serial.print("LIGHTS = "); Serial.println(controls.LIGHTS); 

	watchdog();

} //end of loop





