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

#include "defs.h"
#include "msgID.h"
#include <DallasTemperature.h>		//ds18b20 temp sensor library
#include <Servo.h>

//pin definition for water detection sensor
#define PIN_WATER_INGRESS	A0

//pin definition for pressure transducer
#define PIN_WATER_PRESSURE	A1

//pin definition for the relay
#define PIN_LIGHTS_SWITCH	4

//pin definition for 1-Wire bus
#define PIN_ONE_WIRE_BUS	6

//pin definition for servo signal
#define PIN_SERVO			8

//pin definition for RS485 serial comms
#define PIN_RS485_MODE		10

//tilt camera servo obj
Servo TiltServo;

//commms watchdog time data
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

	//Serial.print("water ingress reading: "); Serial.println(water_ingress_read);//debug
	//Serial.print("water ingress in mm: "); 
	//Serial.print(water_ingress);//debug

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
	for (byte i = 0; i < samples; i++)										
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
	//Serial.println("sending:");
	digitalWrite(PIN_RS485_MODE, HIGH);			//DE=RE=high transmit enabled
	delay(2);

	//send water temperature value
	Serial.write(START_WATERTEMP_MSG_ID);
	for (byte i = 0; i < Float_Size_In_Bytes; i++)
		Serial.write(waterTempArg[i]);
	Serial.write(STOP_WATERTEMP_MSG_ID);

	//send water pressure value
	Serial.write(START_WATERPRESS_MSG_ID);
	for (byte i = 0; i < Float_Size_In_Bytes; i++)
		Serial.write(waterPressArg[i]);
	Serial.write(STOP_WATERPRESS_MSG_ID);
	
	//send water ingress level value
	Serial.write(START_WATERING_MSG_ID);
	Serial.write(waterIngressArg);
	Serial.write(STOP_WATERING_MSG_ID);

	delay(16);								//time needed to clock out last three bytes for bitrate >= 28800 min 6ms
	digitalWrite(PIN_RS485_MODE, LOW);		//DE=RE=low transmit disabled
}


//define struct for received control messages 
struct
{
	byte X_MVMT = LEFT_RIGHT_DEFAULT;			//0 - 128 -> left, 131 - center, 133 - 255 -> right
	byte Y_MVMT = FORWARD_BACKWARD_DEFAULT;		//0 - 122 -> bwd, 124 - center, 126 - 255 -> right
	byte Z_MVMT = 0;			//0 - reset to default (zero speed), 1 - increase diving speed (or decrease surfacing speed)
								//2 - do not change, 3 - increase surfacing speed (or decrease diving speed)

	byte LIGHTS = 0;			//0 - lights OFF, 1 - lights ON

	byte SERVO = 2;				//0 - reset to default (horizontal position), 1 - lower the position, 
								//2 - do not change, 3 - raise the position
} controls;

//define struct for mapped control messages into motion commands
struct
{
	uint16 X_PWM_CMD = 1500;		//1000-1500us to turn left, 1500-2000us to turn right
	uint16 Y_PWM_CMD = 1500;		//1000-1500us to go bwd, 1500-2000us to go fwd
	uint16 Z_PWM_CMD = 1500;		//1000-1500us to rise, 1500-2000us to dive
	bool LIGHTS_CMD = FALSE;		//TRUE to turn the lights on, FALSE to turn them off
	byte SERVO_CMD = 90;			//position of the servo in deg
} commands;

//define byte to store single byte from the message
byte Incoming_Byte = 0;

//define the flags for sending the commands and for new data 
bool SendCmd_Flag = FALSE;
bool NewData_Flag;			//(if there is new data do not attempt to send packet to Topside

/////////////////////////////////////////////////////////////////////////////////////////////////
//receive left/right, fwd/bwd,  up and down info
void receiveTopsideJoystickData()
{	
	if (Serial.available() >= 3)
		NewData_Flag = TRUE;
	else
		NewData_Flag = FALSE;

	while (Serial.available() >= 3)				//3 bytes is the biggest full message
	{
		//data is updated so send commands afterwards
		//will not distinguish if that is noise coming but that is not something of major concern
		SendCmd_Flag = TRUE;

		//process Rx buffer
		Incoming_Byte = Serial.read();

		switch (Incoming_Byte)
		{
			case START_X_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_X_MSG_ID)
					controls.X_MVMT = Incoming_Byte;
				else
					{}	//corrupted packet - ignore
			}
			break;

			case START_Y_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_Y_MSG_ID)
					controls.Y_MVMT = Incoming_Byte;
				else
					{}	//corrupted packet - ignore
			}
			break;

			case START_Z_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_Z_MSG_ID)
					controls.Z_MVMT = Incoming_Byte;
				else
					{}	//corrupted packet - ignore
			}
			break;

			case START_LIGHTS_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_LIGHTS_MSG_ID)
					controls.LIGHTS = Incoming_Byte;
				else
					{}	//corrupted packet - ignore
			}
			break;

			case START_SERVO_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_SERVO_MSG_ID)
					controls.SERVO = Incoming_Byte;
				else
					{}	//corrupted packet - ignore
			}
			break;			
				
			case START_WATCHDOG_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_WATCHDOG_MSG_ID)
				{
					if (Incoming_Byte == WATCHDOG_MSG_ID)
						//there is (meaningful, not corrupted) data, update watchdog
						Wdog_Timestamp = millis();
				}
				else
					{}	//corrupted packet - ignore
			}
			break;	

			default:	//corrupted packet		
			{ 
				//Serial.println("Corrupted packet!");
				blink(1, 10);
			}
				break;
		}//switch
	}//while
}


bool Send_Motors_Cmd = TRUE;
bool Send_Servo_Cmd = TRUE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//interprete and update all values needed to be sent to motors, lights and camera servo
void processControls()
{
	/* motors */

	//convert ctrls to cmds for 3 pair of engines and lights 

	/* lights */

	if (controls.LIGHTS)
		commands.LIGHTS_CMD = TRUE;
	else
		commands.LIGHTS_CMD = FALSE;

	/* servo */
	//reset
	if (controls.SERVO == 0)
	{
		commands.SERVO_CMD = 90;
		Send_Servo_Cmd = TRUE;
		//Serial.println(controls.SERVO); //debug

	}

	//lower
	else if (controls.SERVO == 1)
	{
		if (commands.SERVO_CMD > 75)
		{
			commands.SERVO_CMD -= 1;
			Send_Servo_Cmd = TRUE;
		}
		
		else	//if it reached the end position
			Send_Servo_Cmd = FALSE;
	}

	//higher
	else if (controls.SERVO == 3)
	{
		if (commands.SERVO_CMD < 300)
		{
			commands.SERVO_CMD += 1;
			Send_Servo_Cmd = TRUE;
		}

		else	//if it reached the end position
			Send_Servo_Cmd = FALSE;
	}

	//no change
	else
	{
		//do nothing
		Send_Servo_Cmd = FALSE;
	}
}


//global data for deactivation of the (jittering) servo (holds position)
uint32 Deactivate_Servo_Timestamp = 0;
bool Deactivate_Flag = FALSE;
bool Detached_Flag = FALSE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//send commands to motors, lights, camera servo
void sendCommands()
{
	/* motors */
	if (Send_Motors_Cmd)
	{
		//send each commands on 3 pair of engines and lights

	}

	/* lights */
	if (commands.LIGHTS_CMD)
		digitalWrite(PIN_LIGHTS_SWITCH, HIGH);
	else
		digitalWrite(PIN_LIGHTS_SWITCH, LOW);

	/* servo */
	if (Send_Servo_Cmd)
	{
		if (!TiltServo.attached())
			TiltServo.attach(PIN_SERVO);

		TiltServo.write(commands.SERVO_CMD);

		Deactivate_Servo_Timestamp = millis();
		Detached_Flag = FALSE;
	}

}


bool Recovering_Flag = FALSE;
bool Stopped_Flag = FALSE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//checks when the last comms took place
void watchdog(uint32& timestamp)
{
	if (long(timestamp - Wdog_Timestamp) > 5000)	//30secs lack of comms for Safety-Recovery to kick in
		//do not send command
		safetyRecovery();
	else
		Recovering_Flag = FALSE;

	if (long(timestamp - Wdog_Timestamp) > 5000)	//3sec lack of comms for Safety-Stop to kick in
		safetyStop();
	else
		Stopped_Flag = FALSE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//blinker
void blink(byte tuple, uint16 blinkspan)
{
	for (byte i = 0; i < tuple; i++)
	{
		digitalWrite(PIN_LIGHTS_SWITCH, HIGH);
		delay(blinkspan);
		digitalWrite(PIN_LIGHTS_SWITCH, LOW);
		delay(blinkspan);
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//Auto-Recovery Subsea Safety System
void safetyRecovery()
{	
	if (Recovering_Flag == FALSE) //trigger only once
	{


		//set z to some 1600

		//Serial.println("SAFETY RECOVERY");

		Recovering_Flag = TRUE;
	}

	//flicker the lights while emerging
	blink(1, 500);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//checks when the last comms took place, if more than 10mins, starts surfacing
void safetyStop()
{


	if (Stopped_Flag == FALSE && Recovering_Flag == FALSE) //trigger only once
	{
		//set x to 1500
		//set y to 1500
		//set z to 1500
		
		//set z to some 1600

		//Serial.println("SAFETY STOP");

		Stopped_Flag = TRUE;
	}
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

	//initiate motors, lights, servo with default commands
	sendCommands();

	Serial.begin(BITRATE, SERIAL_8E1);					//open Serial hardware port for RS485 comms
} //end of setup


//global data defining how often to retrieve and send data
uint32 Cycle_Timestamp = 0;	//timestamp of each separate cycle to compare against to

uint32 Measurements_Timestamp = 0;
bool Measurements_Flag = TRUE;

uint32 Send_Timestamp = 0;
bool SendPacket_Flag = TRUE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//main program
void loop()
{
	/* set all time-based flags for each cycle execution */

	Cycle_Timestamp = millis();

	//set flag for measurements update (time in ms)
	if (Cycle_Timestamp - Measurements_Timestamp > 1200)
		Measurements_Flag = TRUE;

	//set flag for sending telemetry data to Topside (time in ms)
	if (Cycle_Timestamp - Send_Timestamp > 1300)
		SendPacket_Flag = TRUE;

	//set flag for deactivating the servo SIG pin (time in ms)
	if (long(Cycle_Timestamp - Deactivate_Servo_Timestamp) > 500)
		Deactivate_Flag = TRUE;

	/* set all time-based flags for each cycle execution */

	receiveTopsideJoystickData();

	if (SendCmd_Flag)
	{
		processControls();
		sendCommands();
		SendCmd_Flag = FALSE;
	}

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

	if (SendPacket_Flag && !NewData_Flag) //only send Subsea data when joystick data stopped arriving
	{
		sendPacket(WaterTemperaturePacket.As_Bytes, WaterPressurePacket.As_Bytes, Water_Ingress_Storage);

		Send_Timestamp = millis();
		SendPacket_Flag = FALSE;
	}
	
	if (Deactivate_Flag)
	{
		if (!Detached_Flag)
			TiltServo.detach();

		Detached_Flag = TRUE;
		Deactivate_Flag = FALSE;
	}

	//joystick stuff//////////////

	//Serial.print("UP = ");  Serial.print(controls.Z1_MVMT); Serial.print("\t");
	//Serial.print("DOWN = "); Serial.println(controls.Z2_MVMT);
	//Serial.print("X = "); Serial.print(controls.X_MVMT);  Serial.print("\t"); 
	//Serial.print("Y = "); Serial.println(controls.Y_MVMT); 
	//Serial.print("LIGHTS = "); Serial.println(controls.LIGHTS); 

	watchdog(Cycle_Timestamp);

} //end of loop





