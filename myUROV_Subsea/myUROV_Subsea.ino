/***********************************************************************************
	Name:
		myUROV_Subsea.ino
	Description:
		Subsea (underwater) controller functionality implementation.
	Version:
		1.0
	Created:
		Aug 2018
	By:
		Jakub Kurkowski
***********************************************************************************/

#include "defs.h"
#include "msgID.h"
									//device				protocol	controller interface pins
#include <DallasTemperature.h>		//temperature sensor	1-Wire		any	DIO 	
#include <Servo.h>					//servo					N/A

/*
 * interface connection defs
 */

//analogue input signals
#define PIN_WATER_INGRESS	A6		//water detection sensor
#define PIN_WATER_PRESSURE	A0		//pressure transducer

//light switch
#define PIN_LIGHTS_SWITCH	A4		//MOSFET driver

//camera tilt servo control
#define PIN_SERVO			9		

//motors controls - 3DOF
#define PIN_MOTORS_FB		7		//forward/backward
#define PIN_MOTORS_LR		5		//yaw
#define PIN_MOTORS_UD		6		//up/down (surface/dive)

//comms
#define PIN_RS485_MODE		2		//Rx/Tx select
#define PIN_ONE_WIRE_BUS	11		//1-Wire protocol	

/*
* global objects, flags and other vars
*/

//temperature sensor setup
OneWire OneWire(PIN_ONE_WIRE_BUS);	//1-Wire bus
DallasTemperature Sensors(&OneWire);//sets bus defaults
DeviceAddress DeviceAddr;			//1-Wire device address 

//servos - speed/angle control
Servo TiltServo;					//camera servo position control
Servo MotorsFB;						//forward/backward		
Servo MotorsLR;						//left/right (yaw)
Servo MotorsUD;						//up down (surface/dive)

//comms buffers
byte Incoming_Byte = 0;

union WaterTemperatureImplicitCast	//water temperature storage and implicit conversion
{
	float As_Float;
	byte As_Bytes[4];
} WaterTemperaturePacket;

union WaterPressureImplicitCast		//water pressure storage and implicit conversion
{
	float As_Float;
	byte As_Bytes[4];
} WaterPressurePacket;

//commms watchdog updates
uint32 Wdog_Timestamp =		0;
bool Get_Wdog_Timestamp =	TRUE;
bool NewData_Flag;					//if there is new incoming data, do not attempt to send packet to Topside
					
//safety mechanism updates
bool Recovering_Flag =		FALSE;
bool Stopped_Flag =			FALSE;

//deactivation of the (jittering) servo (also, saves power)
uint32 Deactivate_Servo_Timestamp = 0;
bool Deactivate_Flag =		FALSE;
bool Detached_Flag =		FALSE;

//data updates
uint32 Cycle_Timestamp =	0;		//timestamp of each separate cycle to compare against to
uint32 Measurements_Timestamp = 0;
bool Measurements_Flag =	TRUE;

//measurements (new/periodic) updates
bool SendPacket_Flag =		TRUE;	//to send to surface controller
uint32 Send_Timestamp =		0;

//re-send commands
bool SendCmd_Flag =			FALSE;
bool Send_Motors_Cmd =		TRUE;
bool Send_Lights_Cmd =		TRUE;
bool Send_Servo_Cmd =		TRUE;

//water ingress value 
byte Water_Ingress_Storage;
const byte Float_Size_In_Bytes = 4;

//struct for received control messages 
struct
{
	byte X_MVMT =			LEFT_RIGHT_DEFAULT;		//0 - 128 -> left, 131 - center, 133 - 255 -> right
	byte Y_MVMT =			FRWRD_BCKWRD_DEFAULT;	//0 - 122 -> bwd, 124 - center, 126 - 255 -> fwd
	byte Z_MVMT =			0;						//0 - reset to default (zero speed), 1 - increase diving speed (or decrease surfacing speed)
	//2 - do not change, 3 - increase surfacing speed (or decrease diving speed)
	byte LIGHTS =			0;						//0 - lights OFF, 1 - lights ON
	byte SERVO =			2;						//0 - reset to default (horizontal position), 1 - lower the position, 
	//2 - do not change, 3 - raise the position
} controls;

//struct for mapped control messages into commands
struct
{
	uint16 X_PWM_CMD =		PWM_CMD_DEFAULT;	//700-1285us to turn left, 1485-2000us to turn right
	uint16 Y_PWM_CMD =		PWM_CMD_DEFAULT;	//700-1285us to go bwd, 1485-2000us to go fwd
	uint16 Z_PWM_CMD =		PWM_CMD_DEFAULT;	//700-1285us to rise, 1485-2000us to dive
	byte LIGHTS_CMD =		0;					//1 lights on, 0 lights off
	byte SERVO_CMD =		SERVO_CMD_DEFAULT;	//position of the servo in deg
} commands;

/*
*  functions
*/

/////////////////////////////////////////////////////////////////////////////////////////////////
//retrieves water temperature measurement via 1-Wire protocol in deg C and returns it in deg C
float getWaterTemperature()
{
	Sensors.requestTemperaturesByAddress(DeviceAddr);		//start conversion in ds18b20 sensor (the only device on the bus)
	float water_temperature = Sensors.getTempC(DeviceAddr);	//assign temp (in deg C)
											
	return water_temperature;								//in degrees Celsius
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//retrieves water ingress level measurement as an analogue value and returns as mm value
byte getWaterIngress()
{
	byte water_ingress = 0;											//range: 0 - 40mm

	uint16 water_ingress_read = analogRead(PIN_WATER_INGRESS);
	delay(1); //delay due to h/w limitations of the onboard ADC 

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
	const byte samples =	30;
	uint16 waterPressureAnaRead[samples];
	float waterPressureAnaVal =	0.0;
	float waterPressurePa =		0.0;

	//averaging over number of samples
	for (byte i = 0; i < samples; i++)										
		{
			waterPressureAnaRead[i] = analogRead(PIN_WATER_PRESSURE);
			delay(1); //delay due to h/w limitations of the onboard ADC 
			//Serial.println(waterPressureAnaRead[i]); //debug

			if (waterPressureAnaRead[i] < 116) //reads up to 116 at 0.0m, see comment below
			{
				waterPressureAnaRead[i] = 116;
			}

			waterPressureAnaVal += waterPressureAnaRead[i];
		}

	//make avg
	waterPressureAnaVal = waterPressureAnaVal / samples;
	//Serial.print("waterPressureAnaVal: "); Serial.println(waterPressureAnaVal, 2); //debug

	/*
	 *	0.5 - 4.5V - sensor output value corresponding to:
	 *	0   - 1.2MPa which corresponds to (in Arduino):
	 *	103 - 922 counts returned by ADC. That yields:
	 *	819 counts to be spread between 0 and 1.2MPa
	 *	BUTT! Both sensor and Arduino ADC are inaccurate due to various reasons e.g. interferences in power supply.
	 *	Sensor returns 0.52V at 0m (atmosphere, 1009hPa) and that is translated to anything between 106 and 116 counts.
	 *	It is assumed that the sensor is perhaps calculating against lower than above ambient pressure (sealed type).
	 *	1.2MPa / 819 counts = 1465.2014 Pa/count OR ~~~ 0.147m / count <==> 14.7cm / count 
	 *	All the above approximations make this measurement quite inaccurate overall but should provide
	 *	a good idea on an actual pressure/depth. 
	 */

	//now see how many counts beyond 0 there are and turn it into Pascals
	waterPressurePa = (waterPressureAnaVal - 116.0) * 1465.2014;
	//waterPressurePa = 1490.68323 * (waterPressurePa / samples) - 172919.25466;//max measured water pressure is 12 000hPa or 1 200 000Pa according to sensor spec
	
	//Serial.print("water pressure bar: "); Serial.println(waterPressurePa / 100000.0, 4);//debug
	
	return waterPressurePa;		//in Pa
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//send measurements to Topside
void sendPacket(byte waterTempArg[], byte waterPressArg[], const byte waterIngressArg)
{
	//Serial.println("sending:");
	digitalWrite(PIN_RS485_MODE, HIGH);		//DE=RE=high transmit enabled
	delay(3);

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

	delay(7);								//6ms is time needed to clock out last (three) bytes for bitrate >= 28800bps
	digitalWrite(PIN_RS485_MODE, LOW);		//DE=RE=low transmit disabled
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//receive left/right, fwd/bwd,  up and down info
void receiveTopsideJoystickData()
{	
	if (Serial.available() >= 3)
		NewData_Flag = TRUE;
	else
		NewData_Flag = FALSE;

	while (Serial.available() >= 3)	//3 bytes is the biggest full message
	{
		//data is updated so send commands afterwards
		//will not distinguish if that is noise coming but that is not something of a concern
		SendCmd_Flag = TRUE;

		//process Rx buffer
		Incoming_Byte = Serial.read();

		switch (Incoming_Byte)
		{
			case START_X_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_X_MSG_ID)
				{
					controls.X_MVMT = Incoming_Byte;

					Get_Wdog_Timestamp = TRUE;
				
					//Serial.println("X MSG!"); //debug

				}

				else
					{}	//corrupted packet - ignore
			}
			break;

			case START_Y_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_Y_MSG_ID)
				{
					controls.Y_MVMT = Incoming_Byte;

					Get_Wdog_Timestamp = TRUE;

					//Serial.println("Y MSG!"); //debug
				}
					
				else
					{}	//corrupted packet - ignore
			}
			break;

			case START_Z_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_Z_MSG_ID)
				{
					controls.Z_MVMT = Incoming_Byte;

					Get_Wdog_Timestamp = TRUE;

					//Serial.println("Z MSG!"); //debug

					//blink(1, 100); //debug
				}
				else
					{}	//corrupted packet - ignore
			}
			break;

			case START_LIGHTS_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_LIGHTS_MSG_ID)
				{
					controls.LIGHTS = Incoming_Byte;
					//Serial.println("Lights MSG!"); //debug

				}
				else
					{}	//corrupted packet - ignore
			}
			break;

			case START_SERVO_MSG_ID:
			{
				Incoming_Byte = Serial.read();
				if (Serial.read() == STOP_SERVO_MSG_ID)
				{
					controls.SERVO = Incoming_Byte;
					//Serial.println("Servo MSG!"); //debug
				}
				else
					{}	//corrupted packet - ignore
			}
			break;			

			default:	//corrupted packet		
			{ 
				//Serial.println("Corrupted packet!"); //debug
				//Serial.println("0");
				//blink(1, 50);
			}
			break;
		}//switch
	}//while
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//interprete and update all values needed to be sent to motors, lights and camera servo
void processControls()
{
	/* motors */

	//X_MVMT
	if (controls.X_MVMT < LEFT_RIGHT_DEFAULT)
		commands.X_PWM_CMD = map(controls.X_MVMT, 0, 128, 700, 1285);

	else if (controls.X_MVMT > LEFT_RIGHT_DEFAULT)
		commands.X_PWM_CMD = map(controls.X_MVMT, 133, 255, 1485, 2000);

	else
		commands.X_PWM_CMD = PWM_CMD_DEFAULT;

	//Y_MVMT
	if (controls.Y_MVMT < FRWRD_BCKWRD_DEFAULT)
		commands.Y_PWM_CMD = map(controls.Y_MVMT, 0, 122, 700, 1285);

	else if (controls.Y_MVMT > FRWRD_BCKWRD_DEFAULT)
		commands.Y_PWM_CMD = map(controls.Y_MVMT, 126, 255, 1485, 2000);

	else
		commands.Y_PWM_CMD = PWM_CMD_DEFAULT;

	//Z_MVMT
	//reset
	if (controls.Z_MVMT == 0)
	{
		commands.Z_PWM_CMD = PWM_CMD_DEFAULT;
	}

	//slower/dive
	else if (controls.Z_MVMT == 1)
	{
		if (commands.Z_PWM_CMD > 700)
		{
			commands.Z_PWM_CMD -= 1;
		}
	}

	//faster/surface
	else if (controls.Z_MVMT == 3)
	{
		if (commands.Z_PWM_CMD < 2000)
		{
			commands.Z_PWM_CMD += 1;
		}
	}

	Send_Motors_Cmd = TRUE;

	/* lights */

	if (controls.LIGHTS == 1)
	{
		if (commands.LIGHTS_CMD == 0)	//send a command if lights are off
		{
			commands.LIGHTS_CMD = 1;
			Send_Lights_Cmd = TRUE;
		}
	}

	else								
	{
		if (commands.LIGHTS_CMD == 1)	//send a command if lights are on	
		{
			commands.LIGHTS_CMD = 0;
			Send_Lights_Cmd = TRUE;
		}
	}

	/* servo */

	//reset
	if (controls.SERVO == 0)
	{
		commands.SERVO_CMD = SERVO_CMD_DEFAULT;		//horizontal position
		Send_Servo_Cmd = TRUE;
	}

	//lower
	else if (controls.SERVO == 1)
	{
		if (commands.SERVO_CMD > 10)
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
		if (commands.SERVO_CMD < 170)
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


/////////////////////////////////////////////////////////////////////////////////////////////////
//send commands to motors, lights, camera servo
void sendCommands()
{
	/* motors */
	if (Send_Motors_Cmd)
	{
		MotorsFB.writeMicroseconds(commands.Y_PWM_CMD);
		MotorsLR.writeMicroseconds(commands.X_PWM_CMD);
		MotorsUD.writeMicroseconds(commands.Z_PWM_CMD);
	}

	/* lights */
	if (Send_Lights_Cmd)
	{
		if (commands.LIGHTS_CMD == 1)
			digitalWrite(PIN_LIGHTS_SWITCH, HIGH);

		else
			digitalWrite(PIN_LIGHTS_SWITCH, LOW);

		Send_Lights_Cmd = FALSE;
	}

	/* servo */
	if (Send_Servo_Cmd)
	{
		if (!TiltServo.attached())
			TiltServo.attach(PIN_SERVO);

		TiltServo.write(commands.SERVO_CMD);

		Send_Servo_Cmd = FALSE;
		
		//to prevent jitter, deactivate the servo after movement
		Deactivate_Servo_Timestamp = millis();
		Detached_Flag = FALSE;
	}

}


/////////////////////////////////////////////////////////////////////////////////////////////////
//checks when the last comms took place
void watchdog(uint32 timestamp)
{
	if (Get_Wdog_Timestamp == TRUE)
	{
		Wdog_Timestamp = millis();
		Get_Wdog_Timestamp = FALSE;
	}

	if ((long)(timestamp - Wdog_Timestamp) > RECOVERY_DELAY)	//time after last motors cmd received
		safetyRecovery();
	else
		Recovering_Flag = FALSE;

	if ((long)(timestamp - Wdog_Timestamp) > STOP_DELAY)		//time after last motors cmd received
		safetyStop();
	else
		Stopped_Flag = FALSE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//blinker
void blink(const byte tuple, const uint16 blinkspan)
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
		commands.Z_PWM_CMD = PWM_CMD_DEFAULT + 200;
		sendCommands();

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
		commands.X_PWM_CMD = PWM_CMD_DEFAULT;
		commands.Y_PWM_CMD = PWM_CMD_DEFAULT;
		commands.Z_PWM_CMD = PWM_CMD_DEFAULT;
		sendCommands();
		
		Stopped_Flag = TRUE;

		//blink(1, 2000); //debug
		//Serial.println("SAFETY STOP!!"); //debug
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//setup
void setup()
{	
	//set pins to input/output
	pinMode(PIN_WATER_INGRESS, INPUT);		//analog input
	pinMode(PIN_WATER_PRESSURE, INPUT);		//analog input

	pinMode(PIN_LIGHTS_SWITCH, OUTPUT);		//lights switch

	pinMode(PIN_RS485_MODE, OUTPUT);		//DE/RE Data Enable/Receive Enable - transmit/receive pin set to output

	pinMode(PIN_MOTORS_FB, OUTPUT); 
	pinMode(PIN_MOTORS_LR, OUTPUT);
	pinMode(PIN_MOTORS_UD, OUTPUT);

	MotorsFB.attach(PIN_MOTORS_FB);
	MotorsLR.attach(PIN_MOTORS_LR);
	MotorsUD.attach(PIN_MOTORS_UD);

	//initiate motors, lights, servo with default commands
	sendCommands();

	//initialise Dallas temp sensor - OneWire protocol
	Sensors.begin();						//locate devices on the bus
	Sensors.setWaitForConversion(FALSE);	//make it async, don't wait for conversion
	Sensors.getAddress(DeviceAddr, 0);
	Sensors.setResolution(DeviceAddr, 10);	//10bit resolution is ~3x fater than 12bit and accurate enough

	Serial.begin(BITRATE, SERIAL_SETTINGS);	//open Serial hardware port for RS485 comms
} //end of setup


/////////////////////////////////////////////////////////////////////////////////////////////////
//main program
void loop()
{
	/* set all time-based flags for each cycle execution */

	Cycle_Timestamp = millis();

	//set flag for measurements update (time in ms)
	if (Cycle_Timestamp - Measurements_Timestamp > MSRMTS_SUBSEA_UPDATE)
		Measurements_Flag = TRUE;

	//set flag for sending telemetry data to Topside (time in ms)
	if (Cycle_Timestamp - Send_Timestamp > SEND_MSRMTS_INTERVAL)
		SendPacket_Flag = TRUE;

	//set flag for deactivating the servo SIG pin (time in ms)
	if (long(Cycle_Timestamp - Deactivate_Servo_Timestamp) > SERVO_STOP)
		Deactivate_Flag = TRUE;

	/* set all time-based flags for each cycle execution */

	receiveTopsideJoystickData();

	if (SendCmd_Flag && !Recovering_Flag)
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
		{
			TiltServo.detach();

			Detached_Flag = TRUE;
			Deactivate_Flag = FALSE;
		}
	}

	//joystick stuff//////////////

	//Serial.print("UP = ");  Serial.print(controls.Z1_MVMT); Serial.print("\t");
	//Serial.print("DOWN = "); Serial.println(controls.Z2_MVMT);
	//Serial.print("X = "); Serial.print(controls.X_MVMT);  Serial.print("\t"); 
	//Serial.print("Y = "); Serial.println(controls.Y_MVMT); 
	//Serial.print("LIGHTS = "); Serial.println(controls.LIGHTS); 

	watchdog(Cycle_Timestamp);

	/*while (Serial.available()) //debug
	{
		blink(1, 10);
		Serial.read();
	}*/

} //loop
