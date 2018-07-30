/***********************************************************************************
	Name:
		myUROV_Subsea.ino
	Description:
		Topside (surface) controller functionality implementation.
	Version:
		1.0
	Created:
		Jul 2018
	By:
		Jakub Kurkowski
***********************************************************************************/

#include "defs.h"
#include "msgID.h"
											//device					protocol	interface pins (controller)
#include <DS1307RTC.h>						//Real Time Clock module	I2C			SCL and SDA hardware pins 
#include <Adafruit_BMP280.h>				//atmospheric PT sensor		I2C			SCL and SDA hardware pins
#include <TFT.h>							//TFT LCD display			SPI			MISO, MOSI, SCL, SS hardware pins

/*
 * interface connection defs
 */

//LCD										//device interface pins (LCD)
#define PIN_SS					53			//CS (Chip (aka Slave) Select)
#define PIN_MISO				50			//RESET
#define PIN_DC					48			//A0 (Data/Command select)
//hardware pin SCK (Serial Clock) 			//SCK (SPI Clock input)	
//hardware pin MOSI (Master Out Slave In)  	//SDA (SPI Data input)

//joystick controls							//device interface pins
#define PIN_BUTTON_UP			A5			//A
#define PIN_BUTTON_DOWN			A3			//C
#define PIN_BUTTON_LIGHTS		A0			//F
#define PIN_BUTTON_SERVO_UP		A4			//B
#define PIN_BUTTON_SERVO_DOWN	A2			//D
#define PIN_BUTTON_SERVO_RESET	A1			//E

#define PIN_JOYSTICK_X			A6			//X
#define PIN_JOYSTICK_Y			A7			//Y
#define PIN_JOYSTICK_BUTTON		A8			//K

//LEDs 
#define PIN_LED_POWER_SUPPLY	7
#define PIN_LED_LEAK_ALARM		6
#define PIN_LED_TX				5
#define PIN_LED_RX				4

//buzzer 
#define PIN_BUZZER_LEAK_ALARM	3

//comms
#define PIN_RS485_MODE			2			 //Rx/Tx select

/*
 * global objects, flags and other vars
 */

//TFT LCD 1.8in 128x160px display 
TFT MyTFT = TFT(PIN_SS, PIN_DC, PIN_MISO);	//3 SPI pins need to be given (only) 	

//BMP280 atmospheric PT sensor
Adafruit_BMP280 BMP;				//I2C protocol
bool foundBmpSensor = FALSE;		//for init

//comms buffers
byte Received_Packet[] =	{ 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //Rx msg max size is 9bytes
byte Incoming_Frame[] =		{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte Incoming_Byte =		0;

uint32 Rx_Good =	0; //debug
uint32 Rx_Incomplete = 0; //debug
uint32 Rx_Rubbish = 0; //debug
bool dispStats = TRUE; //debug

//commms watchdog updates
uint32 Wdog_Timestamp =		0;		//topside controller
bool Get_Wdog_Timestamp =	TRUE;

//data updates
uint32 Cycle_Timestamp =	0;		//used to compare other events against and for comms watchdog
uint32 Runtime_Timestamp =	0;		//runtime updates 
bool RunTime_Flag =			TRUE;
uint32 Time_Timestamp =		0;		//time updates
bool Time_Flag =			TRUE;
uint32 Measurements_Timestamp = 0;	//sensors data updates
bool Measurements_Flag =	TRUE;

//joystick commands (new/periodic) updates
uint32 SendWdog_Timestamp = 0;		//to send to UROV controller
bool Send_Flag =			TRUE;
bool Send_Motors_Flag =		TRUE;
bool Send_Lights_Flag =		TRUE;
bool Send_Servo_Flag =		TRUE;

//default values corresponding to joystick default position readings
//movement
byte Left_Right_Joystick_Motion =		LEFT_RIGHT_DEFAULT;
byte Forward_Backward_Joystick_Motion = FRWRD_BCKWRD_DEFAULT;
byte Up_Button_Motion =		1;
byte Down_Button_Motion =	1;
byte UpDown_Button_Reset =	1;

byte Left_Right_Joystick_Motion_Old;
byte Forward_Backward_Joystick_Motion_Old;
byte UpDown_Motion_Adjust = 0;		//zero speed by default
byte UpDown_Motion_Adjust_Old = 2;	//0 - reset, 1 - slower/dive, 2 - do not change, 3 - faster/surface

//lights
byte Lights_Toggle =		0;		//off by default; 0 - off, 1 - on
byte Lights_Button =		1;
byte Lights_Button_Old =	1;

//servo
byte Servo_Position_Adjust = 0;		//horizontal position by default
byte Servo_Position_Adjust_Old = 2;	//0 - reset, 1 - lower, 2 - do not change, 3 - higher
byte Servo_Button_Lower =	1;
byte Servo_Button_Higher =	1;
byte Servo_Button_Reset =	1;

/*
*  functions
*/

/////////////////////////////////////////////////////////////////////////////////////////////////
//display current runtime
void dispRuntime()
{
	//calculate full hrs and remaining mins and sePIN_SS
	uint32 timer =	millis();				//ms total
	timer =			timer * 0.001;			//s total
	uint16 timerMin = timer / 60;			//full mins only
	byte timerHrs = timerMin / 60;			//full hours only
	timer =			timer - timerMin * 60;	//s only (on top of minutes)
	timerMin =		timerMin - timerHrs * 60;	//min only (on top of hours)

	//display runtime in the format: "hh hrs mm min ss s"
	MyTFT.setTextSize(1);
	MyTFT.setTextColor(ST7735_GREEN);
	MyTFT.setCursor(RUNTIME_X + 50, RUNTIME_Y);
	if (timerHrs >= 0 && timerHrs < 10)
		MyTFT.print(F("0"));
	MyTFT.print(timerHrs);

	MyTFT.setCursor(RUNTIME_X + 71, RUNTIME_Y);
	if (timerMin >= 0 && timerMin < 10)
		MyTFT.print(F("0"));
	MyTFT.print(timerMin);

	MyTFT.setCursor(RUNTIME_X + 92, RUNTIME_Y);
	MyTFT.print(timer);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase runtime
void eraseRuntime()
{
	MyTFT.fillRect(RUNTIME_X + 50, RUNTIME_Y, 12, 7, ST7735_BLACK);
	MyTFT.fillRect(RUNTIME_X + 71, RUNTIME_Y, 12, 7, ST7735_BLACK);
	MyTFT.fillRect(RUNTIME_X + 92, RUNTIME_Y, 12, 7, ST7735_BLACK);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//display current time from RTC
void dispTime() 
{
	tmElements_t Time;						

	if (RTC.read(Time)) 
	{ 
		//get hour
		MyTFT.setTextSize(1);
		MyTFT.setCursor(TIME_X, TIME_Y);
		MyTFT.setTextColor(ST7735_CYAN);
		if (Time.Hour >= 0 && Time.Hour < 10)
			MyTFT.print(F("0"));
		MyTFT.print(Time.Hour);

		MyTFT.setTextColor(ST7735_WHITE);
		MyTFT.print(F(":"));
		
		//get minute
		MyTFT.setTextColor(ST7735_CYAN);
		if (Time.Minute >= 0 && Time.Minute < 10)
			MyTFT.print(F("0"));
		MyTFT.print(Time.Minute);
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase current time
void eraseTime()
{
	MyTFT.fillRect(TIME_X, TIME_Y, 32, 7, ST7735_BLACK);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//display current date from RTC
void dispDate()
{
	tmElements_t Time;						

	if (RTC.read(Time))
	{		
		//display current date
		//day
		MyTFT.setCursor(DATE_X, DATE_Y);
		MyTFT.setTextColor(ST7735_MAGENTA);
		if (Time.Day >= 0 && Time.Day < 10)
			MyTFT.print(F("0"));
		MyTFT.print(Time.Day);

		//month
		MyTFT.setTextColor(ST7735_WHITE);
		MyTFT.print(F("."));
		MyTFT.setTextColor(ST7735_MAGENTA);
		if (Time.Month >= 0 && Time.Month < 10)
			MyTFT.print(F("0"));
		MyTFT.print(Time.Month);

		//year
		MyTFT.setTextColor(ST7735_WHITE);
		MyTFT.print(F("."));
		MyTFT.setTextColor(ST7735_MAGENTA);
		MyTFT.print(tmYearToCalendar(Time.Year));
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//display air pressure and temperature
void dispAirTempPress()
{
	float air_temperature	= 0.0;
	float air_pressure		= 0.0;

	if (foundBmpSensor)
	{
		air_temperature		= (BMP.readTemperature() - 1.2);
		air_pressure		= BMP.readPressure();
	}
	else	//try to locate lost sensor
	{
		foundBmpSensor		= BMP.begin();
		Wire.setClock(400000L); //change from default 100k to 400kHz clock speed for I2C
	}

	//air temperature
	MyTFT.setTextSize(2);
	MyTFT.setCursor(MSRMNTS_X, AIR_TEMP_Y);
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.print(air_temperature, 1);

	//air pressure
	MyTFT.setCursor(MSRMNTS_X, AIR_PRESS_Y);
	MyTFT.print(air_pressure / 100.0, 1);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase air temperature and pressure
void eraseAirTempPress()
{
	//air temperature	
	MyTFT.fillRect(MSRMNTS_X, AIR_TEMP_Y, 73, 14, ST7735_BLACK);
	//air pressure
	MyTFT.fillRect(MSRMNTS_X, AIR_PRESS_Y, 73, 14, ST7735_BLACK);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//display water temperature and pressure
void dispWaterTempPressDpth()
{
	//define variables for water temperature and pressure values storage, bytes casted to float/byte
	float water_temperature = *(float *)Received_Packet;
	float water_pressure = *(float *)(Received_Packet + 4);

	//water temperature
	MyTFT.setTextSize(2);
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.setCursor(MSRMNTS_X, WATER_TEMP_Y);
	MyTFT.print(water_temperature, 1);			//degC

	//water pressure
	MyTFT.setCursor(MSRMNTS_X, WATER_PRESS_Y);
	MyTFT.print(water_pressure / 100000.0, 2);	//bar

	//depth
	MyTFT.setCursor(MSRMNTS_X, DEPTH_Y);
	MyTFT.print(water_pressure / (RHO * G), 2);	//= Pgauge[Pa] / (rho[kg/m^3] * g[m/s^2])

	//max depth
	MyTFT.setTextSize(1);
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.setCursor(MSRMNTS_X, MAX_DEPTH_Y);
	MyTFT.print(water_pressure / (RHO * G), 2);	
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase water temperature and pressure
void eraseWaterTempPressDpth()
{
	//water temperature	
	MyTFT.fillRect(MSRMNTS_X, WATER_TEMP_Y, 73, 14, ST7735_BLACK);
	//water pressure
	MyTFT.fillRect(MSRMNTS_X, WATER_PRESS_Y, 73, 14, ST7735_BLACK);
	//water depth
	MyTFT.fillRect(MSRMNTS_X, DEPTH_Y, 73, 14, ST7735_BLACK);
	//max depth
	MyTFT.fillRect(MSRMNTS_X, MAX_DEPTH_Y, 30, 7, ST7735_BLACK);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//receive water pressure, temperature and leak level data
void receiveSubseaTelemetryData()
{
	while (Serial3.available() >= 6)		//6 bytes defines biggest msg - float (1+4+1)
	{	
		digitalWrite(PIN_LED_RX, HIGH);		//receive LED on

		//process Rx buffer
		Incoming_Byte = Serial3.read();

		switch (Incoming_Byte)
			{
				case START_WATERTEMP_MSG_ID:
				{
					Serial3.readBytes((Incoming_Frame + 0), 4);
					if (Serial3.read() == STOP_WATERTEMP_MSG_ID)
					{
						for (byte i = 0; i < 4; i++)
							Received_Packet[i] = Incoming_Frame[i];

						Rx_Good++; //debug
					}

					else	//corrupted packet - ignore
					{
						//beep(1, 100); Serial3.println("Corrupted packet TEMP"); //debug
						Rx_Incomplete++; //debug
						dispStats = TRUE; //debug
					}	
				}
				break;
							
				case START_WATERPRESS_MSG_ID:
				{
					Serial3.readBytes((Incoming_Frame + 4), 4);
					if (Serial3.read() == STOP_WATERPRESS_MSG_ID)
					{
						for (byte i = 4; i < 8; i++)
							Received_Packet[i] = Incoming_Frame[i];

						Rx_Good++; //debug

					}

					else	//corrupted packet - ignore
					{
						//beep(1, 100); Serial3.println("Corrupted packet PRESS");
						Rx_Incomplete++; //debug
						dispStats = TRUE; //debug
					}	
				}
				break;

				case START_WATERING_MSG_ID:
				{
					Incoming_Frame[8] = Serial3.read();
					if (Serial3.read() == STOP_WATERING_MSG_ID)
					{
						Received_Packet[8] = Incoming_Frame[8];

						//Serial3.println("Water ingress received OK"); //debug
						//when water ingress info is received, kick the watchdog
						Get_Wdog_Timestamp = TRUE;
						Rx_Good++; //debug
					}

					else
					{
						//beep(1, 100); Serial3.println("Corrupted packet INGRESS");
						Rx_Incomplete++; //debug
						dispStats = TRUE; //debug
					}	//corrupted packet - ignore 
				}
				break;

				default:	//corrupted packet								
				{
					//beep(1, 25); //debug
					//Serial3.print("Corrupted packet, buffer: ");
					//Serial3.println(Serial3.available()); 
					Rx_Rubbish++; //debug
					dispStats = TRUE; //debug
				}
				break;
			
		}//switch
	}//while

	delay(2);	//only to make LED light visible
	digitalWrite(PIN_LED_RX, LOW);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//check water ingress level and set alarm
void setWaterLeakAlarm()
{	
	byte Water_Ingress_Alarm = Received_Packet[8];

	if (Water_Ingress_Alarm >= 1 & Water_Ingress_Alarm < 3)	//that indicates humidity in the enclosure most likely
	{
		digitalWrite(PIN_LED_LEAK_ALARM, HIGH);
		analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
	}

	else if (Water_Ingress_Alarm >= 3)						//that indicates probable leak
	{
		digitalWrite(PIN_LED_LEAK_ALARM, HIGH);
		analogWrite(PIN_BUZZER_LEAK_ALARM, 200);
	}

	else
	{
		digitalWrite(PIN_LED_LEAK_ALARM, LOW);
		analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//read joystick command
void getCmd()
{
	/* motors */

	//read in the joystick position, map to 1 byte size value
	Left_Right_Joystick_Motion = map(analogRead(PIN_JOYSTICK_X), 0, 1023, 0, 255);
	Forward_Backward_Joystick_Motion = map(analogRead(PIN_JOYSTICK_Y), 0, 1023, 0, 255);

	//take into account fluctuations on analog signal from the joystick when it is static
	//both when it is in its default (zero) position and when it swerved

	//FWD/BWD
	if (Forward_Backward_Joystick_Motion >= (FRWRD_BCKWRD_DEFAULT - FLUCTUATION_ZERO) &&
		Forward_Backward_Joystick_Motion <= (FRWRD_BCKWRD_DEFAULT + FLUCTUATION_ZERO))
		Forward_Backward_Joystick_Motion = FRWRD_BCKWRD_DEFAULT;

	if (Forward_Backward_Joystick_Motion >= Forward_Backward_Joystick_Motion_Old + FLUCTUATION_NONZERO ||
		Forward_Backward_Joystick_Motion <= Forward_Backward_Joystick_Motion_Old - FLUCTUATION_NONZERO )
	{
		Forward_Backward_Joystick_Motion_Old = Forward_Backward_Joystick_Motion;
		Send_Motors_Flag = TRUE;
	}
	
	//left/right
	if (Left_Right_Joystick_Motion >= (LEFT_RIGHT_DEFAULT - FLUCTUATION_ZERO) &&
		Left_Right_Joystick_Motion <= (LEFT_RIGHT_DEFAULT + FLUCTUATION_ZERO))
		Left_Right_Joystick_Motion = LEFT_RIGHT_DEFAULT;

	if (Left_Right_Joystick_Motion >= Left_Right_Joystick_Motion_Old + FLUCTUATION_NONZERO ||
		 Left_Right_Joystick_Motion <= Left_Right_Joystick_Motion_Old - FLUCTUATION_NONZERO)
	{
		Left_Right_Joystick_Motion_Old = Left_Right_Joystick_Motion;
		Send_Motors_Flag = TRUE;
	}
	
	//up/down

	Up_Button_Motion = digitalRead(PIN_BUTTON_UP);
	Down_Button_Motion = digitalRead(PIN_BUTTON_DOWN);
	UpDown_Button_Reset = digitalRead(PIN_JOYSTICK_BUTTON);

	if (Down_Button_Motion == 0 && Up_Button_Motion == 1)			//allow only 1 button pressed
	{
		UpDown_Motion_Adjust = 1;									//lower the position
		UpDown_Motion_Adjust_Old = UpDown_Motion_Adjust;
		Send_Motors_Flag = TRUE;
	}

	else if (Up_Button_Motion == 0 && Down_Button_Motion == 1)		//allow only 1 button pressed
	{
		UpDown_Motion_Adjust = 3;									//raise the position
		UpDown_Motion_Adjust_Old = UpDown_Motion_Adjust;
		Send_Motors_Flag = TRUE;
	}

	else if (Down_Button_Motion == 0 && Up_Button_Motion == 0)		//both buttons pressed
	{
		UpDown_Motion_Adjust = 2;									//do not move
		if (UpDown_Motion_Adjust_Old != 2)
			Send_Motors_Flag = TRUE;

		UpDown_Motion_Adjust_Old = UpDown_Motion_Adjust;
	}

	else if (UpDown_Button_Reset != 0)								//none pressed and not reset
	{
		UpDown_Motion_Adjust = 2;									//do not move
		if (UpDown_Motion_Adjust_Old != 2 || UpDown_Motion_Adjust_Old == 0)
			Send_Motors_Flag = TRUE;

		UpDown_Motion_Adjust_Old = UpDown_Motion_Adjust;
	}

	//reset button is superior to all others
	if (UpDown_Button_Reset == 0)
	{
		UpDown_Motion_Adjust = 0;									//reset to default position
		Send_Motors_Flag = TRUE;

		UpDown_Motion_Adjust_Old = UpDown_Motion_Adjust;
	}
	
	/* lights */

	Lights_Button = digitalRead(PIN_BUTTON_LIGHTS);

	if (Lights_Button == 0)			//button pressed
	{
		if (Lights_Button_Old == 1) //detect if: 1 -> 0 (just pressed) or 0 -> 0 (is hold)
		{
			Lights_Toggle = Lights_Toggle ? 0 : 1;
			Lights_Button_Old = Lights_Button;
			Send_Lights_Flag = TRUE;
		}
	}

	else							//to detect releasing the button
	{
		Lights_Button_Old = Lights_Button;
	}

	/* servo */

	Servo_Button_Lower = digitalRead(PIN_BUTTON_SERVO_DOWN);
	Servo_Button_Higher = digitalRead(PIN_BUTTON_SERVO_UP);
	Servo_Button_Reset = digitalRead(PIN_BUTTON_SERVO_RESET);

	if (Servo_Button_Lower == 0 && Servo_Button_Higher == 1)		//allow only 1 button pressed
	{
		Servo_Position_Adjust = 1;									//lower the position
		Servo_Position_Adjust_Old = Servo_Position_Adjust;
		Send_Servo_Flag = TRUE;
	}

	else if (Servo_Button_Higher == 0 && Servo_Button_Lower == 1)	//allow only 1 button pressed
	{
		Servo_Position_Adjust = 3;									//raise the position
		Servo_Position_Adjust_Old = Servo_Position_Adjust;
		Send_Servo_Flag = TRUE;
	}

	else if (Servo_Button_Lower == 0 && Servo_Button_Higher == 0)	//both buttons pressed
	{
		Servo_Position_Adjust = 2;									//do not move
		if (Servo_Position_Adjust_Old != 2)
			Send_Servo_Flag = TRUE;
		
		Servo_Position_Adjust_Old = Servo_Position_Adjust;
	}

	else if (Servo_Button_Reset != 0)								//none pressed and not reset
	{
		Servo_Position_Adjust = 2;									//do not move
		if (Servo_Position_Adjust_Old != 2 || Servo_Position_Adjust_Old == 0)
			Send_Servo_Flag = TRUE;

		Servo_Position_Adjust_Old = Servo_Position_Adjust;
	}

	//reset button is superior to all others
	if (Servo_Button_Reset == 0)		
	{
		Servo_Position_Adjust = 0;									//reset to default position
		//if (Servo_Position_Adjust_Old != 0)
			Send_Servo_Flag = TRUE;

		Servo_Position_Adjust_Old = Servo_Position_Adjust;
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//send joystick translated motion data to Subsea
void sendControlsToSubsea(const byte LeftRightArg, const byte FwdBwdArg, const byte UpDownArg, const byte LightsArg, const byte ServoArg)
{
	digitalWrite(PIN_RS485_MODE, HIGH);		//DE=RE=high transmit enabled
	digitalWrite(PIN_LED_TX, HIGH);			//send LED on

	delay(3);

	if (Send_Motors_Flag || Send_Flag)					//send movement related data
	{
		//send Forward/Backword motion data
		Serial3.write(START_X_MSG_ID);
		Serial3.write(LeftRightArg);
		Serial3.write(STOP_X_MSG_ID);

		//send Left/Right motion data
		Serial3.write(START_Y_MSG_ID);
		Serial3.write(FwdBwdArg);
		Serial3.write(STOP_Y_MSG_ID);

		//send Up/Down motion data
		Serial3.write(START_Z_MSG_ID);
		Serial3.write(UpDownArg);
		Serial3.write(STOP_Z_MSG_ID);

		Send_Motors_Flag = FALSE;
		Send_Flag = FALSE;

		SendWdog_Timestamp = millis();
	}

	if (Send_Lights_Flag)					//send lights on/off data
	{
		Serial3.write(START_LIGHTS_MSG_ID);
		Serial3.write(LightsArg);
		Serial3.write(STOP_LIGHTS_MSG_ID);

		Send_Lights_Flag = FALSE;
	}

	if (Send_Servo_Flag)					//send servo positioning data
	{
		Serial3.write(START_SERVO_MSG_ID);
		Serial3.write(ServoArg);
		Serial3.write(STOP_SERVO_MSG_ID);

		Send_Servo_Flag = FALSE;
	}

	delay(7);								//6ms to clock out last three bytes for bitrate >= 28800 min
	digitalWrite(PIN_LED_TX, LOW);			//send LED off
	digitalWrite(PIN_RS485_MODE, LOW);		//DE=RE=low transmit disabled
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//beeper
void beep(byte tuple, uint16 beepspan)
{
	for (byte i = 0; i < tuple; i++)
	{
		analogWrite(PIN_BUZZER_LEAK_ALARM, 100);
		delay(beepspan);
		analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
		delay(beepspan);
	}
}


//flag to indicate that lost comms have already been communicated
bool Beep_Flag = FALSE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//checks when the last comms took place
void watchdog(const uint32 timestamp)
{
	//check if comms were ok
	if (Get_Wdog_Timestamp == TRUE)
	{
		Wdog_Timestamp = millis();
		Get_Wdog_Timestamp = FALSE;
	}

	if (long(timestamp - Wdog_Timestamp) > ZERO_DATA_DELAY) // lack of comms; zero the displayed data
	{
		if (!Beep_Flag)	//beep only once
		{
			for (byte i = 0; i < sizeof(Received_Packet); i++)
				Received_Packet[i] = 0;

			beep(1, ZERO_DATA_BEEP);
		}

		Beep_Flag = TRUE;
	}

	else
		Beep_Flag = FALSE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//setup
void setup()
{	
	//display
	MyTFT.begin();							//initialise LCD
	MyTFT.background(ST7735_BLACK);			//set the background

	//display date
	dispDate();

	//display 'Surface:'
	MyTFT.setTextColor(0x2e8b);
	MyTFT.setCursor(SECTIONS_X, TOPSIDE_Y);
	MyTFT.print(F("Surface:"));

	//display 'Subsea:'
	MyTFT.setCursor(SECTIONS_X, SUBSEA_Y);
	MyTFT.setTextColor(ST7735_RED);
	MyTFT.print(F("UROV:"));

	//display 'degC' for air values
	MyTFT.setTextSize(1);
	MyTFT.setTextColor(0x2e8b);
	MyTFT.setCursor(DEG_AIR_X, DEG_AIR_Y);
	MyTFT.print(F("o"));
	MyTFT.setTextSize(2);
	MyTFT.setCursor(DEG_AIR_X + 8, DEG_AIR_Y);
	MyTFT.print(F("C"));

	//display 'degC' for water values
	MyTFT.setTextSize(1);
	MyTFT.setCursor(DEG_WATER_X, DEG_WATER_Y);
	MyTFT.print(F("o"));
	MyTFT.setTextSize(2);
	MyTFT.setCursor(DEG_WATER_X + 8, DEG_WATER_Y);
	MyTFT.print(F("C"));

	//display 'hPa' for air pressure value
	MyTFT.setCursor(HPA_X, HPA_Y);
	MyTFT.print(F("hPa"));

	//display 'bar' for water pressure
	MyTFT.setCursor(BAR_X, BAR_Y);
	MyTFT.print(F("bar"));

	//display 'm' for depth
	MyTFT.setCursor(METER_X, METER_Y);
	MyTFT.print(F("m"));

	//display 'm' for max depth
	MyTFT.setTextSize(1);
	MyTFT.setCursor(MAX_METER_X, MAX_METER_Y);
	MyTFT.print(F("m"));

	//display runtime related static text
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.setCursor(RUNTIME_X, RUNTIME_Y);
	MyTFT.print(F("Runtime:"));
	MyTFT.setCursor(RUNTIME_X + 63, RUNTIME_Y);
	MyTFT.print(F("h"));
	MyTFT.setCursor(RUNTIME_X + 85, RUNTIME_Y);
	MyTFT.print(F("m"));
	MyTFT.setCursor(RUNTIME_X + 105, RUNTIME_Y);
	MyTFT.print(F("s"));

	//display 'Max:'
	MyTFT.setTextColor(ST7735_BLUE);
	MyTFT.setCursor(SECTIONS_X, MAX_Y);
	MyTFT.print(F("Max:"));

	//set LED pins to output
	pinMode(PIN_LED_POWER_SUPPLY, OUTPUT);
	pinMode(PIN_LED_LEAK_ALARM, OUTPUT);
	pinMode(PIN_LED_TX, OUTPUT);
	pinMode(PIN_LED_RX, OUTPUT);

	//set joystick pins to input
	pinMode(PIN_JOYSTICK_BUTTON, INPUT);
	pinMode(PIN_BUTTON_UP, INPUT);
	pinMode(PIN_BUTTON_DOWN, INPUT);
	pinMode(PIN_BUTTON_LIGHTS, INPUT);
	pinMode(PIN_BUTTON_SERVO_UP, INPUT);
	pinMode(PIN_BUTTON_SERVO_DOWN, INPUT);
	pinMode(PIN_BUTTON_SERVO_RESET, INPUT);

	//enable internal pull-up resistors on inputs
	digitalWrite(PIN_JOYSTICK_BUTTON, HIGH);
	digitalWrite(PIN_BUTTON_UP, HIGH);
	digitalWrite(PIN_BUTTON_DOWN, HIGH);
	digitalWrite(PIN_BUTTON_LIGHTS, HIGH);
	digitalWrite(PIN_BUTTON_SERVO_UP, HIGH);
	digitalWrite(PIN_BUTTON_SERVO_DOWN, HIGH);
	digitalWrite(PIN_BUTTON_SERVO_RESET, HIGH);

	//locate BMP air/press sensor - I2C protocol
	foundBmpSensor = BMP.begin();

	//increase speed of I2C hardware clock from 100k (default) to 400kHz (max)
	//needs to be called after Wire.begin() invoked by I2C sensors
	Wire.setClock(400000L);						

	//RS485
	pinMode(PIN_RS485_MODE, OUTPUT);			//DE/RE Data Enable/Receive Enable transmit/receive pin of RS-485
	Serial3.begin(BITRATE, SERIAL_SETTINGS);	//open Serial Port for RS485 comms
	Serial.begin(BITRATE);						//open Serial Port for RS485 comms

	//light Power Supply LED
	digitalWrite(PIN_LED_POWER_SUPPLY, HIGH);
}//end of setup


/////////////////////////////////////////////////////////////////////////////////////////////////
//main program
void loop()
{
	Cycle_Timestamp = millis();

	//set flag for runtime update (in ms)
	if (Cycle_Timestamp - Runtime_Timestamp > RUNTIME_UPDATE)
		RunTime_Flag = TRUE;

	//set flag for current time update (in ms)
	if (Cycle_Timestamp - Time_Timestamp > TIME_UPDATE)
		Time_Flag = TRUE;

	//set flag for measurements update (in ms)
	if (Cycle_Timestamp - Measurements_Timestamp > MSRMTS_TOPSIDE_UPDATE)
		Measurements_Flag = TRUE;

	//set flag to force sending the joystick data, in case last updated packet was not received by Subsea
	//prevents Auto-Stop Subsea Safety System as well as Auto-Recovery Subsea Safety System to be triggered 
	if (Cycle_Timestamp - SendWdog_Timestamp > SEND_CMDS_INTERVAL)
		Send_Flag = TRUE;

	if (RunTime_Flag)
	{
		eraseRuntime();
		dispRuntime();

		Runtime_Timestamp = millis();
		RunTime_Flag = FALSE;
	}

	if (Time_Flag)
	{
		eraseTime();						
		dispTime();							
			
		Time_Timestamp = millis();			
		Time_Flag = FALSE;					
	}

	if (Measurements_Flag)
	{
		eraseAirTempPress();
		dispAirTempPress();

		eraseWaterTempPressDpth();
		dispWaterTempPressDpth();

		setWaterLeakAlarm();

		Measurements_Timestamp = millis();
		Measurements_Flag = FALSE;
	}

	getCmd();

	if (!Send_Flag)
	{
		receiveSubseaTelemetryData();
	}

	if (Send_Flag || Send_Motors_Flag || Send_Lights_Flag || Send_Servo_Flag)
	{
		sendControlsToSubsea(Left_Right_Joystick_Motion, Forward_Backward_Joystick_Motion, UpDown_Motion_Adjust, Lights_Toggle, Servo_Position_Adjust);
		/* flags set within sendControlsToSubsea() */
	} 

	watchdog(Cycle_Timestamp);

	//joystick stuff//////////////
	//uint16 butup = digitalRead(PIN_BUTTON_UP);
	//uint16 butdown = digitalRead(PIN_BUTTON_DOWN);
	//uint16 butsdown = digitalRead(PIN_BUTTON_SERVO_DOWN);
	//uint16 butsup = digitalRead(PIN_BUTTON_SERVO_UP);
	//uint16 butsres = digitalRead(PIN_BUTTON_SERVO_RESET);
	//uint16 butsurface = digitalRead(PIN_JOYSTICK_Z);

	//Serial3.print("UP = ");  Serial3.print(Up_Button_Motion); Serial3.print("\t");
	//Serial3.print("DOWN = "); Serial3.println(Down_Button_Motion); 
	//if (Left_Right_Joystick_Motion != 131 || Forward_Backward_Joystick_Motion != 124)
	//{
	//	Serial3.print("X = "); Serial3.print(Left_Right_Joystick_Motion); Serial3.print("\t");
	//	Serial3.print("Y = "); Serial3.println(Forward_Backward_Joystick_Motion);
	//}
	//Serial3.print("SERVO DOWN = "); Serial3.print(butsdown); Serial3.print("\t");
	//Serial3.print("SERVO UP = "); Serial3.println(butsup);
	//Serial3.print("SERVO RESET = "); Serial3.println(butsres);
	//Serial3.print("SURFACE (Z) = "); Serial3.println(butsurface);
	//delay(500);

	//Serial3.println("WTF");

	//beep(1, 750);

	//DEBUG:
	if (dispStats == true)
	{
		
		Serial.print("Rx GOOD: "); Serial.print(Rx_Good);
		Serial.print(" Rx Incomplete: "); Serial.print(Rx_Incomplete);
		Serial.print(" Rx Corrupted: "); Serial.println(Rx_Rubbish);
		Serial.print("Rx GOOD : Rx BAD  "); Serial.print(Rx_Good / (Rx_Incomplete + Rx_Rubbish)); Serial.print(":");
		Serial.println(1);
		dispStats = false;
	}

	//debug
	/*while (Serial3.available())
	{
		Serial.println(Serial3.read());
		
	}*/

}//loop
