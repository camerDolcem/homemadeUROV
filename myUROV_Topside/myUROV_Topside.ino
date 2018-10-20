/***********************************************************************************
	Name:
		myUROV_Topside.ino
	Description:
		Topside (surface) controller functionality implementation.
	Version:
		1.0
	Created:
		Aug 2018
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

//joystick controls							//device interface pins
#define PIN_BUTTON_UP			12			//A
#define PIN_BUTTON_DOWN			A3			//C
#define PIN_BUTTON_LIGHTS		A0			//F
#define PIN_BUTTON_SERVO_UP		6			//B
#define PIN_BUTTON_SERVO_DOWN	A2			//D
#define PIN_BUTTON_SERVO_RESET	A1			//E

#define PIN_JOYSTICK_X			A6			//X
#define PIN_JOYSTICK_Y			A7			//Y
#define PIN_JOYSTICK_BUTTON		2//A8		//K

//LEDs 
//#define PIN_LED_POWER_SUPPLY	7
#define PIN_LED_LEAK_ALARM		3//6
#define PIN_LED_TX				4//5
#define PIN_LED_RX				5//4

//buzzer 
//#define PIN_BUZZER_LEAK_ALARM	6//3

//board-to-board comms
#define PIN_RS485_MODE			7//2		//Rx/Tx select

//LCD										//device interface pins (LCD)
#define PIN_SS					10//53		//CS (Chip (aka Slave) Select)
#define PIN_MISO				9//50		//RESET
#define PIN_DC					8//48		//A0 (Data/Command select)
//hardware pin SCK (Serial Clock) 			//SCK (SPI Clock input)	
//hardware pin MOSI (Master Out Slave In)  	//SDA (SPI Data input)

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

/* DEBUG - for comms stats only
uint32 Rx_Good =	0; 
uint32 Rx_Incomplete = 0; 
uint32 Rx_Rubbish = 0; 
bool dispStats = TRUE; 
*/

//commms watchdog updates
uint32 Wdog_Timestamp =		0;		//topside controller
bool Get_Wdog_Timestamp =	TRUE;
bool Beep_Flag =			FALSE;	//to indicate that lost comms have already been reestablished

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
	uint32 msecs =	millis();			//ms total
	uint32 secs =	msecs * 0.001;		//s total
	uint16 mins =	secs / 60;			//full mins only
	byte hrs =		mins / 60;			//full hours only
	secs =			secs - mins * 60;	//s only (on top of minutes)
	mins =			mins - hrs * 60;	//min only (on top of hours)

	//display runtime in the format: "[hh:][mm:]ss"
	MyTFT.setTextSize(1);
	MyTFT.setTextColor(ST7735_GRAY);
	MyTFT.setCursor(RUNTIME_VAL_X, RUNTIME_VAL_Y);
	
	if (hrs == 0)		//if hrs are 0
	{
		if (mins != 0)	//but mins are not
		{
			MyTFT.print(mins);
			MyTFT.print(F(":"));
		}
		if (secs < 10)	//add 0 in front
			MyTFT.print(F("0"));
		MyTFT.print(secs);
	}
	else				//if hrs > 0
	{
		MyTFT.print(hrs);
		MyTFT.print(F(":"));
		if (mins < 10)	//add 0 in front
			MyTFT.print(F("0"));
		MyTFT.print(mins);
		MyTFT.print(F(":"));
		if (secs < 10)	//add 0 in front
			MyTFT.print(F("0"));
		MyTFT.print(secs);
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase runtime
void eraseRuntime()
{

	MyTFT.fillRect(RUNTIME_VAL_X, RUNTIME_VAL_Y, 60, 7, ST7735_BLACK);
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
		MyTFT.setCursor(TIME_VAL_X, TIME_VAL_Y);
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
	MyTFT.fillRect(TIME_VAL_X, TIME_VAL_Y, 32, 7, ST7735_BLACK);
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
		MyTFT.setCursor(DATE_VAL_X, DATE_VAL_Y);
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
		air_temperature		= BMP.readTemperature() - 1.2;
		air_pressure		= BMP.readPressure();
	}
	else	//try to locate sensor not ready at startup (should not occur)
	{
		foundBmpSensor		= BMP.begin();
		Wire.setClock(400000L); //change from default 100k to 400kHz clock speed for I2C
	}

	//air temperature
	MyTFT.setTextSize(2);
	MyTFT.setCursor(AIRTEMP_VAL_X, AIRTEMP_VAL_Y);
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.print(air_temperature, 1);

	//air pressure
	MyTFT.setCursor(AIRPRESS_VAL_X, AIRPRESS_VAL_Y);
	MyTFT.print(air_pressure / 100.0, 1);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase air temperature and pressure
void eraseAirTempPress()
{
	//air temperature	
	MyTFT.fillRect(AIRTEMP_VAL_X, AIRTEMP_VAL_Y, 73, 14, ST7735_BLACK);
	//air pressure
	MyTFT.fillRect(AIRPRESS_VAL_X, AIRPRESS_VAL_Y, 73, 14, ST7735_BLACK);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//display water temperature and pressure
void dispWaterTempPressDpth()
{
	//define variables for water temperature and pressure values storage, bytes casted to float/byte
	float waterTemperature =	*(float *)(Received_Packet + 0);
	float waterPressurePa =		*(float *)(Received_Packet + 4);
	float waterPressureBar =	waterPressurePa / 100000.0;		//Pa to bar
	float waterDepth =			waterPressurePa / (RHO * G);	//Pgauge[Pa] / (rho[kg/m^3] * g[m/s^2])
	
	float tempMaxWaterDepth =	0.0;
	static float maxWaterDepth = 0.0;
	static float lastVals[3] =	{0.0, 0.0, 0.0};
	static byte index =			0;

	//disp water temperature
	MyTFT.setTextSize(2);
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.setCursor(WATERTEMP_VAL_X, WATERTEMP_VAL_Y);
	MyTFT.print(waterTemperature, 1);	//degC

	//disp water pressure
	MyTFT.setCursor(WATERPRESS_VAL_X, WATERPRESS_VAL_Y);
	MyTFT.print(waterPressureBar, 2);	//bar

	//disp depth
	MyTFT.setCursor(DEPTH_VAL_X, DEPTH_VAL_Y);
	MyTFT.print(waterDepth, 2);			//m

	//disp store last 3 measured depths to determine max but ignore spikes
	lastVals[index] =	waterDepth;
	tempMaxWaterDepth = min(lastVals[0], lastVals[1]);
	tempMaxWaterDepth = min(tempMaxWaterDepth, lastVals[2]);	//that is the 'min' max depth from last 3 readings

	maxWaterDepth =		max(tempMaxWaterDepth, maxWaterDepth);	//update actual max recorded depth

	if (index < 2)
	{
		index++;
	}
	else
	{
		index = 0;
	}

	//disp max depth
	MyTFT.setTextSize(1);
	MyTFT.setTextColor(ST7735_GRAY);
	MyTFT.setCursor(MAXDEPTH_VAL_X, MAXDEPTH_VAL_Y);
	MyTFT.print(maxWaterDepth, 2);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase water temperature and pressure
void eraseWaterTempPressDpth()
{
	//water temperature	
	MyTFT.fillRect(WATERTEMP_VAL_X, WATERTEMP_VAL_Y, 73, 14, ST7735_BLACK);
	//water pressure
	MyTFT.fillRect(WATERPRESS_VAL_X, WATERPRESS_VAL_Y, 73, 14, ST7735_BLACK);
	//water depth
	MyTFT.fillRect(DEPTH_VAL_X, DEPTH_VAL_Y, 73, 14, ST7735_BLACK);
	//max depth
	MyTFT.fillRect(MAXDEPTH_VAL_X, MAXDEPTH_VAL_Y, 29, 7, ST7735_BLACK);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//receive water pressure, temperature and leak level data
void receiveSubseaTelemetryData()
{
	while (Serial.available() >= 6)		//6 bytes defines biggest msg - float (1+4+1)
	{	
		digitalWrite(PIN_LED_RX, HIGH);		//receive LED on

		//process Rx buffer
		Incoming_Byte = Serial.read();

		switch (Incoming_Byte)
			{
				case START_WATERTEMP_MSG_ID:
				{
					Serial.readBytes((char *)(Incoming_Frame + 0), 4); //cast to make compiler stop to complain
					if (Serial.read() == STOP_WATERTEMP_MSG_ID)
					{
						for (byte i = 0; i < 4; i++)
							Received_Packet[i] = Incoming_Frame[i];

						//Rx_Good++; //debug
					}

					else	//corrupted packet - ignore
					{
						//Rx_Incomplete++; //debug
						//dispStats = TRUE; //debug
					}	
				}
				break;
							
				case START_WATERPRESS_MSG_ID:
				{
					Serial.readBytes((char *)(Incoming_Frame + 4), 4);
					if (Serial.read() == STOP_WATERPRESS_MSG_ID)
					{
						for (byte i = 4; i < 8; i++)
							Received_Packet[i] = Incoming_Frame[i];

						//Rx_Good++; //debug
					}

					else	//corrupted packet - ignore
					{
						//Rx_Incomplete++; //debug
						//dispStats = TRUE; //debug
					}	
				}
				break;

				case START_WATERING_MSG_ID:
				{
					Incoming_Frame[8] = Serial.read();
					if (Serial.read() == STOP_WATERING_MSG_ID)
					{
						Received_Packet[8] = Incoming_Frame[8];

						//when water ingress info is received, kick the watchdog
						Get_Wdog_Timestamp = TRUE;
						//Rx_Good++; //debug
					}

					else
					{
						//Rx_Incomplete++; //debug
						//dispStats = TRUE; //debug
					}	//corrupted packet - ignore 
				}
				break;

				default:	//corrupted packet								
				{
					//Rx_Rubbish++; //debug
					//dispStats = TRUE; //debug
				}
				break;
			
		}//switch
	}//while

	delay(2);	//only to make LED light visible
	digitalWrite(PIN_LED_RX, LOW);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//check water ingress level and set alarm
void checkWaterLeak()
{	
	byte Water_Ingress_Alarm = Received_Packet[8];
	static bool displayedAlarm = FALSE;

	if (Water_Ingress_Alarm > 0 & Water_Ingress_Alarm < 3)	//that indicates humidity in the enclosure most likely
	{
		//lit the alarm LED but do not buzz
		digitalWrite(PIN_LED_LEAK_ALARM, HIGH);
		//analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
	}

	else if (Water_Ingress_Alarm >= 3)						//that indicates probable leak
	{
		//lit the alarm LED and make noise
		digitalWrite(PIN_LED_LEAK_ALARM, HIGH);
		//analogWrite(PIN_BUZZER_LEAK_ALARM, 200);

		//display "LEAK!!!" alarm
		if (displayedAlarm == FALSE)
		{
			MyTFT.setTextSize(1);
			MyTFT.setTextColor(ST7735_BLUE);
			MyTFT.setCursor(ALARM_TXT_X, ALARM_TXT_Y);
			MyTFT.print(F("***LEAK!***"));
			displayedAlarm = TRUE;
		}
	}

	else
	{
		digitalWrite(PIN_LED_LEAK_ALARM, LOW);
		//analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
	}

	if (Water_Ingress_Alarm < 3 && displayedAlarm == TRUE)	//erase alarm txt
	{
		MyTFT.fillRect(ALARM_TXT_X, ALARM_TXT_Y, 65, 7, ST7735_BLACK);
		displayedAlarm = FALSE;
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
		Serial.write(START_X_MSG_ID);
		Serial.write(LeftRightArg);
		Serial.write(STOP_X_MSG_ID);

		//send Left/Right motion data
		Serial.write(START_Y_MSG_ID);
		Serial.write(FwdBwdArg);
		Serial.write(STOP_Y_MSG_ID);

		//send Up/Down motion data
		Serial.write(START_Z_MSG_ID);
		Serial.write(UpDownArg);
		Serial.write(STOP_Z_MSG_ID);

		Send_Motors_Flag = FALSE;
		Send_Flag = FALSE;

		SendWdog_Timestamp = millis();
	}

	if (Send_Lights_Flag)					//send lights on/off data
	{
		Serial.write(START_LIGHTS_MSG_ID);
		Serial.write(LightsArg);
		Serial.write(STOP_LIGHTS_MSG_ID);

		Send_Lights_Flag = FALSE;
	}

	if (Send_Servo_Flag)					//send servo positioning data
	{
		Serial.write(START_SERVO_MSG_ID);
		Serial.write(ServoArg);
		Serial.write(STOP_SERVO_MSG_ID);

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
		//analogWrite(PIN_BUZZER_LEAK_ALARM, 100);
		analogWrite(PIN_LED_LEAK_ALARM, 100);
		delay(beepspan);
		//analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
		analogWrite(PIN_LED_LEAK_ALARM, 0);
		delay(beepspan);
	}
}


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
	MyTFT.setTextSize(1);
	MyTFT.setTextColor(0x2e8b);
	MyTFT.setCursor(TOPSIDE_TXT_X, TOPSIDE_TXT_Y);
	MyTFT.print(F("Surface:"));

	//display 'Subsea:'
	MyTFT.setCursor(SUBSEA_TXT_X, SUBSEA_TXT_Y);
	MyTFT.setTextColor(ST7735_ORANGE);
	MyTFT.print(F("UROV:"));

	//display 'degC' for air temp value
	MyTFT.setTextColor(0x2e8b);
	MyTFT.setCursor(AIRTEMP_UN_X, AIRTEMP_UN_Y);
	MyTFT.print(F("o"));
	MyTFT.setTextSize(2);
	MyTFT.setCursor(AIRTEMP_UN_X + 8, AIRTEMP_UN_Y);
	MyTFT.print(F("C"));

	//display 'hPa' for air pressure value
	MyTFT.setCursor(AIRPRESS_UN_X, AIRPRESS_UN_Y);
	MyTFT.print(F("hPa"));

	//display 'degC' for water temp
	MyTFT.setTextSize(1);
	MyTFT.setCursor(WATERTEMP_UN_X, WATERTEMP_UN_Y);
	MyTFT.print(F("o"));
	MyTFT.setTextSize(2);
	MyTFT.setCursor(WATERTEMP_UN_X + 8, WATERTEMP_UN_Y);
	MyTFT.print(F("C"));

	//display 'bar' for water pressure value
	MyTFT.setCursor(WATERPRESS_UN_X, WATERPRESS_UN_Y);
	MyTFT.print(F("bar"));

	//display 'm' for depth
	MyTFT.setCursor(DEPTH_UN_X, DEPTH_UN_Y);
	MyTFT.print(F("m"));

	//display 'm' for max depth
	MyTFT.setTextSize(1);
	MyTFT.setCursor(MAXDEPTH_UN_X, MAXDEPTH_UN_Y);
	MyTFT.print(F("m"));

	//display runtime related static text
	MyTFT.setTextColor(ST7735_AQUA);
	MyTFT.setCursor(RUNTIME_TXT_X, RUNTIME_TXT_Y);
	MyTFT.print(F("Run time:"));
	/*MyTFT.setTextColor(0x2e8b);
	MyTFT.setCursor(RUNTIME_H_X, RUNTIME_Y);
	MyTFT.print(F("h"));
	MyTFT.setCursor(RUNTIME_M_X, RUNTIME_Y);
	MyTFT.print(F("m"));
	MyTFT.setCursor(RUNTIME_S_X, RUNTIME_Y); 
	MyTFT.print(F("s")); */

	//display 'Max:'
	MyTFT.setTextColor(ST7735_BLUE);
	MyTFT.setCursor(MAXDEPTH_TXT_X, MAXDEPTH_TXT_Y);
	MyTFT.print(F("Max dpth:")); 

	//set LED pins to output
	//pinMode(PIN_LED_POWER_SUPPLY, OUTPUT);
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
	Serial.begin(BITRATE, SERIAL_SETTINGS);	//open Serial Port for RS485 comms

	//light Power Supply LED
	//digitalWrite(PIN_LED_POWER_SUPPLY, HIGH);
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

		checkWaterLeak();

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

	/*DEBUG - display comms stats:
	if (dispStats == true)
	{
		Serial.print("Rx GOOD: "); Serial.print(Rx_Good);
		Serial.print(" Rx Incomplete: "); Serial.print(Rx_Incomplete);
		Serial.print(" Rx Corrupted: "); Serial.println(Rx_Rubbish);
		Serial.print("Rx GOOD : Rx BAD  "); Serial.print(Rx_Good / (Rx_Incomplete + Rx_Rubbish)); Serial.print(":");
		Serial.println(1);
		dispStats = false;
	} */

}//loop
