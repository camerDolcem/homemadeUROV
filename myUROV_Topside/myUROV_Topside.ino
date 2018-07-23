/***********************************************************************************
 Name:
     myUROV_Subsea.ino
 Description:
     Topside controller
 Version:
     01
 Created:
	2017
 By:
	Jakub Kurkowski
***********************************************************************************/

#include "defs.h"
#include "msgID.h"

#include <TFT.h>							//LCD TFT library
#include <DS1307RTC.h>						//Real Time Clock library
#include <Adafruit_BMP280.h>				//air pressure and temperature sensor library


//pins definition for TFT screen
#define PIN_CS					53			//connects to CS pin
#define PIN_RST					50			//connects to RESET pin
#define PIN_DC					48			//connects to AD pin
//hardware SPI pins	SCK and MOSI (52 and 51 for Mega, 13 and 11 for UNO) connected to pins SCK and SDA of the LCD

//pin definitions for joystick

#define PIN_BUTTON_UP			A5	//A
#define PIN_BUTTON_DOWN			A3	//C
#define PIN_BUTTON_LIGHTS		A0	//F
#define PIN_BUTTON_SERVO_UP		A4	//B
#define PIN_BUTTON_SERVO_DOWN	A2	//D
#define PIN_BUTTON_SERVO_RESET	A1	//E

#define PIN_JOYSTICK_X			A6	//X
#define PIN_JOYSTICK_Y			A7	//Y
#define PIN_JOYSTICK_BUTTON		A8	//K

//pins definition for LEDs
#define PIN_LED_POWER_SUPPLY	7
#define PIN_LED_LEAK_ALARM		6
#define PIN_LED_TX				5
#define PIN_LED_RX				4

//pin definition for the leak alarm buzzer 
#define PIN_BUZZER_LEAK_ALARM	3

//pin definition for RS485 Serial comms
#define PIN_RS485_MODE			2

//for LCD TFT screen manipulations
TFT MyTFT = TFT(PIN_CS, PIN_DC, PIN_RST);	

//commms watchdog time data
bool Get_Wdog_Timestamp = TRUE;
uint32 Wdog_Timestamp = 0;		//Topside controller

uint32 SendWdog_Timestamp = 0;	//to send to Subsea controller


/////////////////////////////////////////////////////////////////////////////////////////////////
//display current runtime
void dispRuntime()
{
	//calculate full hrs and remaining mins and sePIN_CS
	uint32 timer = millis();				//ms total
	timer = timer * 0.001;					//s total
	uint16 timer_Min = timer / 60;			//full mins only
	byte timer_Hrs = timer_Min / 60;		//full hours only
	timer = timer - timer_Min * 60;			//s only (on top of minutes)
	timer_Min = timer_Min - timer_Hrs * 60;	//min only (on top of hours)

	//display runtime in the format: "hh hrs mm min ss s"
	MyTFT.setTextSize(1);
	MyTFT.setTextColor(ST7735_GREEN);
	MyTFT.setCursor(50, 153);
	if (timer_Hrs >= 0 && timer_Hrs < 10)
		MyTFT.print('0');
	MyTFT.print(timer_Hrs);

	MyTFT.setCursor(71, 153);
	if (timer_Min >= 0 && timer_Min < 10)
		MyTFT.print('0'); 
	MyTFT.print(timer_Min);

	MyTFT.setCursor(92, 153);
	MyTFT.print(timer);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase runtime
void eraseRuntime()
{
	MyTFT.fillRect(50, 153, 12, 7, ST7735_BLACK);
	MyTFT.fillRect(71, 153, 12, 7, ST7735_BLACK);
	MyTFT.fillRect(92, 153, 12, 7, ST7735_BLACK);
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
		MyTFT.setCursor(10, 0);
		MyTFT.setTextColor(ST7735_CYAN);
		if (Time.Hour >= 0 && Time.Hour < 10)
			MyTFT.print('0');
		MyTFT.print(Time.Hour);

		MyTFT.setTextColor(ST7735_WHITE);
		MyTFT.print(F(":"));
		
		//get minute
		MyTFT.setTextColor(ST7735_CYAN);
		if (Time.Minute >= 0 && Time.Minute < 10)
			MyTFT.print('0');
		MyTFT.print(Time.Minute);
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase current time
void eraseTime()
{
	MyTFT.fillRect(10, 0, 32, 7, ST7735_BLACK);
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
		MyTFT.setCursor(60, 0);
		MyTFT.setTextColor(ST7735_MAGENTA);
		if (Time.Day >= 0 && Time.Day < 10)
			MyTFT.print('0');
		MyTFT.print(Time.Day);

		//month
		MyTFT.setTextColor(ST7735_WHITE);
		MyTFT.print(F("."));
		MyTFT.setTextColor(ST7735_MAGENTA);
		if (Time.Month >= 0 && Time.Month < 10)
			MyTFT.print('0');
		MyTFT.print(Time.Month);

		//year
		MyTFT.setTextColor(ST7735_WHITE);
		MyTFT.print(F("."));
		MyTFT.setTextColor(ST7735_MAGENTA);
		MyTFT.print(tmYearToCalendar(Time.Year));
	}
}


Adafruit_BMP280 BMP;	//I2C protocol used
bool foundBmpSensor = FALSE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//display air pressure and temperature
void dispAirTemperatureAndPressure()
{
	float air_temperature = 0;
	float air_pressure = 0;

	if (foundBmpSensor)
	{
		air_temperature = (BMP.readTemperature() - 1.2);
		air_pressure = BMP.readPressure();
	}
	else	//try to locate lost sensor
	{
		foundBmpSensor = BMP.begin();
	}

	//temperature
	MyTFT.setTextSize(2);
	MyTFT.setCursor(10, 36);
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.print(air_temperature, 1);

	//pressure
	MyTFT.setCursor(10, 61);
	MyTFT.print(air_pressure / 100.0, 1);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase air temperature and pressure
void eraseAirTemperatureAndPressure()
{
	//temperature	
	MyTFT.fillRect(10, 36, 75, 14, ST7735_BLACK);
	//pressure
	MyTFT.fillRect(10, 61, 75, 14, ST7735_BLACK);
}


//define array for Serial message storage
//{4 bytes for water temp, 4 bytes for water press, 1 byte for water ingress}
byte Received_Packet[] = { 0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4, 0x1 };		
byte Received_Packet_New[] = { 0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4, 0x1 };

//define byte to store single byte from the message
byte Incoming_Byte = 0;
uint32 Rx_Good = 0; //debug
uint32 Rx_Incomplete = 0; //debug
uint32 Rx_Rubbish = 0; //debug
bool dispStats = TRUE; //debug

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
					Serial3.readBytes((Received_Packet_New + 0), 4);
					if (Serial3.read() == STOP_WATERTEMP_MSG_ID)
					{
						for (byte i = 0; i < 4; i++)
							Received_Packet[i] = Received_Packet_New[i];

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
					Serial3.readBytes((Received_Packet_New + 4), 4);
					if (Serial3.read() == STOP_WATERPRESS_MSG_ID)
					{
						for (byte i = 4; i < 8; i++)
							Received_Packet[i] = Received_Packet_New[i];

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
					Received_Packet_New[8] = Serial3.read();
					if (Serial3.read() == STOP_WATERING_MSG_ID)
					{
						Received_Packet[8] = Received_Packet_New[8];

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

	delay(2);				//only to make LED light visible
	digitalWrite(PIN_LED_RX, LOW);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//display water temperature and pressure
void dispWaterTemperaturePressureDepth()
{
	//define variables for water temperature and pressure values storage, bytes casted to float/byte
	float water_temperature = *(float *)Received_Packet;
	float water_pressure = *(float *)(Received_Packet + 4);

	//temperature
	MyTFT.setTextSize(2);
	MyTFT.setCursor(10, 92);
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.print(water_temperature, 1);					//degC

	//pressure
	MyTFT.setCursor(10, 113);
	MyTFT.print(water_pressure / 100000.0, 2);			//bar

	//depth
	MyTFT.setCursor(10, 134);
	MyTFT.print(water_pressure / (RHO * G), 2);			//= Pgauge[Pa] / (rho[kg/m^3] * g[m/s^2])
}														


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase water temperature and pressure
void eraseWaterTemperaturePressureDepth()
{
	//temperature	
	MyTFT.fillRect(10, 92, 75, 14, ST7735_BLACK);
	//pressure
	MyTFT.fillRect(10, 113, 75, 14, ST7735_BLACK);
	//depth
	MyTFT.fillRect(10, 134, 75, 14, ST7735_BLACK);
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


//define variables for initial values corresponding to joystick default position readings
//movement
byte Left_Right_Joystick_Motion = LEFT_RIGHT_DEFAULT;
byte Forward_Backward_Joystick_Motion = FRWRD_BCKWRD_DEFAULT;
byte Up_Button_Motion = 1;
byte Down_Button_Motion = 1;
byte UpDown_Button_Reset = 1;

byte Left_Right_Joystick_Motion_Old;
byte Forward_Backward_Joystick_Motion_Old;
byte UpDown_Motion_Adjust = 0;		//zero speed by default
byte UpDown_Motion_Adjust_Old = 2;	//0 - reset, 1 - slower/dive, 2 - do not change, 3 - faster/surface

//lights
byte Lights_Toggle = 0;				//off by default; 0 - off, 1 - on
byte Lights_Button = 1;				
byte Lights_Button_Old = 1;		

//servo
byte Servo_Position_Adjust = 0;		//horizontal position by default
byte Servo_Position_Adjust_Old = 2;	//0 - reset, 1 - lower, 2 - do not change, 3 - higher
byte Servo_Button_Lower = 1;
byte Servo_Button_Higher = 1;
byte Servo_Button_Reset = 1;

//set flag determining when to send new (only) joystick readings and suppress receiving any data temporarily
bool Send_Flag = TRUE;
bool Send_Motors_Flag = TRUE;
bool Send_Lights_Flag = TRUE;
bool Send_Servo_Flag = TRUE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//read joystick command
void getCmd()
{
	/* motors */

	//read in the joystick position, map to 1 byte size value
	Left_Right_Joystick_Motion = map( analogRead(PIN_JOYSTICK_X), 0, 1023, 0, 255 );
	Forward_Backward_Joystick_Motion = map( analogRead(PIN_JOYSTICK_Y), 0, 1023, 0, 255 );

	//take into account fluctuations on analog signal from the joystick when it is static
	//both when it is in its default (zero) position and when it swerved

	//FWD/BWD
	if ( Forward_Backward_Joystick_Motion >= (FRWRD_BCKWRD_DEFAULT - FLUCTUATION_ZERO ) &&
		Forward_Backward_Joystick_Motion <= (FRWRD_BCKWRD_DEFAULT + FLUCTUATION_ZERO ) )
		Forward_Backward_Joystick_Motion = FRWRD_BCKWRD_DEFAULT;

	if ( Forward_Backward_Joystick_Motion >= Forward_Backward_Joystick_Motion_Old + FLUCTUATION_NONZERO ||
		Forward_Backward_Joystick_Motion <= Forward_Backward_Joystick_Motion_Old - FLUCTUATION_NONZERO )
	{
		Forward_Backward_Joystick_Motion_Old = Forward_Backward_Joystick_Motion;
		Send_Motors_Flag = TRUE;
	}
	
	//left/right
	if (Left_Right_Joystick_Motion >= (LEFT_RIGHT_DEFAULT - FLUCTUATION_ZERO) &&
		Left_Right_Joystick_Motion <= (LEFT_RIGHT_DEFAULT + FLUCTUATION_ZERO))
		Left_Right_Joystick_Motion = LEFT_RIGHT_DEFAULT;

	if ( Left_Right_Joystick_Motion >= Left_Right_Joystick_Motion_Old + FLUCTUATION_NONZERO ||
		 Left_Right_Joystick_Motion <= Left_Right_Joystick_Motion_Old - FLUCTUATION_NONZERO )
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
	MyTFT.background(0, 0, 0);				//black background

	//display runtime related static text
	MyTFT.setTextSize(1);
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.setCursor(0, 153);
	MyTFT.print(F("Runtime:"));
	MyTFT.setCursor(63, 153);
	MyTFT.print(F("h"));
	MyTFT.setCursor(85, 153);
	MyTFT.print(F("m"));
	MyTFT.setCursor(105, 153);
	MyTFT.print(F("s"));

	//display date
	dispDate();

	//display 'Topside:'
	MyTFT.setCursor(0, 24);
	MyTFT.setTextColor(0x2e8b);
	MyTFT.print(F("Topside:"));

	//display 'Subsea:'
	MyTFT.setCursor(0, 80);
	MyTFT.setTextColor(ST7735_RED);
	MyTFT.print(F("Subsea:"));

	//display 'degC' for air and water temperature values
	MyTFT.setTextSize(1);
	MyTFT.setCursor(87, 34);
	MyTFT.setTextColor(0x2e8b);
	MyTFT.print(F("o"));
	MyTFT.setTextSize(2);
	MyTFT.setCursor(95, 36);
	MyTFT.print(F("C"));

	MyTFT.setTextSize(1);
	MyTFT.setCursor(87, 90);
	MyTFT.print(F("o"));
	MyTFT.setTextSize(2);
	MyTFT.setCursor(95, 92);
	MyTFT.print(F("C"));

	//display 'hPa' for air pressure value
	MyTFT.setCursor(90, 61);
	MyTFT.print(F("hPa"));

	//display 'bar' and 'm' for water pressure and depth
	MyTFT.setCursor(90, 113);
	MyTFT.print(F("bar"));
	MyTFT.setCursor(90, 134);
	MyTFT.print(F("m"));

	//set LED pins to output
	pinMode(PIN_LED_POWER_SUPPLY, OUTPUT);
	pinMode(PIN_LED_LEAK_ALARM, OUTPUT);
	pinMode(PIN_LED_TX, OUTPUT);
	pinMode(PIN_LED_RX, OUTPUT);

	//light Power Supply LED
	digitalWrite(PIN_LED_POWER_SUPPLY, HIGH);

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

	//RS485
	pinMode(PIN_RS485_MODE, OUTPUT);		//DE/RE Data Enable/Receive Enable transmit/receive pin of RS-485
	Serial3.begin(BITRATE, SERIAL_SETTINGS);		//open Serial Port for RS485 comms

	Serial.begin(BITRATE); 	//debug
	Serial.print("Bitrate: "); Serial.println(BITRATE); //debug

}//end of setup


//global data to define how often to retrieve and refresh data
uint32 Cycle_Timestamp = 0;	//timestamp of each separate cycle to compare against to

uint32 Runtime_Timestamp = 0;
bool RunTime_Flag = TRUE;

uint32 Time_Timestamp = 0;
bool Time_Flag = TRUE;

uint32 Measurements_Timestamp = 0;
bool Measurements_Flag = TRUE;

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
		eraseAirTemperatureAndPressure();
		dispAirTemperatureAndPressure();

		eraseWaterTemperaturePressureDepth();
		dispWaterTemperaturePressureDepth();

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

}//end of loop



