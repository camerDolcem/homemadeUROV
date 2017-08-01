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
#define PIN_CS					10			//connects to CS pin
#define PIN_RST					9			//connects to RESET pin
#define PIN_DC					8			//connects to AD pin
//hardware SPI pins	SCK and MOSI (13 and 11) connect to pins SCK and SDA

//pin definitions for joystick
#define PIN_BUTTON_DOWN			A3
#define PIN_BUTTON_UP			A2
#define PIN_JOYSTICK_Y			A1
#define PIN_JOYSTICK_X			A0

//pins definition for LEDs
#define PIN_LED_POWER_SUPPLY	7
#define PIN_LED_LEAK_ALARM		6
#define PIN_LED_TX				5
#define PIN_LED_RX				4

//pin definition for the leak alarm buzzer 
#define PIN_BUZZER_LEAK_ALARM	3

//pin definition for RS485 serial comms
#define PIN_RS485_MODE			2

//for LCD TFT screen manipulations
TFT MyTFT = TFT(PIN_CS, PIN_DC, PIN_RST);				


/////////////////////////////////////////////////////////////////////////////////////////////////
//display current runtime
void dispRuntime()
{
	//calculate full hrs and remaining mins and sePIN_CS
	ulong32 timer = millis();			//ms total
	timer = timer * 0.001;					//s total
	uint16 timer_Min = timer / 60;			//full mins only
	byte timer_Hrs = timer_Min / 60;			//full hours only
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


/////////////////////////////////////////////////////////////////////////////////////////////////
//display air pressure and temperature
void dispAirTemperatureAndPressure()
{
	Adafruit_BMP280 BMP;

	if (BMP.begin())
		{
			float air_temperature = (BMP.readTemperature() - 1.2);
			float air_pressure = BMP.readPressure();

			//temperature
			MyTFT.setTextSize(2);
			MyTFT.setCursor(10, 36);
			MyTFT.setTextColor(ST7735_WHITE);
			MyTFT.print(air_temperature, 1);

			//pressure
			MyTFT.setCursor(10, 61);
			MyTFT.print(air_pressure / 100.0, 1);
		}
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


//define array for serial message storage
byte Received_Packet[] = { 0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4, 0x6 };	//{4 bytes for water temp, 4 bytes for water press, 1 byte for water ingress}

//define byte to store single byte from the message
byte Incoming_Byte = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////
//receive water pressure, temperature and leak level data
void receiveSubseaTelemetryData()
{
	digitalWrite(PIN_RS485_MODE, LOW);					//DE=RE=low receive enabled
			
	while (Serial.available() >= 6)						//6 bytes defines biggest type variable message - float
	{	
		digitalWrite(PIN_LED_RX, HIGH);

		Incoming_Byte = Serial.read();

		if (Incoming_Byte == START_MSG_ID)
			{
				Incoming_Byte = Serial.read();

				switch (Incoming_Byte)
					{
						case WATER_TEMPERATURE_ID:
							{
								Serial.readBytes(Received_Packet, 4);
							}
							break;
							
						case WATER_PRESSURE_ID:
							{	
								Serial.readBytes((Received_Packet + 4), 4);
							}
							break;

						case WATER_INGRESS_ID:
							{
								Serial.readBytes((Received_Packet + 8), 1);
							}
							break;

						default:											//corrupted packet								
							{
								analogWrite(PIN_BUZZER_LEAK_ALARM, 100);	//beep for 0.5sec
								delay(35);
								analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
								delay(35);
								analogWrite(PIN_BUZZER_LEAK_ALARM, 100);	
								delay(35);
								analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
							}
							break;
					}//switch
			}//if
	}//while

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
	MyTFT.print(water_pressure / (RHO * G), 2);			//= Pgauge[Pa]/( rho[kg/m^3] * g[m/s^2])
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

	if (Water_Ingress_Alarm >= 3 & Water_Ingress_Alarm <= 5)//that indicates humidity in the enclosure most likely
		{
			digitalWrite(PIN_LED_LEAK_ALARM, HIGH);
			analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
		}

	else if (Water_Ingress_Alarm >= 6)						//that indicates probable leak
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
byte Left_Right_Motion = LEFT_RIGHT_DEFAULT;
byte Forward_Backward_Motion = FORWARD_BACKWARD_DEFAULT;
byte Up_Motion = 1;
byte Down_Motion = 1;
byte Left_Right_Motion_Old;
byte Forward_Backward_Motion_Old;
byte Up_Motion_Old;
byte Down_Motion_Old;

//set flag determining when to send new (only) joystick readings and suppress receiving any data temporarily
bool Send_Flag = TRUE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//read joystick command
void getCmd()
{
	Left_Right_Motion = map(analogRead(PIN_JOYSTICK_X), 0, 1023, 0 , 255);
	Forward_Backward_Motion = map(analogRead(PIN_JOYSTICK_Y), 0, 1023, 0, 255);
	Up_Motion = digitalRead(PIN_BUTTON_UP);
	Down_Motion = digitalRead(PIN_BUTTON_DOWN);

	if (Left_Right_Motion_Old != Left_Right_Motion)
		{
			Left_Right_Motion_Old = Left_Right_Motion;
			Send_Flag = TRUE;
		}
	if (Forward_Backward_Motion_Old != Forward_Backward_Motion)
		{
			Forward_Backward_Motion_Old = Forward_Backward_Motion;
			Send_Flag = TRUE;
		}
	if (Up_Motion_Old != Up_Motion)
		{
			Up_Motion_Old = Up_Motion;
			Send_Flag = TRUE;
		}
	if (Down_Motion_Old != Down_Motion)
		{
			Down_Motion_Old = Down_Motion;
			Send_Flag = TRUE;
		}
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//send joystick translated motion data to Subsea
void sendControlsToSubsea(byte& LeftRightArg, byte& FwdBwdArg, byte& UpArg, byte& DownArg)
{
	digitalWrite(PIN_LED_TX, HIGH);
	digitalWrite(PIN_RS485_MODE, HIGH);		//DE=RE=high transmit enabled
	delay(1);

	//send Forward/Backword motion data
	Serial.write(START_CTRL_ID);
	Serial.write(X_MSG_ID);
	Serial.write(LeftRightArg);

	//send Left/Right motion data
	Serial.write(START_CTRL_ID);
	Serial.write(Y_MSG_ID);
	Serial.write(FwdBwdArg);

	//send Up motion data
	Serial.write(START_CTRL_ID);
	Serial.write(Z1_MSG_ID);
	Serial.write(UpArg);

	//send Down motion data
	Serial.write(START_CTRL_ID);
	Serial.write(Z2_MSG_ID);
	Serial.write(DownArg);

	delay(7);								//safe amount of time to clock out last three bytes
	digitalWrite(PIN_RS485_MODE, LOW);		//DE=RE=low transmit disabled
	digitalWrite(PIN_LED_TX, LOW);
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

	//set LED pins to output
	pinMode(PIN_BUTTON_UP, INPUT);
	pinMode(PIN_BUTTON_DOWN, INPUT);

	//enable internal pull-up resistors
	digitalWrite(PIN_BUTTON_UP, HIGH);
	digitalWrite(PIN_BUTTON_DOWN, HIGH);

	//RS485
	pinMode(PIN_RS485_MODE, OUTPUT);		//DE/RE Data Enable/Receive Enable transmit/receive pin of RS-485
	Serial.begin(BITRATE);					//open Serial Port for RS485 comms
}//end of setup


//global data to define how often to retrieve and refresh data
ulong32 Runtime_Timestamp = 0;
bool RunTime_Flag = TRUE;

ulong32 Time_Timestamp = 0;
bool Time_Flag = TRUE;

ulong32 Measurements_Timestamp = 0;
bool Measurements_Flag = TRUE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//main program
void loop()
{
	//set flag for updating runtime (in ms)
	if (millis() - Runtime_Timestamp >= 2000)
		RunTime_Flag = TRUE;

	//set flag for current time update (in ms)
	if (millis() - Time_Timestamp >= 15000)
		Time_Flag = TRUE;

	//set flag for current time update (in ms)
	if (millis() - Measurements_Timestamp >= 3000)
		Measurements_Flag = TRUE;

	//set flag for retrieving telemetry data from Subsea
	//if (millis() - Retrieve_Timestamp >= 0)
	//	Retrieve_Flag = TRUE;

	//set flag for sending controls data to Subsea
	//if (millis() - Send_Timestamp >= 100)
	//	Send_Flag = TRUE;

	//digitalWrite(PIN_LED_RX, LOW);
	//digitalWrite(PIN_LED_TX, LOW);

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

	if (Send_Flag)
		{
			sendControlsToSubsea(Left_Right_Motion, Forward_Backward_Motion, Up_Motion, Down_Motion);
			Send_Flag = FALSE;
		} 
		

		
	//joystick stuff//////////////
	//uint16 butup = digitalRead(PIN_BUTTON_UP);
	//uint16 butdown = digitalRead(PIN_BUTTON_DOWN);
	//uint16 anaX = analogRead(PIN_JOYSTICK_X);
	//uint16 anaY = analogRead(PIN_JOYSTICK_Y); 
	//Serial.print("UP = ");  Serial.print(butup); Serial.print("\t");
	//Serial.print("DOWN = "); Serial.println(butdown); 
	//Serial.print("X = "); Serial.print(anaX); Serial.print(" or: "); Serial.println(map(anaX, 0, 1023, 0, 255));
	//Serial.print("Y = "); Serial.print(anaY); Serial.print(" or: "); Serial.println(map(anaY, 0, 1023, 0, 255));
	
}//end of loop



