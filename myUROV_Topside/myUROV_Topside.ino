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
//hardware SPI pins	SCK and MOSI (13 and 11) connect to pins SCK and SDA

//pin definitions for joystick

#define PIN_BUTTON_UP			A0
#define PIN_BUTTON_DOWN			A1
#define PIN_BUTTON_LIGHTS		A2
#define PIN_BUTTON_TILT_DOWN	A3
#define PIN_BUTTON_TILT_DOWN	A4
#define PIN_JOYSTICK_Y			A5
#define PIN_JOYSTICK_X			A6

//pins definition for LEDs
#define PIN_LED_POWER_SUPPLY	13
#define PIN_LED_LEAK_ALARM		12
#define PIN_LED_TX				11
#define PIN_LED_RX				10

//pin definition for the leak alarm buzzer 
#define PIN_BUZZER_LEAK_ALARM	9

//pin definition for RS485 serial comms
#define PIN_RS485_MODE			8

//for LCD TFT screen manipulations
TFT MyTFT = TFT(PIN_CS, PIN_DC, PIN_RST);				


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


/////////////////////////////////////////////////////////////////////////////////////////////////
//display air pressure and temperature
void dispAirTemperatureAndPressure()
{
	Adafruit_BMP280 BMP;
	float air_temperature = 0;
	float air_pressure = 0;

	if (BMP.begin())
	{
		air_temperature = (BMP.readTemperature() - 1.2);
		air_pressure = BMP.readPressure();
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


//define array for serial message storage
byte Received_Packet[] = { 0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4, 0x6 };	//{4 bytes for water temp, 4 bytes for water press, 1 byte for water ingress}

//define byte to store single byte from the message
byte Incoming_Byte = 0;

//define receive alarm flag that zeroes the received packet after specified interval 
uint32 Comms_ALarm_Timestamp = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////
//receive water pressure, temperature and leak level data
void receiveSubseaTelemetryData()
{
	digitalWrite(PIN_RS485_MODE, LOW);					//DE=RE=low receive enabled
	
	if (millis() - Comms_ALarm_Timestamp > 10000)
	{
		for (byte i = 0; i < sizeof(Received_Packet); i++)
			Received_Packet[i] = 0;
	}

	while (Serial1.available() >= 6)						//6 bytes defines biggest type variable message - float
	{	
		digitalWrite(PIN_LED_RX, HIGH);
		Comms_ALarm_Timestamp = millis();

		Incoming_Byte = Serial1.read();

		if (Incoming_Byte == START_MSG_ID)
		{
			Incoming_Byte = Serial1.read();

			switch (Incoming_Byte)
				{
					case WATER_TEMPERATURE_ID:
						Serial1.readBytes(Received_Packet, 4);
						break;
							
					case WATER_PRESSURE_ID:
						Serial1.readBytes((Received_Packet + 4), 4);
						break;

					case WATER_INGRESS_ID:
						Serial1.readBytes((Received_Packet + 8), 1);
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
						break;
					}
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

	if (Water_Ingress_Alarm >= 3 & Water_Ingress_Alarm <= 5)	//that indicates humidity in the enclosure most likely
	{
		digitalWrite(PIN_LED_LEAK_ALARM, HIGH);
		analogWrite(PIN_BUZZER_LEAK_ALARM, 0);
	}

	else if (Water_Ingress_Alarm >= 6)							//that indicates probable leak
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
byte Lights_OnOff = 1;
byte Left_Right_Motion_Old;
byte Forward_Backward_Motion_Old;
byte Up_Motion_Old;
byte Down_Motion_Old;
byte Lights_OnOff_Old;

//set flag determining when to send new (only) joystick readings and suppress receiving any data temporarily
bool Send_Flag = TRUE;

/////////////////////////////////////////////////////////////////////////////////////////////////
//read joystick command
void getCmd()
{
	Left_Right_Motion = map(analogRead(PIN_JOYSTICK_X), 0, 1023, 0 , 255);
	Forward_Backward_Motion = map(analogRead(PIN_JOYSTICK_Y), 0, 1023, 0, 255);
	//take into account fluctuations on analog signal from the joystick
	if (Left_Right_Motion >= 129 && Left_Right_Motion <= 132)				
		Left_Right_Motion = 131;
	
	if (Forward_Backward_Motion >= 123 && Forward_Backward_Motion <= 125)
		Forward_Backward_Motion = 124;

	Up_Motion = digitalRead(PIN_BUTTON_UP);
	Down_Motion = digitalRead(PIN_BUTTON_DOWN);
	Lights_OnOff = digitalRead(PIN_BUTTON_LIGHTS);

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
	
	if (Lights_OnOff == 0)	//send always if the button is pressed
	{
		Lights_OnOff_Old = Lights_OnOff;
		Send_Flag = TRUE;
	}

	if (Lights_OnOff_Old == 0 && Lights_OnOff == 1)	//send for change from 0 to 1 to notify button was released faster
	{
		Lights_OnOff_Old = Lights_OnOff;
		Send_Flag = TRUE;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//send joystick translated motion data to Subsea
void sendControlsToSubsea(byte& LeftRightArg, byte& FwdBwdArg, byte& UpArg, byte& DownArg, byte& LightsArg)
{
	digitalWrite(PIN_LED_TX, HIGH);
	digitalWrite(PIN_RS485_MODE, HIGH);		//DE=RE=high transmit enabled
	delay(1);

	//send Forward/Backword motion data
	Serial1.write(START_CTRL_ID);
	Serial1.write(X_MSG_ID);
	Serial1.write(LeftRightArg);

	//send Left/Right motion data
	Serial1.write(START_CTRL_ID);
	Serial1.write(Y_MSG_ID);
	Serial1.write(FwdBwdArg);

	//send Up motion data
	Serial1.write(START_CTRL_ID);
	Serial1.write(Z1_MSG_ID);
	Serial1.write(UpArg);

	//send Down motion data
	Serial1.write(START_CTRL_ID);
	Serial1.write(Z2_MSG_ID);
	Serial1.write(DownArg);

	//send lights on/off cmd
	Serial1.write(START_CTRL_ID);
	Serial1.write(L_MSG_ID);
	Serial1.write(LightsArg);

	delay(7);								//time needed to clock out last three bytes for bitrate >= 28800
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
	pinMode(PIN_BUTTON_LIGHTS, INPUT);

	//enable internal pull-up resistors
	digitalWrite(PIN_BUTTON_UP, HIGH);
	digitalWrite(PIN_BUTTON_DOWN, HIGH);
	digitalWrite(PIN_BUTTON_LIGHTS, HIGH);

	//RS485
	pinMode(PIN_RS485_MODE, OUTPUT);		//DE/RE Data Enable/Receive Enable transmit/receive pin of RS-485
	Serial1.begin(BITRATE, SERIAL_8E1);		//open Serial Port for RS485 comms

	//debug
	//Serial.begin(BITRATE);
}//end of setup


//global data to define how often to retrieve and refresh data
uint32 Runtime_Timestamp = 0;
bool RunTime_Flag = TRUE;

uint32 Time_Timestamp = 0;
bool Time_Flag = TRUE;

uint32 Measurements_Timestamp = 0;
bool Measurements_Flag = TRUE;

uint32 Send_Timestamp = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////
//main program
void loop()
{
	//set flag for updating runtime (in ms)
	if (millis() - Runtime_Timestamp > 2000)
		RunTime_Flag = TRUE;

	//set flag for current time update (in ms)
	if (millis() - Time_Timestamp > 15000)
		Time_Flag = TRUE;

	//set flag for current time update (in ms)
	if (millis() - Measurements_Timestamp > 3000)
		Measurements_Flag = TRUE;

	//set flag to force sending the joystick data, in case last updated packet was not received by Subsea
	//prevents Auto-Stop Subsea Safety System as well as Auto-Recovery Subsea Safety System to be triggered 
	if (millis() - Send_Timestamp > 2000)
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

	if (Send_Flag)
	{
		sendControlsToSubsea(Left_Right_Motion, Forward_Backward_Motion, Up_Motion, Down_Motion, Lights_OnOff);
		Send_Timestamp = millis();
		Send_Flag = FALSE;
	} 
		

		
	//joystick stuff//////////////
	//uint16 butup = digitalRead(PIN_BUTTON_UP);
	//uint16 butdown = digitalRead(PIN_BUTTON_DOWN);
	//uint16 anaX = analogRead(PIN_JOYSTICK_X);
	//uint16 anaY = analogRead(PIN_JOYSTICK_Y); 
	
	/*Serial.print("UP = ");  Serial.print(Up_Motion); Serial.print("\t");
	Serial.print("DOWN = "); Serial.println(Down_Motion);
	Serial.print("X = "); Serial.print(Left_Right_Motion); 
	Serial.print("Y = "); Serial.print(Forward_Backward_Motion);*/
	
}//end of loop



