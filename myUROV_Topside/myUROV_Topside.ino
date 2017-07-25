/*
****************************************************
  UROV v1

  Topside controller

  Created in 2017 by JK
****************************************************
*/

#include "defs.h"
#include "msgID.h"

#include <TFT.h>							//LCD TFT library
#include <DS1307RTC.h>						//Real Time Clock library
#include <Adafruit_BMP280.h>				//air pressure and temperature sensor library

//pins definition for TFT screen
#define PIN_CS				10
#define PIN_DC				9
#define PIN_RST				8 

//pins definition for LEDs
#define PIN_LED_PS			7
#define pin_LED_LEAK_ALARM	6

//pins definition for RS485 serial comms
#define PIN_RS485_MODE		2

//global object for LCD TFT screen manipulations
TFT MyTFT = TFT(PIN_CS, PIN_DC, PIN_RST);				


/////////////////////////////////////////////////////////////////////////////////////////////////
//display current runtime
void dispRuntime()
{
	//calculate full hrs and remaining mins and sePIN_CS
	ulong32 timer = millis();			//ms total
	timer = timer * 0.001;					//s total
	uint16 timerMin = timer / 60;			//full mins only
	byte timerHrs = timerMin / 60;			//full hours only
	timer = timer - timerMin * 60;			//s only (on top of minutes)
	timerMin = timerMin - timerHrs * 60;	//min only (on top of hours)

	//display runtime in the format: "hh hrs mm min ss s"
	MyTFT.setTextSize(1);
	MyTFT.setTextColor(ST7735_GREEN);
	MyTFT.setCursor(50, 153);
	if (timerHrs >= 0 && timerHrs < 10)
		MyTFT.print('0');
	MyTFT.print(timerHrs);

	MyTFT.setCursor(71, 153);
	if (timerMin >= 0 && timerMin < 10)
		MyTFT.print('0'); 
	MyTFT.print(timerMin);

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
	MyTFT.fillRect(10, 36, 47, 14, ST7735_BLACK);
	//pressure
	MyTFT.fillRect(10, 61, 70, 14, ST7735_BLACK);
}


//define array for serial message storage
byte ReceivedPacket[] = { 0x1, 0x2, 0x3, 0x4, 0x1, 0x2, 0x3, 0x4, 0x1 };	//{4 bytes for water temp, 4 bytes for water press, 1 byte for water ingress}

/////////////////////////////////////////////////////////////////////////////////////////////////
//receive telemetry data from Subsea
void receivePacket()
{
	digitalWrite(PIN_RS485_MODE, LOW);					//DE=RE=low receive enabled

	byte incoming_byte = 0;
			
	while (Serial.available() >= 6)						//6 bytes defines biggest type variable - float
	{
		incoming_byte = Serial.read();

		if (incoming_byte == START_MSG_ID)
			{
				incoming_byte = Serial.read();

				switch (incoming_byte)
				{
					case WATER_TEMPERATURE_ID:
						{
							Serial.readBytes(ReceivedPacket, 4);
						}
						break;
							
					case WATER_PRESSURE_ID:
						{	
							Serial.readBytes((ReceivedPacket + 4), 4);
						}
						break;

					case WATER_INGRESS_ID:
						{
							Serial.readBytes((ReceivedPacket + 8), 1);
						}
						break;

					default:						//corrupted packet								
						break;
				}//switch
			}//if
	}//while

	//digitalWrite(PIN_RS485_MODE, HIGH);			//DE=RE=high transmit enabled
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//display water temperature and pressure
void dispWaterTemperaturePressureDepth()
{
	//define variables for water temperature and pressure values storage, bytes casted to float/byte
	float water_temperature = *(float *)ReceivedPacket;
	float water_pressure = *(float *)(ReceivedPacket + 4);

	//temperature
	MyTFT.setCursor(10, 92);
	MyTFT.setTextColor(ST7735_WHITE);
	MyTFT.print(water_temperature, 1);					//degC

	//pressure
	MyTFT.setCursor(10, 113);
	MyTFT.print(water_pressure / 100000.0, 2);			//bar

	//depth
	MyTFT.setCursor(10, 134);
	MyTFT.print(water_pressure / (Rho * G), 2);			//= Pgauge[Pa]/( rho[kg/m^3] * g[m/s^2])
}														


/////////////////////////////////////////////////////////////////////////////////////////////////
//erase water temperature and pressure
void eraseWaterTemperaturePressureDepth()
{
	//temperature	
	MyTFT.fillRect(10, 92, 47, 14, ST7735_BLACK);
	//pressure
	MyTFT.fillRect(10, 113, 70, 14, ST7735_BLACK);
	//depth
	MyTFT.fillRect(10, 134, 70, 14, ST7735_BLACK);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//check water ingress level and set alarm
void checkWaterLeakAlarm()
{	
	byte WaterIngressAlarm = ReceivedPacket[8];
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

	//light Power Supply LED
	pinMode(PIN_LED_PS, OUTPUT);
	digitalWrite(PIN_LED_PS, HIGH);

	//RS485
	pinMode(PIN_RS485_MODE, OUTPUT);		//DE/RE Data Enable/Receive Enable transmit/receive pin of RS-485
	Serial.begin(115200);					//open Serial Port for RS485 comms
}//end of setup


//global data to define how often to retrieve and refresh data
ulong32 Runtime_Timestamp = 0;
bool RunTime_Flag = TRUE;

ulong32 Time_Timestamp = 0;
bool Time_Flag = TRUE;

ulong32 Measurements_Timestamp = 0;
bool Measurements_Flag = TRUE;

ulong32 Retrieve_Timestamp = 0;
bool Retrieve_Flag = TRUE;


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

	//set flag for retrieving data from Subsea
	if (millis() - Retrieve_Timestamp >= 500)
		Retrieve_Flag = TRUE;

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

			Measurements_Timestamp = millis();
			Measurements_Flag = FALSE;
		}

	if (Retrieve_Flag)
		{
			receivePacket();
		
			Retrieve_Timestamp = millis();
			Retrieve_Flag = FALSE;
		}
}//end of loop



