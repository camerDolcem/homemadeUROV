/*
  UROV v1

  Topside controller

  Created in 2017 by JK
  */

#include <TFT.h>							//LCD TFT library
#include <DS1307RTC.h>						//RTC library
#include <Adafruit_BMP280.h>				//air pressure and temperature library

//pins definition for Uno for TFT screen
#define cs					10
#define dc					9
#define rst					8 

//pins definition for LEDs
#define pin_LED_PS			7
#define pin_LED_WI_Alarm	6

//pins definition for RS485 serial comms
#define pin_RS485_mode		2

//global data for LCD TFT
TFT myTFT = TFT(cs, dc, rst);				//create an instance

//global data for BMP280 sensor
Adafruit_BMP280 BMP;						//create an instance

//global data for RS485 telemetry data transmission
const int STRUCT_SIZE = 4;					//the size of struct/packet in bytes
byte expected_ID = 0x02;
byte incoming_byte;
byte buffer[STRUCT_SIZE - 1];

float water_temperature = 0.0;
unsigned int water_pressure = 0;
byte waterIngressAlarm = 0;

////////////////////////////////////////////////////
//display runtime
void dispRuntime()
{
	//calculate full hrs and remaining mins and secs
	unsigned long timer = millis();			//ms total
	timer = timer * 0.001;					//s total
	unsigned int timerMin = timer / 60;		//full mins only
	byte timerHrs = timerMin / 60;			//full hours only
	timer = timer - timerMin * 60;			//s only (on top of minutes)
	timerMin = timerMin - timerHrs * 60;	//min only (on top of hours)

	myTFT.setTextSize(1);
	myTFT.setTextColor(ST7735_GREEN);
	myTFT.setCursor(50, 153);
	myTFT.print(timerHrs);
	myTFT.setCursor(71, 153);
	myTFT.print(timerMin);
	myTFT.setCursor(92, 153);
	myTFT.print(timer);
}


////////////////////////////////////////////////////
//erase runtime
void eraseRuntime()
{
	myTFT.fillRect(50, 153, 12, 7, ST7735_BLACK);
	myTFT.fillRect(71, 153, 12, 7, ST7735_BLACK);
	myTFT.fillRect(92, 153, 12, 7, ST7735_BLACK);
}


////////////////////////////////////////////////////
//display current time from RTC
void dispTime() 
{
	tmElements_t tm;						//create an instance of the tmElements_t class

	if (RTC.read(tm)) 
	{ 
		//hour
		myTFT.setTextSize(1);
		myTFT.setCursor(10, 0);
		myTFT.setTextColor(ST7735_CYAN);
		if (tm.Hour >= 0 && tm.Hour < 10)
			myTFT.print('0');
		myTFT.print(tm.Hour);

		myTFT.setTextColor(ST7735_WHITE);
		myTFT.print(F(":"));
		
		//minute
		myTFT.setTextColor(ST7735_CYAN);
		if (tm.Minute >= 0 && tm.Minute < 10)
			myTFT.print('0');
		myTFT.print(tm.Minute);
	}
}


////////////////////////////////////////////////////
//erase current time
void eraseTime()
{
	myTFT.fillRect(10, 0, 32, 7, ST7735_BLACK);
}


////////////////////////////////////////////////////
//display current date from RTC
void dispDate()
{
	tmElements_t tm;						//create an instance of the tmElements_t class

	if (RTC.read(tm))
	{		
		//display current date
		//day
		myTFT.setCursor(60, 0);
		myTFT.setTextColor(ST7735_MAGENTA);
		if (tm.Day >= 0 && tm.Day < 10)
			myTFT.print('0');
		myTFT.print(tm.Day);

		//month
		myTFT.setTextColor(ST7735_WHITE);
		myTFT.print(F("."));
		myTFT.setTextColor(ST7735_MAGENTA);
		if (tm.Month >= 0 && tm.Month < 10)
			myTFT.print('0');
		myTFT.print(tm.Month);

		//year
		myTFT.setTextColor(ST7735_WHITE);
		myTFT.print(F("."));
		myTFT.setTextColor(ST7735_MAGENTA);
		myTFT.print(tmYearToCalendar(tm.Year));
	}
}


////////////////////////////////////////////////////
//display air pressure and temperature
void dispTempPress()
{
	float pressure = BMP.readPressure() / 100;
	float temperature = BMP.readTemperature();

	//temperature
	myTFT.setTextSize(3);
	myTFT.setCursor(10, 80);
	myTFT.setTextColor(ST7735_WHITE);
	myTFT.print(temperature, 1);

	//pressure
	myTFT.setTextSize(2);
	myTFT.setCursor(5, 35);
	myTFT.print(pressure, 1);
}


////////////////////////////////////////////////////
//erase air temperature and pressure
void eraseTempPress()
{
	//temperature	
	myTFT.fillRect(10, 80, 70, 22, ST7735_BLACK);
	//pressure
	myTFT.fillRect(5, 35, 82, 14, ST7735_BLACK);
}


////////////////////////////////////////////////////
//receive telemtry data from Subsea
void receivePacket()
{
	digitalWrite(pin_RS485_mode, LOW);												//DE=RE=high receive enabled

	if (Serial.available())
		{
			incoming_byte = Serial.read();	//reads first byte only
			
			if (incoming_byte == expected_ID)
			{
				Serial.readBytes(buffer, STRUCT_SIZE - 1);
				processBuffer(buffer);
			}
		}			
}


////////////////////////////////////////////////////
//translate data from buffer
void processBuffer(byte buffer[])
{
	water_temperature = map(buffer[0], 0, 255, 0, 40 * 100) / 100;
	
	water_pressure = map(buffer[1], 0, 255, 0, 12000);
	
	waterIngressAlarm = buffer[2];
}


////////////////////////////////////////////////////
//setup
void setup()
{	
	//display
	myTFT.begin();							//initialise LCD
	myTFT.background(0, 0, 0);				//black background

	//display runtime related static text
	myTFT.setTextSize(1);
	myTFT.setTextColor(ST7735_WHITE);
	myTFT.setCursor(0, 153);
	myTFT.print(F("Runtime:"));
	myTFT.setCursor(63, 153);
	myTFT.print(F("h"));
	myTFT.setCursor(85, 153);
	myTFT.print(F("m"));
	myTFT.setCursor(105, 153);
	myTFT.print(F("s"));

	//display date
	dispDate();

	//display 'degC'
	myTFT.setTextSize(2);
	myTFT.setCursor(90, 75);
	myTFT.setTextColor(0x2e8b);
	myTFT.print(F("o"));
	myTFT.setTextSize(3);
	myTFT.setCursor(105, 80);
	myTFT.print(F("C"));

	//display 'hPa'
	myTFT.setTextSize(2);
	myTFT.setCursor(91, 35);
	myTFT.print(F("hPa"));

	//initialise BMP280 sensor
	BMP.begin();

	//light Power Supply LED
	pinMode(pin_LED_PS, OUTPUT);
	digitalWrite(pin_LED_PS, HIGH);

	//RS485
	pinMode(pin_RS485_mode, OUTPUT);		//DE/RE Data Enable/Receive Enable transmit/receive pin of RS-485
	Serial.begin(9600);						//open Serial1 Port for RS485 comms
}


//global data to define how often to retrieve and refresh data
unsigned long runtime_timestamp = 0;
boolean runtime_flag = true;

unsigned long time_timestamp = 0;
boolean time_flag = true;

unsigned long retrieve_timestamp = 0;
boolean retrieve_flag = true;


////////////////////////////////////////////////////
//main program
void loop()
{
	//set flag for the time to elapse before another update of value displayed (in ms)
	if (millis() - runtime_timestamp >= 2000)
		runtime_flag = true;

	//set flag for the time to elapse before another update of value displayed (in ms)
	if (millis() - time_timestamp >= 3000)
		time_flag = true;

	//set flag for retrieving data from Subsea
	if (millis() - retrieve_timestamp >= 250)
		retrieve_flag = true;


	if (time_flag)
		{
			eraseTime();					//erase current time
			dispTime();						//display current time
			
			eraseTempPress();				//erase temperature and pressure
			dispTempPress();				//display current t
			
			time_timestamp = millis();		//update timestamp
			time_flag = false;				//set the flag back to false
		}

	if (runtime_flag)
		{
			eraseRuntime();					//erase runtime
			dispRuntime();					//display runtime

			runtime_timestamp = millis();	//update timestamp
			runtime_flag = false;			//set the flag back to false
		}




	if (retrieve_flag)
	{
		receivePacket();

		//temperature	
		myTFT.fillRect(70, 140, 35, 7, ST7735_BLACK);
		myTFT.fillRect(70, 130, 35, 7, ST7735_BLACK);
		myTFT.fillRect(70, 120, 35, 7, ST7735_BLACK);

		myTFT.setTextSize(1);
		myTFT.setCursor(0, 140);
		myTFT.setTextColor(0x2e8b);
		myTFT.print("water temp: ");
		myTFT.print(water_temperature);

		myTFT.setCursor(0, 130);
		myTFT.print("water pres: ");
		myTFT.print(water_pressure);

		myTFT.setCursor(0, 120);
		myTFT.print("water ingr: ");
		myTFT.print(waterIngressAlarm);

		retrieve_timestamp = millis();
		retrieve_flag = false;
	}



}



