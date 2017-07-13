/*
  UROV v1

  Topside controller

  Created in 2017 by JK
  */

#include <TFT.h>							//LCD TFT library
#include <DS1307RTC.h>						//RTC library
#include <Adafruit_BMP280.h>				//air pressure and temperature library

//pins definition for Uno for TFT screen
#define cs   10
#define dc   9
#define rst  8 

//pins definition for LEDs
#define pin_LED_PS			7
#define pin_LED_WI_Alarm	6 //if over AI 200

//global data for LCD TFT
TFT myTFT = TFT(cs, dc, rst);				//create an instance

//global data for BMP280 sensor
Adafruit_BMP280 BMP;						//create an instance

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
	myTFT.setCursor(10, 35);
	myTFT.print(pressure, 0);
}


////////////////////////////////////////////////////
//erase air temperature and pressure
void eraseTempPress()
{
	//temperature	
	myTFT.fillRect(10, 80, 70, 22, ST7735_BLACK);
	//pressure
	myTFT.fillRect(10, 35, 55, 14, ST7735_BLACK);
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
	myTFT.setCursor(65, 35);
	myTFT.print(F("hPa"));

	//initialise BMP280 sensor
	BMP.begin();

	//light Power Supply LED
	pinMode(pin_LED_PS, OUTPUT);
	digitalWrite(pin_LED_PS, HIGH);
}


//global data to define how often to refresh info on the screen
unsigned long runtime_timestamp = 0;
unsigned long time_timestamp = 0;
boolean runtimeFlag = true;
boolean timeFlag = true;


////////////////////////////////////////////////////
//main program
void loop()
{
	//set flag for the time to elapse before another update of value displayed (in ms)
	if (millis() - runtime_timestamp >= 2000)
		runtimeFlag = true;

	//set flag for the time to elapse before another update of value displayed (in ms)
	if (millis() - time_timestamp >= 3000)
		timeFlag = true;

	if (timeFlag)
		{
			eraseTime();					//erase current time
			dispTime();						//display current time
			
			eraseTempPress();				//erase temperature and pressure
			dispTempPress();				//display current t
			
			time_timestamp = millis();		//update timestamp
			timeFlag = false;				//set the flag back to false
		}

	if (runtimeFlag)
		{
			eraseRuntime();					//erase runtime
			dispRuntime();					//display runtime

			runtime_timestamp = millis();	//update timestamp
			runtimeFlag = false;			//set the flag back to false
		}
}



