/*
  UROV v1

  Created in 2017 by JayK
  */

#include <TFT.h>  // Arduino LCD TFT library
#include <DS1307RTC.h>

// pin definition for Uno for TFT screen
#define cs   10
#define dc   9
#define rst  8 


TFT myTFT = TFT(cs, dc, rst);	//create an instance of the TFT class


////////////////////////////////////////////////////
//display runtime
void dispRuntime()
{
	char arr_elapsedSec[3];
	char arr_elapsedMin[3];
	char arr_elapsedHrs[3];

	//calculate full hrs and remaining mins and secs
	unsigned long timer = millis();			//ms total
	timer = timer * 0.001;					//s total
	unsigned int timerMin = timer / 60;		//full mins only
	byte timerHrs = timerMin / 60;			//full hours only
	timer = timer - timerMin * 60;			//s only (on top of minutes)
	timerMin = timerMin - timerHrs * 60;	//min only (on top of hours)

	//convert to strings
	String s_elapsedSec = String(timer);
	String s_elapsedMin = String(timerMin);
	String s_elapsedHrs = String(timerHrs);
	//convert to char arrays
	s_elapsedSec.toCharArray(arr_elapsedSec, 3);
	s_elapsedMin.toCharArray(arr_elapsedMin, 3);
	s_elapsedHrs.toCharArray(arr_elapsedHrs, 3);

	//display elapsed time
	myTFT.setTextColor(ST7735_GREEN);
	myTFT.setCursor(50, 153);
	myTFT.print(arr_elapsedHrs);
	myTFT.setCursor(71, 153);
	myTFT.print(arr_elapsedMin);
	myTFT.setCursor(92, 153);
	myTFT.print(arr_elapsedSec);
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
	tmElements_t tm;				//create an instance of the tmElements_t class

	if (RTC.read(tm)) 
	{ 
		myTFT.setCursor(10, 0);
		myTFT.setTextColor(ST7735_CYAN);
		if (tm.Hour >= 0 && tm.Hour < 10)
			myTFT.print('0');
		myTFT.print(tm.Hour);
		myTFT.setTextColor(ST7735_WHITE);
		myTFT.print(F(":"));
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
	myTFT.fillRect(10, 0, 12, 7, ST7735_BLACK);
	myTFT.fillRect(27, 0, 12, 7, ST7735_BLACK);
}


////////////////////////////////////////////////////
//display current date from RTC
void dispDate()
{
	tmElements_t tm;				//create an instance of the tmElements_t class

	if (RTC.read(tm))
	{		
		//display current date
		myTFT.setCursor(60, 0);
		myTFT.setTextColor(ST7735_MAGENTA);
		if (tm.Day >= 0 && tm.Day < 10)
			myTFT.print('0');
		myTFT.print(tm.Day);
		myTFT.setTextColor(ST7735_WHITE);
		myTFT.print(F("."));
		myTFT.setTextColor(ST7735_MAGENTA);
		if (tm.Month >= 0 && tm.Month < 10)
			myTFT.print('0');
		myTFT.print(tm.Month);
		myTFT.setTextColor(ST7735_WHITE);
		myTFT.print(F("."));
		myTFT.setTextColor(ST7735_MAGENTA);
		myTFT.print(tmYearToCalendar(tm.Year));
	}
}


////////////////////////////////////////////////////
//setup
void setup()
{	
	// initiate LCD and make bg black
	myTFT.begin();
	myTFT.background(0, 0, 0);

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
	//myTFT.setTextSize(1);
	//myTFT.setTextColor(ST7735_WHITE);
	dispDate();
}

//variables to define how often to refresh info on the screen
unsigned long runtime_timestamp = 0;
unsigned long time_timestamp = 0;
boolean runtimeFlag = true;
boolean timeFlag = true;


////////////////////////////////////////////////////
//main program
void loop()
{
	//set flag for the time to elapse before another update of display (in ms)
	if (millis() - runtime_timestamp >= 2000)
		runtimeFlag = true;

	//set flag for the time to elapse before another update of display (in ms)
	if (millis() - time_timestamp >= 5000)
		timeFlag = true;

	if (timeFlag)
	{
		eraseTime();					//erase current time
		dispTime();						//display current time
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



