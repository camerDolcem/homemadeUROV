/*
  UROV v1

  Subsea controller

  Created in 2017 by JK
  */

#include <DallasTemperature.h>				//ds18b20 temp sensor library

//pins definition for 1-Wire bus
#define ONE_WIRE_BUS 2

//global data for 1-Wire
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


////////////////////////////////////////////////////
//setup
void setup()
{	
	//1-Wire
	sensors.begin();						//locate device(s) on the bus

	Serial.begin(9600); 
}

//global data to define how often to retrieve data
unsigned long timestamp = 0;
boolean tempFlag = true;

////////////////////////////////////////////////////
//main program
void loop()
{
	//set flag for update of temp value retrieval (in ms)
	if (millis() - timestamp >= 1000)
		tempFlag = true;

	if (tempFlag)
		{
			printTemperature();			

			timestamp = millis();
			tempFlag = false;
		}
}

void printTemperature()
{
	sensors.requestTemperatures();
	float tempC = sensors.getTempCByIndex(0);
	Serial.print(tempC);
	//Serial.print(char(176));
	Serial.print("degC\n");
}



