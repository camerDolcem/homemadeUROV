/*
  UROV v1

  Subsea controller

  Created in 2017 by JK
  */

#include <DallasTemperature.h>				//ds18b20 temp sensor library

//pins definition for 1-Wire bus
#define ONE_WIRE_BUS 2

//pins definition for water detection sensor
#define pin_WI A7

//global data for 1-Wire
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


////////////////////////////////////////////////////
//get temperature measurement
void Temperature()
{
	sensors.requestTemperatures();
	float tempC = sensors.getTempCByIndex(0);
	Serial.print(tempC); //del later
	//Serial.print(char(176)); //del later
	Serial.println("degC");
}


////////////////////////////////////////////////////
//setup
void setup()
{	
	//1-Wire
	sensors.begin();						//locate device(s) on the bus

	//water ingress
	pinMode(pin_WI, INPUT);

	Serial.begin(9600); //del later
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
			Temperature();			
			unsigned int liquid_level = analogRead(pin_WI);
			Serial.print(liquid_level);
			Serial.println(" of water");
			timestamp = millis();
			tempFlag = false;
		}
}





