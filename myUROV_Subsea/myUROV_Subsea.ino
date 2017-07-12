/*
  UROV v1

  Subsea controller

  Created in 2017 by JK
  */

#include <DallasTemperature.h>				//ds18b20 temp sensor library

//pins definition for 1-Wire bus
#define ONE_WIRE_BUS 3

//pins definition for water detection sensor
#define pin_WI A7

//pins definition for pressure transducer
#define pin_PR A6

//global data for 1-Wire
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


////////////////////////////////////////////////////
//get temperature measurement
float Temperature()
{
	sensors.requestTemperatures();
	float tempC = sensors.getTempCByIndex(0);
	
	return tempC;
}


////////////////////////////////////////////////////
//get water ingress level measurement
byte waterIngress()
{
	unsigned int water_ingress_read = analogRead(pin_WI);
	byte water_ingress = map(water_ingress_read, 0, 435, 0, 40);	//max measured vertical ingress level is 40mm 

	return water_ingress;
}


////////////////////////////////////////////////////
//get pressure measurement
int Pressure()
{
	int water_pressure = 0;
	const byte counter = 5;
	unsigned int water_pressure_read[counter];

	for (byte i = 1; i <= counter; i++)
		{
			water_pressure_read[i] = analogRead(pin_PR);
			water_pressure += water_pressure_read[i];
		}
		
	water_pressure = map((water_pressure / counter), 102, 922, 0, 12000);	//max measured vertical ingress level is 40mm 

	return water_pressure;
}


////////////////////////////////////////////////////
//setup
void setup()
{	
	//1-Wire
	sensors.begin();						//locate devices on the bus

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
	if (millis() - timestamp >= 500)
		tempFlag = true;

	if (tempFlag)
		{
			Serial.print("Temperature: ");
			Serial.print(Temperature(), 1);
			Serial.println(" (degC)");

			Serial.print("Water pressure: ");
			Serial.print(Pressure());
			Serial.println(" (hPa)");

			Serial.print("Water ingress: ");
			Serial.print(waterIngress());
			Serial.println(" (mm)");

			timestamp = millis();
			tempFlag = false;
		}
}





