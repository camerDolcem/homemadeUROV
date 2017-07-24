/*
****************************************************
  UROV v1

  Subsea controller

  Created in 2017 by JK
****************************************************

Jake's style guide.

	Variable names.
----------------------------------------------------
	Local scope variables:
int water_temperature;
	Global variables:
int Water_Pressure;
	Constants:
const char k_Help_String[] = "Text";					-global scope
	Static variables:
static const char sk_Help_String[] = "Text";			-global scope

	Enumerator and union names.
----------------------------------------------------
enum ListOfIDs {START_MSG_ID};							-global scope
union DataPacket1 {float As_Float} TemperaturePacket;	-global scope

	Macro names.
----------------------------------------------------
#define PIN_ALARM_LED 1			

	Function names.
----------------------------------------------------
getWaterPressure();

	Object names.
----------------------------------------------------
OneWire OneWire;
*/

#include <DallasTemperature.h>							//ds18b20 temp sensor library

//general definitions
#define TRUE				true
#define FALSE				false

//pins definition for 1-Wire bus
#define PIN_ONE_WIRE_BUS	2

//pins definition for water detection sensor
#define PIN_WATER_INGRESS	7

//pins definition for pressure transducer
#define PIN_WATER_PRESSURE	6

//pins definition for RS485 serial comms
#define PIN_RS485_MODE		3


/////////////////////////////////////////////////////////////////////////////////////////////////
//retrieves water temperature measurement via 1-Wire protocol in deg C and returns it in deg C
float getWaterTemperature()
{
	OneWire OneWire(PIN_ONE_WIRE_BUS);						//setup 1-Wire bus
	DallasTemperature Sensors(&OneWire);					//setup temperature sensor with 1-Wire params
	Sensors.begin();										//locate devices on the bus
	Sensors.requestTemperaturesByIndex(0);					//start conversion in ds18b20 sensor (the only device on the bus)
	
	float water_temperature = Sensors.getTempCByIndex(0);	//assign temp (in deg C)
	
	return water_temperature;								//in degrees Celsius
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//retrieves water ingress level measurement as an analogue value and returns as mm value
byte getWaterIngress()
{
	unsigned int water_ingress_read = analogRead(PIN_WATER_INGRESS);
	byte water_ingress = 0;											//range 0 - 40mm

	//scaling of the sensor
	if (water_ingress_read <= 20)
		water_ingress = 0;											//values reported for 0mm
	else if (water_ingress_read > 20 && water_ingress_read < 200)
		water_ingress = map(water_ingress_read, 20, 200, 1, 5);		//in range of 1 to 5mm
	else
		water_ingress = map(water_ingress_read, 200, 500, 5, 40);	//in range of 5 to 40mm 

	return water_ingress;											//in mm
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//retrieves water pressure measurement as an analogue value and returns as hPa value
float getWaterPressure()
{
	static const byte samples = 5;
	unsigned int water_pressure_read[samples];
	float water_pressure = 0.0;

	//averaging over number of samples
	for (byte i = 1; i <= samples; i++)										
		{
			water_pressure_read[i] = analogRead(PIN_WATER_PRESSURE);
			
			if (water_pressure_read[i] <= 150)
				water_pressure_read[i] = 150;
			water_pressure += water_pressure_read[i];
		}
	
	water_pressure = 15.564 * (water_pressure / samples) - 2334.63;	//max measured water pressure is 12 000hPa or 1 200 000Pa according to sensor spec

	return water_pressure;													//in hPa
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//This section is connected to Serial comms. 

//list of (start) delimiters/identifiers for serial messages
enum ListOfMsgIDs
	{ 
		START_MSG_ID = 100, 
		WATER_TEMPERATURE_ID = 200,
		WATER_PRESSURE_ID = 201,
		WATER_INGRESS_ID = 202,
	};

//define packet for water temperature value storage and type substitution
union WaterTemperatureImplicitCast
{
	float As_Float;
	byte As_Bytes[4];
} WaterTemperaturePacket;

//define packet for water pressure value storage and type substitution
union WaterPressureImplicitCast
{
	float As_Float;
	byte As_Bytes[4];
} WaterPressurePacket;

//define variable for water ingress level value storage
byte Water_Ingress_Storage;

const byte Float_Size_In_Bytes = 4;						//size of byte array to store float, constant value

/////////////////////////////////////////////////////////////////////////////////////////////////
//send data to Topside
void sendPacket(byte waterTempArg[], byte waterPressArg[], byte& waterIngressArg)
{
	digitalWrite(PIN_RS485_MODE, HIGH);					//DE=RE=high transmit enabled

	//send water temperature value
	Serial1.write(START_MSG_ID);
	Serial1.write(WATER_TEMPERATURE_ID);
	for (byte i = 0; i < Float_Size_In_Bytes; i++)
		Serial1.write(waterTempArg[i]);

	//send water pressure value
	Serial1.write(START_MSG_ID);
	Serial1.write(WATER_PRESSURE_ID);
	for (byte i = 0; i < Float_Size_In_Bytes; i++)
		Serial1.write(waterPressArg[i]);
	
	//send water ingress level value
	Serial1.write(START_MSG_ID);
	Serial1.write(WATER_INGRESS_ID);
	Serial1.write(waterIngressArg);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//setup
void setup()
{	
	//set pins to input/output
	pinMode(PIN_WATER_INGRESS, INPUT);					//analog input
	pinMode(PIN_WATER_PRESSURE, INPUT);					//analog input
	pinMode(PIN_RS485_MODE, OUTPUT);					//DE/RE Data Enable/Receive Enable - transmit/receive pin set to output
	
	Serial1.begin(115200);								//open Serial1 hardware port for RS485 comms
} //end of setup


//global data defining how often to retrieve and send data
unsigned long Measurements_Timestamp = 0;
boolean Measurements_Flag = TRUE;

unsigned long Send_Timestamp = 0;
boolean Send_Flag = TRUE;


/////////////////////////////////////////////////////////////////////////////////////////////////
//main program
void loop()
{
	//set flag for measurements update (in ms)
	if (millis() - Measurements_Timestamp >= 250)
		Measurements_Flag = TRUE;

	//set flag for sending telemetry data to Topside (in ms)
	if (millis() - Send_Timestamp >= 1000)
		Send_Flag = TRUE;

	//retrieve the measurements
	if (Measurements_Flag)
		{
			//update water temperature, pressure and ingress level
			WaterTemperaturePacket.As_Float = getWaterTemperature();
			WaterPressurePacket.As_Float = getWaterPressure();
			Water_Ingress_Storage = getWaterIngress();

			Measurements_Timestamp = millis();
			Measurements_Flag = FALSE;
		}

	if (Send_Flag)
		{
			sendPacket(WaterTemperaturePacket.As_Bytes, WaterPressurePacket.As_Bytes, Water_Ingress_Storage);

			Send_Timestamp = millis();
			Send_Flag = FALSE;
		}
} //end of loop





