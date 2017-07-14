/*
  UROV v1

  Subsea controller

  Created in 2017 by JK
  */

#include <DallasTemperature.h>								//ds18b20 temp sensor library

//pins definition for 1-Wire bus
#define ONE_WIRE_BUS	2

//pins definition for water detection sensor
#define pin_WI			A7

//pins definition for pressure transducer
#define pin_PR			A6

//pins definition for RS485 serial comms
#define pin_RS485_mode	3

//global data for 1-Wire
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//global data for RS485 telemetry data transmission
struct telemetryData 
{
	byte ID =					0x02;						//identity of the packet
	byte temperature;
	byte pressure;
	byte waterIngressAlarm;									//0 - ok, 1 - alarm (if over ~5mm or AI = 40)
} sendTopside;

const int STRUCT_SIZE =			4;							//the size of struct/packet in bytes
float water_temperature =		0.0;
unsigned int water_pressure =	0;
byte water_ingress =			0;


////////////////////////////////////////////////////
//get temperature measurement
float temperature()
{
	sensors.requestTemperatures();
	water_temperature = sensors.getTempCByIndex(0);
	
	return water_temperature;								//in degrees Celsius
}


////////////////////////////////////////////////////
//get water ingress level measurement
byte waterIngress()
{
	unsigned int water_ingress_read = analogRead(pin_WI);
	water_ingress = map(water_ingress_read, 0, 435, 0, 40);	//max measured vertical ingress level is 40mm 

	return water_ingress;									//in mm
}


////////////////////////////////////////////////////
//get pressure measurement
unsigned int pressure()
{
	const byte samples = 5;
	int water_pressure_read[samples];

	for (byte i = 1; i <= samples; i++)						//average over number of samples
		{
			water_pressure_read[i] = analogRead(pin_PR);
			if (water_pressure_read[i] < 0)
				water_pressure_read[i] = 0;
			water_pressure += water_pressure_read[i];
		}
		
	water_pressure = map((water_pressure / samples), 102, 922, 0, 12000);	//max measured water pressure is 12 000hPa 

	return water_pressure;													//in hPa
}


////////////////////////////////////////////////////
//send telemtry data to Topside
void sendPacket()
{
	digitalWrite(pin_RS485_mode, HIGH);												//DE=RE=high transmit enabled

	//convert water temperature to byte
	water_temperature = water_temperature * 100;									//to make it look like int
	sendTopside.temperature = (byte)map(water_temperature, 0, 40 * 100, 0, 255);	//map it as if it was int to byte

	//convert water pressure to byte
	sendTopside.pressure = (byte)map(water_pressure, 0, 12000, 0, 255);				//map int to byte

	//convert water ingress to water ingress alarm if water ingress level >= 5mm
	if (water_ingress >= 5)
		sendTopside.waterIngressAlarm = 1;
	else
		sendTopside.waterIngressAlarm = 0;

	//fill the buffer
	byte buffer[STRUCT_SIZE] = { sendTopside.ID, sendTopside.temperature, sendTopside.pressure, sendTopside.waterIngressAlarm };

	//send the buffer
	Serial1.write(buffer, STRUCT_SIZE);
}


////////////////////////////////////////////////////
//setup
void setup()
{	
	//1-Wire sensors
	sensors.begin();										//locate devices on the bus

	//water ingress
	pinMode(pin_WI, INPUT);

	//RS485
	pinMode(pin_RS485_mode, OUTPUT);						//DE/RE Data Enable/Receive Enable transmit/receive pin of RS-485
	Serial1.begin(9600);									//open Serial1 Port for RS485 comms

	Serial.begin(9600); //del later
}


//global data to define how often to retrieve and send data
unsigned long measurements_timestamp = 0;
boolean measurements_flag = true;

unsigned long send_timestamp = 0;
boolean send_flag = true;


////////////////////////////////////////////////////
//main program
void loop()
{
	//set flag for update of temp value retrieval (in ms)
	if (millis() - measurements_timestamp >= 100)
		measurements_flag = true;

	if (measurements_flag)
		{
			//Serial.print("Temperature: ");
			//Serial.print(temperature(), 2);
			//Serial.println(" (degC)");

			water_temperature = temperature();				//in deg C

			//Serial.print("Water pressure: ");
			//Serial.print(pressure());
			//Serial.println(" (hPa)");

			water_pressure = pressure();					//in hPa

			//Serial.print("Water ingress: ");
			//Serial.print(waterIngress());
			//Serial.println(" (mm)");

			water_ingress = waterIngress();					//in mm

			measurements_timestamp = millis();
			measurements_flag = false;
		}

	//set flag for sending data to Topside
	if (millis() - send_timestamp >= 100)
		send_flag = true;

	if (send_flag)
		{
			sendPacket();

			send_timestamp = millis();
			send_flag = false;
		}
}





