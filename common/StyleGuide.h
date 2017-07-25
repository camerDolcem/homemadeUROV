/*
**********************************************************************
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
**********************************************************************
*/