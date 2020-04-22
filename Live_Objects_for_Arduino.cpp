

/******************************************************************************
   INCLUDE
 ******************************************************************************/

#include "LiveObjects_for_Arduino.h"

/******************************************************************************
   CTOR/DTOR
 ******************************************************************************/

LiveObjects::LiveObjects(LiveObjects_security security = LiveObjects_security::NONE, const bool debug)
{
	_apiKey = SECRET_LIVEOBJECTS_API_KEY;
	MQTTsecurityLevel = security;
	_debug = debug;
}

/******************************************************************************
   PUBLIC MEMBER FUNCTIONS
 ******************************************************************************/

void connect()
{
	networkConnect(LiveObjects_security MQTTsecurityLevel, const bool debug);

	// if network OK
	mqttConnect(LiveObjects_security MQTTsecurityLevel, const bool debug);

	// if mqtt OK
	// send config
}

void update()
{
	conMan.check();
	if (conMan.getStatus() >= 2)
	{
		// check MQTT state
		// check incoming msg
	}
	// else
		// do nothing
}



void disconnect() {}
void update() {}

/******************************************************************************
   PRIVATE MEMBER FUNCTIONS
 ******************************************************************************/


void sendData(const String& payload)
	{
		sendData(payload.c_str());
	}