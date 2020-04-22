

#ifndef _LIVEOBJECTS_FOR_ARDUINO_H_
#define _LIVEOBJECTS_FOR_ARDUINO_H_

#ifdef ARDUINO_SAMD_MKR1000
  #include <WiFi101.h>
  #include <WiFiUdp.h>

  #define BOARD_HAS_WIFI
  #define NETWORK_HARDWARE_ERROR WL_NO_SHIELD
  #define NETWORK_IDLE_STATUS WL_IDLE_STATUS
  #define NETWORK_CONNECTED WL_CONNECTED
  #define WIFI_FIRMWARE_VERSION_REQUIRED WIFI_FIRMWARE_REQUIRED
#endif

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT)
  #include <WiFiNINA.h>
  #include <WiFiUdp.h>

  #define BOARD_HAS_WIFI
  #define NETWORK_HARDWARE_ERROR WL_NO_MODULE
  #define NETWORK_IDLE_STATUS WL_IDLE_STATUS
  #define NETWORK_CONNECTED WL_CONNECTED
  #define WIFI_FIRMWARE_VERSION_REQUIRED WIFI_FIRMWARE_LATEST_VERSION
#endif

#ifdef ARDUINO_SAMD_MKRGSM1400
  #include <MKRGSM.h>
  #define BOARD_HAS_GSM
  #define NETWORK_HARDWARE_ERROR GPRS_PING_ERROR
  #define NETWORK_IDLE_STATUS GSM3_NetworkStatus_t::IDLE
  #define NETWORK_CONNECTED GSM3_NetworkStatus_t::GPRS_READY
#endif

#ifdef ARDUINO_SAMD_MKRNB1500
  #include <MKRNB.h>
  #define BOARD_HAS_NB
  #define NETWORK_HARDWARE_ERROR
  #define NETWORK_IDLE_STATUS NB_NetworkStatus_t::IDLE
  #define NETWORK_CONNECTED NB_NetworkStatus_t::GPRS_READY
#endif

#if defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310)
  #include <MKRWAN.h>
  #define BOARD_HAS_LORA
#endif

#if    defined(ARDUINO_ESP8266_ESP12)    \
    || defined(ESP8266)                  \
    || defined(ESP8266_ESP01)            \
    || defined(ESP8266_ESP13)            \
    || defined(ESP8266_GENERIC)          \
    || defined(ESP8266_ESPRESSO_LITE_V1) \
    || defined(ESP8266_ESPRESSO_LITE_V2) \
    || defined(ESP8266_PHOENIX_V1)       \
    || defined(ESP8266_PHOENIX_V2)       \
    || defined(ESP8266_NODEMCU)          \
    || defined(MOD_WIFI_ESP8266)         \
    || defined(ESP8266_THING)            \
    || defined(ESP8266_THING_DEV)        \
    || defined(ESP8266_ESP210)           \
    || defined(ESP8266_WEMOS_D1MINI)     \
    || defined(ESP8266_WEMOS_D1MINIPRO)  \
    || defined(ESP8266_WEMOS_D1MINILITE) \
    || defined(ESP8266_WEMOS_D1R1)       \
    || defined(ESP8266_ESP12)            \
    || defined(WIFINFO)                  \
    || defined(ESP8266_ARDUINO)          \
    || defined(GEN4_IOD)                 \
    || defined(ESP8266_OAK)              \
    || defined(WIFIDUINO_ESP8266)        \
    || defined(AMPERKA_WIFI_SLOT)        \
    || defined(ESP8266_WIO_LINK)         \
    || defined(ESP8266_ESPECTRO_CORE)
  
  #define BOARD_ESP8266
#endif

#if defined(BOARD_ESP8266)
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>

  #define BOARD_HAS_WIFI
  #define NETWORK_HARDWARE_ERROR WL_NO_SHIELD
  #define NETWORK_IDLE_STATUS WL_IDLE_STATUS
  #define NETWORK_CONNECTED WL_CONNECTED
  #define WIFI_FIRMWARE_VERSION_REQUIRED WIFI_FIRMWARE_REQUIRED
#endif

/*
#if defined(BOARD_HAS_WIFI)
WiFiConnectionHandler conMan(SECRET_SSID, SECRET_PASS);
#elif defined(BOARD_HAS_GSM)
GSMConnectionHandler conMan(SECRET_APN, SECRET_PIN, SECRET_GSM_USER, SECRET_GSM_PASS);
#elif defined(BOARD_HAS_NB)
NBConnectionHandler conMan(SECRET_APN, SECRET_PIN, SECRET_GSM_USER, SECRET_GSM_PASS);
#elif defined(BOARD_HAS_LORA)
LoRaConnectionHandler conMan(SECRET_APP_EUI, SECRET_APP_KEY);
#endif
*/

/******************************************************************************
   INCLUDES
 ******************************************************************************/

#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <ArduinoSTL.h>
#include <vector>
#include "LiveObjectsCert.h"

#if defined(BOARD_HAS_WIFI)
  #include "Arduino_WiFiConnectionHandler.h"
#elif defined(BOARD_HAS_GSM)
  #include "Arduino_GSMConnectionHandler.h"
#elif defined(BOARD_HAS_NB)
  #include "Arduino_NBConnectionHandler.h"
#elif defined(BOARD_HAS_LORA)
  #include "Arduino_LoRaConnectionHandler.h"
#endif

/******************************************************************************
   TYPEDEFS
 ******************************************************************************/

enum LiveObjects_parameterType {
  INTEGER,
  UNSIGNED_INTEGER,
  BINARY,
  STRING,
  DECIMAL
};

enum LiveObjects_variableType {
  t_bool,
  t_char,
  t_short,
  t_int8,
  t_uint8,
  t_int16,
  t_uint16,
  t_int32,
  t_uint32,
  t_double,
  t_float,
  t_charArray,
  t_String
};

enum LiveObjects_security {
  NONE,
  TLS,
  DTLS
};

struct LiveObjects_parameter {
  const char* label;
  LiveObjects_parameterType type;
  void value; // variable address
  LiveObjects_variableType variableType;
};

struct LiveObjects_command {
  const char* label;
  void callback; // callback address
};

/******************************************************************************
   CONSTANTS
 ******************************************************************************/

LiveObjects_security MQTTsecurityLevel;

/******************************************************************************
   CLASS DECLARATION
 ******************************************************************************/

class LiveObjects
{
  public:

    LiveObjects(LiveObjects_security security = LiveObjects_security::NONE, bool const debug = false);
  
    void connect();
    void disconnect();
    void update();

//    void addParameter(String& label, LiveObjects_parameterType type, &user_variable);
//    void addParameter(char* label, LiveObjects_parameterType type, &user_variable);
//    void addCommand(String& label, user_callback);
//    void addCommand(char* label, user_callback);

//    void writePayload(char* name, int8_t value);
/*
    namespace
    {
*/
      template<typename LO_payloadItem>

      void writePayload(const String& itemName, LO_payloadItem itemValue)   // ideally, liveobjects.print()
      {
        writePayload(itemName.c_str(), itemValue);
      }
      void writePayload(const char* itemName, LO_payloadItem itemValue)
      {
        // code...
        // storage of serialized JSON in vectors?
        // or always reuse the same JSONdoc?
      }
      
/*
    }
*/

    void addLocation(float latitude, float longitude, float altitude);
    void addTimestamp(uint32_t timestamp);

    void sendData(const String& payload);
    void sendData(const char* payload);
    void sendData();
    
    void clearPayload();

  private:
    std::vector<LiveObjects_parameter> _parameters;
    std::vector<LiveObjects_command> _commands;

    const char* _apiKey;
    const bool _debug;

    MqttClient _mqttClient(network.getclient());
}


/*

void addCallback(NetworkConnectionEvent const event, OnNetworkEventCallback callback);
// onConfigUpdate
// onCommand

*/

#endif