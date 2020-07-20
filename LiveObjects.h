/******************************************************************************
   DEFAULT VALUES
 ******************************************************************************/

#define PARAMETERS_NB_MAX 10
#define COMMANDS_NB_MAX 10
#define PAYLOAD_DEVMGT_SIZE 256
#define PAYLOAD_DATA_SIZE 512

#define SW_REVISION "1.9.0"
#define LIVE_OBJECTS_LOOP_LIMITER 1000

/******************************************************************************
   COMPATIBILITY
 ******************************************************************************/

#ifdef ARDUINO_SAMD_MKR1000
  #include <SPI.h>
  #include <WiFi101.h>
  #define BOARD_HAS_WIFI
#endif

#if    defined(ARDUINO_SAMD_MKRWIFI1010) \
    || defined(ARDUINO_SAMD_NANO_33_IOT) \
    || defined(ARDUINO_SAMD_MKRVIDOR4000)
  #include <SPI.h>
  #include <WiFiNINA.h>
  #define BOARD_HAS_WIFI
#endif

#ifdef BOARD_HAS_WIFI
  char wifi_ssid[] = SECRET_WIFI_SSID;
  char wifi_pass[] = SECRET_WIFI_PASS;
#endif

#ifdef ARDUINO_SAMD_MKRNB1500
  #include <MKRNB.h>
  #define BOARD_HAS_CELLULAR
  char cellular_pin[] = SECRET_CELLULAR_PIN;
  char cellular_apn[] = SECRET_CELLULAR_APN;
  char cellular_apn_user[] = SECRET_CELLULAR_APN_USER;
  char cellular_apn_pass[] = SECRET_CELLULAR_APN_PASS;
#endif

#ifdef ARDUINO_SAMD_MKRGSM1400
  #include <MKRGSM.h>
  #define BOARD_HAS_CELLULAR
  char cellular_pin[] = SECRET_CELLULAR_PIN;
  char cellular_apn[] = SECRET_CELLULAR_APN;
  char cellular_apn_user[] = SECRET_CELLULAR_APN_USER;
  char cellular_apn_pass[] = SECRET_CELLULAR_APN_PASS;
#endif

/******************************************************************************
   INCLUDES
 ******************************************************************************/

#include "LiveObjectsCert.h"

/******************************************************************************
   TYPEDEFS
 ******************************************************************************/

enum LiveObjects_networkStatus {
  CONNECTED,
  DISCONNECTED
};

enum LiveObjects_parameterType {
  INTEGER,
  UNSIGNED_INTEGER,
  BINARY,
  STRING,
  DECIMAL,
  IMPLICIT
};

enum LiveObjects_variableType {
  T_BOOL,
  T_CHAR,
  T_INT,
  T_INT8,
  T_INT16,
  T_INT32,
  T_UINT,
  T_UINT8,
  T_UINT16,
  T_UINT32,
  T_DOUBLE,
  T_FLOAT,
  T_STRING,
};

typedef void (*onParameterUpdateCallback)();
typedef void (*onCommandCallback)(const String, String&);

struct LiveObjects_parameter {
  const char* label;
  void *value;
  LiveObjects_parameterType type;
  LiveObjects_variableType variableType;
  onParameterUpdateCallback callback;
} parameters[PARAMETERS_NB_MAX];
uint8_t paramNb = 0;

struct LiveObjects_command {
  const char* label;
  onCommandCallback callback;
} commands[COMMANDS_NB_MAX];
uint8_t cmdNb = 0;

/******************************************************************************
   CLIENTS
 ******************************************************************************/

#if defined(ARDUINO_SAMD_MKRNB1500)
  NB cellularAccess;
  #if defined(MQTT_TLS)   // for TLS MQTT connection to Live Objects
    NBSSLClient networkClient;   
  #else   // for plain MQTT connection to Live Objects
    NBClient networkClient;
  #endif
#endif

#if defined(ARDUINO_SAMD_MKRGSM1400)
  GSM cellularAccess;
  GPRS gprs;
  #if defined(MQTT_TLS)   // for TLS MQTT connection to Live Objects
    GSMSSLClient networkClient;   
  #else   // for plain MQTT connection to Live Objects
    GSMClient networkClient;
  #endif
#endif

#if defined(BOARD_HAS_WIFI)
  #if defined(MQTT_TLS)   // for TLS MQTT connection to Live Objects
    WiFiSSLClient networkClient;
  #else   // for plain MQTT connection to Live Objects
    WiFiClient networkClient;
  #endif
#endif

MqttClient mqttClient(networkClient);

/******************************************************************************
   CONSTANTS
 ******************************************************************************/

// Live Objects constants
char mqtt_id[16] = "MKR_board";
const char mqtt_broker[] = "liveobjects.orange-business.com";
const char mqtt_user[] = "json+device";    // MQTT username for 'device' role
const char mqtt_pass[] = SECRET_LIVEOBJECTS_API_KEY;
#if defined(MQTT_TLS)
  #if defined(ARDUINO_SAMD_MKR1000)
    #error TLS not supported on Arduino MKR1000 WIFI, please disable TLS when using this board!
  #endif
  const uint16_t mqtt_port = 8883;         // TLS MQTT port
#else
  const uint16_t mqtt_port = 1883;         // default MQTT port
#endif
const char* mqtt_pubdata = "dev/data";     // topic for publishing data
const char* mqtt_subcfg = "dev/cfg/upd";   // subscribed topic for configuration updates
const char* mqtt_pubcfg = "dev/cfg";       // topic used to publish confuguration
const char* mqtt_subcmd = "dev/cmd";       // subscribed topic for commands
const char* mqtt_pubcmd = "dev/cmd/res";   // topic used to confirm commands

// Live Objects' JSON constants
const char* JSONcid = "cid";
const char* JSONcfg = "cfg";
const char* JSONcfgValue = "v";
const char* JSONcfgType = "t";
const char* JSONmodel = "model";
const char* JSONvalue = "value";
const char* JSONmodelName = "github_sample_MKR_family";

/******************************************************************************
   VARIABLES
 ******************************************************************************/

bool DigiCert_rootCA_loaded = false;  // controls if the root CA certificate has already been stored in the SARA module
bool initialConfigDone = false;       // controls if the inital config has been sent to Live Objects
unsigned long lastLiveObjectsLoop = 0;
LiveObjects_networkStatus networkStatus = DISCONNECTED;
StaticJsonDocument<PAYLOAD_DATA_SIZE> easyDataPayload;

void publishMessage(const char* topic, JsonDocument& payload);

/******************************************************************************
   FUNCTION TEMPLATES
 ******************************************************************************/

template<typename LOtA>
void addParameter(const char* name, LOtA &variable) {
  onParameterUpdateCallback ptr = NULL;
  paramTyper(name, &variable, IMPLICIT, ptr);
}
template<typename LOtB>
void addParameter(const char* name, LOtB &variable, LiveObjects_parameterType type) {
  onParameterUpdateCallback ptr = NULL;
  paramTyper(name, &variable, type, ptr);
}
template<typename LOtC>
void addParameter(const char* name, LOtC &variable, onParameterUpdateCallback callback) {
  paramTyper(name, &variable, IMPLICIT, callback);
}
template<typename LOtD>
void addParameter(const char* name, LOtD &variable, onParameterUpdateCallback callback, LiveObjects_parameterType type) {
  paramTyper(name, &variable, type, callback);
}

template<typename LOtE>
void addTypedParam(const char* name, LOtE *variable, LiveObjects_parameterType type, LiveObjects_variableType variableType, onParameterUpdateCallback callback) {
  parameters[paramNb++] = (LiveObjects_parameter) { name, variable, type, variableType, callback };
}

template<typename LOtF>
void updateParameter(const LiveObjects_parameter param, LOtF* ptr, const JsonDocument& configIn, JsonDocument& configOut) {
  if (!configIn.isNull())
    *ptr = configIn[JSONcfg][param.label][JSONcfgValue].as<LOtF>();
  configOut[JSONcfg][param.label][JSONcfgValue] = *ptr;
}

template<typename LOtH>
void addToPayload(const String label, LOtH value) {
  easyDataPayload[JSONvalue][label] = value;
}

/******************************************************************************
   FUNCTIONS DECLARATION
 ******************************************************************************/

void networkConnect();
void networkDisconnect();
void networkCheck();
void mqttConnect();
void onMQTTmessage(int messageSize);
void sendData();
void sendData(const String customPayload);
void publishMessage(const char* topic, JsonDocument& payload);
void LiveObjects_connect();
void LiveObjects_disconnect();
void LiveObjects_loop();

/******************************************************************************
   PARAM TYPERS
 ******************************************************************************/

void paramTyper(const char* name, bool* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, UNSIGNED_INTEGER, T_BOOL, callback);
  else
    addTypedParam(name, variable, type, T_BOOL, callback);
}
void paramTyper(const char* name, char* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, INTEGER, T_CHAR, callback);
  else
    addTypedParam(name, variable, type, T_CHAR, callback);
}
void paramTyper(const char* name, int* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, INTEGER, T_INT, callback);
  else
    addTypedParam(name, variable, type, T_INT, callback);
}
void paramTyper(const char* name, int8_t*variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, INTEGER, T_INT8, callback);
  else
    addTypedParam(name, variable, type, T_INT8, callback);
}
void paramTyper(const char* name, int16_t* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, INTEGER, T_INT16, callback);
  else
    addTypedParam(name, variable, type, T_INT16, callback);
}
void paramTyper(const char* name, int32_t* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, INTEGER, T_INT32, callback);
  else
    addTypedParam(name, variable, type, T_INT32, callback);
}
void paramTyper(const char* name, unsigned int* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, UNSIGNED_INTEGER, T_UINT, callback);
  else
    addTypedParam(name, variable, type, T_UINT, callback);
}
void paramTyper(const char* name, uint8_t* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, UNSIGNED_INTEGER, T_UINT8, callback);
  else
    addTypedParam(name, variable, type, T_UINT8, callback);
}
void paramTyper(const char* name, uint16_t* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, UNSIGNED_INTEGER, T_UINT16, callback);
  else
    addTypedParam(name, variable, type, T_UINT16, callback);
}
void paramTyper(const char* name, uint32_t* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, UNSIGNED_INTEGER, T_UINT32, callback);
  else
    addTypedParam(name, variable, type, T_UINT32, callback);
}
void paramTyper(const char* name, double* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, DECIMAL, T_DOUBLE, callback);
  else
    addTypedParam(name, variable, type, T_DOUBLE, callback);
}
void paramTyper(const char* name, float* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, DECIMAL, T_FLOAT, callback);
  else
    addTypedParam(name, variable, type, T_FLOAT, callback);
}
void paramTyper(const char* name, String* variable, LiveObjects_parameterType type, onParameterUpdateCallback callback) {
  if (type == IMPLICIT)
    addTypedParam(name, variable, STRING, T_STRING, callback);
  else
    addTypedParam(name, variable, type, T_STRING, callback);
}

/******************************************************************************
   POINTER TYPERS
 ******************************************************************************/

void ptrTyperBool(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  bool* ptr = (bool*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperChar(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  char* ptr = (char*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperInt(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  int* ptr = (int*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperInt8(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  int8_t* ptr = (int8_t*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperInt16(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  int16_t* ptr = (int16_t*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperInt32(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  int32_t* ptr = (int32_t*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperUint(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  unsigned int* ptr = (unsigned int*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperUint8(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  uint8_t* ptr = (uint8_t*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperUint16(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  uint16_t* ptr = (uint16_t*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperUint32(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  uint32_t* ptr = (uint32_t*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperDouble(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  double* ptr = (double*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperFloat(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  float* ptr = (float*)param.value;
  updateParameter(param, ptr, configIn, configOut);
}
void ptrTyperString(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  String* ptr = (String*)param.value;
  if (!configIn.isNull())
    *ptr = configIn[JSONcfg][param.label][JSONcfgValue].as<String>();
  configOut[JSONcfg][param.label][JSONcfgValue] = (*ptr).c_str();
}

void ptrTyperInit(const LiveObjects_parameter param, const JsonDocument& configIn, JsonDocument& configOut) {
  switch (param.variableType) {
    case T_BOOL:
      ptrTyperBool(param, configIn, configOut);
      break;
    case T_CHAR:
      ptrTyperChar(param, configIn, configOut);
      break;
    case T_INT:
      ptrTyperInt(param, configIn, configOut);
      break;
    case T_INT8:
      ptrTyperInt8(param, configIn, configOut);
      break;
    case T_INT16:
      ptrTyperInt16(param, configIn, configOut);
      break;
    case T_INT32:
      ptrTyperInt32(param, configIn, configOut);
      break;
    case T_UINT:
      ptrTyperUint(param, configIn, configOut);
      break;
    case T_UINT8:
      ptrTyperUint8(param, configIn, configOut);
      break;
    case T_UINT16:
      ptrTyperUint16(param, configIn, configOut);
      break;
    case T_UINT32:
      ptrTyperUint32(param, configIn, configOut);
      break;
    case T_DOUBLE:
      ptrTyperDouble(param, configIn, configOut);
      break;
    case T_FLOAT:
      ptrTyperFloat(param, configIn, configOut);
      break;
    case T_STRING:
      ptrTyperString(param, configIn, configOut);
      break;
  }
}

/******************************************************************************
   CONFIGURATION MANAGER
 ******************************************************************************/

void configurationManager(int messageSize = -1) {
  StaticJsonDocument<PAYLOAD_DEVMGT_SIZE> configOut;
  if (messageSize >= 0) { // config update received
    StaticJsonDocument<PAYLOAD_DEVMGT_SIZE> configIn;
    deserializeJson(configIn, mqttClient);

    serializeJsonPretty(configIn, Serial);
    Serial.print('\n');

    configOut = configIn;
    for (uint8_t i = 0; i < paramNb; i++)
      if (configIn[JSONcfg].containsKey(parameters[i].label)) {
        ptrTyperInit(parameters[i], configIn, configOut);
        if (parameters[i].callback != NULL)
          parameters[i].callback();
      }
    publishMessage(mqtt_pubcfg, configOut);
  }
  else { // messageSize==-1, compose & send initial config
    StaticJsonDocument<0> configIn;
    for (uint8_t i = 0; i < paramNb; i++) {
      switch (parameters[i].type) {
        case INTEGER:
          configOut[JSONcfg][parameters[i].label][JSONcfgType] = F("i32");
          break;
        case UNSIGNED_INTEGER:
          configOut[JSONcfg][parameters[i].label][JSONcfgType] = F("u32");
          break;
        case STRING:
          configOut[JSONcfg][parameters[i].label][JSONcfgType] = F("str");
          break;
        case BINARY:
          configOut[JSONcfg][parameters[i].label][JSONcfgType] = F("bin");
          break;
        case DECIMAL:
          configOut[JSONcfg][parameters[i].label][JSONcfgType] = F("f64");
          break;
      }
      ptrTyperInit(parameters[i], configIn, configOut);
    }
    publishMessage(mqtt_pubcfg, configOut);
  }
}

/******************************************************************************
   COMMAND MANAGEMENT
 ******************************************************************************/

void addCommand(const char* name, onCommandCallback callback) {
  commands[cmdNb++] = (LiveObjects_command) {
    name, callback
  };
}

void commandManager() {
  StaticJsonDocument<PAYLOAD_DEVMGT_SIZE> cmdIn;
  StaticJsonDocument<PAYLOAD_DEVMGT_SIZE> cmdOut;
  deserializeJson(cmdIn, mqttClient);

  serializeJsonPretty(cmdIn, Serial);
  Serial.print('\n');

  for (uint8_t i = 0; i < cmdNb; i++) // only with MQTT or SMS !!
    if (cmdIn[F("req")] == commands[i].label) {
      cmdOut[JSONcid] = cmdIn[JSONcid];
      String response;
      commands[i].callback(cmdIn[F("arg")].as<String>(), response);
      if (response.length() != 0)
        cmdOut[F("res")] = serialized(response);
      break;
    }
  publishMessage(mqtt_pubcmd, cmdOut);
}

/******************************************************************************
   CONNECTION MANAGER
 ******************************************************************************/

byte hexToASCII(byte in) {return (in >= 10 ? in - 10 + 'A' : in + '0');}

void getClientId() {
  #if defined(BOARD_HAS_CELLULAR)
    #if defined(ARDUINO_SAMD_MKRNB1500)
      NBModem modem;
    #elif defined(ARDUINO_SAMD_MKRGSM1400)
      GSMModem modem;
    #endif
    delay(200);
    if(modem.begin()) {
      String imei = modem.getIMEI();
      strncpy(mqtt_id, imei.c_str(), 16);
    }
    else
      strncpy(mqtt_id, "defaultMKRboard", 16);
  #endif

  #if defined(BOARD_HAS_WIFI)
    byte mac[6];
    WiFi.macAddress(mac);
    byte j = 0;
    for (int i = 5; i >= 0; i--) {
      mqtt_id[j++] = hexToASCII(mac[i] >> 4);
      mqtt_id[j++] = hexToASCII(mac[i] & 0xF);
    }
    mqtt_id[j] = '\0';
  #endif
}

void networkConnect() {
  Serial.print(F("\nConnecting to "));
  #if defined(BOARD_HAS_CELLULAR)
    Serial.print(F("cellular network"));
    #if defined(ARDUINO_SAMD_MKRNB1500)
      while (cellularAccess.begin(cellular_pin, cellular_apn, cellular_apn_user, cellular_apn_pass) != NB_READY) {
    #endif
    #if defined(ARDUINO_SAMD_MKRGSM1400)
      while ((cellularAccess.begin(cellular_pin) != GSM_READY) ||
          (gprs.attachGPRS(cellular_apn, cellular_apn_user, cellular_apn_pass) != GPRS_READY)) {
    #endif
  #endif
  #if defined(BOARD_HAS_WIFI)
    Serial.print(F("Wi-Fi network '"));
    Serial.print(wifi_ssid);
    Serial.print(F("'"));
    int status = WL_IDLE_STATUS;
    while (status != WL_CONNECTED) {
      status = WiFi.begin(wifi_ssid, wifi_pass);
  #endif
      delay(1000);
      Serial.print(F(".")); }
  Serial.println(F("\nYou're connected to the network"));

  //#if defined(ARDUINO_SAMD_MKRNB1500)
  #if defined(BOARD_HAS_CELLULAR)
    #if defined(MQTT_TLS)    // only for TLS ; loads DigiCert CA into SARA module
      if (!DigiCert_rootCA_loaded) {
        Serial.println(F("Loading DigiCert Root CA certificate"));
        MODEM.sendf("AT+USECMNG=0,0,\"%s\",%d", LO_ROOT_CERT.name, LO_ROOT_CERT.size);
        if (MODEM.waitForPrompt() != 1) {
          Serial.print(F("Problem loading certificate!\nStopping here"));
          while (1) ;
        }
        else {
          MODEM.write(LO_ROOT_CERT.data, LO_ROOT_CERT.size);
          int ready;
          while (!MODEM.ready()) ;
          DigiCert_rootCA_loaded = true;
          Serial.println(F("Certificate loaded"));
        }
      }
    #endif
  #endif
  networkStatus = CONNECTED;
}

void networkDisconnect() {
  Serial.println(F("Disconnecting from network..."));
  #if defined(BOARD_HAS_CELLULAR)
    cellularAccess.shutdown();
  #endif
  #if defined(BOARD_HAS_WIFI)
    WiFi.end();
  #endif
  Serial.println(F("Offline"));
  
  networkStatus = DISCONNECTED;
}

void networkCheck() {
  #if defined(BOARD_HAS_CELLULAR)
    if (cellularAccess.isAccessAlive() != 1)
  #endif
  #if defined(BOARD_HAS_WIFI)
    if (WiFi.status() != WL_CONNECTED)
  #endif
  	networkConnect();

  if (!mqttClient.connected())
  	mqttConnect();
}

/******************************************************************************
   MQTT FUNCTIONS
 ******************************************************************************/

void mqttConnect() {
  if (!initialConfigDone) {
    mqttClient.setId(mqtt_id);
    mqttClient.setUsernamePassword(mqtt_user, mqtt_pass);
    mqttClient.onMessage(onMQTTmessage);
  }

  Serial.print(F("Connecting to MQTT broker '"));
  Serial.print(mqtt_broker);
  Serial.print(F("' with port "));
  Serial.print(mqtt_port);

  while (!mqttClient.connect(mqtt_broker, mqtt_port)) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println(F("\nYou're connected to Live Objects"));

  networkStatus = CONNECTED;

  mqttClient.subscribe(mqtt_subcfg);
  mqttClient.subscribe(mqtt_subcmd);
  
  if (!initialConfigDone) {
    configurationManager();
    initialConfigDone = true;
  }

  mqttClient.poll();
}

void onMQTTmessage(int messageSize) {
  String topic = mqttClient.messageTopic();

  Serial.print(F("Received a message on topic '"));
  Serial.print(topic);
  Serial.println(F("':"));

  if (topic == mqtt_subcfg)
    configurationManager(messageSize);
  else if (topic == mqtt_subcmd)
    commandManager();
}

void sendData() {
  easyDataPayload[JSONmodel] = JSONmodelName;
  publishMessage(mqtt_pubdata, easyDataPayload);
  easyDataPayload.clear();
}

void sendData(const String customPayload) {
  StaticJsonDocument<PAYLOAD_DATA_SIZE> payload;
  deserializeJson(payload, customPayload);
  if (!payload.containsKey(JSONmodel))
    payload[JSONmodel] = JSONmodelName;
  publishMessage(mqtt_pubdata, payload);
}

void publishMessage(const char* topic, JsonDocument& payload) {
  LiveObjects_networkStatus lastStatus = networkStatus;
  if (networkStatus != CONNECTED)
    LiveObjects_connect();

  Serial.print(F("Publishing message on topic '"));
  Serial.print(topic);
  Serial.println(F("':"));
  serializeJsonPretty(payload, Serial);
  Serial.print('\n');
  mqttClient.beginMessage(topic);
  serializeJson(payload, mqttClient);
  mqttClient.endMessage();

  if (lastStatus == DISCONNECTED)
    LiveObjects_disconnect();
}

/******************************************************************************
   LIVE OBJECTS
 ******************************************************************************/

void LiveObjects_connect() {  
  if (!initialConfigDone) {
    if (sizeof(mqtt_pass) != 33) {   // check if API key seems correct
      Serial.print(F("Please check your Live Objects API key in 'arduino_secrets.h' file).\nStopping here."));
      while (1);
    }
  	getClientId();
  }

  networkConnect();
  mqttConnect();
}

void LiveObjects_disconnect() {
  Serial.println(F("\nClosing MQTT connection..."));
  mqttClient.stop();
  networkDisconnect();
}

void LiveObjects_loop() {
  if (networkStatus == CONNECTED)
    if (millis() - lastLiveObjectsLoop > LIVE_OBJECTS_LOOP_LIMITER) {
      networkCheck();
      mqttClient.poll();
      lastLiveObjectsLoop = millis();
    }
}
