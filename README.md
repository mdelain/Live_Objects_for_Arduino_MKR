# Live Objects code sample for Arduino MKR family
**[Beta]** Library and code samples for Live Objects on Arduino &amp; compatible boards based on SAMD21

## Compatibility ##
| Board | MQTT | MQTTS |
| :--- | :---: | :---: |
| Arduino MKR1000 WIFI | OK | - |
| Arduino MKR 1010 WiFi | not tested | not tested* |
| Arduino MKR 1400 GSM | OK | OK |
| Arduino MKR 1500 NB | OK | OK |
| Arduino MKR VIDOR 4000 | OK | OK* |
| Arduino Nano 33 IoT | not tested | not tested* |

*You need to import the Root CA certificate using the Firmware updater from the example of the WiFiNINA library, and follow [this tutorial](https://www.arduino.cc/en/Tutorial/WiFiNINAFirmwareUpdater) to add the domain ``liveobjects.orange-business.com:443``.
