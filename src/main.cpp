

#include <Arduino.h>

#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
//#include <MQTT.h>
//#include <WiFi.h>
//#include <WiFiSSLClient.h>
#include <WiFiNINA.h>

#include "arduino_secrets.h"

//#include <ArduinoJson.h>

// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC "arduino/outgoing"
#define AWS_IOT_SUBSCRIBE_TOPIC "arduino/incoming"

WiFiClient    wifiClient;                   // Used for the TCP socket connection
BearSSLClient sslClient(wifiClient);        // Used for SSL/TLS connection, integrates with ECC508
MqttClient    client(sslClient);

bool connectWifi()
{
    int status = false; // the WiFi radio's connection status

    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE)
    {
        Serial.println("Communication with WiFi module failed!");
        // don't continue
        while (true)
            ;
    }

    String fv = WiFi.firmwareVersion();
    Serial.print("Wifi Firmware: ");
    Serial.println(fv);

    if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    {
        Serial.print("Expected Firnwmare version:");
        Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
        Serial.println("Please upgrade the firmware");
    }

    // attempt to connect to WiFi network:
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(WIFI_SSID);
        // Connect to WPA/WPA2 network:
        status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    return status;
}

void printMacAddress(byte mac[])
{
    for (int i = 5; i >= 0; i--)
    {
        if (mac[i] < 16)
        {
            Serial.print("0");
        }
        Serial.print(mac[i], HEX);
        if (i > 0)
        {
            Serial.print(":");
        }
    }
    Serial.println();
}

void printCurrentWifi()
{
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print the MAC address of the router you're attached to:
    byte bssid[6];
    WiFi.BSSID(bssid);
    Serial.print("BSSID: ");
    printMacAddress(bssid);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.println(rssi);

    // print the encryption type:
    byte encryption = WiFi.encryptionType();
    Serial.print("Encryption Type:");
    Serial.println(encryption, HEX);
    Serial.println();
}

unsigned long getTime() {
    // get the current time from the WiFi module  
    return WiFi.getTime();
}

void messageHandler(int messageSize)
{
      // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(client.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (client.available()) {
    Serial.print((char)client.read());
  }
  Serial.println();

  Serial.println();

}

void publishMessage()
{
    //StaticJsonDocument<200> doc;
    //doc["time"] = millis();
    //doc["sensor_a0"] = analogRead(0);
    //char jsonBuffer[512];
    //serializeJson(doc, jsonBuffer); // print to client

    //client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);

    // send message, the Print interface can be used to set the message contents
    client.beginMessage(AWS_IOT_PUBLISH_TOPIC);
    client.print("hello ");
    client.print(millis());
    client.endMessage();
}

bool connectAws()
{
    int status = false;
    Serial.println("Connecting to AWS");

        // checks to  see if the crypto chip is working
    if (!ECCX08.begin()) {
        Serial.println("No ECCX08 present!");
        while (1);
    }
    else
    {
        Serial.println("ECCX08 copnnected");
    }

    // Set a callback to get the current time
    // used to validate the servers certificate
    ArduinoBearSSL.onGetTime(getTime); //WiFi.getTime());

    // Set the ECCX08 slot to use for the private key
    // and the accompanying public certificate for it
    sslClient.setEccSlot(0, SECRET_CERTIFICATE); //certificate);

    return status;
}

bool connectThing() {
    int status = false;

    // Optional, set the client id used for MQTT,
    // each device that is connected to the broker
    // must have a unique client id. The MQTTClient will generate
    // a client id for you based on the millis() value if not set
    //
    // client.setId("clientId");

    Serial.println("Connecting thing to AWS");

    while (!client.connect(AWS_IOT_ENDPOINT, 8883)) //THINGNAME))
    {
        Serial.print(".");
        delay(100);
    }

    if (!client.connected())
    {
        Serial.println("AWS IoT Timeout!");
        status = false;
    }
    else {
        Serial.println("Connected to AWS");
        status = true;
    }

    return status;
}


void setup()
{

    Serial.begin(115200);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    connectWifi();

    connectAws();

    Serial.println("Setting up MQTT receiver");
    // Set the message callback, this function is
    // called when the MQTTClient receives a message
    client.onMessage(messageHandler);

    connectThing();

    Serial.println("Subscribing to the topic");
    // Subscribe to a topic
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);


}

void loop()
{
    Serial.println("Wifi status: ");
    printCurrentWifi();

    // poll for new MQTT messages and send keep alives
    client.poll();

    publishMessage();
    delay(5000);
}
