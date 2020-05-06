/****************************************
   Include Libraries
 ****************************************/
#include <WiFi.h>
#include <PubSubClient.h>
#include "HardwareSerial.h"

HardwareSerial myserial(2);

/****************************************
   Define Constants
 ****************************************/
#define WIFISSID "Androidpablo1" // Put your WifiSSID here
#define PASSWORD "gaviria7" // Put your wifi password here
#define TOKEN "BBFF-o2Hl9pzXqqc8vfC9smjimW9IUZkEVJ" // Put your Ubidots' TOKEN
#define MQTT_CLIENT_NAME "INNSMED_P06" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
#define DEVICE_LABEL "IMS_P06" // Assig the device label

char mqttBroker[]  = "industrial.api.ubidots.com";
char payload[100];
char topic[150];

bool sendFlag;
bool readP;
bool readF;
bool readI;

bool readPip;
bool readPeep;
bool readN;

char inChar;

String inputString;

float pressure;
float pip;
float peep;
long numCiclos;

int frec;
float ieRatio;

#define RXD2 4
#define TXD2 17

/****************************************
   Auxiliar Functions
 ****************************************/

WiFiClient ubidots;
PubSubClient client(ubidots);

void callback(char* topic, byte* payload, unsigned int length) {
  //  char p[length + 1];
  //  memcpy(p, payload, length);
  //  p[length] = NULL;
  //  String message(p);
  //  Serial.write(payload, length);
  //  Serial.println(topic);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");

    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

/****************************************
   Main Functions
 ****************************************/
void setup() {
  Serial.begin(115200);

  delay(500);

  WiFi.begin(WIFISSID, PASSWORD);
  
  myserial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println();
  Serial.print("Wait for WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
}

void loop() {

  if (myserial.available()) {
    inChar = myserial.read();
//    Serial.print(inChar);

    if (readP || readF || readI || readPip || readPeep || readN)
      inputString += inChar;

    if (readP && (inChar == 'b')) {
      pressure = inputString.toFloat();
      Serial.print("Presion: ");
      Serial.println(pressure);
      inputString = "";
      readP = LOW;
    }
    else if (readF && (inChar == 'r')) {
      frec = inputString.toInt();
      Serial.print("Frec: ");
      Serial.println(frec);
      inputString = "";
      readF = LOW;
    }
    else if (readI && (inChar == 'i')) {
      ieRatio = inputString.toFloat();
      Serial.print("I:E: ");
      Serial.println(ieRatio);
      inputString = "";
      readI = LOW;
    }
    if (readPip && (inChar == 'e')) {
      pip = inputString.toFloat();
      Serial.print("PIP: ");
      Serial.println(pip);
      inputString = "";
      readPip = LOW;
    }
    else if (readPeep && (inChar == 'n')) {
      peep = inputString.toFloat();
      Serial.print("PEEP: ");
      Serial.println(peep);
      inputString = "";
      readPeep = LOW;
    }
    else if (readN && (inChar == ';')) {
      numCiclos = inputString.toInt();
      Serial.print("#Ciclos: ");
      Serial.println(numCiclos);
      inputString = "";
      readN = LOW;
      sendFlag = HIGH;
    }


    if (inChar == 13)
      sendFlag = HIGH;

    if (inChar == 'p') {
      readP = HIGH;
    }

    if (inChar == 'b') {
      readF = HIGH;
    }

    if (inChar == 'r') {
      readI = HIGH;
    }

    if (inChar == 'i') {
      readPip = HIGH;
    }

    if (inChar == 'e') {
      readPeep = HIGH;
    }

    if (inChar == 'n') {
      readN = HIGH;
    }
  }


  if (!client.connected()) {
    reconnect();
  }

  if (sendFlag) {

    sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", ""); // Cleans the payload

    sprintf(payload, "{\"%s\": %s", "p", String(pressure)); // Adds the variable label

    sprintf(payload, "%s, \"%s\": %s", payload, "f", String(frec)); // Adds the variable label

    sprintf(payload, "%s , \"%s\": %s", payload, "r", String(ieRatio)); // Adds the variable label

    sprintf(payload, "%s, \"%s\": %s", payload, "i", String(pip)); // Adds the variable label

    sprintf(payload, "%s, \"%s\": %s", payload, "e", String(peep)); // Adds the variable label

    sprintf(payload, "%s , \"%s\": %s", payload, "n", String(numCiclos)); // Adds the variable label


    sprintf(payload, "%s }", payload); // Closes the dictionary brackets
    Serial.println(payload);
    client.publish(topic, payload);
    sendFlag = LOW;
  }
  client.loop();
}
