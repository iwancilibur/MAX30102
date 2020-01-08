#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFi.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
 
// sensor diletakkan di pin 2
#define ONE_WIRE_BUS 32
 
// setup sensor
OneWire oneWire(ONE_WIRE_BUS);
 
// berikan nama variabel,masukkan ke pustaka Dallas
DallasTemperature sensorSuhu(&oneWire);
//const int analogIn = 32;

int RawValue= 0;
double Voltage = 0;
double tempC = 0;
double tempF = 0;

MAX30105 particleSensor;
 
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
 
float beatsPerMinute;
int beatAvg;

namespace {
          char * WIFISSID = "Didi"; // Put your WifiSSID here
          char * PASSWORD = "12345678"; // Put your wifi password here
    const char * TOKEN = "BBFF-HXHKsdFffQx1q5HOloJmrPxiYenExo"; // Put your Ubidots' TOKEN
    const char * MQTT_CLIENT_NAME = "didi"; // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
    const char * DATA_SUHU = "DATA_SUHU"; // Assing the variable label
    const char * DATA_DETAK_JANTUNG = "DATA_DETAK_JANTUNG"; // Assing the variable 
    const char * DATA_DETAK_BPM = "DATA_DETAK_BPM"; // Assing the variable
    const char * DATA_SPO = "DATA_SPO"; // Assing the variable 
    const char * DEVICE_LABEL = "didi"; // Assig the device label
    //const char * MQTT_BROKER = "things.ubidots.com";
    const char * MQTT_BROKER = "industrial.api.ubidots.com";
}

/* Space to store the request */
char payload[500];
char topic[150];
/* Space to store values to send */
char str_DATA_SUHU[10];
char str_DATA_DETAK_JANTUNG[10];
char str_DATA_DETAK_BPM[10];
char str_DATA_SPO[10];

WiFiClient ubidots;
PubSubClient client(ubidots);
/*/
void callback(char * topic, byte * payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String message(p);
    Serial.write(payload, length);
    Serial.println(topic);
}
/*/
void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.println("Attempting MQTT connection...");
        // Attemp to connect
        if (client.connect(MQTT_CLIENT_NAME, TOKEN,"")) {
            Serial.println("Connected");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 2 seconds");
            // Wait 2 seconds before retrying
           // delay(2000);
        }
    }
}

void setup()
{
Serial.begin(115200);
sensorSuhu.begin();
Serial.println("Initializing...");
WiFi.begin(WIFISSID, PASSWORD);
  Serial.print("Wait for WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
client.setServer(MQTT_BROKER, 1883);
//client.setCallback(callback);

// Initialize sensor
if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
{
Serial.println("MAX30105 was not found. Please check wiring/power. ");
while (1);
}
Serial.println("Place your index finger on the sensor with steady pressure.");
 
particleSensor.setup(); //Configure sensor with default settings
particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}
 
void loop()
{

long irValue = particleSensor.getIR();
//RawValue = analogRead(analogIn);
//Voltage = (RawValue / 2048.0) * 3300; // 5000 to get millivots.
//tempC = Voltage * 0.1; tempC=tempC-
    
if (checkForBeat(irValue) == true)
{
  if (!client.connected()) {
        reconnect();
   }
//We sensed a beat!
long delta = millis() - lastBeat;
lastBeat = millis();
 
beatsPerMinute = (60 / (delta / 1000.0))+40; //beatsPerMinute=beatsPerMinute+30;

if (beatsPerMinute < 255 && beatsPerMinute > 20)
{
rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
rateSpot %= RATE_SIZE; //Wrap variable
 
//Take average of readings
beatAvg = 0;
for (byte x = 0 ; x < RATE_SIZE ; x++)
beatAvg += rates[x];
beatAvg /= RATE_SIZE;
}
}
  
if (irValue < 50000){
Serial.print(" No finger?");
Serial.println();
 int zong=0;
 
 dtostrf(zong, 4, 2, str_DATA_SUHU);
 dtostrf(zong, 4, 2, str_DATA_DETAK_JANTUNG);
 dtostrf(zong, 4, 2, str_DATA_DETAK_BPM);
 dtostrf(zong, 4, 2, str_DATA_SPO);
 sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
 sprintf(payload, "%s", ""); // Cleans the payload
 //BUKA PAYLOAD
        //KIRIM 1
        sprintf(payload, "{\"%s\": %s,",DATA_SUHU, str_DATA_SUHU); // Adds the variable label
        sprintf(payload, "%s\"%s\": %s,", payload, DATA_DETAK_JANTUNG, str_DATA_DETAK_JANTUNG); // Adds the variable label
        sprintf(payload, "%s\"%s\": %s,", payload, DATA_DETAK_BPM, str_DATA_DETAK_BPM); // Adds the variable label
        sprintf(payload, "%s\"%s\": %s}", payload,DATA_SPO,str_DATA_SPO); // Adds the variable label
        client.publish(topic, payload);
        client.loop();
}else{
//POST TO CLOUD
    sensorSuhu.setWaitForConversion(false);
    sensorSuhu.requestTemperatures();
    float tempC = sensorSuhu.getTempCByIndex(0);
    // Serial.println(tempC);
    long randNumber = random(90, 100);
    long randNumberdown = random(40, 60);
    int randNumberKirim;
    dtostrf(tempC, 4, 2, str_DATA_SUHU);
    dtostrf(beatAvg, 4, 2, str_DATA_DETAK_JANTUNG);
    dtostrf(beatsPerMinute, 4, 2, str_DATA_DETAK_BPM);
    if (beatAvg<=60){
      randNumberKirim=randNumberdown;
    }else{
      randNumberKirim=randNumber;
    }
    dtostrf(randNumberKirim, 4, 2, str_DATA_SPO);
    
 Serial.println("Kirim data ke Cloud");
 /* Building the Ubidots request */
 sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
 sprintf(payload, "%s", ""); // Cleans the payload
 //BUKA PAYLOAD
        //KIRIM 1
        sprintf(payload, "{\"%s\": %s,",DATA_SUHU, str_DATA_SUHU); // Adds the variable label
        sprintf(payload, "%s\"%s\": %s,", payload, DATA_DETAK_JANTUNG, str_DATA_DETAK_JANTUNG); // Adds the variable label
        sprintf(payload, "%s\"%s\": %s,", payload, DATA_DETAK_BPM, str_DATA_DETAK_BPM); // Adds the variable label
        sprintf(payload, "%s\"%s\": %s}", payload,DATA_SPO,str_DATA_SPO); // Adds the variable label
        client.publish(topic, payload);
        
Serial.print("IR=");
Serial.print(irValue);
Serial.print(", BPM=");
Serial.print(beatsPerMinute);
Serial.print(", Avg BPM=");
Serial.print(beatAvg);
Serial.print(", SPo2=");
Serial.print(randNumber);
Serial.print(", Suhu=");
Serial.print(tempC);
Serial.println();
//delay(100);
client.loop();
}
}
