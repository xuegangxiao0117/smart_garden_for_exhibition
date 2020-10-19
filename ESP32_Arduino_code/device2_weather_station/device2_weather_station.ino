/*设备3 步道
   设备状态  /device2/status
   sensor:
   - 气压         /weather_station/BMP388/pressure
   - 户外温度     /weather_station/dht22/temperature
   - 户外湿度     /weather_station/dht22/humidity
   - 紫外线       /weather_station/UV
   - （电池电量）
   actuator:

*/
#include <PubSubClient.h>
#include<WiFi.h>
#include <DHTesp.h>
#include "DFRobot_BMP280.h"
#include "Wire.h"

typedef DFRobot_BMP280_IIC    BMP;    // ******** use abbreviations instead of full names ********

BMP   bmp(&Wire, BMP::eSdo_low);


#define MQTT_SERVER "192.168.199.142"
#define SEA_LEVEL_PRESSURE    1015.0f

WiFiClient wifiClient;
DHTesp dht;

char* ssid = "MushroomCloud";
char* password = "12345678";
//测试
//char* ssid = "the way out2_5G";
//char* password = "dfrobot2017";
char* device2_status_Topic = "/device2/status";
char* pressure_Topic = "/weather_station/BMP388/pressure";
char* humidity_Topic = "/weather_station/dht22/humidity";
char* temperature_Topic = "/weather_station/dht22/temperature";
char* uv_Topic = "/weather_station/UV";



int uv_pin = 36; //A0
int dht22_pin = 25;//D2




void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);


void printLastOperateStatus(BMP::eStatus_t eStatus)
{
  switch (eStatus) {
    case BMP::eStatusOK:    Serial.println("everything ok"); break;
    case BMP::eStatusErr:   Serial.println("unknow error"); break;
    case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
    case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
    default: Serial.println("unknow status"); break;
  }
}

void setup() {
  Serial.begin(115200);

  bmp.reset();
  while (bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp begin faild");
    printLastOperateStatus(bmp.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bmp begin success");
  delay(100);

  pinMode(uv_pin, INPUT);
  dht.setup(dht22_pin, DHTesp::DHT22);


  //start wifi subsystem
  WiFi.begin(ssid, password);
  reconnect();
  //wait a bit before starting the main loop
  delay(2000);



}

void loop() {
  if (!client.connected() && WiFi.status() == 3) {
    reconnect();
  }
  //maintain MQTT connection
  client.loop();
  pub_device2_status();
  pub_humidity();
  pub_uv();
  pub_temperature();
  pub_pressure();
  delay(200);

}


void callback(char* topic, byte* payload, unsigned int length) {

}



void pub_device2_status() {
  client.publish(device2_status_Topic, "在线");
  Serial.println("pub pub_device2_status");
}

void pub_uv() {
  client.publish(uv_Topic, String(getuv()).c_str());
  Serial.println("pub_uv: ");
  Serial.println(getuv());
}


void pub_humidity() {
  client.publish(humidity_Topic, String(dht.getHumidity()).c_str());
  Serial.println("pub_humidity: ");
  Serial.println(dht.getHumidity());

}

void pub_temperature() {
  client.publish(temperature_Topic, String(dht.getTemperature()).c_str());
  Serial.println("pub_temp: ");
  Serial.println(dht.getTemperature());
}

void pub_pressure() {
  client.publish(pressure_Topic, String(getpressure()).c_str());
}


float getuv() {
  int uvLevel = averageAnalogRead(uv_pin);
  float outputVoltage = 3.3 * uvLevel / 4095;
  //float outputVoltage = 5.0 * uvLevel/1024;
  float uvIntensity = mapfloat(outputVoltage, 0.80, 2.9, 0.0, 15.0);
  return uvIntensity;
}

int getpressure() {
  uint32_t    pressure = bmp.getPressure();
  return pressure;
}


void reconnect() {

  //attempt to connect to the wifi if connection is lost
  if (WiFi.status() != WL_CONNECTED) {
    //debug printing
    Serial.print("Connecting to ");
    Serial.println(ssid);

    //loop while we wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    //print out some more debug once connected
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

  //make sure we are connected to WIFI before attemping to reconnect to MQTT
  if (WiFi.status() == WL_CONNECTED) {
    // Loop until we're reconnected to the MQTT server
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");

      // Generate client name based on MAC address and last 8 bits of microsecond counter
      String clientName;
      clientName += "esp32 - ";
      uint8_t mac[6];
      WiFi.macAddress(mac);
      clientName += macToStr(mac);

      //if connected, subscribe to the topic(s) we want to be notified about
      if (client.connect((char*) clientName.c_str())) {
        Serial.println("\tMTQQ Connected");
        digitalWrite(LED_BUILTIN, HIGH);
        //        client.subscribe(fan_Topic);
        //        client.subscribe(pump_Topic);
      }

      //otherwise print failed for debugging
      else {
        Serial.println("\tFailed.");
        abort();
      }
    }
  }
}

//generate unique name from MAC addr
String macToStr(const uint8_t* mac) {

  String result;

  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);

    if (i < 5) {
      result += ':';
    }
  }

  return result;
}

int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
