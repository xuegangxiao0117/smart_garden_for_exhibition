/*设备1温室
   设备状态  /device1/status
   sensor:
   - CO2       /green_house/ccs811/co2
   - 土壤湿度  /green_house/soil_moisture
   - 门锁状态  /home/door
   - （O2）
   actuator:
   - 风扇      /green_house/fan
   - 水泵      /green_house/pump
*/
#include <PubSubClient.h>
#include<WiFi.h>
#include "DFRobot_CCS811.h"
#include <ESP32_Servo.h>


#define MQTT_SERVER "192.168.199.142"

DFRobot_CCS811 sensor;
WiFiClient wifiClient;
Servo myservo;

char* ssid = "MushroomCloud";
char* password = "12345678";
//测试
//char* ssid = "the way out2_5G";
//char* password = "dfrobot2017";
char* device1_status_Topic = "/device1/status";
char* co2_Topic = "/green_house/ccs811/co2";
char* soil_moisture_Topic = "/green_house/soil_moisture";
char* door_Topic = "/home/door";

char* fan_Topic = "/green_house/fan";
char* pump_Topic = "/green_house/pump";

int moisture_pin = 36; //A0
int door_pin = 25;//D2
int fan_pin = 26;//D3
int servoPin = 27;//D4

int co2_value;
const int AirValue = 3050;
const int WaterValue = 2000;
int intervals = (AirValue - WaterValue) / 3;
int soilMoistureValue = 0;

void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);



void setup() {

  //  ccs811 setup
  while (sensor.begin() != 0) {
    Serial.println("failed to init chip, please check if the chip connection is fine");
    delay(1000);
  }
  sensor.setMeasCycle(sensor.eCycle_250ms);
  delay(100);

  Serial.begin(115200);
  //start wifi subsystem
  pinMode(door_pin, INPUT);
  pinMode(fan_pin, OUTPUT);
  WiFi.begin(ssid, password);
  reconnect();
  //wait a bit before starting the main loop
  delay(2000);


  myservo.attach(servoPin, 500, 2400);
}

void loop() {
  if (!client.connected() && WiFi.status() == 3) {
    reconnect();
  }
  //maintain MQTT connection
  client.loop();
  pub_device1_status();
  pub_co2();
  pub_soil_moisture();
  pub_door();
  delay(200);

}


void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = topic;

  //Print out some debugging info
  Serial.println("Callback update.");
  Serial.print("Topic: ");
  Serial.println(topicStr);

  char* fan_Topic = "/green_house/fan";
  char* pump_Topic = "/green_house/pump";

  //  if (topicStr == "" ) {
  //    if (payload[0] == '0') {
  //
  //    } else if (payload[0] == '1') {
  //
  //    }
  //  }

  if (topicStr == "/green_house/fan" ) {
    if (payload[0] == '0') {
      digitalWrite(fan_pin, LOW);
    } else if (payload[0] == '1') {
      digitalWrite(fan_pin, HIGH);
    }
  }

  if (topicStr == "/green_house/pump" ) {
    if (payload[0] == '0') {
      myservo.write(150);
      delay(500);
    } else if (payload[0] == '1') {
      myservo.write(30);
      delay(500);
    }
  }



}



void pub_device1_status() {
  client.publish(device1_status_Topic, "在线");
}

void pub_co2() {
  client.publish(co2_Topic, String(getCO2()).c_str());
}


void pub_soil_moisture() {
  int temp = getMoisture();
  if (temp > WaterValue && soilMoistureValue < (WaterValue + intervals))
  {
    client.publish(soil_moisture_Topic, "非常湿润");
  }
  else if (temp > (WaterValue + intervals) && soilMoistureValue < (AirValue - intervals))
  {
    client.publish(soil_moisture_Topic, "湿润");
  }
  else if (temp < AirValue && soilMoistureValue > (AirValue - intervals))
  {
    client.publish(soil_moisture_Topic, "干燥");
  }
}

void pub_door() {
  int temp = digitalRead(door_pin);
  if (temp == LOW) {
    client.publish(door_Topic, "ON");
  } else {
    client.publish(door_Topic, "OFF");
  }

}


int getCO2() {
  if (sensor.checkDataReady() == true) {
    co2_value = sensor.getCO2PPM();
  } else {
    co2_value = 0;
  }
  sensor.writeBaseLine(0xC47A);
  delay(5000);
  return co2_value;
}

int getMoisture() {
  soilMoistureValue = analogRead(moisture_pin);
  return soilMoistureValue;
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
        client.subscribe(fan_Topic);
        client.subscribe(pump_Topic);
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
