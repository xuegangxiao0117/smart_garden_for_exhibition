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

#include "DFRobot_MAX17043.h"
#include "Wire.h"

#ifdef __AVR__
#define ALR_PIN       2
#else
#define ALR_PIN       D2
#endif

#define PRINT_INTERVAL        2000

DFRobot_MAX17043        gauge;
uint8_t       intFlag = 0;

void interruptCallBack()
{
  intFlag = 1;
}


#define MQTT_SERVER "192.168.199.142"

WiFiClient wifiClient;

char* ssid = "MushroomCloud";
char* password = "12345678";
//测试
//char* ssid = "the way out2_5G";
//char* password = "dfrobot2017";
char* device3_status_Topic = "/device3/status";
char* human_detect_Topic = "/foot_path/human";
char* ambient_light_Topic = "/foot_path/ambient_light";
char* light_Topic = "/foot_path/light";
char* bat_Topic ="/device1/battery";




int ambient_light_pin = 36; //A0
int light_pin = 25;//D2
int human_detect_pin = 26;//D3


void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);


void setup() {
  Serial.begin(115200);

  pinMode(light_pin, OUTPUT);
  pinMode(human_detect_pin, INPUT);


  //start wifi subsystem
  WiFi.begin(ssid, password);
  reconnect();
  //wait a bit before starting the main loop
  delay(2000);

  while (gauge.begin() != 0) {
    Serial.println("gauge begin faild!");
    delay(2000);
  }
  delay(2);
  Serial.println("gauge begin successful!");



}

void loop() {
  if (!client.connected() && WiFi.status() == 3) {
    reconnect();
  }
  //maintain MQTT connection
  client.loop();
  pub_device3_status();
  pub_human_detection();
  pub_ambient_light();
  pub_device_battery();
  delay(200);

}


void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = topic;

  //Print out some debugging info
  Serial.println("Callback update.");
  Serial.print("Topic: ");
  Serial.println(topicStr);

  if (topicStr == "/foot_path/light" ) {
    Serial.println(String(payload[0]));
    if (payload[0] == '0') {
      digitalWrite(light_pin, LOW);
    } else if (payload[0] == '1') {
      digitalWrite(light_pin, HIGH);
    }
  }
}



void pub_device3_status() {
  client.publish(device3_status_Topic, "在线");
  Serial.println("pub pub_device2_status");
}

void pub_human_detection() {
  if (digitalRead(human_detect_pin) == HIGH) {
    Serial.println("pub 检测到行人");
    client.publish(human_detect_Topic, "检测到行人");
  } else {
    Serial.println("pub 未检测到行人");
    client.publish(human_detect_Topic, "未检测到行人");
  }

}


void pub_ambient_light() {
  client.publish(ambient_light_Topic, String(analogRead(ambient_light_pin)).c_str());
  Serial.println("pub_ambient_light: ");
  Serial.println(analogRead(ambient_light_pin));

}

void pub_device_battery() {
  client.publish(bat_Topic, String(get_bat_gauge()).c_str());
  Serial.println("pub_device_battery");
}

float get_bat_gauge() {
  static unsigned long lastMillis = 0;
  if ((millis() - lastMillis) > PRINT_INTERVAL) {
    lastMillis = millis();

    return gauge.readPercentage();
  }

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
        client.subscribe(light_Topic);
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
