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
#include "DFRobot_OxygenSensor.h"

#define Oxygen_IICAddress ADDRESS_3

#define MQTT_SERVER "192.168.199.142"

#define TdsSensorPin 35
#define VREF 3.3    // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

DFRobot_OxygenSensor Oxygen;
WiFiClient wifiClient;

char* ssid = "MushroomCloud";
char* password = "12345678";
//测试
//char* ssid = "the way out2_5G";
//char* password = "dfrobot2017";
char* device4_status_Topic = "/device4/status";
char* tds_Topic = "/water_quality/tds";
char* ds18b20_Topic = "/water_quality/ds18b20";
char* o2_Topic = "/green_house/o2";




int ds18b20_pin = 26;//D3


void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);


void setup() {
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);

  while (!Oxygen.begin(Oxygen_IICAddress)) {
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");
  Oxygen.SetKeys(20.9 / 102.3);
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
  pub_device4_status();
  pub_tds();
  pub_o2();
  delay(200);

}


void callback(char* topic, byte* payload, unsigned int length) {

}



void pub_device4_status() {
  client.publish(device4_status_Topic, "在线");
  Serial.println("pub pub_device4_status");
}

void pub_tds() {
  client.publish(tds_Topic, String(gettds()).c_str());
  delay(500);
  //Serial.println("pub_tds:");
  //Serial.println(gettds());
}


void pub_o2() {
  client.publish(o2_Topic, String(geto2()).c_str());
  Serial.println("pub_o2_Topic: ");
  Serial.println(geto2());

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
        //client.subscribe(light_Topic);
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

float gettds() {

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V   ");
    //Serial.print("TDS Value:");
    //Serial.print(tdsValue, 0);
    //Serial.println("ppm");
    return tdsValue;
  }
}


int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

float geto2() {

  float oxygenData = Oxygen.ReadOxygenData();
  return oxygenData;
}
