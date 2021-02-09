
//-------SENSORS INCLUDES----

#include <stm32l475e_iot01.h>
#include <stm32l475e_iot01_accelero.h>
#include <stm32l475e_iot01_gyro.h>
#include <stm32l475e_iot01_hsensor.h>
#include <stm32l475e_iot01_magneto.h>
#include <stm32l475e_iot01_psensor.h>
#include <stm32l475e_iot01_qspi.h>
#include <stm32l475e_iot01_tsensor.h>

//-------WIFI INCLUDES-------

#include <SPI.h>
#include <WiFiST.h>

//-------MQTT INCLUDES-------

#include <MQTT.h>
#include <MQTTClient.h>

//-------GLOBAL VARIABLES----

//*********Connection********
WiFiClient net;
MQTTClient client;
SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);
char ssid[] = "Hichri";          //  network name
char pass[] = "AR404318";              //  network password
char serverIPAddress[] = "192.168.1.11"; //  IP address of MQTT broker(a.k.a server)
char login[] = "";                   //  Login of server
char password[] = "";            //  Password of server
char clientName[] = "DIS_L4IOT";         //  Name of board(a.k.a client)
int status = WL_IDLE_STATUS;             //  Wifi radio's status

//**********Sensors**********
int16_t MagnetoDataXYZ[3]  = {0,0,0};  //Magneto value list
float GyroDataXYZ[3]       = {0,0,0};  //Gyro value list
int16_t AcceleroDataXYZ[3] = {0,0,0};  //Accelero value list
float fTemperatureValue,fHumidityValue,fPressureValue;  //Sensor float variables
String sAccValues,sGyroValues,sMagnetoValues,sTemperatureValue,sHumidityValue,sPressureValue; //Sensor string variables

//-------SETUP---------------

void setup() {
  initSerial();     // initialize serial communication and wait for serial monitor to open
  initWifi();       // initialize the WiFi module
  initSensors();    // initialize sensors
  connectToBroker();// connect to MQTT broker
}

//-------LOOP----------------

void loop() {
  reconnect();
  publishTemperature();
  publishHumidity();
  publishMagneto();
  publishGyro();
  publishAccelero();
}

//-------FUNCTION DEFINITIONS----
//---------------INIT------------
void initSerial(){
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
}

void initWifi(){
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi module not detected");
    // don't continue:
    while (true);
  }

  // Check firmware version:
  String fv = WiFi.firmwareVersion();
  if (fv != "C3.5.2.5.STM") {
    Serial.println("Please upgrade the firmware");
  }

  // Attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA/WPA2 network: ");
    Serial.println(ssid);
    // Connect to WPA2 network:
    status = WiFi.begin(ssid, pass);
    if (status != WL_CONNECTED) {
      // Connect to WPA (TKIP) network:
      
      //status = WiFi.begin(ssid, pass, ES_WIFI_SEC_WPA);
    }

    // wait 10 seconds for connection:
    delay(1000);
  }

  // you're connected now, so print out the connection info:
  Serial.println("Connected.");
}

void initSensors(){
  BSP_TSENSOR_Init();
  BSP_HSENSOR_Init();
  BSP_PSENSOR_Init();
  BSP_MAGNETO_Init();
  BSP_GYRO_Init();
  BSP_ACCELERO_Init(); 
}

void connectToBroker() {
  Serial.print("Checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  client.begin(serverIPAddress, net);
  Serial.print("\nConnecting to MQTT Broker...");
  while (!client.connect(clientName,login,password)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  //client.subscribe("/test/#");
  // client.unsubscribe("/hello");
}

//---------------LOOP------------

void publishTemperature(){
 fTemperatureValue = BSP_TSENSOR_ReadTemp();
 Serial.print("\n TEMPERATURE = ");
 Serial.print(fTemperatureValue);
 sTemperatureValue = String(fTemperatureValue, 1);
 client.publish("/temperature", sTemperatureValue);
}

void publishHumidity(){
 fHumidityValue = BSP_HSENSOR_ReadHumidity();
 Serial.print("\n HUMIDITY = ");
 Serial.print(fHumidityValue);
 sHumidityValue = String(fHumidityValue, 1);
 client.publish("/humidity", sHumidityValue);
}

void publishGyro(){
  BSP_GYRO_GetXYZ(GyroDataXYZ);
  Serial.print("\n GYRO_X = ");
  Serial.print(GyroDataXYZ[0]);
  Serial.print("\n GYRO_Y = ");
  Serial.print(GyroDataXYZ[1]);
  Serial.print("\n GYRO_Z = ");
  Serial.print(GyroDataXYZ[2]);
  //format the string
  sGyroValues = '(' + String(GyroDataXYZ[0], 1) + ',' + String(GyroDataXYZ[1], 1) + ',' + String(GyroDataXYZ[2], 1) + ')';
  client.publish("/gyro", sGyroValues);
}

void publishAccelero(){
  BSP_ACCELERO_AccGetXYZ(AcceleroDataXYZ);
  Serial.print("\n ACC_X = ");
  Serial.print(AcceleroDataXYZ[0]);
  Serial.print("\n ACC_Y = ");
  Serial.print(AcceleroDataXYZ[1]);
  Serial.print("\n ACC_Z = ");
  Serial.print(AcceleroDataXYZ[2]);
  //format the string
  sAccValues = '(' + String(AcceleroDataXYZ[0], 1) + ',' + String(AcceleroDataXYZ[1], 1) + ',' + String(AcceleroDataXYZ[2], 1) + ')';
  client.publish("/accelero", sAccValues);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Connection lost,Attempting to reconnect...");
    if (client.connect(clientName, login, password)) {
      Serial.println("Connected");
      } else {
      Serial.print("Failed ,will try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
