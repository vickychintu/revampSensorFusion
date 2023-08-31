#include "RTClib.h"
#include <Adafruit_MPU6050.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <String.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//
#include <WiFi.h>
#include <HTTPClient.h>
TaskHandle_t task1Handle;
TaskHandle_t task2Handle;
//
Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;

//ultrasonic sensor setup
const int trigPin = 13;
const int echoPin = 12;
//define sound speed in cm/uS
#define SOUND_SPEED 0.034
//
#define FORMAT_LITTLEFS_IF_FAILED true
//temp file to log sensor data
const char* tempFileName = "/temp.json";
const char* ssid = "IKP-1stFloor";
const char* password = "ikigai21";
String serverName = "http://16.170.246.20:4000/update-sensor";
//ntp server data
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;
//creating a mutex
SemaphoreHandle_t myMutex;

int i=0;
void datalogger(void *pvParameters){
  while(1){
    Serial.print(i);
    if(i==20){
      i=0;
      //renaming file in send format
      // if (xSemaphoreTake(myMutex, portMAX_DELAY) == pdTRUE) {
        renameTempFile();
        // xSemaphoreGive(myMutex);
      // }
    }
    else{
      readSensorData();
      i++;
    }
    //reading sensor data
    
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
 
}
void datasender(void *pvParameters){
  // Serial.print("in data sender");
  while(1){
    if (WiFi.status() != WL_CONNECTED) {
      reconnect();
    }
    if (WiFi.status() == WL_CONNECTED) {
      sendPerFilesToServer();
      Serial.println("Jack dosent live here");
    } else {
      Serial.print("try in next attempt");
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
  
}
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return (0);
  }
  Serial.println("time obtained successfully");
  time(&now);
  Serial.println(now);
  return now;
}
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  int i = 0;
  while (WiFi.status() != WL_CONNECTED && i < 30) {
    delay(500);
    Serial.print(".");
    i++;
    if (i == 9) {
      return;
    }
  }
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //
  unsigned long testT=getTime();
  if(testT != 0){
    Serial.println("setting time");
    rtc.adjust(DateTime(testT));

  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void setup() {
  //setting up baud rate 
  Serial.begin(115200);
  while(!Serial)
    delay(10);
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("LittleFS mount failed");
    return;
  }
  else{
    Serial.println("LittleFS mount succeded");
  }
  //initilizing mpu
  if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
  ESP.restart();
    for(;;) {
      delay(10);
    }
  }
  //MPU setup
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //rtc startup
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
      ESP.restart();
    Serial.flush();
    while (1) delay(10);
  }
  //setUp rtc if it lost power
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);
  setup_wifi();
  myMutex = xSemaphoreCreateMutex();
  xTaskCreate(datalogger, "Task 1", 10000, NULL, 1, &task1Handle);
  xTaskCreate(datasender, "Task 2", 10000, NULL, 2, &task2Handle);
}
//main function one that logs data
void reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }
}
void loop() {
  // Serial.println("int1.......");
}

void renameTempFile(){
  String permFileName="/per";
  permFileName=permFileName+rtc.now().unixtime()+"e.json";
  if (LittleFS.rename(tempFileName,permFileName )) {
    Serial.println("File renamed successfully");
  } else {
    Serial.println("Failed to rename file");
  }
}
//main function two that sends data

void sendPerFilesToServer() {
  Serial.println("in send function");
  File root = LittleFS.open("/");
    File file = root.openNextFile();
    if(file.isDirectory()){
      return;  
    } 
    String fileName = file.name();
    
    if (fileName.startsWith("per")) {
      if (file) {
        String fileContent = file.readString();
        file.close();
        Serial.print(fileName);
        // Serial.print(fileContent);
        HTTPClient http;
        http.begin("http://16.170.246.20:4000/getSensorData");
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        int httpCode = http.POST("logData=[" + fileContent+"]");
        String response = http.getString();
        response.trim();
        Serial.print("Return code = ");
        Serial.println(httpCode);
        http.end();
        Serial.print('done sending');
        if (httpCode == 200) {
          fileName="/"+fileName;
          if (LittleFS.remove(fileName)) {
            Serial.println("File removed successfully");
          } else {
            Serial.println("Failed to remove file");
          }
        } else {
          Serial.print('failed to send data');
        }
        
      }
    }
  
}
//
void readSensorData(){
  sensors_event_t a, g, temp;
  float distanceCm = ultrasonicDistance();
  mpu.getEvent(&a, &g, &temp);
  uint32_t time = rtc.now().unixtime();
  jsonToFile(a.acceleration.x,a.acceleration.y,a.acceleration.z,g.gyro.x,g.gyro.y,g.gyro.z,time,distanceCm);
}
void jsonToFile( float x, float y, float z, float gx, float gy, float gz, uint32_t time,float distance){
  File queueFile = LittleFS.open(tempFileName, "a+");
  if (!queueFile) {
    Serial.println("Failed to open queue file");
    return;
  }
  DynamicJsonDocument jsonDoc(200);
  jsonDoc["x"]  = x;
  jsonDoc["y"]  = y;
  jsonDoc["z"]  = z;
  jsonDoc["gx"] = gx;
  jsonDoc["gy"] = gy;
  jsonDoc["gz"] = gz;
  jsonDoc["time"]=time;
  jsonDoc["distance"]=ultrasonicDistance();
  jsonDoc["machine"]=2;

  serializeJson(jsonDoc, queueFile);
  queueFile.println();
  queueFile.close();
  return;
}
float ultrasonicDistance() {
  digitalWrite(trigPin, LOW);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float echoTime = pulseIn(echoPin, HIGH);
  float echoDistance = echoTime * SOUND_SPEED / 2;
  Serial.println(echoDistance);
  return echoDistance;
}