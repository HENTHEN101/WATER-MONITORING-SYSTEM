#include <WiFi.h>
#include "time.h"
#include <SPI.h>
#include <LoRa.h>
#include <Nextion.h>

//pump start time
int startH;
int startM;
bool pump_triggered = false;
//pump stop time
int stopH;
int stopM;

//wifi credentials
const char* ssid     = "Shane's A15";
const char* password = "Meforever123";

const char* ntpServer = "jm.pool.ntp.org";
const long  gmtOffset_sec = -18000;
const int   daylightOffset_sec = 0;//GMT+5:30

//NEXTION DISPLAY OBJECTS
NexPage page1    = NexPage(1, 0, "page1");

void setup(){
  nexInit();
  dbSerial.begin(9600);
  Serial.begin(115200);
  delay(500);
  page1.show();


  syncTimeFromWiFi();
}

void loop(){
  delay(1000);
  printLocalTime();
}

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  checkPump(timeinfo);

}

void syncTimeFromWiFi(){
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

void checkPump(struct tm &timeinfo){
  if (!pump_triggered && timeinfo.tm_hour ==startH && timeinfo.tm_min == startM && timeinfo.tm_sec ==0 ){
    Serial.print("pump gogo");
  }else{
    return;
  }
  pump_triggered = true;
}
