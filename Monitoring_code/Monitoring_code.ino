#include <SPI.h>
#include <HardwareSerial.h>
#include <LoRa.h>

HardwareSerial Ultrasonic_Sensor(2);//   TX2 (pin 17), RX2 (pin 16)
// Define connections to sensor
int pinRX = 16;  // Choose a suitable pin for RX
int pinTX = 17;  // Choose a suitable pin for TX 

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
int distance;

//packet counter
int counter = 0;
String LoRaData;
String outgoing;              // outgoing message
//int relay1Status;
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 50;    
String Mymessage;
// Array to store incoming serial data
unsigned char data_buffer[4] = {0};
// Variable to hold checksum
unsigned char CS;
 
void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  Ultrasonic_Sensor.begin(9600, SERIAL_8N1, pinRX, pinTX); // Initialize the hardware serial
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433E6)) {             
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
 
  Serial.println("LoRa init succeeded.");

}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - lastSendTime > interval) {
    if (Ultrasonic_Sensor.available() > 0) {
      delay(4);
       // Check for packet header character 0xff
      if (Ultrasonic_Sensor.read() == 0xff) {
        // Insert header into array
        data_buffer[0] = 0xff;
        // Read remaining 3 characters of data and insert into array
        for (int i = 1; i < 4; i++) {
         data_buffer[i] = Ultrasonic_Sensor.read();
        }
        //Compute checksum
        CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
        // If checksum is valid compose distance from data
        if (data_buffer[3] == CS) {
        distance = (data_buffer[1] << 8) + data_buffer[2];
        // Print to serial monitor
        distance= distance / 10; // cm
        }
      }
    }
    Serial.print("Distance: ");
    Serial.println(distance);
   //Print the distance on the Serial Monitor (Ctrl+Shift+M):
   int waterLevelPer = map(distance,22,51, 100, 0);
   if(waterLevelPer<0){
    waterLevelPer=0;
   }
   if(waterLevelPer>100){
    waterLevelPer=100;
   }
   Serial.print("Distance = ");
   Serial.print(distance);
   Serial.println(" cm");
   Serial.println(waterLevelPer);
   Mymessage = Mymessage + distance +","+ waterLevelPer;  
   sendMessage(Mymessage);
   Mymessage = "";
   //Serial.println("Sending " + message);
   lastSendTime = millis();            // timestamp the message
   interval = random(50) + 100;
  }
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());  

}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
   if (packetSize == 0) return;          // if there's no packet, return
   // read packet header bytes:
    int recipient = LoRa.read();          // recipient address
    byte sender = LoRa.read();            // sender address
    byte incomingMsgId = LoRa.read();     // incoming msg ID
    byte incomingLength = LoRa.read();    // incoming msg length
  
    String incoming = "";

    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    if (incomingLength != incoming.length()) {   // check length for error
      Serial.println("error: message length does not match length");
      ;
      return;                             // skip rest of function
    }
    // if the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0xFF) {
     Serial.println("This message is not for me.");
      ;
      return;                             // skip rest of function
    }
  //type getvalue code here
 }

String getValue(String data, char separator, int index){
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;
 
    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
 }
  