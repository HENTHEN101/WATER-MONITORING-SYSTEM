#include <SPI.h>
#include <HardwareSerial.h> 
#include <LoRa.h>


//DEFINED VARIBALES
//lora pins
#define RST 25
#define DIO0 26 //12//27//13
#define SS 5 
#define STUCK_WINDOW 20
#define MIN_VARIANCE 0.05f  // cm²
//data and ack packets bytes 
#define MSG_DATA         0x01 
#define MSG_ACK          0x02 
#define MSG_CON          0x03 
#define DATA_TANK_LEVEL  0x04 
#define DATA_CALIBRATION 0x05 
#define SYS_AVAILABLE    0x07
#define CAL_SETTINGS     0x08
#define CAL_MARK_EMPTY   0x09
#define CAL_MARK_FULL    0x10
#define CAL_ESTIMATED    0x11
#define MSG_REQ_TANKLVL  0x12
//calobration stages
#define CAL_STAGE_NONE 0
#define CAL_STAGE_ESTIMATED 1
#define CAL_STAGE_PARTIAL   2
#define CAL_STAGE_FULL      3
//LED pins
#define CON_LED 27
#define RED_LED 13
#define BLUE_LED 14
#define GREEN_LED 12

//SYSTEM ADDRESSES
byte localaddress = 0xBB;  // address of this device 
byte destinationtdsply = 0xFF; // destination to send to 

//TIMER VARIABLES
const unsigned long CONNECTION_TIMEOUT = 120000; // 2 minutes
const unsigned long ACK_TIMEOUT = 5000; 
unsigned long lastSend =0;
const unsigned long SEND_INTERVAL = 10000;
long lastSendTime = 0; // last send time
int interval = 10000;
unsigned long lastConnectionSeen = 0;

//DATA FLAGS
volatile bool loraPacketReady = false;
bool geometryInitialized = false;
bool connected = false;
bool datarequested = false;

//LORA DATA HOLDERS
uint8_t lastRxSeq = 255;   // invalid initial value
volatile int loraPacketSize = 0;
uint8_t loraRxBuffer[21];   // safe size
byte msgcount = 0;  // count of outgoing messages 

//SYSTEM DATA HOLDERS
float sensorOffset = -1;  // cm, unknown initially
float tankHeight =-1; //cm,unknown inittially
int calEmptyDistance = -1;
int calFullDistance  = -1;
uint8_t calibrationStage = CAL_STAGE_ESTIMATED;
const float SENSOR_MIN_DISTANCE = 3.0;  // cm 
const float SENSOR_MAX_DISTANCE = 450.0; // cm
unsigned char data_buffer[4] = {0}; 

//SENSOR PINS
HardwareSerial Ultrasonic_Sensor(2);// TX2 (pin 17), RX2 (pin 16) 
int pinRX = 16; // Choose a suitable pin for RX 
int pinTX = 17; // Choose a suitable pin for TX

//LORA DATA PACKETS STRUCTURES
struct pendingTX{ 
  bool active; 
  byte dest = destinationtdsply; 
  byte type; 
  byte txID; 
  uint8_t retriesleft; 
  unsigned long lastsendtime; 
  uint8_t payload[16]; 
  uint8_t payloadLen; 
};
pendingTX tx{};

enum CalConfidence : uint8_t {
  CONF_NONE = 0,
  CONF_LOW  = 1,   // geometry / estimated
  CONF_MED  = 2,   // observed repeatedly
  CONF_HIGH = 3    // user-marked
};
struct CalPoint {
  float distance;             // cm
  CalConfidence confidence;
  uint8_t stableHits;       // how many times we observed this
};
CalPoint calFull  = { -1, CONF_NONE, 0 };
CalPoint calEmpty = { -1, CONF_NONE, 0 };

struct TankLevelPayload { 
  uint8_t percent;
  uint8_t cal_stage; 
}; 
TankLevelPayload tnklvl;


struct CalibrationSettings { 
  uint8_t mode;   // what the user did
  uint8_t value;  // % if estimated, unused otherwise 
  uint16_t tankHeightCm;  // usable water height
};
CalibrationSettings tnkset;


enum SystemState {
  STATE_UNCALIBRATED,
  STATE_CALIBRATING,
  STATE_READY
};
SystemState systemState = STATE_UNCALIBRATED;


void setup() { 
  //initialize Serial Monitor 
  Serial.begin(9600);
  pinMode(CON_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(BLUE_LED,OUTPUT);
  pinMode(GREEN_LED,OUTPUT);
  digitalWrite(CON_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW); 
  Ultrasonic_Sensor.begin(9600, SERIAL_8N1, pinRX, pinTX); // Initialize the hardware serial  
  LoRa.setPins(SS, RST, DIO0); 
  if (!LoRa.begin(525E6)) { 
    Serial.println("LoRa init failed. Check your connections."); 
    while (true){
    Serial.print("."); 
    delay(100);
    } // if failed, do nothing 
    Serial.println("LoRa init succeeded."); 
  } 
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("setup done.");
  //starttransaction(nullptr,0,SYS_AVAILABLE);
}
void loop() { 
  unsigned long now = millis();
  if (loraPacketReady) {
    loraPacketReady = false;
    processLoRaPacket(loraRxBuffer, loraPacketSize);
    LoRa.receive();   // return to RX mode
  }

  if (connected && systemState == STATE_READY && datarequested && !tx.active) { 
    sendTankLevel(); 
    datarequested = false;
    //lastSend = now + SEND_INTERVAL; 
  }  
  processtx(); 
}


bool readsensor(float &outDistance) {
  static uint8_t idx = 0;
  static uint8_t buf[4];
  unsigned long start = millis();

  while (Ultrasonic_Sensor.available()) {
    uint8_t b = Ultrasonic_Sensor.read();

    // Timeout protection (VERY important)
    if (millis() - start > 30) {
      idx = 0;
      return false;
    }

    if (idx == 0 && b != 0xFF) {
      continue; // wait for frame header
    }

    buf[idx++] = b;

    if (idx == 4) {
      idx = 0;

      uint8_t sum = (buf[0] + buf[1] + buf[2]) & 0xFF;
      if (sum != buf[3]) return false;

      float d = ((buf[1] << 8) | buf[2]) / 10.0f;
      if (d < SENSOR_MIN_DISTANCE || d > SENSOR_MAX_DISTANCE) return false;

      outDistance = d;
      Serial.print("Distance = ");
      Serial.println(d);
      return true;
    }
  }
  return false;
}
bool isCalibrated() {
  return (calFull.distance > 0 && calEmpty.distance > 0 && calEmpty.distance > calFull.distance);
}


void resetTx() { 
  tx = pendingTX{}; // C++ value-initialize (clears everything) 
  //tnkset = CalibrationSettings{};
  //tnkcal=CalibrationPayload {};
}


uint8_t computePercent(float d){
  
 if (calFull.distance < 0 || calEmpty.distance < 0)
    return 0;

  if (d <= calFull.distance)  return 100;
  if (d >= calEmpty.distance) return 0;

  return (uint8_t)(
    100.0f * (calEmpty.distance - d) /
    (calEmpty.distance - calFull.distance)
  );
}
void observeCalibration(float d) {
  static int lastFullCandidate  = calFull.distance;
  static int lastEmptyCandidate = calEmpty.distance;

  // --- FULL observation ---
  if (calFull.confidence < CONF_HIGH &&
      d < calFull.distance - 2) {

    if (abs(d - lastFullCandidate) <= 2) {
      calFull.stableHits++;
    } else {
      calFull.stableHits = 1;
    }

    lastFullCandidate = d;

    if (calFull.stableHits >= 3) {
      calFull.distance   = d;
      calFull.confidence = CONF_MED;
      Serial.printf("FULL refined to %d cm (observed)\n", d);
    }
  }

  // --- EMPTY observation ---
  if (calEmpty.confidence < CONF_HIGH &&
      d > calEmpty.distance + 2) {

    if (abs(d - lastEmptyCandidate) <= 2) {
      calEmpty.stableHits++;
    } else {
      calEmpty.stableHits = 1;
    }

    lastEmptyCandidate = d;

    if (calEmpty.stableHits >= 3) {
      calEmpty.distance   = d;
      calEmpty.confidence = CONF_MED;
      Serial.printf("EMPTY refined to %d cm (observed)\n", d);
    }
  }
}


bool isSensorStuck(float d) {
  static float samples[STUCK_WINDOW];
  static uint8_t idx = 0;
  static bool filled = false;

  samples[idx++] = d;
  if (idx >= STUCK_WINDOW) {
    idx = 0;
    filled = true;
  }

  if (!filled) return false;

  float mean = 0;
  for (int i = 0; i < STUCK_WINDOW; i++) mean += samples[i];
  mean /= STUCK_WINDOW;

  float variance = 0;
  for (int i = 0; i < STUCK_WINDOW; i++) {
    float diff = samples[i] - mean;
    variance += diff * diff;
  }
  variance /= STUCK_WINDOW;

  return variance < MIN_VARIANCE;
}
void processtx(){ 
  if (!tx.active) return; 
  unsigned long now = millis(); 
  if (now - tx.lastsendtime < ACK_TIMEOUT) return; 
  if(tx.retriesleft ==0){
    Serial.println("Transaction failed");
    connected = false; 
    digitalWrite(CON_LED, LOW);
    resetTx(); 
    return; 
  } 
  Serial.println("Retrying transaction.."); 
  tx.retriesleft --; 
  sendRAW(tx); 
}

//tank level pack
uint8_t packTankLevel(uint8_t* buf, const TankLevelPayload& p){
    buf[0] = p.percent;
    buf[1] = p.cal_stage;
    return 2;
}
void SendTanklevel(){
  uint8_t payload[2];

  uint8_t len = packTankLevel(payload, tnklvl);

  starttransaction(payload, len, DATA_TANK_LEVEL);
}
void sendTankLevel() { 
  //if(distance==lastreaddistance)return;
  //lastreaddistance = distance;
  if (systemState != STATE_READY) return;
   float d;

  if (!readSensorWithRetry(d)) {
    Serial.println("Tank level read failed");
    return;
  }
  // if (isSensorStuck(d)) {
  // Serial.println("Sensor variance too low — likely stuck");
  // return;
  // }

  //observeCalibration(d);
  uint8_t percent = computePercent(d);
  tnklvl.percent = percent;
  tnklvl.cal_stage = calibrationStage;
  SendTanklevel();
  //starttransaction(&tnklvl,sizeof(TankLevelPayload),DATA_TANK_LEVEL);
  //Serial.print("Sent tank level: "); 
  //Serial.println(percent); 
}

//calibration settings pack
uint8_t packCalibrationSettings(uint8_t* buf, const CalibrationSettings& c){
    buf[0] = c.mode;
    buf[1] = c.value;

    buf[2] = c.tankHeightCm & 0xFF;
    buf[3] = (c.tankHeightCm >> 8) & 0xFF;

    return 4;
}
void unpackCalibrationSettings(const uint8_t* buf, CalibrationSettings& c){
    c.mode = buf[0];
    c.value = buf[1];
    c.tankHeightCm = buf[2] | (buf[3] << 8);
}
bool getCalibratioNSettings(const uint8_t *payload, uint8_t len){
  if (len < 4) return false;
  unpackCalibrationSettings(payload,tnkset);
  return processCalibration(tnkset);
}
bool processCalibration(CalibrationSettings& c) {
  float d;

  if (!readSensorWithRetry(d)) {
    Serial.println("Calibration failed: sensor unstable");
    //systemState = STATE_UNCALIBRATED;
    return false;
  }
  uint16_t TNKH = c.tankHeightCm;
  uint8_t MODE = c.mode;
  uint8_t VALUE = c.value;

  if(tankHeight ==-1 || tankHeight !=TNKH){
    if (!geometryInitialized || TNKH > 0) {
      geometryInitialized = initFromGeometry(TNKH);
    }
  }
  switch (MODE) {
    case CAL_MARK_FULL:
      markFull(d);
      break;

    case CAL_MARK_EMPTY:
      markEmpty(d);
      break;

    case CAL_ESTIMATED:
      applyEstimatedPercent(d,VALUE);
      break;
  }
  getCalibrationStage();
  if (isCalibrated()) {
    systemState = STATE_READY;
    Serial.println("System calibrated and READY");
    printCalibrationState();
    return true;
    //starttransaction(&tnkcal, sizeof(CalibrationPayload), DATA_CALIBRATION);
  } else {
    Serial.println("System uncalibrated and READY");
    systemState = STATE_UNCALIBRATED;
    printCalibrationState();
    return false;
  }
  return false;
  //delay(20);
}
void getCalibrationStage() {

  bool fullHigh  = (calFull.confidence  == CONF_HIGH);
  bool emptyHigh = (calEmpty.confidence == CONF_HIGH);
  bool fullAny   = (calFull.confidence  > CONF_NONE);
  bool emptyAny  = (calEmpty.confidence > CONF_NONE);

  if (fullHigh && emptyHigh) {
    calibrationStage = CAL_STAGE_FULL;
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  }
  else if (fullHigh || emptyHigh) {
    calibrationStage = CAL_STAGE_PARTIAL;
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
  }
  else if (fullAny || emptyAny) {
    calibrationStage = CAL_STAGE_ESTIMATED;
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
  }
  else {
    calibrationStage = CAL_STAGE_NONE;
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
  }
  Serial.print("The calibration stage is: ");
  Serial.println(calibrationStage);
}


bool starttransaction(const void *payload,uint8_t payloadLen, byte type){ 
  if(!connected) return false;
  if (tx.active){ 
    //Serial.println("Transaction already active"); 
    return false; 
  } 
  resetTx(); 
  tx.active = true; 

  //tx.dest = dest;
  tx.type=type; 
  //tx.payload[0] = payload; 
  tx.retriesleft = 5; 
  tx.payloadLen = payloadLen; 
  tx.txID = msgcount++; 
  if(payloadLen >0 && payload !=nullptr){ 
    if (payloadLen > sizeof(tx.payload)) return false;
    memcpy(tx.payload, payload, payloadLen); 
  }
  if(type!=SYS_AVAILABLE){
  sendRAW(tx); 
  } 
  //Serial.println("Start transaction success"); 
  return true;
}
void sendRAW(pendingTX &t){ 
  LoRa.beginPacket(); 
  LoRa.write(t.dest); 
  LoRa.write(localaddress); 
  LoRa.write(t.txID); 
  LoRa.write(t.type); 
  LoRa.write(t.payloadLen); 
  //LoRa.write(t.payload.length()); 
  LoRa.write(t.payload, t.payloadLen); 
  LoRa.endPacket(true); 
  t.lastsendtime = millis(); //+ backoff;
  //waitingforAck = true; 
  //Serial.println("data sent..");
  delay(10);
  LoRa.receive();
}
void sendACK(byte msgID){ 
  LoRa.beginPacket(); 
  LoRa.write(destinationtdsply); 
  LoRa.write(localaddress); 
  LoRa.write(msgID); 
  LoRa.write(MSG_ACK); 
  LoRa.write(0); // no payload 
  //LoRa.write(subtype); 
  LoRa.endPacket(); 
  //delay(10); 
  Serial.println("ACK sent"); 
  LoRa.receive();
}


void clearSensorBuffer() {
  while (Ultrasonic_Sensor.available()) {
    Ultrasonic_Sensor.read();
  }
  //delay(50);
}
bool readSensorWithRetry(float &outDistance) {
  const uint8_t MAX_RETRIES = 10;

  for (uint8_t i = 0; i < MAX_RETRIES; i++) {
    clearSensorBuffer();   // 🔥 CRITICAL
    delay(30);
    //Serial.print("Sensor read attempt ");
    //Serial.println(i + 1);

    if (readsensor(outDistance)) {
      return true;
    }
    //delay(50); // small settling delay
  }

  return false;
}
bool updateCalPoint(CalPoint &p, float newDist, CalConfidence newConf){
  if (newDist < SENSOR_MIN_DISTANCE || newDist > SENSOR_MAX_DISTANCE)
    return false;

  // Authority rules
  if (p.confidence > newConf)
    return false;   // cannot downgrade authority

  p.distance = newDist;
  p.confidence = newConf;

  if (newConf == CONF_HIGH)
    p.stableHits = 255;

  return true;
}


void markFull(float d){
  if (!geometryInitialized) return;
  if (!updateCalPoint(calFull, d, CONF_HIGH))
    return;

  sensorOffset = d;

  if (geometryInitialized && calEmpty.confidence < CONF_HIGH){
    float derivedEmpty = d + tankHeight;
    updateCalPoint(calEmpty, derivedEmpty, CONF_LOW);
  }
}
void markEmpty(float d){
  if (!geometryInitialized) return;
  if (!updateCalPoint(calEmpty, d, CONF_HIGH))
    return;

  if (geometryInitialized && calFull.confidence < CONF_HIGH){
    float derivedFull = d - tankHeight;
    updateCalPoint(calFull, derivedFull, CONF_LOW);

    sensorOffset = derivedFull;
  }
}
void applyEstimatedPercent(float distance, uint32_t percent) {

  if (!geometryInitialized) return;
  if (percent <= 0 || percent >= 100) return;

  //tank percentage in decimal
  float P = percent / 100.0f;

  // water height from bottom
  float waterHeight = P * tankHeight;

  // distance from water to bottom
  float distFromBottom = tankHeight - waterHeight;

  // solve offset
  float x = distance - distFromBottom;
  Serial.print("The offset calculated is: ");
  Serial.println(x);
  if (x < SENSOR_MIN_DISTANCE || x > SENSOR_MAX_DISTANCE){
    Serial.println("i ends here");
    return;
  }

  // ONLY set if nothing high exists
  if (calFull.confidence < CONF_HIGH){
    updateCalPoint(calFull, x, CONF_LOW);
  }

  if (calEmpty.confidence < CONF_HIGH){
    updateCalPoint(calEmpty, x + tankHeight, CONF_LOW);
  }
  sensorOffset = x;
}


bool initFromGeometry(float tnkHeight) {
  if (tnkHeight == 0) return false;
  if (tnkHeight > (SENSOR_MAX_DISTANCE - SENSOR_MIN_DISTANCE)) {
    Serial.println("Geometry rejected: tank too tall for sensor");
    return false;
  }
  tankHeight = tnkHeight;

  calFull.distance  = -1;
  calEmpty.distance = -1;
  calFull.confidence  = CONF_NONE;
  calEmpty.confidence = CONF_NONE;
  sensorOffset = -1;

  //Serial.println("Geometry known,offset unknown");
  return true;
}
void printCalibrationState(){
  Serial.println("=== CAL STATE ===");
  Serial.printf("Full:  %.2f cm  Conf=%d\n", calFull.distance, calFull.confidence);
  Serial.printf("Empty: %.2f cm  Conf=%d\n", calEmpty.distance, calEmpty.confidence);
}

void handleDataPacket(byte type, uint8_t *payload,uint8_t len, byte sender, byte txID){

  // Duplicate detection (except ACKs)
  if (type != MSG_ACK) {
    if (txID == lastRxSeq) {
      Serial.println("Duplicate packet detected — re-ACK only");
      sendACK(txID);
      return;
    }
  }

  bool processed = false;

  switch(type){ 
    //Serial.println("now matching types"); 
    case MSG_ACK: 
      if (!tx.active) return; 
      if (sender != tx.dest) return; 
      if (txID != tx.txID) return; 
      Serial.println("ACK confirmed"); 
      resetTx(); break; 
    case SYS_AVAILABLE: 
      connected = true;
      digitalWrite(CON_LED,HIGH);
      processed = true;  
      break;
    case CAL_SETTINGS:
      if(!connected) return;
      if (sender !=destinationtdsply || len <4) return;
      systemState = STATE_CALIBRATING;
      processed = getCalibratioNSettings(payload,len);
      //delay(10);
      break;
    case MSG_REQ_TANKLVL:
      if(!connected) return;
      if (sender !=destinationtdsply) return;
      datarequested = true;
      processed = true;
      break; 
    default: 
      Serial.println("Unknown packet type"); 
      break; 
  }
  if(processed){
      lastRxSeq = txID;   //  ONLY update if processed
      sendACK(txID);      //  ACK AFTER success
      delay(100);
  }
  //lastRxSeq = txID;
}
void processLoRaPacket(const uint8_t* data, int len) {
  //Serial.println("it seems we got data");
  if (len < 5) return;
  int idx = 0;

  byte recipient = data[idx++];
  byte sender = data[idx++];
  byte txID = data[idx++];
  byte type = data[idx++];
  byte payloadLen = data[idx++];

  if (recipient != localaddress) return;
  if (payloadLen > 16) return;

  uint8_t payload[16];
  if (idx + payloadLen > len) return;
  memcpy(payload, &data[idx], payloadLen);

  handleDataPacket(type, payload, payloadLen, sender, txID);
}
void onReceive(int packetSize) {
  if (packetSize == 0) return;
  if (packetSize > sizeof(loraRxBuffer)) return;

  loraPacketSize = packetSize;

  for (int i = 0; i < packetSize && LoRa.available(); i++) {
    loraRxBuffer[i] = LoRa.read();
  }

  loraPacketReady = true;   // signal loop()
}


