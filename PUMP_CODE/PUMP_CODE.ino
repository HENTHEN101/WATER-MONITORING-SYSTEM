#include <SPI.h>
#include <LoRa.h>

//DEFINED VARIBALES
//lora pins
#define RST 25
#define DIO0 26 //12//27//13
#define SS 5
//types of data packets 
#define MSG_ACK          0x02 
#define MSG_CON          0x03 
#define DATA_TANK_LEVEL  0x04 
#define DATA_CALIBRATION 0x05 
#define DATA_PUMP_STATUS 0x06 
#define SYS_AVAILABLE 0x07
#define CAL_SETTINGS     0x08
#define CAL_MARK_EMPTY   0x09
#define CAL_MARK_FULL    0x10
#define CAL_ESTIMATED    0x11
#define MSG_REQ_TANKLVL  0x12
#define MSG_REQ_PUMPSTAT 0x13
//pump switch and LEDs
#define PUMP_SWITCH 13//13//33//25//27
#define CON_LED 17//2//0//4
#define E_STOP 32
#define ON_OFF 33
#define MODE 16//change pin


//PUMP FLAG HOLDERS
uint8_t g_pumpState   = 0;
volatile bool estopPressed = false;
volatile bool onoffPressed = false;

//SYSTEM ADDRESSES
byte localaddress = 0xAA;  // address of this device 
byte destinationtdsply = 0xFF; // destination to send to 

//DATA FLAGS
volatile bool loraPacketReady = false;
bool connected = false;
bool datarequested = false;
bool pump_triggered = false;


//TIMER VALUES
unsigned long lastRetryTime =0;
unsigned long lastTimeupdate =0;
const unsigned long timeinterval =1000;
unsigned long acktimer =0; 
const unsigned long ACK_TIMEOUT = 5000; 
unsigned long lastPumpCommandTime = 0;
const unsigned long MAX_PUMP_RUNTIME = 300000; // 5 min
unsigned long pumpStartTime = 0;
const unsigned long MIN_PUMP_SWITCH_INTERVAL = 3000; // 3 seconds
volatile unsigned long lastEStopTime = 0;
volatile unsigned long lastOnOffTime = 0;
const unsigned long debounceDelay = 50;


//LORA DATA HOLDERS
uint8_t lastRxSeq = 255;   // invalid initial value
volatile int loraPacketSize = 0;
uint8_t loraRxBuffer[21];   // safe size
byte msgcount = 0;  // count of outgoing messages 



enum PumpState {
  NORMAL,
  FAULT   // E-stop active
};
PumpState currentState = NORMAL;
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

//pumpstatus packet 
struct PumpStatusPayload {
  uint8_t state;
  uint8_t mode;
};
PumpStatusPayload pmpstat;

void IRAM_ATTR handleStopButton(){
  unsigned long now = millis();
  if(now - lastEStopTime > debounceDelay){
    estopPressed = true;
    lastEStopTime = now;
  }
}
void IRAM_ATTR handleONOFFButton(){
  unsigned long now = millis();
  if(now - lastOnOffTime > debounceDelay){
    onoffPressed = true;
    lastOnOffTime = now;
  }
}

void setup() {
  //initialize Serial Monitor 
  Serial.begin(9600);
  pinMode(PUMP_SWITCH,OUTPUT);
  pinMode(CON_LED,OUTPUT);
  pinMode(MODE,OUTPUT);
  pinMode(E_STOP,INPUT_PULLUP);
  pinMode(ON_OFF,INPUT_PULLUP);
  digitalWrite(PUMP_SWITCH, LOW);  // force pump OFF at boot
  digitalWrite(MODE,LOW);
  digitalWrite(CON_LED,LOW);
  attachInterrupt(digitalPinToInterrupt(E_STOP), handleStopButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(ON_OFF), handleONOFFButton, FALLING);
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
}

void loop() {
  change_mode();
  if (loraPacketReady) {
    loraPacketReady = false;
    //Serial.println(loraRxBuffer[1]);
    processLoRaPacket(loraRxBuffer, loraPacketSize);
    LoRa.receive();   // return to RX mode
  }

  checkPumpFailsafe();

  if (connected && datarequested && !tx.active) { 
    pmpstat.state = digitalRead(PUMP_SWITCH);
    pmpstat.mode = currentState;
    // Serial.print("the mode of the pump is: ");
    // Serial.println(pmpstat.mode);
    sendPumpstatus(); 
    datarequested = false;
  } 

  processtx(); 
}


void change_mode(){
  if(estopPressed){
    Serial.println("Pump is in manual override");
    estopPressed = false;
    if(currentState != FAULT){
      currentState = FAULT;
      digitalWrite(MODE,HIGH);
      digitalWrite(PUMP_SWITCH, LOW);
      datarequested = true;
      g_pumpState =0;
    }else{
      currentState = NORMAL;
      digitalWrite(MODE,LOW);
      datarequested = true;
    }
  }else if(onoffPressed){
    Serial.println("ON_OFF button pressed");
    onoffPressed = false;
    if(currentState != FAULT){
      datarequested = true;
      g_pumpState = !g_pumpState;
      // Serial.print("Pump physiocally was turned: ");
      // Serial.println(g_pumpState? "ON":"OFF");
      digitalWrite(PUMP_SWITCH,g_pumpState);
      handlePumpStatus(g_pumpState);
    }
  }
}
void processtx(){ 
  if (!tx.active) return; 
  unsigned long now = millis(); 
  if (now - tx.lastsendtime < ACK_TIMEOUT) return; 
  if(tx.retriesleft ==0){
    Serial.println("Transaction failed");
    connected = false;
    digitalWrite(CON_LED,LOW); 
    resetTx(); 
    return; 
  } 
  Serial.println("Retrying transaction.."); 
  tx.retriesleft --; 
  sendRAW(tx); 
}

void resetTx() { 
  tx = pendingTX{}; // C++ value-initialize (clears everything) 
  //tnkset = CalibrationSettings{};
  //tnkcal=CalibrationPayload {};
}

//pump packet handlers
uint8_t packPumpStatus(uint8_t* buf, const PumpStatusPayload& p){
    buf[0] = p.state;
    buf[1] = p.mode;
    return 2;
}
void unpackPumpStatus(const uint8_t* buf, PumpStatusPayload& p){
    p.state = buf[0];
}
bool GetPumpmstatus(const uint8_t *payload, uint8_t len){
  //PumpStatusPayload p;
  if (len < 1) return false;
  unpackPumpStatus(payload, pmpstat);
  return handlePumpStatus(pmpstat.state);
}
void sendPumpstatus(){
  uint8_t payload[2];

 uint8_t len = packPumpStatus(payload,pmpstat);

  starttransaction(payload, len, DATA_PUMP_STATUS);
}
bool handlePumpStatus(uint8_t state){
  //int state = payload.toInt();
  state = (state != 0) ? 1 : 0;
  unsigned long now = millis();

  // Prevent rapid toggling
  if(now - pumpStartTime < MIN_PUMP_SWITCH_INTERVAL){
    Serial.println("Pump toggle blocked (too soon)");
    return false;
  }
  //lastPumpCommandTime = now;   // record last valid command

  if(state != g_pumpState){
    // Serial.print("Pump state changed to: ");
    // Serial.println(state);
    g_pumpState = state;
    pump_triggered = (state ==1);
    digitalWrite(PUMP_SWITCH,state);
    if(state == 1){
      pumpStartTime = now;
      Serial.println("Pump turned ON");
    } else {
      Serial.println("Pump turned OFF");
    }
    return true;
  }
  return false;
}
void checkPumpFailsafe(){
  if(g_pumpState == 1){   // only care if pump is ON
    if(millis() - pumpStartTime > MAX_PUMP_RUNTIME){
      Serial.println("FAILSAFE: No command received FOR 10MIN. Turning pump OFF.");
      digitalWrite(PUMP_SWITCH, LOW);
      g_pumpState = 0;
      pump_triggered = false;
    }
  }
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
  Serial.println("transaction successful"); 
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

void handleDataPacket(byte type, uint8_t *payload,uint8_t len, byte sender, byte txID){

  // Duplicate detection (except ACKs)
  if (type != MSG_ACK) {
    if (txID == lastRxSeq) {
      Serial.println("Duplicate packet detected — re-ACK only");
      sendACK(txID);
      return;
    }
  }

  if(type==MSG_ACK){
    if (!tx.active) return; 
    if (sender != tx.dest) return; 
    if (txID != tx.txID) return; 
    Serial.println("ACK confirmed"); 
    resetTx(); 
    return;
  }

  bool processed = false;
  if(currentState == FAULT)return;
  switch(type){ 
    //Serial.println("now matching types"); 
    case SYS_AVAILABLE: 
      connected = true;
      Serial.println("tank sys mssg recieved");
      digitalWrite(CON_LED,HIGH);
      processed = true; 
      break;
    case MSG_REQ_PUMPSTAT:
      if(!connected) return;
      if (sender !=destinationtdsply|| len < 1) return;
      processed = GetPumpmstatus(payload,len);
      datarequested = processed;
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
