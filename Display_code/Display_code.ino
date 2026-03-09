#include <WiFi.h>
#include "time.h"
#include <SPI.h>
#include <LoRa.h>
#include <Nextion.h>
#include <Preferences.h>
#include <stdarg.h>

Preferences prefs;

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
#define SYS_AVAILABLE    0x07
#define CAL_SETTINGS     0x08
#define CAL_MARK_EMPTY   0x09
#define CAL_MARK_FULL    0x10
#define CAL_ESTIMATED    0x11
#define MSG_REQ_TANKLVL  0x12
#define MSG_REQ_PUMPSTAT 0x13
//lora data packet container
#define RX_QUEUE_SIZE 6
#define MAX_PACKET_SIZE 21
#define RX_NEXT(i) ((uint8_t)((i + 1) % RX_QUEUE_SIZE))
//Popup queue implementation
#define POPUP_QUEUE_SIZE 8
#define POPUP_TEXT_LEN 200

// #define BATTERY_PIN 34
// const float R1 = 100000.0;
// const float R2 = 33000.0;
// const float ADC_REF = 3.3;
// float alpha = 0.1;   // 0.05 = very smooth, 0.2 = faster response

// Backup values (for rollback if remote calibration fails)
float backup_TANKHEIGHT = 0;
//CalibrationSettings backup_tnkset = {0};
bool backup_full_pressed = false;
bool backup_empty_pressed = false;
bool backup_tnkcalibrated = false;
//bool calPending = false;        // true while waiting for ACK for CAL_SETTINGS


//DATA FLAGS
volatile bool loraPacketReady = false;
bool tankConnected = false;
bool pumpConnected = false;
bool pump_triggered = false;
//bool g_popupActive = false;
//bool g_popupPendingUpdate = false;
bool waitingTankUpdate = false;
bool waitingPumpUpdate = false;
bool tnkcalibrated = false;
bool Pmp_manual_override = false;
bool emptying_fulling = false;
bool full_pressed = false;
bool empty_pressed = false;
bool timedateset = false;
bool loraactive = false;
bool wifiactive = false;
bool blynkactive = false;

//TIMER VALUES
unsigned long lastRetryTime =0;
unsigned long lastTimeupdate =0;
//const unsigned long timeinterval =60000;
const unsigned long HEARTBEAT_INTERVAL = 20000;
unsigned long lastheartbeat =0;
unsigned long acktimer =0; 
unsigned long nextClockUpdate = 0;
const unsigned long ACK_TIMEOUT = 5000; 
const unsigned long DATA_TIMEOUT = 30000;
unsigned long getupdate = 0;
struct tm currentTime;
//WIFI CREDENTIALS
char ssid[32]="MonaConnect";
char password[63]="";

//DATE AND TIME SETTERS
const char* ntpServer = "jm.pool.ntp.org";
const long  gmtOffset_sec = -18000;
const int   daylightOffset_sec = 0;//GMT+5:30

//SYSTEM ADDESSES
byte localaddress = 0xFF; 
byte destinationtnk = 0xBB;
byte destinationpmp = 0xAA; 

//DISPLAY DATA HOLDERS
float TANKHEIGHT = 0;
int startperc =0, stopperc=0;
uint8_t g_tankPercent = 0;
uint8_t g_pumpMode    = 0;   // 0 = MANUAL, 1 = AUTO
uint8_t g_pumpState   = 0;   // 0 = OFF, 1 = ON
uint8_t prev_g_pumpMode    = 0;   // 0 = MANUAL, 1 = AUTO
uint8_t prev_g_pumpState   = 0;   // 0 = OFF, 1 = ON
uint8_t g_blynkstste = 0; //0=OFF, 1=ON
uint8_t prev_g_blynkstste =0; // 0 = OFF, 1 = ON
uint8_t currpg = 0;//current page
uint8_t prevcurrpg = 0;//current page
char g_popupHead[20] = {0};
char g_popupContent[200] = {0};

//LORA DATA MANAGERS
uint8_t lastRxSeq = 255;   // invalid initial value
uint8_t lastRxSeqTank = 255;
uint8_t lastRxSeqPump = 255;
volatile int loraPacketSize = 0;
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
uint8_t loraRxBuffer[21];   // safe size
byte msgcount = 0; 
 

//NEXTION DISPLAY OBJECTS
//page0 home
NexPage page0 = NexPage(0, 0, "page0");//**
NexPicture cloudicon =  NexPicture(0,7,"pcloud");
NexPicture radioicon =  NexPicture(0,8,"pradio");
NexPicture wifiicon =  NexPicture(0,9,"pwifi");
NexButton settingbtn = NexButton(0,2,"set");//**
NexButton tankbtn = NexButton(0,3,"tank");//**
NexButton pumpbtn = NexButton(0,4,"pump");//**
NexText Time = NexText(0,5,"time");//**
NexText Date = NexText(0,6,"date");//**

//NexVariable currentpage = NexVariable(0,7,"currpage");

//page1 tank
NexPage page1 = NexPage(1,0,"page1");//**
NexButton taddbtn = NexButton(1,1,"tadd");//**
NexButton tbackbtn = NexButton(1,2,"tback");//**
NexButton tcalbtn = NexButton(1,12,"tcal");//**
NexText tlinktxt = NexText(1,5,"tlink");//shows if the tank sys is connected**
NexText tcaltxt = NexText(1,7,"tbrat");//depends on user input + ALU
NexText theighttxt = NexText(1,9,"theight");//time comes from tank system
NexText tnametxt = NexText(1,3,"tname");//user iput
NexNumber tperctxt = NexNumber(1,14,"tperc");//comes from tank sys**
NexSlider tlvl = NexSlider(1,11,"tlevel");//comes from tank sys**

//page2 pump
NexPage page2 = NexPage(2,0,"page2");//**
NexButton pbackbtn = NexButton(2,1,"pback");//**
NexButton paddbtn = NexButton(2,10,"padd");//**
NexButton pstrtstp = NexButton(2,3,"pstrtstp");//used to set start time**
NexButton emergstop = NexButton(2,13,"Estop");//turns off pump & reset mode and status**
NexDSButton pmodebtn = NexDSButton(2,14,"pmode");//use selects mode**
NexDSButton pstatbtn = NexDSButton(2,15,"pstat");//is pump on or off**
NexText plinktxt = NexText(2,12,"plink");//shows if the pump sys is connected**
NexText pstarttxt = NexText(2,8,"pstart");//shows pump start time**
NexText pstptxt = NexText(2,9,"pstop");//shows pump stop time**
NexText pnametxt = NexText(2,2,"pname");//user input

//page3 setting
NexPage page3 = NexPage(3,0,"page3");
NexButton sbackbtn = NexButton(3,9,"sback");
NexDSButton blynkbtn = NexDSButton(3,3,"blynk");
NexButton chngwifi = NexButton(3,8,"chngwifi");
//NexButton tdchng = NexButton(3,5,"tdchange");
NexText wifiname = NexText(3,10,"wifiname");

//page4 tank calibration 
NexPage page4 = NexPage(4,0,"page4");//**
NexText theight = NexText(4,12,"tnkheight");
NexDSButton tfullbtn = NexDSButton(4,7,"full");//user input to calibrate tank
NexDSButton temptybtn = NexDSButton(4,8,"empty");//user input to calibrate tank
//NexButton restcal = NexButton(3,13,"restcal");
NexButton tconfirm = NexButton(4,5,"tcon");//confirms and set tank size**
NexButton tcancel = NexButton(4,4,"tcan");//leave calibration seeting no variable update**
NexNumber tapprox = NexNumber(4,9,"approx");//user input via sub and plus button**
NexVariable fulemp = NexVariable(4,13,"fulemp");//**

//page5 pump start and stop time
NexPage page5 = NexPage(5,0,"page5");//**
//NexButton hradd = NexButton(4,5,"hadd");//used to add hour
//NexButton minadd = NexButton(4,6,"madd");//used to add min
//NexButton hrsub = NexButton(4,10,"hsub");//used to sub hour
//NexButton minsub = NexButton(4,11,"msub");//used to sub min
NexButton pconfirm = NexButton(5,11,"pcon");//confirm and set time**
NexButton pcancel = NexButton(5,12,"pcan");//leave time settings, no variable update**
NexNumber hrnum = NexNumber(5,8,"hnum");// used to show hour**
NexNumber minnum = NexNumber(5,7,"mnum");//used to show min**

//page6 popup page
NexPage page6 = NexPage(6,0,"page6");//
NexText pophead = NexText(6,1,"head");
NexText popcontent = NexText(6,2,"content");
NexHotspot exitpop = NexHotspot(6,3,"exitpop");
NexText msgcounter = NexText(6,4,"msgcount");
//NexVariable popstyle = NexVariable(5,5,"style");

//page7 popup page
NexPage page7 = NexPage(7,0,"page7");//

//page8 wifi calibration
NexPage page8 = NexPage(8,0,"page8");//
NexText wifiID = NexText(8,8,"SSID");
NexText wifipass = NexText(8,7,"PASS");
NexButton wconfirm = NexButton(8,3,"wcon");
NexButton wcancel = NexButton(8,4,"wcan");


//SYSTEM STATE HOLDERS
//popupmsg prioprity levels
enum PopupPriority {
  POP_LOW = 0,
  POP_NORMAL = 1,
  POP_HIGH = 2
};
PopupPriority g_popupCurrentPriority = POP_LOW;

//connection and calibration states
typedef enum {
  NONE,
  DISCON_EST,
  CON_PART,
  CON_FULL,
}Con_calStates;
//Con_calStates connection = DISCON_EST;
Con_calStates g_tankConnection =  NONE;
Con_calStates g_pumpConnection = NONE;
Con_calStates calibration = NONE;

//local time retrival state
typedef enum{
  TIME_NORMAL,
  TIME_RECONNECTING
}TimeSyncState;
//error messages formm tank
enum Errortype{
  NO_ERROR,
  UNKNOWN_ERROR,
  FAULTY_SENSOR,
  INVALID_TNKHEIGHT,
  INVALID_OFFSET
};
Errortype ERROR = NO_ERROR;
enum TextRule {
  RULE_SSID,
  RULE_PASSWORD,
  RULE_NUMBER
};
static TimeSyncState timeState = TIME_NORMAL;
//connection status 
const char* connectiontext[]={
  "--",
  "Disconnected",
  "Connecting",
  "Connected"
};
//tank calibration status
const char* calibrationtext[] ={
  "--",
  "Estimated",
  "Partial",
  "Fully"
};
// tank Error strings
const char* TankErrorStrings[] = {
    "No Error",
    "System malcfunction due to unknown fault",
    "Tank sensor not operating properly",
    "Tank size recieved by tank invalid",
    "Ensure system is installed over 3cm above highest water level",
    "Tank is uncalibrated"
};
//pump error strings
const char* PumpErrorStrings[]={
  "No Error",
  "System malcfunction due to unknown fault",
  "Toggling pump too fast please wait 3 seconds.",
  "Tank is in manual override"
};
//LORA DATA PACKET structures
struct RxPacket {
  uint8_t data[MAX_PACKET_SIZE];
  uint8_t len;
};
RxPacket rxQueue[RX_QUEUE_SIZE];
//transaction packet 
struct pendingTX{ 
  bool active; 
  byte dest; 
  byte type; 
  byte txID; 
  uint8_t retriesleft; 
  unsigned long lastsendtime; 
  uint8_t payload[16]; 
  uint8_t payloadLen; 
};
pendingTX tx{};

//tanklvel packet 
struct TankLevelPayload {
  uint8_t percent;
  uint8_t cal_stage; 
};
TankLevelPayload tnklvl{};

//pumpstatus packet 
struct PumpStatusPayload {
  uint8_t state;
  uint8_t mode;
};
PumpStatusPayload pmpstat{};

//tank calibrtion level packet 
struct ErrorPayload { 
  uint8_t ERR; 
};
ErrorPayload errotype{};

//tank calibration packet
struct CalibrationSettings { 
  uint8_t mode;   // what the user did
  uint8_t value;  // % if estimated, unused otherwise 
  uint16_t tankHeightCm;  // usable water height
};
CalibrationSettings tnkset{};

struct PopupMessage {
  uint32_t severity;           // 0..2 (for setPopupHeadFromValue)
  char content[POPUP_TEXT_LEN];
};
struct PopupManager {
  PopupMessage queue[POPUP_QUEUE_SIZE];
  uint8_t head = 0;
  uint8_t tail = 0;
  uint8_t count = 0;
  bool active = false;
  uint8_t currentIndex = 0;
};
PopupManager popup;

//DISPLAY UI INTERATION LISTENER
NexTouch *nex_listen_list[]={
  //page0
  &tankbtn,
  &pumpbtn,
  &settingbtn,
  //page1
  &tbackbtn,
  &tcalbtn,
  &taddbtn,
  //page2
  &pbackbtn,
  &paddbtn,
  &pstrtstp,
  &emergstop,
  &pmodebtn,
  &pstatbtn,
  //page3
  &sbackbtn,
  &blynkbtn,
  &chngwifi,
  //&tdchng,
  //page4
  &tcancel,
  &tconfirm,
  //page5
  &pcancel,
  &pconfirm,
  //page6
  &exitpop,
  //page8
  &wconfirm,
  &wcancel,
  NULL
};

void saveSettings() {

  prefs.begin("system", false);   // namespace "system", RW mode

  prefs.putFloat("tankH", TANKHEIGHT);
  prefs.putInt("startP", startperc);
  prefs.putInt("stopP", stopperc); 
  prefs.putBool("calib", tnkcalibrated);

  prefs.end();

  Serial.println("Settings saved to flash");
}
void loadSettings() {

  prefs.begin("system", true);   // read-only mode

  TANKHEIGHT     = prefs.getFloat("tankH", 0.0);
  startperc      = prefs.getInt("startP", 0);
  stopperc       = prefs.getInt("stopP", 0);
  tnkcalibrated  = prefs.getBool("calib", false);


  prefs.end();

  Serial.println("Settings loaded from flash");
  Serial.printf("The tank height is:  %.2f",TANKHEIGHT);
  Serial.printf("the start and stop times are:%d and %d",startperc,stopperc);
}
void connectblynk(){ 
}
bool checkwifi(){
  Serial.print("Attemptig to Connect to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  wl_status_t status;
  while ((status = WiFi.status()) != WL_CONNECTED) {
    if (status == WL_NO_SSID_AVAIL) {
        //Serial.println("SSID not found");
        popupWithFmt(1,"%s not found",ssid);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        return false;
    } else if (status == WL_CONNECT_FAILED) {
        //Serial.println("Connection failed, wrong password?");
        popupWithFmt(1,"Connection failed, wrong password?");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        return false;
    }
    if (millis() - start > 20000) {
        //Serial.println("WiFi connection timeout, check credentials or signal");
        popupWithFmt(1,"WiFi connection timeout, check credentials or signal");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        return false;
    }
    delay(100);
    Serial.println(".");
  }
  //Serial.println("\nWiFi exists and connected");
  if(!timedateset){
    loadClock();
    return true;
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("disconnecting form wifi");
  return true;
}

//SETUP FUNCTION
void setup(){
  nexInit();
  Serial.begin(9600);
  showpopuppg(0);
  //SPI.begin(SCK,MISO,MOSI); 
  LoRa.setPins(SS,RST,DIO0);
  if (!LoRa.begin(525E6)){ 
    Serial.println("Lora init failed. check connections"); 
    while(true){ 
      Serial.println("LoRa init failed - retry hardware/ wiring"); 
      delay(100);
      } 
  }
  Serial.println("LoRa init succeeded.");
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.onReceive(onReceive);
  LoRa.receive();
  loraactive = true;
  //syncTimeFromWiFi();
  if(!timedateset){
    loadClock();
  }
  Attachpush();
  updatehomepg();
  Serial.print("setup done");
  delay(100);
}
//LOOP FUNCTION
void loop(){
  nexLoop(nex_listen_list); 
  if (rxTail != rxHead){//(rxTail != rxHead) {
    RxPacket &pkt = rxQueue[rxTail];
    //loraPacketReady = false;
    processLoRaPacket(pkt.data, pkt.len);
    rxTail = RX_NEXT(rxTail);
       // retu  rn to RX mode
  }
  unsigned long now = millis();
  if(tankConnected && !tx.active && !waitingTankUpdate && tnkcalibrated){
    if(now-lastheartbeat >= HEARTBEAT_INTERVAL){
      starttransaction(nullptr,0,destinationtnk,MSG_REQ_TANKLVL);
      lastheartbeat = now;
    }
  }
  //LoRa.receive();
  processtx(); 
  Autopumpcheck();
  if(timedateset){
    updateClock();
  } 
}

//LORA DATA RETRIVAL AND PROCESSING
void onReceive(int packetSize) {
  if (packetSize == 0) return;
  if (packetSize > MAX_PACKET_SIZE) return;

  //if (packetSize > sizeof(loraRxBuffer)) return;
  uint8_t nextHead = RX_NEXT(rxHead);
  if (nextHead == rxTail) {
    Serial.println("RX queue full — packet dropped");
    return;  // queue overflow protection
  }

  //loraPacketSize = packetSize;
  rxQueue[rxHead].len = packetSize;
  for (int i = 0; i < packetSize; i++) {
    rxQueue[rxHead].data[i] = LoRa.read();
  }

  // for (int i = 0; i < packetSize && LoRa.available(); i++) {
  //   loraRxBuffer[i] = LoRa.read();
  // }
  rxHead = nextHead;
  //loraPacketReady = true;   // signal loop()
}
void processLoRaPacket(const uint8_t* data, int len) {

  if (len < 5) return;
  int idx = 0;

  byte recipient = data[idx++];
  byte sender = data[idx++];
  byte txID = data[idx++];
  byte type = data[idx++];
  byte payloadLen = data[idx++];

  if (recipient != localaddress) return;
  if (payloadLen > 16) return;
  if (idx + payloadLen > len) return; // defensive check

  uint8_t payload[16];
  memcpy(payload, &data[idx], payloadLen);

  handleDataPacket(type, payload, payloadLen, sender, txID);
}
void handleDataPacket(byte type, const uint8_t *payload,uint8_t len, byte sender, byte txID){ 

  if(!isConnectedToSender(sender)){
    //Serial.println("system not connected");
    if(type==MSG_ACK && txID == tx.txID && tx.active && tx.type ==SYS_AVAILABLE && sender==tx.dest){
      //Serial.println("Attempting connection");
      uint8_t ERTPE=Geterror(payload, len);
      en_connection(sender,ERTPE);
      resetTx();
    }
    return;
  }
  bool processed = false;

  if (type != MSG_ACK ) {
    if (isDuplicate(sender, txID)){//if (txID == lastRxSeq) {
      Serial.println("Duplicate packet detected — re-ACK only");
      sendACK(sender,txID);
      return;
    }
    switch(type){ 
      case DATA_TANK_LEVEL: 
        if(sender !=destinationtnk || len < 2 ||!waitingTankUpdate) return; 
        //sendACK(destinationtnk, txID); 
        processed = GetTanklevel(payload, len);
        waitingTankUpdate = !processed;
        break; 
      case DATA_PUMP_STATUS: 
        Serial.println("pump packet recived");
        if(sender !=destinationpmp || len < 2 ) return;  
        //sendACK(destinationpmp, txID);
        processed = GetPumpmstatus(payload, len);
        waitingPumpUpdate = !processed; 
        break; 
      default: 
        Serial.println("Unknown packet type"); 
        break; 
    }
  }

  if(type==MSG_ACK){
    uint8_t ERTPE=Geterror(payload, len);
    if(sender==destinationtnk && waitingTankUpdate && tankConnected){
      waitingTankUpdate = false;
      popupWithFmt(2,"Tank update Error: %s",getTankErrorString(ERTPE));
      return;
    }else if(sender==destinationpmp && waitingPumpUpdate && pumpConnected){
      waitingPumpUpdate = false;
      popupWithFmt(2,"Pump updtae Error: %s",getPumpErrorString(ERTPE));
      return;
    }
    if (!tx.active) return; 
    if (sender != tx.dest) return; 
    if (txID != tx.txID) return;
    if(len<1)return;
    switch(tx.type){
      case CAL_SETTINGS:
        if(ERTPE==0){
           tnkcalibrated = true;
           TANKHEIGHT=backup_TANKHEIGHT;
           popupWithFmt(0,"Tank calibration successful");
        }
        else{
          rollbackCalibration();
          popupWithFmt(2,"Calibration failed: %s",getTankErrorString(ERTPE));
        }
        break;
      case MSG_REQ_TANKLVL:
        if(ERTPE==0){
          getupdate = millis();
          waitingTankUpdate = true;
        }
        else if(ERTPE!=0){
          popupWithFmt(2,"Tank update request failed: %s",getTankErrorString(ERTPE));
          break;
        }
        break;
      case MSG_REQ_PUMPSTAT:
        if(ERTPE==0) {
          waitingPumpUpdate = true;
          getupdate = millis();
        }
        else if(ERTPE!=0){
          popupWithFmt(1,"Pump update request failed: %s", getPumpErrorString(ERTPE));
        }
        break;
      default: 
        // other tx types — fall back to generic handling
        if (ERTPE == 0) {
          popupWithFmt(0,"ACK OK for type %u", tx.type);
        } else {
          popupWithFmt(2,"%s", getPumpErrorString(ERTPE));
        }
        break;
    }
    //ACKpopupmsg(0,tx.type,POP_LOW);
    Serial.println("ACK confirmed"); 
    resetTx(); 
    return; 
  }
  if(processed){
      savetxID(sender,txID);   //  ONLY update if processed
      sendACK(sender,txID);      //  ACK AFTER success
      resetTx();
  } 
}
void savetxID(byte sender,byte txID){
  if(sender == destinationtnk){
    lastRxSeqTank = txID;
    return;
  }
  if(sender == destinationpmp){
    lastRxSeqPump = txID;
    return;
  }
}
bool isDuplicate(byte sender, byte txID) {
  //uint8_t *lastSeq = nullptr;

  if(sender == destinationtnk){
    if(txID == lastRxSeqTank) return true;
    //lastRxSeqTank = txID;
    return false;
  }
  if(sender == destinationpmp){
    if(txID == lastRxSeqPump) return true;
    //lastRxSeqPump = txID;
    return false;
  }
  return false; // other nodes
}

//RESETTERS 
void resetTx() { 
  tx = pendingTX{}; 
}
void processtx(){
  unsigned long now = millis();
  if(waitingPumpUpdate || waitingTankUpdate){
    if (now - getupdate < DATA_TIMEOUT) return;
    if(waitingTankUpdate){
      tankConnected = false;
      waitingTankUpdate = false;
      g_tankConnection = CON_PART;
      popupWithFmt(1,"No Update recieved form tank,checking connection");
      resetTx();
      starttransaction(nullptr,0,destinationtnk,SYS_AVAILABLE);
      return;
    }
    if(waitingPumpUpdate){
      pumpConnected = false;
      waitingPumpUpdate = false;
      g_pumpConnection = CON_PART;
      //g_pumpMode = prev_g_pumpMode;
      g_pumpState = prev_g_pumpState;
      popupWithFmt(1,"Pump not responding,checking connection");
      resetTx();
      starttransaction(nullptr,0,destinationpmp,SYS_AVAILABLE);
      return;
    }

  }
  if (!tx.active) return;  
  if (now - tx.lastsendtime < ACK_TIMEOUT) return; 
  if(tx.retriesleft ==0){ 
    Serial.println("Transaction failed"); 
    if(tx.dest == destinationtnk){ 
      tankConnected = false;
      waitingTankUpdate = false;
    }
    if (tx.dest == destinationpmp){ 
      pumpConnected = false; 
      waitingPumpUpdate = false; 
      //g_pumpMode = prev_g_pumpMode;
      g_pumpState = prev_g_pumpState;
    } 
    ACKpopupmsg(2,tx.type);
    return; 
  } 
  Serial.println("Retrying transaction.."); 
  tx.retriesleft --; 
  sendRAW(tx); 
}

//Calibration data ahndlers
void backupCalibration() {
  backup_TANKHEIGHT = TANKHEIGHT;
  //backup_tnkset = tnkset;
  backup_full_pressed = full_pressed;
  backup_empty_pressed = empty_pressed;
  backup_tnkcalibrated = tnkcalibrated;
  // mark that a calibration is pending so we can rollback on failure
  //calPending = true;
}
void rollbackCalibration() {
  // restore backups
  //TANKHEIGHT = backup_TANKHEIGHT;
  //tnkset = backup_tnkset;
  full_pressed = backup_full_pressed;
  empty_pressed = backup_empty_pressed;
  tnkcalibrated = backup_tnkcalibrated;
  //calPending = false;

  // update UI and notify user
  //updatecalipg();
}

//NEXTION BUTTON INTERATION
void pagebtn(void *ptr){
  switch(currpg){
    case 0:
      if(ptr == &tankbtn) Serial.println("tank button pressed"),showupdatepage(1);
      else if(ptr == &pumpbtn) Serial.println("pump button pressed"),showupdatepage(2);
      else if(ptr==&settingbtn) Serial.println("setting button pressed"),showupdatepage(3);
      break;
    case 1:
      if(ptr == &tcalbtn) Serial.println("tank cal button pressed"),showupdatepage(4);
      else if(ptr == &tbackbtn) Serial.println("tank back button pressed"),showupdatepage(0);
      else if(ptr==&taddbtn) connect_systemsbtn(&taddbtn);
      break;
    case 2:
      if(ptr ==&pbackbtn) Serial.println("pump back button pressed"),showupdatepage(0);
      else if(ptr==&pstrtstp) Serial.println("pump set buttons pressed"),showupdatepage(5);
      else if(ptr==&emergstop || ptr==&pmodebtn || ptr==&pstatbtn) pumpcontrolbtn(ptr);
      else if(ptr==&paddbtn)connect_systemsbtn(&paddbtn);
      break;
    case 3:
      if(ptr ==&sbackbtn) Serial.println("setting back button pressed"),showupdatepage(0);
      else if(ptr==&blynkbtn)Serial.println("blynk button pressed"),showupdatepage(45);
      else if(ptr==&chngwifi) Serial.println("wifichng button pressed"),showupdatepage(8);
      //else if(ptr==&tdchng)Serial.println("timedate button pressed"),showupdatepage(45);
      break;
    case 4:
      if(ptr ==&tcancel) Serial.println("tankcal cancel button pressed"),showupdatepage(1);
      else if(ptr == &tconfirm) Serial.println("tankcal confirm button pressed"),savedatabtn(&tconfirm);
      //else if(ptr == &tfullbtn || ptr == &temptybtn), savedatabtn(ptr);
      break;
    case 5:
      if(ptr ==&pcancel) Serial.println("pumpset cancel button pressed"),showupdatepage(2);
      else if(ptr == &pconfirm) Serial.println("pumpset confirm button pressed"),savedatabtn(&pconfirm);
      break;
    case 8:
      if(ptr ==&wconfirm) Serial.println("setting cancel button pressed"),savedatabtn(&wconfirm);
      else if(ptr == &wcancel) Serial.println("seting confirm button pressed"),showupdatepage(3);
      break;
    default:
      Serial.print("invalid page");
      break;  
  }
}
void showupdatepage(uint8_t page){
  currpg = page;
  switch(page){
    case 0:
      //printLocalTime();
      page0.show();
      updatehomepg();
      break;
    case 1:
      page1.show();
      updatetankpg();
      break;
    case 2:
      page2.show();
      updatepumppg();
      break;
    case 3:
      page3.show();
      updatesettingpg();
      break;
    case 4:
      page4.show();
      updatecalipg();
      break;
    case 5:
      page5.show();
      break;
    case 8:
      page8.show();
      break;
    default:
      currpg=0;
      Serial.println("current page not one of the big 3");
      page0.show();
      break;
  }
  delay(50);
}
void savedatabtn(void *ptr){

  if(ptr==&tconfirm){
    uint32_t Number1,Number2,Number3;
    if(!tankConnected){
    popupWithFmt(1,"Tank Disconnected,plz connect to send data");
    return;
    }
    if(tx.active){
      popupWithFmt(2,"Transaction in progress plz wait a few seconds and try again");
      return;
    }

    float newHeightMeters = 0;
    bool gotNewHeight = getIntFromText(theight, newHeightMeters);
    // ---- FIRST CALIBRATION CHECK ----
    // If stored height is invalid AND user didn't enter a valid one
    if (TANKHEIGHT <= 0.0) {
      if (!gotNewHeight || newHeightMeters <= 0.0) {
        popupWithFmt(2,"Tank height required for first calibration");
        return;
      }
      // valid first height → store temporarily
      backup_TANKHEIGHT = newHeightMeters;
    }else {
      // Tank height already exists
      if (gotNewHeight) {
        if (newHeightMeters <= 0.0) {
          popupWithFmt(2,"Invalid tank height");
          return;
        }
        // user is changing tank size
        backup_TANKHEIGHT = newHeightMeters;
      }
      else {
        // user not changing tank → use existing
        backup_TANKHEIGHT = newHeightMeters;
      }
    }
    float valCm =newHeightMeters*100;
    if(valCm > 450){
      popupWithFmt(1,"Tank height too large");
      return;
    }

    if(valCm <= 3){
      popupWithFmt(1,"Tank height too small");
      return;
    }
    // Serial.print("The size of the tank is: ");
    // Serial.println(TANKHEIGHT*100);

    tnkset.tankHeightCm = (uint16_t)(valCm);
    // Serial.print("The size of the tank in cm is: ");
    // Serial.println(tnkset.tankHeightCm);
    tapprox.getValue(&Number1);
    fulemp.getValue(&Number3);
    //Number3=-1;
    if(Number3!=-1 && Number1==0){
      if(Number3==0){
        tnkset.mode=CAL_MARK_EMPTY;
        empty_pressed = true;
      }else if(Number3==1){
        tnkset.mode=CAL_MARK_FULL;
        full_pressed = true;
      }else {
        Serial.println("Invalid text calibration input");
        return;
      }
    }else if (Number3==-1 && Number1!=0){
      tnkset.mode=CAL_ESTIMATED;
      tnkset.value=(uint8_t)Number1;
      //starttransaction(&tnkset,sizeof(CalibrationSettings),destinationtnk,CAL_SETTINGS);
    }else if((Number3==-1 && Number1==0)){
      Serial.println("indicate level");
      return;
    }
    sendCalibratioNSettings();
    showupdatepage(1);
    return;
  }else if(ptr==&pconfirm){
    uint32_t startH, stopM;
    hrnum.getValue(&startH);
    minnum.getValue(&stopM);
    if(startH==stopM){
      popupWithFmt(1,"start and stop % can not be the same");
      //Serial.println("Pump start and stop cannot be equal");
      return;
    }else if(startH!=stopM){
      startperc = startH;
      stopperc = stopM;
      if(startperc > stopperc){
        emptying_fulling = true;
      }else if(startperc < stopperc){
        emptying_fulling = false;
      }
    }
    //Serial.printf("Pump auto range saved: START=%d STOP=%d\n",startperc, stopperc);
    showupdatepage(2);
    return;
  }else if(ptr==&wconfirm){
    bool validID = checktext(wifiID,ssid,sizeof(ssid),RULE_SSID);
    bool validpass = checktext(wifipass,password,sizeof(password),RULE_PASSWORD);
    if(!validID||!validpass){
      Serial.println("invalid credentails enetered");
      popupWithFmt(1,"%s invalid credentails enetered",validID? "wifiname":"password");
      return;
    }
    if(!checkwifi()){
      Serial.println("WiFi didnt connect");
      return;
    }else{
      showupdatepage(3);
      popupWithFmt(0,"%s was found",ssid);
      return;
    }
  }else{
    return;
  } 
}
void connect_systemsbtn(void *ptr){
  if (tx.active){ 
    popupWithFmt(2,"Transaction in progress plz wait a few seconds and try again"); 
    return; 
  } 
  if (ptr==&taddbtn){
    dbSerial.println("add tank button pressed");
    g_tankConnection = CON_PART;
    tankConnected = false;
    starttransaction(nullptr,0,destinationtnk,SYS_AVAILABLE);
    tlinktxt.setText(connectiontext[g_tankConnection]);

  }else if(ptr==&paddbtn){
    dbSerial.println("add pump button pressed");
    g_pumpConnection = CON_PART;
    starttransaction(nullptr,0,destinationpmp,SYS_AVAILABLE);
    plinktxt.setText(connectiontext[g_pumpConnection]);
  }else{
    return;
  }
}
bool isConnectedToSender(byte sender) {
  if(sender==destinationtnk) return tankConnected;
  else if(sender ==destinationpmp) return pumpConnected;
  else return false;
}
//atachpush signal to buttions
void Attachpush(){
  //page0
  tankbtn.attachPop(pagebtn,&tankbtn);
  pumpbtn.attachPop(pagebtn,&pumpbtn);
  settingbtn.attachPop(pagebtn,&settingbtn);
  //page1
  tbackbtn.attachPop(pagebtn,&tbackbtn);
  tcalbtn.attachPop(pagebtn,&tcalbtn);
  taddbtn.attachPop(pagebtn,&taddbtn);
  //page2
  pbackbtn.attachPop(pagebtn,&pbackbtn);
  paddbtn.attachPop(pagebtn,&paddbtn);
  pstrtstp.attachPop(pagebtn,&pstrtstp);
  emergstop.attachPop(pagebtn,&emergstop);
  pmodebtn.attachPop(pagebtn,&pmodebtn);
  pstatbtn.attachPop(pagebtn,&pstatbtn);
  //page3
  sbackbtn.attachPop(pagebtn,&sbackbtn);
  blynkbtn.attachPop(pagebtn,&blynkbtn);
  chngwifi.attachPop(pagebtn,&chngwifi);
  //tdchng.attachPop(pagebtn,&tdchng);
  //page4
  tcancel.attachPop(pagebtn,&tcancel);
  tconfirm.attachPop(pagebtn,&tconfirm);
  //page5 
  pcancel.attachPop(pagebtn,&pcancel);
  pconfirm.attachPop(pagebtn,&pconfirm);
  //page6
  exitpop.attachPop(exitpopuppg,&exitpop);
  //page8
  wconfirm.attachPop(pagebtn,&wconfirm);
  wcancel.attachPop(pagebtn,&wcancel);
}


//Popupmessage handlers
// Show next popup if queue has something and no popup currently active
void showNextPopupIfAny(){
  if (popup.count == 0) {
    popup.active = false;
    return;
  }
  PopupMessage &msg = popup.queue[popup.head];
  strncpy(g_popupContent, msg.content, sizeof(g_popupContent)-1);
  //setPopupHeadFromValue(msg.severity);
  popup.currentIndex++;
  popup.active = true;
  // show page and update display
  showpopuppg(msg.severity);
}
// Enqueue an already-formatted message (head and message must be NUL terminated)
void popup_enqueue(uint32_t severity, const char* message) {

  if (popup.count == POPUP_QUEUE_SIZE) {
    // queue full → overwrite oldest message (advance head, reuse insertIdx)
    Serial.println("Popup queue full — overwriting oldest popup");
    // queue full → remove oldest
    popup.head = (popup.head + 1) % POPUP_QUEUE_SIZE;
    popup.count--;
  }
  // copy into queue slot
  PopupMessage &slot = popup.queue[popup.tail];
  slot.severity = severity;
  if (message) {
    strncpy(slot.content, message,  POPUP_TEXT_LEN - 1);
    slot.content[POPUP_TEXT_LEN - 1] = '\0';
  } else {
    slot.content[0] = '\0';
  }
  popup.tail = (popup.tail + 1) % POPUP_QUEUE_SIZE;
  popup.count++;
  if(currpg==6){
    char Counter[15];
    sprintf(Counter, "Msg 1 of %d",popup.count);
    msgcounter.setText(Counter);
  }
  // if nothing is currently shown, immediately pop & show the new one
  if (!popup.active) {
    showNextPopupIfAny();
  }
}
void showpopuppg(uint32_t value){
  if (g_popupContent[0] == '\0') return;
  if(currpg!=6){
  prevcurrpg = currpg;
  }
  page6.show();
  setPopupHeadFromValue(value);
  currpg =6;
  updatePopupPage();
}
void exitpopuppg(void *ptr){
  if(ptr == &exitpop){
    Serial.println("exitpopup pressed");
    if (popup.count == 0){
      return;
    }
    popup.head = (popup.head + 1) % POPUP_QUEUE_SIZE;
    popup.count--;
    // If there are more messages queued, show the next one
    if (popup.count > 0) {
      showNextPopupIfAny();
    } else {
      popup.active = false;
      g_popupContent[0] = '\0';
      showupdatepage(prevcurrpg);
    }
  }
} 
void setPopupHeadFromValue(uint32_t value){
  switch(value){
    case 0:
      strncpy(g_popupHead, "Information", sizeof(g_popupHead)-1);
      sendCommand("head.bco=2024");
      break;

    case 1:
      strncpy(g_popupHead, "Warning", sizeof(g_popupHead)-1);
      sendCommand("head.bco=65504");
      break;

    case 2:
    default:
      strncpy(g_popupHead, "Error", sizeof(g_popupHead)-1);
      sendCommand("head.bco=63488");
      break;
  }
}
void ACKpopupmsg(uint32_t value, byte type){
  // Compose the message locally into a buffer (same logic you had)
  char contentBuf[POPUP_TEXT_LEN];
  contentBuf[0] = '\0';
  switch(type){
    case SYS_AVAILABLE:
      if(tx.dest == destinationtnk){
        g_tankConnection = DISCON_EST;
        tankConnected = false;
        strncpy(contentBuf,"Tank Connection failed,if issue persists check systems", sizeof(contentBuf)-1);
      }else if(tx.dest == destinationpmp){
        g_pumpConnection = DISCON_EST;
        pumpConnected = false;
        strncpy(contentBuf,"Pump Connection failed, if issue persists check systems", sizeof(contentBuf)-1);
      }
      break;
    case CAL_SETTINGS:
      rollbackCalibration();
      g_tankConnection = CON_PART;
      resetTx();
      starttransaction(nullptr,0,destinationtnk,SYS_AVAILABLE);
      strncpy(contentBuf,"Calibration failed: Tank didnt get calibration data", sizeof(contentBuf)-1);
      break;
    case MSG_REQ_TANKLVL:
      g_tankConnection = CON_PART;
      resetTx();
      starttransaction(nullptr,0,destinationtnk,SYS_AVAILABLE);
      strncpy(contentBuf, "Tank not responding,checking connection", sizeof(contentBuf)-1);
      break;
    case MSG_REQ_PUMPSTAT:
      g_pumpConnection = CON_PART;
      resetTx();
      starttransaction(nullptr,0,destinationpmp,SYS_AVAILABLE);
      strncpy(contentBuf, "Pump not responding,checking connections", sizeof(contentBuf)-1);
      break;
    default:
      strncpy(contentBuf, "Unknown popup type", sizeof(contentBuf)-1);
      break;
  }
  contentBuf[sizeof(contentBuf)-1] = '\0';

  popup_enqueue(value, contentBuf);
}
void popupWithFmt(uint32_t severity, const char *fmt, ...){
  // format into a buffer then enqueue
  char buf[POPUP_TEXT_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf) - 1, fmt, args);
  buf[sizeof(buf) - 1] = '\0';
  va_end(args);

  //const char* headstr = setPopupHeadFromValue(severity);
  popup_enqueue(severity, buf);
}

//Display pages updaters
void updatePopupPage(){
  pophead.setText(g_popupHead);
  popcontent.setText(g_popupContent);
  char Counter[15];
  sprintf(Counter, "Msg 1 of %d",popup.count);
  msgcounter.setText(Counter);
}
void updatetankpg(){
  tlvl.setValue(g_tankPercent);
  tperctxt.setValue(g_tankPercent);
  tlinktxt.setText(connectiontext[g_tankConnection]);
  tcaltxt.setText(calibrationtext[calibration]);

  if(TANKHEIGHT > 0){
    setTextMeters(theighttxt, TANKHEIGHT);
  }else{
    theighttxt.setText("--");
  }
}
void disable_enablepmpstatbtn(uint8_t mode){
  if(mode==1){
    sendCommand("tsw pstat,0");
    sendCommand("pstat.bco=50712");
    sendCommand("pstat.bco2=50712");
    return;
  }else{
    sendCommand("tsw pstat,1");
    sendCommand("pstat.bco=63488");
    sendCommand("pstat.bco2=1024");
    return;
  }
}
void updatepumppg(){
  plinktxt.setText(connectiontext[g_pumpConnection]);

  pmodebtn.setValue(g_pumpMode);
  pstatbtn.setValue(g_pumpState);
  disable_enablepmpstatbtn(g_pumpMode);

  pmodebtn.setText(g_pumpMode ? "AUTO" : "MANUAL");
  pstatbtn.setText(g_pumpState ? "ON" : "OFF");


  setTextfromint(pstarttxt, startperc);
  setTextfromint(pstptxt,stopperc);
}
void updatecalipg(){
  if (TANKHEIGHT <= 0.00f) {
    theight.setText("");   // leave blank
    return;
  }
  setTextfromfloat(theight,TANKHEIGHT);
  // if(tnkcalibrated){
  //   sendCommand("tsw tnkheight,0");
  //   sendCommand("tnkheight.bco=61277");
  //   if(full_pressed && empty pressed){
  //     sendCommand("tsw full,0");
  //     sendCommand("full.bco=1024");
  //     sendCommand("tsw empty,0");
  //     sendCommand("empty.bco=1024");
  //     return;
  //   }
  //   if(full_pressed){
  //     sendCommand("tsw full,0");
  //     sendCommand("full.bco=1024");
  //   }
  //   if(empty_pressed){
  //     sendCommand("tsw empty,0");
  //     sendCommand("empty.bco=1024");
  //   }
  //   return
  // }else{
  //   return;
  // }
}
void updatesettingpg(){
  wifiname.setText(ssid);
  blynkbtn.setValue(g_blynkstste);
  blynkbtn.setText(g_blynkstste? "ON":"OFF");
}
void updatehomepg(){
  cloudicon.setPic(blynkactive? 5:6);
  radioicon.setPic(loraactive? 3:4);
  wifiicon.setPic(wifiactive? 7:8);
  if(timedateset){
    showTime_date(currentTime);
  }else{
    showNoTime();
  }
}
//DATA conversions
void setTextMeters(NexText &txt, float meters){
  char buf[16];

  snprintf(buf, sizeof(buf), "%.2fm", meters); // up to 2 decimals

  txt.setText(buf);
}
void setTextfromint(NexText &txt, int value){
  char buf[10];
  snprintf(buf, sizeof(buf), "%d", value);
  txt.setText(buf);
}
void setTextfromfloat(NexText &txt, float value){
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", value);
  txt.setText(buf);
}
bool getIntFromText(NexText &txt, float &outValue) {
  char buf[10] = {0};
  uint32_t len = sizeof(buf)-1;

  txt.getText(buf, len);
  buf[len] = '\0';
  Serial.print("The value inside the height text is: ");
  Serial.println(buf);

  if (buf[0] == '\0') {
    Serial.println("ERROR: Empty text received");
    return false;
  }

  //if (buf[0] == '\0') return false;   // empty string protectionth

  char *start = buf;
  while (*start == ' ') start++;

  if (*start == '\0') return false;

  char *endPtr;
  float valMeters = strtof(start, &endPtr);

  if (endPtr == start) return false;

  // Skip trailing spaces
  while (*endPtr == ' ') endPtr++;

  if (*endPtr != '\0') return false;  // invalid characters found 

  outValue = valMeters;
  //theight.setText("");
  return true;
}
bool checktext(NexText &txt,char* target,size_t maxLen,TextRule rule){
  char buf[65] = {0};     
  uint32_t buflen = sizeof(buf) - 1;

  txt.getText(buf, buflen);
  buf[buflen] = '\0';

  Serial.print(" entered: "); Serial.println(buf);
   // Trim leading spaces
  char *start = buf;
  while (*start == ' ') start++;
   // Trim trailing spaces
  char *end = start + strlen(start) - 1;
  while(end > start && *end == ' ') *end-- = '\0';

  size_t len = strlen(start);

  // -------- RULES --------
  size_t minLen = 0;
  size_t maxAllowed = maxLen - 1;

  switch(rule)
  {
    case RULE_SSID:
      minLen = 1;
      maxAllowed = 32;
      break;

    case RULE_PASSWORD:
      minLen = 0;      // allow open networks
      maxAllowed = 63;
      break;
  }

  if (len < minLen || len > maxAllowed) {
    Serial.println("ERROR: length invalid");
    return false;
  }



  // Validate characters
  for (int i = 0; start[i] != '\0'; i++) {
    char c = start[i];

    switch(rule){
      case RULE_SSID:
        if (!(isalnum(c) || c=='_' || c=='-' || c==' ' || c=='\'' || c=='.'))
          return false;
        break;

      case RULE_PASSWORD:
        if(strlen(start) < 8 && strlen(start) != 0){
          return false;
        }
        else if(!isprint(c))  // allow any printable char
          return false;
        break;
    }
  }
  // Copy valid SSID
  strncpy(target,start,maxLen-1);
  target[maxLen-1] = '\0';
  Serial.println(target);

  return true;
}
const char* getTankErrorString(uint8_t code) {
    if (code < sizeof(TankErrorStrings)/sizeof(TankErrorStrings[0])) {
        return TankErrorStrings[code];
    }
    return "Invalid error code";
}
const char* getPumpErrorString(uint8_t code) {
    if (code < sizeof(PumpErrorStrings)/sizeof(PumpErrorStrings[0])) {
        return PumpErrorStrings[code];
    }
    return "Invalid error code";
}

//Status check handlers
void Autopumpcheck(){
  if(!pumpConnected) return;

  if(g_pumpMode!=1) return;

  unsigned long now = millis();
  if(emptying_fulling){
    if(!pump_triggered && g_tankPercent >= startperc){
      Serial.println("AUTO: Starting pump");
      pmpstat.state =1;
      prev_g_pumpState = g_pumpState;
      g_pumpState = 1;
      pump_triggered = true;
      if(currpg==2){
        pstatbtn.setValue(g_pumpState);
        pstatbtn.setText("ON");
      }
      sendPumpstatus();
    }else if(pump_triggered && g_tankPercent <= stopperc){
      Serial.println("AUTO: Stopping pump");
      pmpstat.state =0;
      prev_g_pumpState = g_pumpState;
      g_pumpState = 0;
      pump_triggered = false;
      if(currpg==2){
        pstatbtn.setValue(g_pumpState);
        pstatbtn.setText("OFF");
      }
    }
  }else if(!emptying_fulling){
    if(!pump_triggered && g_tankPercent <= startperc){
      Serial.println("AUTO: Starting pump");
      pmpstat.state =1;
      prev_g_pumpState = g_pumpState;
      g_pumpState = 1;
      pump_triggered = true;
      if(currpg==2){
        pstatbtn.setValue(g_pumpState);
        pstatbtn.setText("ON");
      }
      sendPumpstatus();
    }else if(pump_triggered && g_tankPercent >= stopperc){
      Serial.println("AUTO: Stopping pump");
      pmpstat.state =0;
      prev_g_pumpState = g_pumpState;
      g_pumpState = 0;
      pump_triggered = false;
      if(currpg==2){
        pstatbtn.setValue(g_pumpState);
        pstatbtn.setText("OFF");
      }
    }
  }
  sendPumpstatus();
}
void pumpcontrolbtn(void *ptr){
  uint32_t btnval1;
  uint32_t btnval2;
  if(Pmp_manual_override && pumpConnected){
    popupWithFmt(1,"Pump has benn manually overriden,reset in order to send data");
    return;
  }
  if(ptr==&emergstop){
    resetTx();
    prev_g_pumpState = g_pumpState;
    g_pumpMode = 0;
    g_pumpState = 0;
    disable_enablepmpstatbtn(0);
    pmpstat.state=g_pumpState;
    pmodebtn.setValue(g_pumpMode);
    pstatbtn.setValue(g_pumpState);
    pmodebtn.setText(g_pumpMode ? "AUTO" : "MANUAL");
    pstatbtn.setText(g_pumpState ? "ON" : "OFF");
    sendPumpstatus();
  }else if(ptr==&pmodebtn){
    pmodebtn.getValue(&btnval1);
    g_pumpMode = btnval1;
    disable_enablepmpstatbtn(g_pumpMode);
    pmodebtn.setText(g_pumpMode ? "AUTO" : "MANUAL");
  }else if(ptr==&pstatbtn){
    prev_g_pumpState = g_pumpState;
    pstatbtn.getValue(&btnval2);
    g_pumpState = btnval2;
    pmpstat.state=g_pumpState;
    //pstatbtn.setValue(g_pumpState);
    pstatbtn.setText(g_pumpState ? "ON" : "OFF");
    if(!pumpConnected){
      popupWithFmt(1,"Pump Disconnected,plz connect to send data");
      g_pumpState = prev_g_pumpState;
      return;
    }
    sendPumpstatus();
  }
}
void en_connection(byte fromwhere,uint8_t code){
  if(fromwhere==destinationtnk){
    tankConnected = true;
    g_tankConnection = CON_FULL;

    if(code==5){
      tnkcalibrated = false;
      popupWithFmt(0,"Tank connection successful:Tank uncalibrated");
    }else{
      tnkcalibrated = true;
      popupWithFmt(0,"Tank connection successful:Tank calibrated and ready");
    } 
  }else if(fromwhere==destinationpmp){ 
    pumpConnected = true; 
    g_pumpConnection = CON_FULL; 
    if(code==2){
      Pmp_manual_override = true;
      popupWithFmt(0,"Pump connection successful.Pump manual override active");
    }else{
      Pmp_manual_override = false;
      popupWithFmt(0,"Pump connection successful.Pump ready");
    } 
  } else return;
}

//tank packet handlers
void unpackTankLevel(const uint8_t* buf, TankLevelPayload& p){
    p.percent = buf[0];
    p.cal_stage = buf[1];
}
bool GetTanklevel(const uint8_t *payload, uint8_t len){
  if (len < 2) return false;
  unpackTankLevel(payload, tnklvl);
  return handleTankLevel(tnklvl);
}
bool handleTankLevel(const TankLevelPayload& p){
  g_tankPercent = p.percent;
  calibration = (Con_calStates)p.cal_stage;
  // Serial.print("The calibration stage is: ");
  // Serial.println(calibration);
  // Serial.print("the tank level is: ");
  // Serial.println(g_tankPercent);
  if (currpg==1){
  tlvl.setValue(g_tankPercent);
  tperctxt.setValue(g_tankPercent);
  tcaltxt.setText(calibrationtext[calibration]);
  }
  return true;
}
uint8_t packCalibrationSettings(uint8_t* buf, const CalibrationSettings& c){
    buf[0] = c.mode;
    buf[1] = c.value;

    buf[2] = c.tankHeightCm & 0xFF;
    buf[3] = (c.tankHeightCm >> 8) & 0xFF;

    return 4;
}
void sendCalibratioNSettings(){
  uint8_t payload[4];

  uint8_t len = packCalibrationSettings(payload, tnkset);

  // Backup current state in case the tank rejects or times out
  //backupCalibration();

  starttransaction(payload, len, destinationtnk, CAL_SETTINGS);
}

//pump packet handlers
uint8_t packPumpStatus(uint8_t* buf, const PumpStatusPayload& p){
    buf[0] = p.state;
    return 1;
}
void unpackPumpStatus(const uint8_t* buf, PumpStatusPayload& p){
    p.state = buf[0];
    p.mode = buf[1];
}
bool GetPumpmstatus(const uint8_t *payload, uint8_t len){
  if (len < 2) return false;
  unpackPumpStatus(payload, pmpstat);
  return handlePumpStatus(pmpstat.state,pmpstat.mode);
}
void sendPumpstatus(){
  uint8_t payload[1];

  uint8_t len = packPumpStatus(payload,pmpstat);
  starttransaction(payload, len, destinationpmp, MSG_REQ_PUMPSTAT);
}
bool handlePumpStatus(uint8_t state,uint8_t mode){
  state = (state != 0) ? 1 : 0;
  mode = (mode!=0) ? 1 : 0;

  bool man_override = (mode==1);

  if(man_override!=Pmp_manual_override){
    Pmp_manual_override= (mode==1);
    if(Pmp_manual_override){
      pump_triggered = false;
      g_pumpState = 0;
      g_pumpMode = 0;
      //popupWithFmt(0,"Pump manual override actived",POP_HIGH);
      //return true;
    }//else{
    //   //popupWithFmt(0,"Pump manual override deactived",POP_HIGH);
    //   return true;
    // }
    popupWithFmt(0,"Pump manual override %s",(Pmp_manual_override ? "activated" : "deactivated"));
    return true;
  }else{
    g_pumpState = state;
    pump_triggered = (g_pumpState ==1);
  }

  if(state !=g_pumpState && waitingPumpUpdate){
    popupWithFmt(1,"Pump status didnt change, plz check pump");
    return true;
  }else if(!waitingPumpUpdate){
    // if(g_pumpState==1){
    //   popupWithFmt(0,"Pump has been turned ON maunally",POP_HIGH);
    // }else if(g_pumpState==0){
    //   popupWithFmt(0,"Pump has been turned OFF maunally",POP_HIGH);
    // }
    popupWithFmt(0,"Pump has been turned %s manually",(g_pumpState ? "ON" : "OFF"));
    return true;
  }else{
    return false;
  }
  return false;
}

//Erro packet handlers
void unpackError(const uint8_t* buf, ErrorPayload& p){
  p.ERR = buf[0];
}
uint8_t Geterror(const uint8_t *payload, uint8_t len){
  if (len < 1) return 0xFF;
  unpackError(payload, errotype);
  ERROR = (Errortype)errotype.ERR;
  return errotype.ERR;;
  //return handleerror(errotype);
}
void handleerror(const ErrorPayload& p){
  ERROR = (Errortype)p.ERR;
}

//LORA DATA TRANSMISSION
bool starttransaction(const void *payload,uint8_t payloadLen, byte dest, byte type){ 
  if (tx.active){ 
    popupWithFmt(2,"Data transaction in progress plz wait a few second and try again"); 
    return false; 
  }  

  if (type != SYS_AVAILABLE) { 
    if(!isConnectedToSender(dest)){
      if(!tankConnected){
        popupWithFmt(1,"Tank disconnected,plz connect to send data");
      }else if(!pumpConnected){
        popupWithFmt(1,"Pump disconnected,plz connect to send data");
      }
      return false;
    } 
  }
  resetTx();
  tx.active = true; 
  tx.dest = dest; 
  tx.type=type; 
  //tx.payload[0] = payload; 
  tx.retriesleft = 5;
  tx.payloadLen = payloadLen; 
  tx.txID = msgcount++; 
  if(payloadLen >0 && payload !=nullptr){
    if (payloadLen > sizeof(tx.payload)) return false; 
    memcpy(tx.payload, payload, payloadLen); 
  } 
  sendRAW(tx); 
  return true; 
}
void sendRAW(pendingTX &t){ 
  LoRa.beginPacket(); 
  LoRa.write(t.dest); 
  LoRa.write(localaddress); 
  LoRa.write(t.txID); 
  LoRa.write(t.type); 
  LoRa.write(t.payloadLen); 
  LoRa.write(t.payload, t.payloadLen); 
  LoRa.endPacket(true); 
  Serial.println("data sent"); 
  t.lastsendtime = millis();
  LoRa.receive();
}
void sendACK(byte where, byte txID){ 
  LoRa.beginPacket(); 
  LoRa.write(where); 
  LoRa.write(localaddress); 
  LoRa.write(txID); 
  LoRa.write(MSG_ACK); 
  LoRa.write(0); 
  LoRa.endPacket(); 
  LoRa.receive(); 
}

// void setNextionButtonState(const char* name, bool enabled, uint32_t colorActive, uint32_t colorDisabled) {
//   // assume button object names on page3 are "full" and "empty"
//   char cmd[64];
//   // enable/disable touch
//   snprintf(cmd, sizeof(cmd), "tsw %s,%d", name, enabled ? 1 : 0);
//   sendCommand(cmd);
//   // set background color to show disabled
//   snprintf(cmd, sizeof(cmd), "%s.bco=%lu", name, enabled ? colorActive : colorDisabled);
//   sendCommand(cmd);
// }
// void updatecalipg(){
//   // If fully calibrated, disable both
//   if (tnkcalibrated) {
//     setNextionButtonState("full", false, 50712, 63488);
//     setNextionButtonState("empty", false, 50712, 63488);
//     // also disable height editing if you want:
//     sendCommand("tnkheight.en=0");
//     return;
//   }

//   // If the user already pressed full, disable full; empty remains to be pressed
//   setNextionButtonState("full", !full_pressed, 50712, 63488);
//   setNextionButtonState("empty", !empty_pressed, 50712, 63488);

//   // height editing: if either full or empty already pressed and you want to prevent edits
//   // choose desired behaviour. Example: allow editing but warn user that edits imply re-calibration.
//   sendCommand("tnkheight.en=1");
// }

//battery percentagehandlers
// uint8_t BatteryVoltage(){
//   int total = 0;
//   for(int i=0; i<10; i++){
//     total += analogRead(BATTERY_PIN);
//   }
//   int adcValue = total / 20;

//   float vMeasured = (adcValue / 4095.0) * ADC_REF;
//   float vBattery = vMeasured * ((R1 + R2) / R2);

//   Serial.print("Battery Voltage: ");
//   Serial.println(vBattery);

//   float percent = (vBattery - 7.0) / (9.6 - 7.0) * 100.0;

//   if (percent > 100) percent = 100;
//   if (percent < 0) percent = 0;
//   return percent;
// }
// void smoothVolatge(){
//   float smoothedPercent = 0;
//   float newPercent = BatteryVoltage();  // your existing function

//   smoothedPercent = smoothedPercent + alpha * (newPercent - smoothedPercent);

//   Serial.println(smoothedPercent);
// }

// //DTAE AND TIME HANDLERS
void showTime_date(struct tm &timeinfo){
  //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  char timebuf[10];
  char datebuf[20];

  strftime(timebuf, sizeof(timebuf), "%I:%M%p", &timeinfo);
  strftime(datebuf, sizeof(datebuf), "%Y/%m/%d", &timeinfo);

  Time.setText(timebuf);
  Date.setText(datebuf);
}
void showNoTime(){
  Time.setText("--:--");
  Date.setText("----/--/--");
}
void loadClock() {
  if (getLocalTime(&currentTime)) {
    Serial.println("Time obtain sucessfully");
    timedateset = true;
    return;
  }else{
    Serial.println("RTC time not available, retrieving from WiFi...");
    syncTimeFromWiFi();
  }
}
void updateClock(){
  if(millis() >= nextClockUpdate){
    currentTime.tm_min++;            // increment manually
    currentTime.tm_sec = 0;
    mktime(&currentTime);
    if(currpg==0){
    showTime_date(currentTime);
    }
    alignMinuteUpdate();
  }
}
bool syncTimeFromWiFi(){
  // Check if SSID is blank/null
  if (ssid[0] == '\0') {
      Serial.println("No WiFi SSID set, skipping time sync.");
      timedateset = false;
      return false;
  }

  if(WiFi.getMode()==WIFI_MODE_NULL || WiFi.status() !=WL_CONNECTED){
    WiFi.mode(WIFI_STA);
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    unsigned long start = millis();

    while(WiFi.status() != WL_CONNECTED){
      if(millis() - start > 20000){
        Serial.println("WiFi connection timeout");
        timedateset = false;
        return false;
      }

      delay(500);
      Serial.print(".");
    }
  }

  Serial.println("\nWiFi connected");
  static int retries = 0;
  while(retries<10){
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    if(getLocalTime(&currentTime)){
      //alignMinuteUpdate();
      Serial.println("Time synced from NTP");
      timedateset = true;

      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);

      Serial.println("WiFi disconnected");
      retries=0;
      return true;
    }else{
      Serial.println("Failed to obtain NTP time, retrying...");
      retries++;
      delay(100); // small delay before retry
    }
  }
  Serial.println("Failed to sync time after 10 retries");
  timedateset = false;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  retries = 0; // reset for next attempt
  return false;
}
void alignMinuteUpdate(){
  int secondsRemaining = 60 - currentTime.tm_sec;
  nextClockUpdate = millis() + (secondsRemaining * 1000);
}
void setManualTime(int year,int month,int day,int hour,int minute,int second){

  struct tm t;

  t.tm_year = year - 1900;
  t.tm_mon  = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min  = minute;
  t.tm_sec  = second;

  time_t newTime = mktime(&t);
  struct timeval now = { newTime, 0 };
  settimeofday(&now, NULL);
  currentTime = t;   // update smooth clock
  Serial.println("Manual time set");
  //showTime_date(currentTime);
  timedateset = true;
  alignMinuteUpdate();
}
