#include <WiFi.h>
#include "time.h"
#include <SPI.h>
#include <LoRa.h>
#include <Nextion.h>

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

//DATA FLAGS
volatile bool loraPacketReady = false;
bool tankConnected = false;
bool pumpConnected = false;
bool pump_triggered = false;
bool g_popupActive = false;
bool g_popupPendingUpdate = false;
bool waitingTankUpdate = false;
bool waitingPumpUpdate = false;
bool tnkcalibrated = false;
bool Pmp_manual_override = false;

//TIMER VALUES
unsigned long lastRetryTime =0;
unsigned long lastTimeupdate =0;
const unsigned long timeinterval =1000;
const unsigned long HEARTBEAT_INTERVAL = 30000;
unsigned long lastheartbeat =0;
unsigned long acktimer =0; 
const unsigned long ACK_TIMEOUT = 5000; 
const unsigned long DATA_TIMEOUT = 30000;
unsigned long getupdate = 0;

//WIFI CREDENTIALS
const char* ssid     = "Shane's A15";
const char* password = "Meforever123";

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

//page3 tank calibration 
NexPage page3 = NexPage(3,0,"page3");//**
NexText theight = NexText(3,12,"tnkheight");
//NexDSButton tfullbtn = NexDSButton(3,7,"full");//user input to calibrate tank
//NexDSButton temptybtn = NexDSButton(3,8,"empty");//user input to calibrate tank 
NexButton tconfirm = NexButton(3,5,"tcon");//confirms and set tank size**
NexButton tcancel = NexButton(3,4,"tcan");//leave calibration seeting no variable update**
NexNumber tapprox = NexNumber(3,9,"approx");//user input via sub and plus button**
NexVariable fulemp = NexVariable(3,13,"fulemp");//**

//page4 pump start and stop time
NexPage page4 = NexPage(4,0,"page4");//**
//NexButton hradd = NexButton(4,5,"hadd");//used to add hour
//NexButton minadd = NexButton(4,6,"madd");//used to add min
//NexButton hrsub = NexButton(4,10,"hsub");//used to sub hour
//NexButton minsub = NexButton(4,11,"msub");//used to sub min
NexButton pconfirm = NexButton(4,11,"pcon");//confirm and set time**
NexButton pcancel = NexButton(4,12,"pcan");//leave time settings, no variable update**
NexNumber hrnum = NexNumber(4,8,"hnum");// used to show hour**
NexNumber minnum = NexNumber(4,7,"mnum");//used to show min**

//page5 popup page
NexPage page5 = NexPage(5,0,"page5");//
NexText pophead = NexText(5,1,"head");
NexText popcontent = NexText(5,2,"content");
NexHotspot exitpop = NexHotspot(5,4,"exitpop");
//NexVariable popstyle = NexVariable(5,5,"style");


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
pendingTX tx = {0};

//tanklvel packet 
struct TankLevelPayload {
  uint8_t percent;
  uint8_t cal_stage; 
};
TankLevelPayload tnklvl;

//pumpstatus packet 
struct PumpStatusPayload {
  uint8_t state;
  uint8_t mode;
};
PumpStatusPayload pmpstat;

//tank calibrtion level packet 
struct CalibrationPayload { 
  uint8_t level; 
};
CalibrationPayload tnkcal;

//tank calibration packet
struct CalibrationSettings { 
  uint8_t mode;   // what the user did
  uint8_t value;  // % if estimated, unused otherwise 
  uint16_t tankHeightCm;  // usable water height
};
CalibrationSettings tnkset;


//DISPLAY UI INTERATION LISTENER
NexTouch *nex_listen_list[]=
{
  //page0
  &tankbtn,
  &pumpbtn,
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
  &tcancel,
  &tconfirm,
  //page4
  &pcancel,
  &pconfirm,
  //page5
  &exitpop,
  NULL
};

//SETUP FUNCTION
void setup(){
  nexInit();
  Serial.begin(9600);
  sendCommand("dims=75");
  page0.show();
  //SPI.begin(SCK,MISO,MOSI); 
  LoRa.setPins(SS,RST,DIO0);
  if (!LoRa.begin(525E6)){ 
    Serial.println("Lora init failed. check connections"); 
    while(true){ 
      Serial.println("no"); 
      delay(100); // } 
      } 
    Serial.println("LoRa init succeeded.");
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.onReceive(onReceive);
  LoRa.receive();
  //syncTimeFromWiFi();

  Attachpush();
  //printLocalTime();
  Serial.print("setup done");
  delay(100);
}
//LOOP FUNCTION
void loop(){
  nexLoop(nex_listen_list); 
  // if (currpg == 0){ 
  //   updatetime(); 
  // }
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

  if (type != MSG_ACK) {
    if (isDuplicate(sender, txID)){//if (txID == lastRxSeq) {
      Serial.println("Duplicate packet detected — re-ACK only");
      sendACK(sender,txID);
      return;
    }
  }
  if(type==MSG_ACK){
      if (!tx.active) return; 
      if (sender != tx.dest) return; 
      if (txID != tx.txID) return;
      ACKpopupmsg(0,tx.type,POP_LOW);
      switch(tx.type){
        case SYS_AVAILABLE:
          en_connection(sender);
          break;
        case CAL_SETTINGS:
          tnkcalibrated = true;
          break;
        case MSG_REQ_TANKLVL:
          waitingTankUpdate = true;
          getupdate = millis();
          break;
        case MSG_REQ_PUMPSTAT:
          waitingPumpUpdate = true;
          getupdate = millis();
          break;
        default: 
          //Serial.println("Unknown ACK packet type"); 
          break;
      }
      Serial.println("ACK confirmed"); 
      resetTx(); 
      return; 
  }
  if(!isConnectedToSender(sender)) return;
  switch(type){ 
    case DATA_TANK_LEVEL: 
      if(sender !=destinationtnk || len < 2 ||!waitingTankUpdate ) return; 
      sendACK(destinationtnk, txID); 
      GetTanklevel(payload, len);
      waitingTankUpdate = false;
      //delay(10);
      break; 
    case DATA_PUMP_STATUS: 
      Serial.println("pump packet recived");
      if(sender !=destinationpmp || len < 2 ) return;  
      sendACK(destinationpmp, txID);
      GetPumpmstatus(payload, len);
      waitingPumpUpdate = false; 
      //delay(10);
      break; 
    default: 
      Serial.println("Unknown packet type"); 
      break; 
  } 
}
bool isDuplicate(byte sender, byte txID) {
  uint8_t *lastSeq = nullptr;

  if(sender == destinationtnk){
    if(txID == lastRxSeqTank) return true;
    lastRxSeqTank = txID;
    return false;
  }
  if(sender == destinationpmp){
    if(txID == lastRxSeqPump) return true;
    lastRxSeqPump = txID;
    return false;
  }
  return false; // other nodes
}

//RESETTERS 
void resetTx() { 
  tx = pendingTX{}; 
  // tnkset = CalibrationSettings{};
  // pmpstat=PumpStatusPayload {};
  // tnkcal=CalibrationPayload {};
  //waitingforAck = false; 
}
void processtx(){
  unsigned long now = millis();
  if(waitingPumpUpdate || waitingTankUpdate){
    if (now - getupdate < DATA_TIMEOUT) return;
    if(waitingTankUpdate){
      tankConnected = false;
      waitingTankUpdate = false;
      g_tankConnection = DISCON_EST;
      popupmsgCustom(1,"No Update recieved form tank",POP_HIGH);
    }
    if(waitingPumpUpdate){
      pumpConnected = false;
      waitingPumpUpdate = false;
      g_pumpConnection = DISCON_EST;
      //g_pumpMode = prev_g_pumpMode;
      g_pumpState = prev_g_pumpState;
      popupmsgCustom(1,"No Update recieved form pump",POP_HIGH);
    }
  }
  if (!tx.active) return;  
  if (now - tx.lastsendtime < ACK_TIMEOUT) return; 
  if(tx.retriesleft ==0){ 
    Serial.println("Transaction failed"); 
    if(tx.dest == destinationtnk){ 
      tankConnected = false;
      g_tankConnection = DISCON_EST;
      popupmsgCustom(1,"Tank not responding try again",POP_HIGH);
      //waitingTankUpdate = false;
    }
    if (tx.dest == destinationpmp){ 
      pumpConnected = false; 
      g_pumpConnection = DISCON_EST; 
      //g_pumpMode = prev_g_pumpMode;
      g_pumpState = prev_g_pumpState;
      popupmsgCustom(1,"Pump not responding try again",POP_HIGH);
      // pmodebtn.setValue(prev_g_pumpMode);
      // pmodebtn.setText(prev_g_pumpMode ? "AUTO" : "MANUAL");
    } 
    byte failedType = tx.type; 
    resetTx();
    ACKpopupmsg(2,failedType,POP_HIGH);
    return; 
  } 
  Serial.println("Retrying transaction.."); 
  tx.retriesleft --; 
  sendRAW(tx); 
}


//DTAE AND TIME HANDLERS
void showTime_date(struct tm &timeinfo){
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  char timebuf[10];
  char datebuf[20];

  strftime(timebuf, sizeof(timebuf), "%I:%M%p", &timeinfo);
  strftime(datebuf, sizeof(datebuf), "%Y/%m/%d", &timeinfo);

  Time.setText(timebuf);
  Date.setText(datebuf);
}
void updatetime(){
  unsigned long now = millis();

  if(now-lastTimeupdate >= timeinterval){
    lastTimeupdate = now;
    printLocalTime();
  }
}
void printLocalTime(){
  struct tm timeinfo;
  static int retries = 0;
  static bool wifiTempConnected = false;
  if (timeState == TIME_NORMAL) {
    if(!getLocalTime(&timeinfo) && retries <10){
      Serial.println("Failed to obtain time");
      retries++;
      //syncTimeFromWiFi();
    // return;
      if(retries>=10){
        Serial.print("FAILURE TO GET TIME OVERALL");
        retries=0;
          if(WiFi.getMode()==WIFI_MODE_NULL || WiFi.status() !=WL_CONNECTED){
            WiFi.mode(WIFI_STA);
            WiFi.begin(ssid, password);
            //wifiTempConnected = true;
            timeState = TIME_RECONNECTING; 
            Serial.println("reconnecting to wifi.");
          }
      }
      return;
    }
    retries=0;
    showTime_date(timeinfo);
    return;
  }

  if(timeState == TIME_RECONNECTING){
    if(WiFi.status() !=WL_CONNECTED){
      return;
    }
    wifiTempConnected = true;
    Serial.println("WiFi connected, syncing time");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    if (getLocalTime(&timeinfo)) {
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
      showTime_date(timeinfo);


      if(wifiTempConnected){
        Serial.println("Disconnecting WiFi after time sync");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        Serial.println("WiFi disconnected.");
        wifiTempConnected = false;
      }
      timeState = TIME_NORMAL;
    }
  }
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
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  showTime_date(timeinfo);

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi disconnected.");
}

//NEXTION BUTTON INTERATION
void pagebtn(void *ptr){
  switch(currpg){
    case 0:
      if(ptr == &tankbtn) Serial.println("tank button pressed"),showupdatepage(1);
      else if(ptr == &pumpbtn) Serial.println("pump button pressed"),showupdatepage(2);
      break;
    case 1:
      if(ptr == &tcalbtn) Serial.println("tank cal button pressed"),showupdatepage(3);
      else if(ptr == &tbackbtn) Serial.println("tank back button pressed"),showupdatepage(0);
      else if(ptr==&taddbtn) connect_systemsbtn(&taddbtn);
      break;
    case 2:
      if(ptr ==&pbackbtn) Serial.println("pump back button pressed"),showupdatepage(0);
      else if(ptr==&pstrtstp) Serial.println("pump set buttons pressed"),showupdatepage(4);
      else if(ptr==&emergstop || ptr==&pmodebtn || ptr==&pstatbtn) pumpcontrolbtn(ptr);
      else if(ptr==&paddbtn)connect_systemsbtn(&paddbtn);
      break;
    case 3:
      if(ptr ==&tcancel) Serial.println("tankcal cancel button pressed"),showupdatepage(1);
      else if(ptr == &tconfirm) Serial.println("tankcal confirm button pressed"),savedatabtn(&tconfirm);
      //else if(ptr == &tfullbtn || ptr == &temptybtn), savedatabtn(ptr);
      break;
    case 4:
      if(ptr ==&pcancel) Serial.println("pumpset cancel button pressed"),showupdatepage(2);
      else if(ptr == &pconfirm) Serial.println("pumpset confirm button pressed"),savedatabtn(&pconfirm);
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
      break;
    case 4:
      page4.show();
      break;
    default:
      Serial.println("current page not one of the big 3");
      page0.show();
      break;
  }
  delay(50);
}
void savedatabtn(void *ptr){
  uint32_t Number1,Number2,Number3;

  if(ptr==&tconfirm){
    if(!tankConnected){
    popupmsgCustom(0,"Tank Disconnected,plz connect to send data",POP_NORMAL);
    return;
    }
    if(tx.active){
      Serial.println("mssg already in progress");
      return;
    }

    float newHeightMeters = 0;
    bool gotNewHeight = getIntFromText(theight, newHeightMeters);
    // FIRST EVER CALIBRATION → must have height
    if (TANKHEIGHT < 0.1f && !gotNewHeight) {
      //Serial.println("First calibration requires tank height");
      popupmsgCustom(2,"Tank height unknown/invalid",POP_HIGH);
      return;
    }
    float valCm =newHeightMeters*100;
    if(valCm > 450){
    popupmsgCustom(2,"Tank height too large",POP_NORMAL);
    return;
    }

    if(valCm <= 3){
      popupmsgCustom(2,"Tank height too small",POP_NORMAL);
      return;
    }

    if (gotNewHeight) {
      TANKHEIGHT = newHeightMeters;
    }
    // Serial.print("The size of the tank is: ");
    // Serial.println(TANKHEIGHT*100);

    tnkset.tankHeightCm = (uint16_t)(TANKHEIGHT * 100);
    // Serial.print("The size of the tank in cm is: ");
    // Serial.println(tnkset.tankHeightCm);
    tapprox.getValue(&Number1);
    fulemp.getValue(&Number3);
    //Number3=-1;
    if(Number3!=-1 && Number1==0){
      if(Number3==0){
        tnkset.mode=CAL_MARK_EMPTY;
      }else if(Number3==1){
        tnkset.mode=CAL_MARK_FULL;
      }else {
        Serial.println("Invalid text calibration input");
        return;
      }
    }else if (Number3==-1 && Number1!=0){
      tnkset.mode=CAL_ESTIMATED;
      tnkset.value=(uint8_t)Number1;
      //starttransaction(&tnkset,sizeof(CalibrationSettings),destinationtnk,CAL_SETTINGS);
    }else{
     return;
    }
    sendCalibratioNSettings();
    showupdatepage(1);
  }else if(ptr==&pconfirm){
    uint32_t startH, stopM;
    hrnum.getValue(&startH);
    minnum.getValue(&stopM);
    if(startH==stopM){
      popupmsgCustom(1,"start and stop % can not be the same",POP_NORMAL);
      //Serial.println("Pump start and stop cannot be equal");
      return;
    }else if(startH!=stopM){
      startperc = startH;
      stopperc = stopM;
    }
    Serial.printf("Pump auto range saved: START=%d STOP=%d\n",startperc, stopperc);
    showupdatepage(2);
  }else{
    return;
  } 
}
void connect_systemsbtn(void *ptr){
  if (ptr==&taddbtn){
    dbSerial.println("add tank button pressed");
    g_tankConnection = CON_PART;
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
  tcancel.attachPop(pagebtn,&tcancel);
  tconfirm.attachPop(pagebtn,&tconfirm);
  //page4 
  pcancel.attachPop(pagebtn,&pcancel);
  pconfirm.attachPop(pagebtn,&pconfirm);
  //page5
  exitpop.attachPush(exitpopuppg,&exitpop);
}

//Popupmessage handlers
void showpopuppg(){
  if(currpg!=5){
  prevcurrpg = currpg;
  }
  page5.show();
  currpg =5;
  updatePopupPage();
}
void exitpopuppg(void *ptr){
  if(ptr == &exitpop){
  Serial.println("exitpopup pressed");
  g_popupActive = false;
  g_popupCurrentPriority = POP_LOW;
  showupdatepage(prevcurrpg);
  }else return;
}
void setPopupHeadFromValue(uint32_t value){
  switch(value){
    case 0:
      strncpy(g_popupHead, "Information", sizeof(g_popupHead)-1);
      break;

    case 1:
      strncpy(g_popupHead, "Warning", sizeof(g_popupHead)-1);
      break;

    case 2:
    default:
      strncpy(g_popupHead, "Error", sizeof(g_popupHead)-1);
      break;
  }
}
void popupmsgCustom(uint32_t severity, const char* message,PopupPriority prio){

  // If popup already active and new one is lower → ignore
  if (g_popupActive && prio < g_popupCurrentPriority) {
    return;
  }

  // Set popup head using same system
  setPopupHeadFromValue(severity);

  // Copy custom message safely
  strncpy(g_popupContent, message, sizeof(g_popupContent) - 1);
  g_popupContent[sizeof(g_popupContent) - 1] = '\0';

  g_popupPendingUpdate = true;
  g_popupActive = true;
  g_popupCurrentPriority = prio;

  showpopuppg();
}
void ACKpopupmsg(uint32_t value, byte type,PopupPriority prio){

  // If popup already active and new one is lower → ignore
  if (g_popupActive && prio < g_popupCurrentPriority) {
    return;
  }

  // Set HEAD based on severity
  setPopupHeadFromValue(value);

  // Set CONTENT based on packet type + severity
  switch(type){
    case CAL_SETTINGS:
      if(value == 0){
        strncpy(g_popupContent, "Tank received calibration data", sizeof(g_popupContent)-1);
      }
      else if(value == 1){
        strncpy(g_popupContent, "Tank calibration delayed", sizeof(g_popupContent)-1);
      }
      else{ // POP_ERROR
        strncpy(g_popupContent, "Tank did not receive calibration data", sizeof(g_popupContent)-1);
      }
      g_popupPendingUpdate = true;
      showpopuppg();
      break;
    case MSG_REQ_TANKLVL:
      if(value == 0) break;
      else if(value == 1) break;
      else if(value == 2){
        strncpy(g_popupContent, "Tank not responding,plz check system", sizeof(g_popupContent)-1);
      }
      g_popupPendingUpdate = true;
      showpopuppg();
      break;
    case MSG_REQ_PUMPSTAT:
      if(value == 0) break;
      else if(value == 1) break;
      else if(value == 2){
        strncpy(g_popupContent, "Pump not responding,plz check system", sizeof(g_popupContent)-1);
      }
      g_popupPendingUpdate = true;
      showpopuppg();
      break;
    default:
      Serial.println("unknow popupmsg");
      //strncpy(g_popupContent, "Unknown system message", sizeof(g_popupContent)-1);
      break;
  }
}

//Display pages updaters
void updatePopupPage(){
  if(!g_popupPendingUpdate) return;

  pophead.setText(g_popupHead);
  popcontent.setText(g_popupContent);

  g_popupPendingUpdate = false;
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
  }else{
    sendCommand("tsw pstat,1");
    sendCommand("pstat.bco=63488");
    sendCommand("pstat.bco2=1024");
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

//Status check handlers
void Autopumpcheck(){
  if(!pumpConnected) return;

  if(g_pumpMode!=1) return;

  if(startperc==stopperc) return;

  unsigned long now = millis();
  if(!pump_triggered && g_tankPercent >= startperc){
    Serial.println("AUTO: Starting pump");
    pmpstat.state =1;
    prev_g_pumpState = g_pumpState;
    g_pumpState = 1;
    pump_triggered = true;
    if(currpg==2){
      pstatbtn.setValue(g_pumpState);
      pstatbtn.setText(g_pumpState ? "ON" : "OFF");
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
      pstatbtn.setText(g_pumpState ? "ON" : "OFF");
    }
    sendPumpstatus();
  }
}
void pumpcontrolbtn(void *ptr){
  uint32_t btnval1;
  uint32_t btnval2;
  if(Pmp_manual_override){
    popupmsgCustom(0,"Pump has benn manually overriden,reset in orderr to send data",POP_NORMAL);
    return;
  }else if(!pumpConnected){
    popupmsgCustom(0,"Pump Disconnected,plz connect to send data",POP_NORMAL);
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
    if(tx.active){
      return;
    }
    pstatbtn.getValue(&btnval2);
    prev_g_pumpState = g_pumpState;
    g_pumpState = btnval2;
    pmpstat.state=g_pumpState;
    //pstatbtn.setValue(g_pumpState);
    pstatbtn.setText(g_pumpState ? "ON" : "OFF");
    sendPumpstatus();
  }
}
void en_connection(byte fromwhere){
  if(fromwhere==destinationtnk){
    tankConnected = true;
    g_tankConnection = CON_FULL;
    popupmsgCustom(0,"Tank connection successful",POP_NORMAL);
    Serial.println("tank Connection made");  
  }
  else if(fromwhere==destinationpmp){ 
    pumpConnected = true; 
    g_pumpConnection = CON_FULL; 
    popupmsgCustom(0,"Pump connection successful",POP_NORMAL); 
    Serial.println("Pump Connection made");
  } else return;
}

//tank packet handlers
void unpackTankLevel(const uint8_t* buf, TankLevelPayload& p){
    p.percent = buf[0];
    p.cal_stage = buf[1];
}
void GetTanklevel(const uint8_t *payload, uint8_t len){
  if (len < 2) return;
  unpackTankLevel(payload, tnklvl);
  handleTankLevel(tnklvl);
}
void handleTankLevel(const TankLevelPayload& p){
  g_tankPercent = p.percent;
  uint8_t cal = p.cal_stage;
  calibration = (Con_calStates)cal;
  Serial.print("The calibration stage is: ");
  Serial.println(calibration);
  Serial.print("the tank level is: ");
  Serial.println(g_tankPercent);
  if (currpg==1){
  tlvl.setValue(g_tankPercent);
  tperctxt.setValue(g_tankPercent);
  tcaltxt.setText(calibrationtext[calibration]);
  }
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
void GetPumpmstatus(const uint8_t *payload, uint8_t len){
  if (len < 2) return;
  unpackPumpStatus(payload, pmpstat);
  handlePumpStatus(pmpstat.state,pmpstat.mode);
}
void sendPumpstatus(){
  uint8_t payload[1];

 uint8_t len = packPumpStatus(payload,pmpstat);

 starttransaction(payload, len, destinationpmp, MSG_REQ_PUMPSTAT);
}
void handlePumpStatus(uint8_t state,uint8_t mode){
  state = (state != 0) ? 1 : 0;
  mode = (mode!=0) ? 1 : 0;

  bool man_override = (mode==1);

  if(man_override!=Pmp_manual_override){
    Pmp_manual_override= (mode==1);
    if(Pmp_manual_override){
      pump_triggered = false;
      g_pumpState = 0;
      g_pumpMode = 0;
      popupmsgCustom(0,"Pump manual override actived",POP_HIGH);
      return;
    }else{
      popupmsgCustom(0,"Pump manual override deactived",POP_HIGH);
      return;
    }
  }else{
    g_pumpState = state;
    pump_triggered = (g_pumpState ==1);
  }

  if(state !=g_pumpState && waitingPumpUpdate){
    popupmsgCustom(1,"Pump did status dint change, plz check pump",POP_HIGH);
  }else if(!waitingPumpUpdate){
    popupmsgCustom(0,"Pump has been turned ON/OFF maunally",POP_HIGH);
  }
}

//LORA DATA TRANSMISSION
bool starttransaction(const void *payload,uint8_t payloadLen, byte dest, byte type){ 
  if (tx.active){ 
    Serial.println("Transaction already active"); 
    return false; 
  }  

  if (type != SYS_AVAILABLE) { 
    if(!isConnectedToSender(dest)){
      if(!tankConnected){
        popupmsgCustom(0,"Tank disconnected,plz connect to send data",POP_NORMAL);
      }else if(!pumpConnected){
        popupmsgCustom(0,"Pump disconnected,plz connect to send data",POP_NORMAL);
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
