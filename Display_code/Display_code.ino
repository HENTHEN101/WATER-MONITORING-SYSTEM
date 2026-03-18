#include <WiFi.h>
#include <WiFiClient.h>
#include "time.h"
#include <SPI.h>
#include <LoRa.h>
#include <Nextion.h>
#include <Preferences.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

Preferences prefs;

//DEFINED VARIBALES
//lora pins
#define RST 25
#define DIO0 26 //12//27//13
#define SS 5
#define MISO 19
#define MOSI 23
#define SCK 18
#define CS 5
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
#define POPUP_QUEUE_SIZE 10
#define POPUP_TEXT_LEN 300
#define BLYNK_PRINT Serial
//que msg types for blynk
// #define MSG_TANK_LEVEL  1
// #define MSG_PUMP_STATE  2
// #define MSG_PUMP_MODE   3
// #define MSG_TANK_CON    4
// #define MSG_PUMP_CON    5
// #define MSG_TANK_CAL    6
// #define MSG_START_PER   7
// #define MSG_STOP_PER    8
//#define MSG_CON_TANK    9
//#define MSG_CON_PUMP    10
// #define MSG_BUTTON_SYNC 11
#define ACTION_DATA   1
#define ACTION_EVENT  2
#define ACTION_BOTH   3
#define ACTION_PROPERTY 4

#define LORA_EVT_RX        (1 << 0)
#define LORA_EVT_REINIT    (1 << 1)
#define LORA_EVT_TXREADY   (1 << 2)
SPIClass loraSPI(HSPI);
#define SHORT_RETRY_MS 200   // when SPI busy, retry after this many ms (keeps retry loop from draining retries instantly)
SemaphoreHandle_t spiMutex;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

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
bool firstbootup = true;
bool tankConnected = false;
bool pumpConnected = false;
bool pump_triggered = false;
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
bool blynkConfigured = false;
bool wifigood = false;
bool prev_con_g_blynkstate =false; 
bool ssidScanStarted = false;
//volatile bool loraReinitRequested = false;   // set by WiFi task, handled by LoRa task

// Task handles
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t loraTaskHandle = NULL;

//TIMER VALUES
unsigned long wifiStart = 0;
uint8_t ntpRetries = 0;
uint8_t wifiRetryCount = 0;
unsigned long lastRetryTime =0;
unsigned long lastTimeupdate =0;
//const unsigned long timeinterval =60000;
const uint32_t HEARTBEAT_INTERVAL = 20000;
unsigned long lastheartbeat =0;
unsigned long acktimer =0; 
unsigned long nextClockUpdate = 0;
const unsigned long ACK_TIMEOUT = 5000; 
const unsigned long DATA_TIMEOUT = 5000;
// telemetry timing
unsigned long  lastCloudSend = 0;
const uint32_t CLOUD_HEARTBEAT = 300000; // 5 minutes
//wifitimeout
unsigned long lastwifiuse= 0;
const uint32_t WIFI_TIMEOUT = 30000; 
const uint32_t CONN_TIMEOUT = 5000; 
int scanResultCount = -1;
uint8_t savedBSSID[6];
int savedChannel = 0;
bool wifiFastConnectReady = false;
// LoRa health monitor
volatile uint32_t lastlastinit= 0;
const uint32_t LORATIMEOUT=300000;
int loraFailCount = 0;
const int LORA_FAIL_LIMIT = 3;
void IRAM_ATTR onReceive(int packetSize);

unsigned long getupdate = 0;
struct tm currentTime;
//WIFI CREDENTIALS
char ssid[32];//="MonaConnect";
char password[63];//="";
//BLYNK CREDENTIALS
#define BLYNK_TEMPLATE_ID "TMPL2cXYtefV0"
#define BLYNK_TEMPLATE_NAME "WATER MONITORING SYSTEM"
#define BLYNK_AUTH_TOKEN "w_xZ6gKh5uJkLh9qKv08zDcpJtWDbfgT"
#include <BlynkSimpleEsp32.h>

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
int startperc =10, stopperc=10;
uint8_t g_tankPercent = 0;
uint8_t g_pumpMode    = 0;   // 0 = MANUAL, 1 = AUTO
uint8_t g_pumpState   = 0;   // 0 = OFF, 1 = ON
uint8_t prev_g_pumpMode    = 0;   // 0 = MANUAL, 1 = AUTO
uint8_t prev_g_pumpState   = 0;   // 0 = OFF, 1 = ON
uint8_t g_blynkstate = 0; //0=OFF, 1=ON
uint8_t prev_g_blynkstate =0; // 0 = OFF, 1 = ON
uint8_t currpg = 0;//current page
uint8_t prevcurrpg = 0;//current page
char g_popupHead[20] = {0};
char g_popupContent[POPUP_TEXT_LEN] = {0};

//LORA DATA MANAGERS
uint8_t lastRxSeq = 255;   // invalid initial value
uint8_t lastRxSeqTank = 255;
uint8_t lastRxSeqPump = 255;
volatile int loraPacketSize = 0;
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
uint8_t lastTankPercent = 255;
uint8_t lastcalibration = 255;

//uint8_t loraRxBuffer[21];   // safe size
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
NexButton sbackbtn = NexButton(3,7,"sback");//7
NexDSButton blynkbtn = NexDSButton(3,3,"blynk");
NexButton chngwifi = NexButton(3,9,"chngwifi");//9
NexButton tdchng = NexButton(3,10,"tdchange");
NexText wifiname = NexText(3,8,"wifiname");//8

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
NexButton entercalinfo = NexButton(4,11,"calinfo");

//page5 pump start and stop time
NexPage page5 = NexPage(5,0,"page5");//**
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

//page7 popup page
NexPage page7 = NexPage(7,0,"page7");//
NexHotspot exitcalinfo = NexHotspot(7,3,"calinfoex");

//page8 wifi calibration
NexPage page8 = NexPage(8,0,"page8");
NexText wifiID = NexText(8,8,"SSID");
NexText wifipass = NexText(8,7,"PASS");
NexButton wconfirm = NexButton(8,3,"wcon");
NexButton wcancel = NexButton(8,4,"wcan");

//page9 time/date change
NexPage page9 = NexPage(9,0,"page9");//
NexNumber thour = NexNumber(9,13,"hour");
NexNumber tminute = NexNumber(9,14,"minute");
NexNumber dday = NexNumber(9,9,"day");
NexNumber dmonth = NexNumber(9,7,"month");
NexNumber dyear = NexNumber(9,8,"year");
NexDSButton AMPM =NexDSButton(9,18,"ampm");
NexButton tdconfirm = NexButton(9,3,"tdcon");
NexButton tdcancel = NexButton(9,4,"tdcan");

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
Con_calStates g_tankConnection =  NONE;
Con_calStates g_pumpConnection = NONE;
Con_calStates calibration = NONE;

//local time retrival state
enum WiFiTaskState {
  WIFI_IDLE,
  WIFI_CONNECTING,
  WIFI_CONNECTED,
  WIFI_NTP_WAIT,
  WIFI_RUNNING,
  WIFI_DONE,
  WIFI_FAIL
};
WiFiTaskState wifiState = WIFI_IDLE;
enum BlynkState {
  BLYNK_IDLE,
  BLYNK_CHECK_WIFI,
  BLYNK_CONNECT,
  BLYNK_RUNNING,
  BLYNK_DISCONNECT
};
BlynkState blynkState = BLYNK_IDLE;
enum WiFiTaskMode {
  WIFI_VERIFY_ONLY,
  WIFI_SYNC_TIME,
  WIFI_CON_STAY
};
WiFiTaskMode wifiMode;

enum TextRule {
  RULE_SSID,
  RULE_PASSWORD,
};
enum MsgType{
  MSG_TANK_LEVEL,
  MSG_PUMP_STATE,
  MSG_PUMP_MODE,
  MSG_TANK_CAL,
  MSG_PUMP_CON,
  MSG_TANK_CON,
  MSG_CON_PUMP,
  MSG_CON_TANK,
  MSG_START_PER,
  MSG_STOP_PER,
  MSG_MAX
};
MsgType blynkmsg;
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
  "Tank is in manual override",
  "Toggling pump too fast please wait 3 seconds."
};
struct TelemetryMap{
  uint8_t pin;
};

TelemetryMap telemetryMap[MSG_MAX] = {
  {V0}, // MSG_TANK_LEVEL
  {V1}, // MSG_PUMP_STATE
  {V2}, // MSG_PUMP_MODE
  {V3}, // MSG_TANK_CAL
  {V4}, // MSG_PUMP_CON
  {V5}, // MSG_TANK_CON
  {V6}, //MSG_CON_PUMP
  {V7}, //MSG_CON_TANK
  {V8}, // MSG_START_PER
  {V9}  // MSG_STOP_PER
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
//msg struct to communicat between core 1 and 2
struct SystemMessage{
  uint8_t type;   // what kind of message
  uint8_t value;  // value
  const char *event; //event name
  char Msg[64];  //msg for event
  uint8_t action;   //action
};


QueueHandle_t telemetryQueue;   // LoRa -> WiFi
QueueHandle_t commandQueue;     // WiFi -> LoRa
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
  &tdchng,
  //page4
  &tcancel,
  &tconfirm,
  &entercalinfo,
  //page5
  &pcancel,
  &pconfirm,
  //page6
  &exitpop,
  //page7
  &exitcalinfo,
  //page8
  &wconfirm,
  &wcancel,
  //page9
  &tdconfirm,
  &tdcancel,
  NULL
};


void SendMsg(uint8_t type,uint8_t value,uint8_t action,const char *event_property="",const char *fmt="",...){
  if(!blynkactive && !Blynk.connected()){
    return;
  }
    SystemMessage msg;

    msg.type = type;
    msg.value = value;
    msg.event = event_property;
    msg.action = action;

    
    if(fmt[0] != '\0'){
      va_list args;
      va_start(args, fmt);
      vsnprintf(msg.Msg, sizeof(msg.Msg), fmt, args);
      va_end(args);
    } else {
      msg.Msg[0] = '\0';
    }
    xQueueSend(telemetryQueue, &msg, 0);
    lastCloudSend = millis();
}
BLYNK_WRITE_DEFAULT(){
  int pin = request.pin;     // which virtual pin triggered
  int value = param.asInt(); // value from widget

  SystemMessage cmd;

  switch(pin)
  {
    case V1:
      cmd.type = MSG_PUMP_STATE;
      break;
    case V2:
      cmd.type = MSG_PUMP_MODE;
      break;
    case V6:
      cmd.type = MSG_CON_PUMP;
      break;
    case V7:
      cmd.type = MSG_CON_TANK;
      break;
    case V8:
      cmd.type = MSG_START_PER;
      break;
    case V9:
      cmd.type =  MSG_STOP_PER;
      break;
    default:
      return; // ignore unknown pins
  }

  cmd.value = value;
  xQueueSend(commandQueue, &cmd, 0);
}
BLYNK_CONNECTED(){
  SendMsg(MSG_PUMP_CON,g_pumpConnection, ACTION_DATA);
  SendMsg(MSG_TANK_CON,g_tankConnection, ACTION_DATA);
  SendMsg(MSG_PUMP_STATE, g_pumpState, ACTION_DATA);
  SendMsg(MSG_PUMP_MODE, g_pumpMode, ACTION_DATA);
  SendMsg(MSG_TANK_CAL, calibration, ACTION_DATA);
  SendMsg(MSG_START_PER, startperc, ACTION_DATA);
  SendMsg(MSG_STOP_PER, stopperc, ACTION_DATA);
  if(g_pumpMode==1){
    SendMsg(MSG_PUMP_MODE,1,ACTION_PROPERTY,"isDisabled");
  }else{
    SendMsg(MSG_PUMP_MODE,0,ACTION_PROPERTY,"isDisabled");
  }
}

//SETUP FUNCTION
void setup(){
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.begin(9600);
  Serial2.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  WiFi.persistent(false);
  nexInit();
  Attachpush();
  telemetryQueue = xQueueCreate(20,sizeof(SystemMessage));
  commandQueue   = xQueueCreate(20,sizeof(SystemMessage));
  if(!telemetryQueue || !commandQueue){
    Serial.println( "Queue creation failed");
    while(1);
  }
  spiMutex = xSemaphoreCreateMutex();
  loraSPI.setFrequency(4000000);
  loraSPI.begin(SCK,MISO,MOSI,CS);   // SCK, MISO, MOSI, CS
  //LoRa.setSPIFrequency(4000000);
  LoRa.setSPI(loraSPI);
  //SPI.begin(SCK,MISO,MOSI,CS); 
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
  showupdatepage(0);
  // start tasks: wifiTask on core 0, loraTask on core 1
  xTaskCreatePinnedToCore(
    wifiTask,          // function
    "wifiTask",        // name
    8192,              // stack size (bytes)
    NULL,              // pvParameters
    1,                 // priority
    &wifiTaskHandle,   // task handle
    0                  // run on core 0
  );

  xTaskCreatePinnedToCore(
    loraTask,
    "loraTask",
    8192,
    NULL,
    2,
    &loraTaskHandle,
    1                  // run on core 1
  );
  //updatehomepg();
  if(firstbootup){
    bootmsg();
  }else if(!timedateset && !firstbootup){
    startTimeSync();
  }
  lastlastinit= millis();
  firstbootup = false;
  Serial.println("setup done");
  //delay(100);
}
//LOOP FUNCTION
void loop(){
}
//TASKS HANDLER
//wifi/blynk task(runs on core0)
void wifiTask(void * pvParameters){
  (void) pvParameters;
  Serial.println("wifiTask started on core: " + String(xPortGetCoreID()));

  // small delay to let LoRa task/initialization finish
  vTaskDelay(pdMS_TO_TICKS(200));
  while(1){
    //Serial.println("wifi task called");
    if(Blynk.connected()){
      SystemMessage msg;
      while(xQueueReceive(telemetryQueue,&msg,0)==pdTRUE){
        if(msg.action ==ACTION_DATA || msg.action == ACTION_BOTH){
          if(msg.type < MSG_MAX){
            Blynk.virtualWrite(telemetryMap[msg.type].pin, msg.value);
          }
        }
        if(msg.action == ACTION_EVENT || msg.action == ACTION_BOTH){
          if(msg.event[0] != '\0'){
            Blynk.logEvent(msg.event,msg.Msg);
            Serial.printf("Event logged: %s\n",msg.event);
          }
        }
        if(msg.action == ACTION_PROPERTY){
          if(msg.type < MSG_MAX && msg.event[0] != '\0'){
            Blynk.setProperty(telemetryMap[msg.type].pin,msg.event,msg.value);
          }
        }
      }
    }
    // 1) handle WiFi state machine
    handleWifiTask();

    // 2) handle Blynk state machine (connect/disconnect/run)
    handleBlynk();

    // small sleep to yield CPU (10 ms)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  // never reaches here
  vTaskDelete(NULL);
}
//lora/nextion tasks(runs on core1)
void loraTask(void * pvParameters){
  (void) pvParameters;
  Serial.println("loraTask started on core: " + String(xPortGetCoreID()));

  SystemMessage cmd;
  uint32_t events;

  while(1){
    //Serial.println("lora task called");
    // at start of loraTask loop, check for notification (non-blocking)
    xTaskNotifyWait(
        0,              // don't clear bits on entry
        UINT32_MAX,     // clear bits on exit
        &events,
        pdMS_TO_TICKS(1)
    );
    if(events & LORA_EVT_REINIT)
    {
      reinitLoRa();
    }
    //ispacketwaiting();
    ispacketqueued();

    while(xQueueReceive(commandQueue, &cmd,0)){
      if(cmd.type==MSG_CON_PUMP&& cmd.value==1){
        connect_systemsbtn(&paddbtn);
        continue;
      }else if(cmd.type==MSG_CON_TANK&& cmd.value==1){
        connect_systemsbtn(&taddbtn);
        continue;
      }else if(Pmp_manual_override){
        //Blynk.logEvent("system_warning","PUMP MANUAL OVERRIDE ACTIVE");
        SendMsg(0,0,ACTION_BOTH,"pump_override","Pump override active");
        continue;
      }
      switch(cmd.type){
        case MSG_PUMP_STATE:
          if(!pumpConnected){
            SendMsg(MSG_PUMP_STATE,g_pumpState,ACTION_BOTH,"pump_disconnected","Pump disconnected,plz connect to operate");
            continue;
          }
          prev_g_pumpState = g_pumpState;
          g_pumpState = cmd.value;
          pmpstat.state=g_pumpState;
          if(currpg==2){
            pstatbtn.setValue(g_pumpState);
            pstatbtn.setText(g_pumpState ? "ON" : "OFF");
          }
          sendPumpstatus();
          break;
        case MSG_PUMP_MODE:
          if(!pumpConnected){
            SendMsg(MSG_PUMP_MODE,g_pumpMode,ACTION_BOTH,"pump_disconnected","Pump disconnected,plz connect to operate");
            continue;
          }
          g_pumpMode = cmd.value;
          if(currpg==2){
            pmodebtn.setValue(g_pumpMode);
            disable_enablepmpstatbtn(g_pumpMode);
            pmodebtn.setText(g_pumpMode ? "AUTO" : "MANUAL");
          }
          break;
        case MSG_START_PER:{
          if(!pumpConnected){
            SendMsg(MSG_START_PER,startperc,ACTION_BOTH,"pump_disconnected","Pump disconnected,plz connect to operate");
            continue;
          }
          bool stchange = pumptype(cmd.value,stopperc);
          if(!stchange) {
            SendMsg(MSG_START_PER,startperc,ACTION_BOTH,"system_info","Pump start and stop percentages cant be the same");
            continue;
          }
          if(currpg==2){
            setTextfromint(pstarttxt, startperc);
          }
          break;
        }
        case MSG_STOP_PER:{
          if(!pumpConnected){
            SendMsg(MSG_STOP_PER,stopperc,ACTION_BOTH,"pump_disconnected","Pump disconnected,plz connect to operate");
            continue;
          }
          bool spchange = pumptype(startperc,cmd.value);
          if(!spchange){
            SendMsg(MSG_STOP_PER,stopperc,ACTION_BOTH,"system_info","Pump start and stop percentages cant be the same");
            continue;
          }
          if(currpg==2){
            setTextfromint(pstptxt,stopperc);
          }
          break;
        }
      }
    }
    // Do the LoRa + UI work that used to be in loop():
    nexLoop(nex_listen_list);

    // process incoming RX queue items (safe — on core 1)
    sendtankrequest();
    // transaction retry hanlder
    processtx();
    checkLoRaHealth();
    //autopump checks
    Autopumpcheck();
    //clock update
    if(timedateset){
    updateClock();
    }
    // small sleep to yield CPU — keep tight enough to be responsive
     // --- yield CPU if idle ---
    if(!tx.active && uxQueueMessagesWaiting(commandQueue) == 0){
        vTaskDelay(pdMS_TO_TICKS(2));  // only sleep if nothing pending
    }
  }
  // never reaches here
  vTaskDelete(NULL);
}
void bootmsg(){
  popupWithFmt(0,"HOLA,this is live water demo.A system design  to help with water managament  which entails a display system(me),pump and tank monitoring system");
  popupWithFmt(0,"If you also wish to monitor your water remotely go to setting,enter your wifi credentails,wait until wifi is confirmed before connecting to the blynk App, your local time and date will auto update from the wifi.");
  popupWithFmt(0,"If you do not wish to use remote monitoring please set your local time and date in setting.");
}
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
void connectblynk(void *ptr){ 
  if(ptr==&blynkbtn){
    Serial.println("blyn btn code working");
    uint32_t btnval2;
    prev_g_blynkstate = g_blynkstate;
    blynkbtn.getValue(&btnval2);
    g_blynkstate = btnval2;
    //blynkbtn.state=g_blynkstate;
    //pstatbtn.setValue(g_pumpState);
    blynkbtn.setText(g_blynkstate ? "ON" : "OFF");
  }
}
void reinitLoRa(){
  Serial.println("Reinitializing LoRa");
  LoRa.sleep();        // reset radio state
  delay(10);

  LoRa.end();          // release SPI
  delay(10);
  loraSPI.begin(SCK,MISO,MOSI,CS);   // SCK, MISO, MOSI, CS
  LoRa.setSPI(loraSPI);
  if (!LoRa.begin(525E6)) {
    Serial.println("LoRa restart failed");
    loraactive = false;
    return;
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(17);
  LoRa.enableCrc();

  LoRa.onReceive(onReceive);
  LoRa.receive();
  loraactive = true;
  lastlastinit= millis();
  Serial.println("LoRa ready again");
}
void checkLoRaHealth(){
  if(loraFailCount >= LORA_FAIL_LIMIT){
    Serial.println("LoRa fail too many times detected");
    xTaskNotify(
        loraTaskHandle,
        LORA_EVT_REINIT,
        eSetBits
    );
    loraFailCount = 0; // prevent repeated triggers
    return;
  }else if(millis()-lastlastinit>=LORATIMEOUT){
    Serial.println("LoRa timeout detected");
    xTaskNotify(
        loraTaskHandle,
        LORA_EVT_REINIT,
        eSetBits
    );
     return;
  }
}


//LORA DATA RETRIVAL AND PROCESSING
void IRAM_ATTR onReceive(int packetSize) {
  lastlastinit= millis();
  // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // // xTaskNotifyFromISR(
  // //     loraTaskHandle,
  // //     LORA_EVT_RX,
  // //     eSetBits,
  // //     &xHigherPriorityTaskWoken
  // // );
  // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  uint8_t nextHead = RX_NEXT(rxHead);
  // protect update to rxQueue/rxHead
  portENTER_CRITICAL_ISR(&rxMux);
  if (nextHead == rxTail) {
    portEXIT_CRITICAL_ISR(&rxMux);
    Serial.println("RX queue full — packet dropped");
    return;  // queue overflow protection
  }
  rxQueue[rxHead].len = packetSize;
  for (int i = 0; i < packetSize; i++) {
    rxQueue[rxHead].data[i] = LoRa.read();
  }
  rxHead = nextHead;
  portEXIT_CRITICAL_ISR(&rxMux);
}
void ispacketqueued(){
  // guard read of rxTail/rxHead
  while(true){
     RxPacket pkt;
    portENTER_CRITICAL(&rxMux);
    if (rxTail == rxHead) {
        // queue empty
        portEXIT_CRITICAL(&rxMux);
        break;
      }//(rxTail != rxHead) {
      // safely read the packet (copy to local struct to minimize time in critical)
      pkt = rxQueue[rxTail];
      rxTail = RX_NEXT(rxTail); // advance tail (protected)
      portEXIT_CRITICAL(&rxMux);
      processLoRaPacket(pkt.data, pkt.len);
      //rxTail = RX_NEXT(rxTail);
        // retu  rn to RX mode
      // update localTail for loop
  }
}
void processLoRaPacket(const uint8_t* data, int len) {

  //loraFailCount = millis();

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
  if((!isConnectedToSender(sender))||(isConnectedToSender(sender))){
    //Serial.println("system not connected");
    if(type==MSG_ACK && txID ==tx.txID && tx.active && tx.type ==SYS_AVAILABLE && sender==tx.dest ){//tx.txID && tx.active && tx.type ==SYS_AVAILABLE && sender==tx.dest
      //Serial.println("Attempting connection");
      uint8_t ERTPE=Geterror(payload, len);
      en_connection(sender,ERTPE);
      resetTx();
      return;
    }
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
        //Serial.println("pump packet recived");
        if(sender !=destinationpmp || len < 2 ) return;  
        //sendACK(destinationpmp, txID);
        //Serial.println("sender is pump and len is correct");
        processed = GetPumpmstatus(payload, len);
        waitingPumpUpdate = !processed; 
        lastheartbeat = pump_triggered ? 0 : lastheartbeat;
        //Serial.println("end of puump dtat process");
         Serial.println(processed);
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
           lastheartbeat=0;
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
          g_pumpState = prev_g_pumpState;
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
  //Serial.println("reset pendingTX");
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
      SendMsg(0,0,ACTION_EVENT,"system_unresponsive","Tank not responding,checking connection");
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
      SendMsg(0,0,ACTION_EVENT,"system_unresponsive","Pump not responding,checking connection");
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
    Serial.println("lorafailed increased by 1");
    loraFailCount++;
    return; 
  } 
  bool sent = sendRAW(tx);
  if (sent) {
    Serial.println("Retry send attempted; retries left will be decremented");
    tx.retriesleft--;
  } else {
    // SPI was busy — throttle a short backoff before trying again (don't consume a retry)
    // make next allowed retry occur after SHORT_RETRY_MS
    tx.lastsendtime = now - (ACK_TIMEOUT - SHORT_RETRY_MS);
    Serial.println("sendRAW busy — will retry shortly without consuming a retry");
  }
  // Serial.println("Retrying transaction.."); 
  // tx.retriesleft --; 
  //sendRAW(tx); 
}

//Calibration data ahndlers
void backupCalibration() {
  backup_full_pressed = full_pressed;
  backup_empty_pressed = empty_pressed;
}
void rollbackCalibration() {
  full_pressed = backup_full_pressed;
  empty_pressed = backup_empty_pressed;

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
      else if(ptr==&blynkbtn)Serial.println("blynk button pressed"),connectblynk(ptr);
      else if(ptr==&chngwifi) Serial.println("wifichng button pressed"),showupdatepage(8);
      else if(ptr==&tdchng)Serial.println("timedate button pressed"),showupdatepage(9);
      break;
    case 4:
      if(ptr ==&tcancel) Serial.println("tankcal cancel button pressed"),showupdatepage(1);
      else if(ptr == &tconfirm) Serial.println("tankcal confirm button pressed"),savedatabtn(&tconfirm);
      else if(ptr==&entercalinfo)Serial.println("pumpset confirm button pressed"),showupdatepage(7);
      break;
    case 5:
      if(ptr ==&pcancel) Serial.println("pumpset cancel button pressed"),showupdatepage(2);
      else if(ptr == &pconfirm) Serial.println("pumpset confirm button pressed"),savedatabtn(&pconfirm);
      break;
    case 7:
      if(ptr==&exitcalinfo)Serial.println("pumpset confirm button pressed"),currpg=4;
      break;
    case 8:
      if(ptr ==&wconfirm) Serial.println("setting cancel button pressed"),savedatabtn(&wconfirm);
      else if(ptr == &wcancel) Serial.println("seting confirm button pressed"),showupdatepage(3);
      break;
    case 9:
      if(ptr ==&tdconfirm) Serial.println("time/date cancel button pressed"),savedatabtn(&tdconfirm);
      else if(ptr == &tdcancel) Serial.println("time/date confirm button pressed"),showupdatepage(3);
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
    case 7:
      page7.show();
      break;
    case 8:
      page8.show();
      break;
    case 9:
      page9.show();
      break;
    default:
      currpg=0;
      Serial.println("current page not one of the big 3");
      page0.show();
      break;
  }
  //delay(100);
}
void savedatabtn(void *ptr){

  if(ptr==&tconfirm){
    uint32_t Number1,Number2,Number3;
    if(!tankConnected){
    popupWithFmt(1,"Tank disconnected,plz connect to send data");
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
    backupCalibration();
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
    bool change = pumptype(startH,stopM);
    if(change){
      SendMsg(MSG_START_PER,startperc,ACTION_DATA);
      SendMsg(MSG_STOP_PER,stopperc,ACTION_DATA);
    }else{
      popupWithFmt(1,"start and stop % can not be the same");
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
    startCheckWiFi();
    showupdatepage(3);
    popupWithFmt(0,"Attempting to connect to %s",ssid);
    return;
  }else if(ptr==&tdconfirm){
    uint32_t hr,min,ampm,dy,mnth,yr;
    thour.getValue(&hr);
    tminute.getValue(&min);
    dday.getValue(&dy); 
    dmonth.getValue(&mnth);
    dyear.getValue(&yr); 
    AMPM.getValue(&ampm);
    int hour24 = (hr % 12) + (ampm ? 12 : 0);
    //Serial.printf("%d/%d/%d %d:%d\n",dy,mnth,yr,hr,min);
    //Serial.println(hour24);
    bool set = setManualTime(yr,mnth,dy,hour24,min,0);
    if(set){
      showupdatepage(3);
      popupWithFmt(0,"Time set sucessful");
    }else{
      popupWithFmt(2,"Time set unsucessful");
    }
    return;
  }else{
    return;
  } 
}
void connect_systemsbtn(void *ptr){
  if (tx.active){ 
    resetTx();
    // popupWithFmt(2,"Transaction in progress plz wait a few seconds and try again");
  } 
  if (ptr==&taddbtn){
    dbSerial.println("add tank button pressed");
    g_tankConnection = CON_PART;
    tankConnected = false;
    if(currpg==1){
      tlinktxt.setText(connectiontext[g_tankConnection]);
    }
    starttransaction(nullptr,0,destinationtnk,SYS_AVAILABLE);
    SendMsg(MSG_TANK_CON,g_tankConnection,ACTION_DATA);

  }else if(ptr==&paddbtn){
    dbSerial.println("add pump button pressed");
    g_pumpConnection = CON_PART;
    if(currpg==2){
      plinktxt.setText(connectiontext[g_pumpConnection]);
    }
    starttransaction(nullptr,0,destinationpmp,SYS_AVAILABLE);
    SendMsg(MSG_PUMP_CON,g_pumpConnection,ACTION_DATA);
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
  tdchng.attachPop(pagebtn,&tdchng);
  //page4
  tcancel.attachPop(pagebtn,&tcancel);
  tconfirm.attachPop(pagebtn,&tconfirm);
  entercalinfo.attachPop(pagebtn,&entercalinfo);
  //page5 
  pcancel.attachPop(pagebtn,&pcancel);
  pconfirm.attachPop(pagebtn,&pconfirm);
  //page6
  exitpop.attachPop(exitpopuppg,&exitpop);
  //page7
  exitcalinfo.attachPop(pagebtn,&exitcalinfo);
  //page8
  wconfirm.attachPop(pagebtn,&wconfirm);
  wcancel.attachPop(pagebtn,&wcancel);
  //page9
  tdconfirm.attachPop(pagebtn,&tdconfirm);
  tdcancel.attachPop(pagebtn,&tdcancel);
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
  Serial.println("popup msg triggered");
  char contentBuf[POPUP_TEXT_LEN];
  contentBuf[0] = '\0';
  switch(type){
    case SYS_AVAILABLE:
      if(tx.dest == destinationtnk){
        resetTx();
        g_tankConnection = DISCON_EST;
        tankConnected = false;
        if(pump_triggered){
          g_pumpMode =0;
          pump_triggered = false;
          g_pumpState = 0;
          prev_g_pumpState =0;
          pmpstat.state =0;
          sendPumpstatus();
          
        }
        pump_triggered?SendMsg(MSG_PUMP_MODE,g_pumpMode,ACTION_BOTH,"tank_disconnected","Tank disconnected..turning off pump"):SendMsg(0,0,ACTION_EVENT,"tank_disconnected","Tank disconnected");
        strncpy(contentBuf,"Tank Connection failed,if issue persists check systems", sizeof(contentBuf)-1);
      }else if(tx.dest == destinationpmp){
        resetTx();
        g_pumpConnection = DISCON_EST;
        pumpConnected = false;
        SendMsg(0,0,ACTION_EVENT,"pump_disconnected","Pump disconnected");
        strncpy(contentBuf,"Pump Connection failed, if issue persists check systems", sizeof(contentBuf)-1);
      }
      break;
    case CAL_SETTINGS:
      resetTx();
      rollbackCalibration();
      g_tankConnection = CON_PART;
      strncpy(contentBuf,"Calibration failed: Tank didnt get calibration data", sizeof(contentBuf)-1);
      starttransaction(nullptr,0,destinationtnk,SYS_AVAILABLE);
      break;
    case MSG_REQ_TANKLVL:
      resetTx();
      g_tankConnection = CON_PART;
      strncpy(contentBuf, "Tank not responding,checking connection", sizeof(contentBuf)-1);
      SendMsg(0,0,ACTION_EVENT,"system_unresponsive","Tank not responding,checking connection");
      starttransaction(nullptr,0,destinationtnk,SYS_AVAILABLE);
      break;
    case MSG_REQ_PUMPSTAT:
      resetTx();
      g_pumpConnection = CON_PART;
      strncpy(contentBuf, "Pump not responding,checking connections", sizeof(contentBuf)-1);
      SendMsg(0,0,ACTION_EVENT,"system_unresponsive","Pump not responding, checking connection");
      starttransaction(nullptr,0,destinationpmp,SYS_AVAILABLE);
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
    SendMsg(MSG_PUMP_STATE,1,ACTION_PROPERTY,"isDisabled");
    return;
  }else{
    sendCommand("tsw pstat,1");
    sendCommand("pstat.bco=63488");
    sendCommand("pstat.bco2=1024");
    SendMsg(MSG_PUMP_STATE,0,ACTION_PROPERTY,"isDisabled");
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
  }else{
    setTextfromfloat(theight,TANKHEIGHT);
  }
  tapprox.setValue(0);
}
void updatesettingpg(){
  wifiname.setText(ssid);
  blynkbtn.setValue(g_blynkstate);
  blynkbtn.setText(g_blynkstate? "ON":"OFF");
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
  if(!pumpConnected || tx.active||startperc==stopperc||!tankConnected) return;

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
      //sendPumpstatus();
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
    SendMsg(MSG_PUMP_STATE,g_pumpState,ACTION_DATA);
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
      //sendPumpstatus();
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
  if(Pmp_manual_override){
    if(pumpConnected){
      popupWithFmt(1,"Pump has been manually overriden,disbale to control pump");
    }else if(!pumpConnected){
      popupWithFmt(1,"Pump had been manually overriden,reconnect to know when disabled");
    }
    return;
  }else if(!pumpConnected){
    //g_pumpState = prev_g_pumpState;
    popupWithFmt(1,"Pump disconnected,plz connect to send data");
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
    SendMsg(MSG_PUMP_MODE,g_pumpMode,ACTION_DATA);
    //SendMsg(0,0,ACTION_EVENT,"system_info","pump reseted");
  }else if(ptr==&pmodebtn){
    if(startperc==stopperc){
      g_pumpMode = 0;
      popupWithFmt(2,"You need to set the start and stop percentages before been able to set auto mode");
      return;
    }
     Serial.println("pump state btn pressed");
    pmodebtn.getValue(&btnval1);
    g_pumpMode = btnval1;
    disable_enablepmpstatbtn(g_pumpMode);
    pmodebtn.setText(g_pumpMode ? "AUTO" : "MANUAL");
    SendMsg(MSG_PUMP_MODE,g_pumpMode,ACTION_DATA);
  }else if(ptr==&pstatbtn){
    Serial.println("pump state btn pressed");
    prev_g_pumpState = g_pumpState;
    pstatbtn.getValue(&btnval2);
    g_pumpState = btnval2;
    pmpstat.state=g_pumpState;
    //pstatbtn.setValue(g_pumpState);
    pstatbtn.setText(g_pumpState ? "ON" : "OFF");
    if(tx.active){
    //g_pumpState = prev_g_pumpState;
    popupWithFmt(1,"Data transaction in progress plz wait a few second and try again");
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
      popupWithFmt(0,"Tank connection successful");
    } 
    if(currpg==1){
      tlinktxt.setText(connectiontext[g_tankConnection]);
    }
    tnkcalibrated? SendMsg(MSG_TANK_CON,g_tankConnection,ACTION_DATA): SendMsg(MSG_TANK_CON,g_tankConnection,ACTION_BOTH,"system_info","Tank connection successful:Tank uncalibrated");
    SendMsg(MSG_TANK_CAL,calibration,ACTION_DATA);
    lastheartbeat=0;
  }else if(fromwhere==destinationpmp){ 
    pumpConnected = true; 
    g_pumpConnection = CON_FULL; 
    if(code==2){
      Pmp_manual_override = true;
      pump_triggered = false;
      g_pumpState = 0;
      g_pumpMode = 0;
      SendMsg(MSG_PUMP_CON,g_pumpConnection,ACTION_BOTH,"pump_override","Pump connection successful.Pump manual override active");
      popupWithFmt(0,"Pump connection successful.Pump manual override active");
    }else if(code==4){
      Pmp_manual_override = false;
      pump_triggered = true;
      g_pumpState = 1;
      SendMsg(MSG_PUMP_CON,g_pumpConnection,ACTION_BOTH,"system_info","Pump connection successful.Pump is currenlty ON");
      SendMsg(MSG_PUMP_STATE,g_pumpState,ACTION_DATA);
      popupWithFmt(0,"Pump connection successful.Pump is currenlty ON");
    }else if(code ==5){
      Pmp_manual_override = false;
      pump_triggered = false;
      g_pumpState = 0;
      SendMsg(MSG_PUMP_CON,g_pumpConnection,ACTION_BOTH);
      SendMsg(MSG_PUMP_STATE,g_pumpState,ACTION_DATA);
      popupWithFmt(0,"Pump connection successful.");
    } 
    if(currpg==2){
       plinktxt.setText(connectiontext[g_pumpConnection]);
    }
  } else return;
}
bool pumptype(int start,int stop){
  if(start==stop){
      //Serial.println("Pump start and stop cannot be equal");
      return false;
  }else if(start!=stop){
    startperc = start;
    stopperc = stop;
    if(startperc > stopperc){
      emptying_fulling = true;
    }else if(startperc < stopperc){
      emptying_fulling = false;
    }
    return true;
  }
}

//tank packet handlers
void sendtankrequest(){
  if(!tankConnected || tx.active || waitingTankUpdate || !tnkcalibrated){
    //Serial.println("not applicabel to send request");
    return;
  }
  unsigned long now = millis();

  uint32_t interval = pump_triggered? 5000 :HEARTBEAT_INTERVAL;
  if(now-lastheartbeat >= interval){
    Serial.print("The interval timeout is now:");
    Serial.println(interval);
    starttransaction(nullptr,0,destinationtnk,MSG_REQ_TANKLVL);
    lastheartbeat = now;
  }
}
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
  static int Freminder =0;
  static int Ereminder =0;
  if (currpg==1){
    tlvl.setValue(g_tankPercent);
    tperctxt.setValue(g_tankPercent);
    tcaltxt.setText(calibrationtext[calibration]);
  }

  bool calibrationchanged = (calibration!=lastcalibration);
  bool valueChanged = (g_tankPercent != lastTankPercent);
  bool heartbeat = (millis() - lastCloudSend > CLOUD_HEARTBEAT);

  if(((!full_pressed&&empty_pressed)||(!full_pressed&&!empty_pressed))&&Freminder<1){
    if(g_tankPercent>=90){
      popupWithFmt(0,"Reminder to indicate when tank full level to fully calibrate system");
      Freminder =1;
    }
  }
  if(((full_pressed&&!empty_pressed)||(!full_pressed&&!empty_pressed))&&Ereminder<1){
    if(g_tankPercent<=10){
      popupWithFmt(0,"Reminder to indicate when tank empty level to fully calibrate system");
      Ereminder =1;
    }
  }
  // send only if value changed
  if(valueChanged||heartbeat){
    // Serial.println("tnak level chnaged ready to send to blynk");
    // Serial.println(valueChanged);
    // Serial.println(heartbeat);
    lastTankPercent = g_tankPercent;
    //Serial.println("Blynk connected sending queue date to core2");
    SendMsg(MSG_TANK_LEVEL,g_tankPercent,ACTION_DATA);
    }
  if(calibrationchanged){
    lastcalibration=calibration;
    SendMsg(MSG_TANK_CAL,calibration,ACTION_DATA);
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
  backupCalibration();

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
  Serial.println("inside get pump,en is good");
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

  if(state !=g_pumpState && waitingPumpUpdate){
    g_pumpState = prev_g_pumpState;
    popupWithFmt(1,"Pump didnt turn %s, plz check pump",g_pumpState?"OFF":"ON");
    SendMsg(MSG_PUMP_STATE,g_pumpState,ACTION_BOTH,"pump_unresponsive","Pump did not turn %s",g_pumpState?"OFF":"ON");
    return true;
  }else if(state ==g_pumpState && waitingPumpUpdate){
    g_pumpState = state;
    pump_triggered = (g_pumpState ==1);
    Serial.println("the data has been process it seems");
    SendMsg(MSG_PUMP_STATE,g_pumpState,ACTION_DATA);
    return true;
  }else if(!waitingPumpUpdate){
    if(man_override!=Pmp_manual_override){
      Pmp_manual_override= (mode==1);
      if(Pmp_manual_override){
        pump_triggered = false;
        g_pumpState = 0;
        g_pumpMode = 0;
      }
      Pmp_manual_override?SendMsg(0,0,ACTION_EVENT,"pump_override","Pump manual override activiated"):SendMsg(0,0,ACTION_EVENT,"system_info","Pump manual override deactiviated");
      popupWithFmt(0,"Pump manual override %s",(Pmp_manual_override ? "activated" : "deactivated"));
      return true;
    }
    g_pumpState = state;
    pump_triggered = (g_pumpState ==1);
    popupWithFmt(0,"Pump has been turned %s manually",(g_pumpState ? "ON" : "OFF"));
    SendMsg(MSG_PUMP_STATE,g_pumpState,ACTION_BOTH,"system_info","Pump has been turned %s manually",g_pumpState ? "ON" : "OFF");
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
  //ERROR = (Errortype)errotype.ERR;
  return errotype.ERR;;
  //return handleerror(errotype);
}

//LORA DATA TRANSMISSION
bool starttransaction(const void *payload,uint8_t payloadLen, byte dest, byte type){ 
  if (tx.active){ 
    //popupWithFmt(2,"Data transaction in progress plz wait a few second and try again"); 
    return false; 
  }  

  if (type != SYS_AVAILABLE) { 
    if(!isConnectedToSender(dest)){
      return false;
    } 
  }
  resetTx();
  //Serial.println("The destination is %s")
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
bool sendRAW(pendingTX &t){ 
  if(xSemaphoreTake(spiMutex, pdMS_TO_TICKS(5)) == pdTRUE){
    LoRa.beginPacket(); 
    LoRa.write(t.dest); 
    LoRa.write(localaddress); 
    LoRa.write(t.txID); 
    LoRa.write(t.type); 
    LoRa.write(t.payloadLen); 
    LoRa.write(t.payload, t.payloadLen); 
    int epRes=LoRa.endPacket(); 
    //Serial.println("data sent"); 
    t.lastsendtime = millis();
    LoRa.receive();
    xSemaphoreGive(spiMutex);
    if (epRes == 0) { // endPacket returns 0 on success in most libs; if uncertain, still log
      Serial.println("data sent (blocking)");
      return true;
    } else {
      Serial.printf("endPacket returned %d\n", epRes);
      return true; // we still transmitted — keep true for retries logic
    }
  } else {
    Serial.println("sendRAW: SPI busy, packet skipped");
    return false;
  }
}
bool sendACK(byte where, byte txID){ 
  if(xSemaphoreTake(spiMutex, pdMS_TO_TICKS(5)) == pdTRUE){
    LoRa.beginPacket(); 
    LoRa.write(where); 
    LoRa.write(localaddress); 
    LoRa.write(txID); 
    LoRa.write(MSG_ACK); 
    LoRa.write(0); 
    LoRa.endPacket(); 
    LoRa.receive(); 
    Serial.println("ACK sent");
    xSemaphoreGive(spiMutex);
    return true;
  } else {
    Serial.println("sendRAW: SPI busy, packet skipped");
    return false;
  }
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
    startTimeSync();
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
void alignMinuteUpdate(){
  int secondsRemaining = 60 - currentTime.tm_sec;
  nextClockUpdate = millis() + (secondsRemaining * 1000);
}
bool setManualTime(int year,int month,int day,int hour,int minute,int second){

  struct tm t={0};

  t.tm_year = year - 1900;
  t.tm_mon  = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min  = minute;
  t.tm_sec  = second;
  t.tm_isdst =-1;

  time_t newTime = mktime(&t);
  struct timeval now = { newTime, 0 };
  settimeofday(&now, NULL);
  currentTime = t;   // update smooth clock
  Serial.println("Manual time set");
  //showTime_date(currentTime);
  timedateset = true;
  alignMinuteUpdate();
  return true;
}
void startCheckWiFi() {
  wifiactive = false;
  wifiMode = WIFI_VERIFY_ONLY;
  wifiState = WIFI_CONNECTING;
}
void startTimeSync() {
  wifiactive = false;
  wifiMode = WIFI_SYNC_TIME;
  wifiState = WIFI_CONNECTING;
}
void startStayCon(){
  wifiactive = false;
  wifiMode = WIFI_CON_STAY;
  wifiState = WIFI_CONNECTING;
}
void handleWifiTask(){
  if(wifiState == WIFI_IDLE) return;
  wl_status_t status = WiFi.status();
  switch(wifiState){
    case WIFI_CONNECTING:
      if(ssid[0]=='\0'){
        Serial.println("No SSID set");
        if (prev_con_g_blynkstate){
          g_blynkstate = 0;
          blynkState = BLYNK_IDLE;
          //blynkState = BLYNK_DISCONNECT;
        }
        popupWithFmt(0,"No wifi credentails, plz enter them in settings");
        wifiState = WIFI_IDLE;
        wifiactive = false;
        wifigood = false;
        break;
      }
      // if(status ==WL_CONNECTED){
      //   Serial.print("Wifi already connected");
      //   wifiState = WIFI_CONNECTED;
      //   break;
      // }

      if(!wifigood){
        if(!ssidScanStarted){
          WiFi.scanNetworks(true);
          ssidScanStarted = true;
          scanResultCount = -1;
          break;
        }
      }else if(wifigood){
        Serial.print("Using fast channel,Connecting to ");
        //popupWithFmt(0,"Attempting to connect to %s",ssid);
        Serial.println(ssid);

        WiFi.begin(ssid, password, savedChannel, savedBSSID, true);

        wifiStart = millis();
        wifiState = WIFI_CONNECTED;
        break;
      }
      if(scanResultCount==-1 &&ssidScanStarted){
        scanResultCount = WiFi.scanComplete();
        if(scanResultCount==WIFI_SCAN_RUNNING){
          break;
        }
        bool found = false;
        for(int i =0;i< scanResultCount; i++){
          if(WiFi.SSID(i)==String(ssid)){
            found = true;
            break;
          }
        }
        WiFi.scanDelete();
        if(!found){
          Serial.println("SSID not available!");
          popupWithFmt(1,"%s not found", ssid);
          //wifiState = WIFI_FAIL;
          wifiactive = false;
          wifigood = false;
          ssidScanStarted = false;
          scanResultCount = -1;
          wifiState = WIFI_IDLE;
          break;
        }
        Serial.println("SSID found, retrying connection");
        popupWithFmt(0,"%s was not found retrying..",ssid);
        ssidScanStarted = false;
        scanResultCount = -1;
        Serial.print("Using nromal channel,Connecting to ");
        Serial.println(ssid);

        WiFi.begin(ssid, password);

        wifiStart = millis();
        wifiState = WIFI_CONNECTED;
      }
      break;
    case WIFI_CONNECTED:{
      if(status ==WL_CONNECTED){
        //Serial.println("Wifi connected");
        memcpy(savedBSSID, WiFi.BSSID(), 6);
        savedChannel = WiFi.channel();
        //wifiFastConnectReady = true;

        Serial.print("Saved BSSID: ");
        Serial.println(WiFi.BSSIDstr());

        Serial.print("Saved channel: ");
        Serial.println(savedChannel);
        wifigood = true;
        wifiactive = true;
        if(wifiMode == WIFI_VERIFY_ONLY){
          if(!timedateset){
            wifiMode = WIFI_SYNC_TIME;
            wifiState = WIFI_CONNECTED;
          }else{
            wifiState = WIFI_RUNNING;
            lastwifiuse = millis();
          }
          //wifiState = WIFI_RUNNING;
          popupWithFmt(0,"%s was found and connected",ssid);
        }else if(wifiMode ==WIFI_SYNC_TIME){
          Serial.println("Getting time from NTP");
          configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
          ntpRetries =0;
          wifiState = WIFI_NTP_WAIT;
        }else if(wifiMode ==WIFI_CON_STAY){
          wifiState = WIFI_RUNNING;
          lastwifiuse = millis();
        }
        xTaskNotify(
        loraTaskHandle,
        LORA_EVT_REINIT,
        eSetBits
        );
      }
      if (status == WL_CONNECT_FAILED||status==WL_CONNECTION_LOST) {
        wifiRetryCount++;
        if(wifiRetryCount >= 5){
          wifiState = WIFI_FAIL;
        }
        //popupWithFmt(1,"Connection failed, wrong password?,%d retries left",wifiRetryCount);
        break;
      }
      // connection timeout
      if(millis() - wifiStart > CONN_TIMEOUT){
        Serial.println("WiFi timeout");
        wifiState = WIFI_FAIL;
      }

      break;
    }
    case WIFI_NTP_WAIT:
      if(getLocalTime(&currentTime)){
        alignMinuteUpdate();
        timedateset = true;
        Serial.println("Time synced from NTP");
        popupWithFmt(0,"Time and date sync successful for wifi");
        wifiState = WIFI_RUNNING;
        lastwifiuse = millis();
        break;
      }
      if(++ntpRetries >10){
        Serial.println("Failed to sync time");
        popupWithFmt(0,"Time and date sync unsuccessful from wifi, input time and date in settings");
        timedateset=false;
        wifiState = WIFI_RUNNING;
        lastwifiuse = millis();
      }
      break;
    case WIFI_RUNNING:
      if(millis()-lastwifiuse >=WIFI_TIMEOUT){
        if(!blynkactive){
          Serial.println("Wifi time out,no blynk connection");
          wifiState = WIFI_DONE;
        }else{
          Serial.println("Wifi time out,but blynk connect,continuing");
          lastwifiuse = millis();
        }
      }else if(status !=WL_CONNECTED){
        Serial.println("Wifi diconnected without blnk been on");
        if(!blynkactive){
          wifiState = WIFI_IDLE;
        }
      }
      break;
    case WIFI_DONE:
      WiFi.disconnect(true);

      Serial.println("WiFi task finished");
      wifiState = WIFI_IDLE;
      wifiactive = false;
      break;
    case WIFI_FAIL:
      //WiFi.disconnect(true);
      Serial.println("WiFi task failed");
      popupWithFmt(1,"Couldn't connect to wifi ensure password is correct and sinal is good");
      if (prev_con_g_blynkstate){
        g_blynkstate = 0;
        blynkState = BLYNK_IDLE;
        //blynkState = BLYNK_DISCONNECT;
      }
      wifiState = WIFI_IDLE;
      wifiactive = false;
      wifigood = false;
      wifiRetryCount =0;
      break;
  }
  //loraFailCount = millis(); // prevent repeated triggers
}
void handleBlynk(){
  switch(blynkState){
    case BLYNK_IDLE:
      if(g_blynkstate){
        if(tx.active){
          popupWithFmt(0,"Conneting to blynk...");
          prev_g_blynkstate = g_blynkstate;
          return;
        }
        blynkState = BLYNK_CHECK_WIFI;
      }
      break;
    case BLYNK_CHECK_WIFI:
      if (!g_blynkstate){
        blynkState = BLYNK_IDLE;
        break;
      }
      if(!wifigood){
        Serial.println("checking if wifi is active, dont press blynk on again plz wait");
        //popupWithFmt(0,"failed to connect %s, ensure wifi is active or credentails are correct",ssid);
        startStayCon();
        blynkState = BLYNK_IDLE;
        prev_con_g_blynkstate = true;
        g_blynkstate = 0;
        break;
      }
      if(WiFi.status() != WL_CONNECTED){
        if (wifiState == WIFI_IDLE) {
          startStayCon();   // start WiFi connection only once
        }
        break;
      }else{
        blynkState = BLYNK_CONNECT;
      }
      break;
    case BLYNK_CONNECT:
      if (!g_blynkstate){
        blynkState = BLYNK_IDLE;
        break;
      }
      if(!blynkConfigured){
        Blynk.config(BLYNK_AUTH_TOKEN);
        blynkConfigured = true;
      }
      if (Blynk.connect(2000)){  // try connect for 2s
        Serial.println("Blynk connected");
        popupWithFmt(0,"Blynk connected succesfully");
        blynkState = BLYNK_RUNNING;
        blynkactive = true;
      }else{
        blynkState = BLYNK_IDLE;
        g_blynkstate=0;
        popupWithFmt(1,"Failed to connect to blynk");
      }
      break;
    case BLYNK_RUNNING:
      if (!g_blynkstate){
        blynkState = BLYNK_DISCONNECT;
        break;
      }
      if (WiFi.status() != WL_CONNECTED)
      {
        blynkState = BLYNK_CHECK_WIFI;
        break;
      }
      if (!Blynk.connected()){
        blynkState = BLYNK_CONNECT;
        blynkactive = false;
        popupWithFmt(1,"Blynk disconnected,attempting to reconnect..");
        break;
      }
      Blynk.run();
      break;
    case BLYNK_DISCONNECT:
      Blynk.disconnect();
      wifiState = WIFI_RUNNING;
      Serial.println("Blynk disconnected");
      blynkState = BLYNK_IDLE;
      blynkactive = false;
      blynkConfigured = false;
      g_blynkstate = 0;
      break;
  }
}
