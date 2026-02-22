#include <Wire.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <FluxGarage_RoboEyes.h>
#include <base64.h>
#include <WiFiClientSecure.h>

//  Hardware
#define SDA_PIN    22
#define SCL_PIN    19
#define OLED_ADDR  0x3C
#define BTN_LEFT   15
#define BTN_MID     2
#define BTN_RIGHT  13
#define BUZZER_PIN 17

// Dino obstacle type (must be declared early so Arduino auto-prototyper sees it)
struct DObs { float x; int8_t w,h; bool active,bird; };

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// -- RoboEyes adapter: provides Adafruit GFX API for FluxGarage_RoboEyes --
// RoboEyes calls fillRoundRect / fillTriangle which don't exist in U8g2.
// This thin wrapper maps them to U8g2 equivalents.
class RoboEyesDisplay {
public:
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C& u8g2;
  explicit RoboEyesDisplay(U8G2_SSD1306_128X64_NONAME_F_HW_I2C& d) : u8g2(d) {}

  void clearDisplay()        { u8g2.clearBuffer(); }
  void display()             { u8g2.sendBuffer(); }
  int16_t width()  const     { return 128; }
  int16_t height() const     { return 64;  }

  void fillRect(int16_t x,int16_t y,int16_t w,int16_t h,uint16_t color){
    u8g2.setDrawColor(color?1:0); u8g2.drawBox(x,y,w,h); }
  void drawRect(int16_t x,int16_t y,int16_t w,int16_t h,uint16_t color){
    u8g2.setDrawColor(color?1:0); u8g2.drawFrame(x,y,w,h); }
  void fillRoundRect(int16_t x,int16_t y,int16_t w,int16_t h,int16_t r,uint16_t color){
    u8g2.setDrawColor(color?1:0); u8g2.drawRBox(x,y,w,h,r); }
  void drawRoundRect(int16_t x,int16_t y,int16_t w,int16_t h,int16_t r,uint16_t color){
    u8g2.setDrawColor(color?1:0); u8g2.drawRFrame(x,y,w,h,r); }
  void drawLine(int16_t x0,int16_t y0,int16_t x1,int16_t y1,uint16_t color){
    u8g2.setDrawColor(color?1:0); u8g2.drawLine(x0,y0,x1,y1); }
  void drawPixel(int16_t x,int16_t y,uint16_t color){
    u8g2.setDrawColor(color?1:0); u8g2.drawPixel(x,y); }
  void fillCircle(int16_t x,int16_t y,int16_t r,uint16_t color){
    u8g2.setDrawColor(color?1:0); u8g2.drawDisc(x,y,r); }
  void drawCircle(int16_t x,int16_t y,int16_t r,uint16_t color){
    u8g2.setDrawColor(color?1:0); u8g2.drawCircle(x,y,r); }
  void drawTriangle(int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t x2,int16_t y2,uint16_t color){
    u8g2.setDrawColor(color?1:0); u8g2.drawTriangle(x0,y0,x1,y1,x2,y2); }
  void fillTriangle(int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t x2,int16_t y2,uint16_t color){
    u8g2.setDrawColor(color?1:0);
    // scanline fill (U8g2 has no fillTriangle)
    if(y0>y1){int16_t t;t=y0;y0=y1;y1=t;t=x0;x0=x1;x1=t;}
    if(y0>y2){int16_t t;t=y0;y0=y2;y2=t;t=x0;x0=x2;x2=t;}
    if(y1>y2){int16_t t;t=y1;y1=y2;y2=t;t=x1;x1=x2;x2=t;}
    int16_t totalH=y2-y0; if(totalH==0)return;
    for(int16_t i=0;i<totalH;i++){
      bool sec=(i>y1-y0)||(y1==y0);
      int16_t segH=sec?(y2-y1):(y1-y0); if(segH==0)continue;
      float alpha=(float)i/totalH;
      float beta=(float)(i-(sec?y1-y0:0))/segH;
      int16_t ax=x0+(int16_t)((x2-x0)*alpha);
      int16_t bx=sec?x1+(int16_t)((x2-x1)*beta):x0+(int16_t)((x1-x0)*beta);
      if(ax>bx){int16_t t=ax;ax=bx;bx=t;}
      u8g2.drawHLine(ax,y0+i,bx-ax+1);
    }
  }
  // Stubs for methods RoboEyes may call on the font/text side
  void setTextSize(uint8_t)    {}
  void setTextColor(uint16_t)  {}
  void setCursor(int16_t,int16_t) {}
  void print(const char*)      {}
};

RoboEyesDisplay roboEyesDisplay(display);
RoboEyes<RoboEyesDisplay> roboEyes(roboEyesDisplay);

// -- U8g2 helpers --
void setDisplayFont(uint8_t sz) {
  switch(sz) {
    case 0: display.setFont(u8g2_font_5x7_tr);        break;
    case 2: display.setFont(u8g2_font_9x18_tr);       break;
    case 3: display.setFont(u8g2_font_logisoso20_tr);  break;
    default:display.setFont(u8g2_font_6x13_tr);       break;
  }
  display.setFontPosTop();
}

// Standalone scanline fillTriangle (for app code, uses display directly)
void u8g2FillTriangle(int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t x2,int16_t y2){
  if(y0>y1){int16_t t=y0;y0=y1;y1=t;t=x0;x0=x1;x1=t;}
  if(y0>y2){int16_t t=y0;y0=y2;y2=t;t=x0;x0=x2;x2=t;}
  if(y1>y2){int16_t t=y1;y1=y2;y2=t;t=x1;x1=x2;x2=t;}
  int16_t totalH=y2-y0; if(totalH==0)return;
  for(int16_t i=0;i<totalH;i++){
    bool sec=(i>y1-y0)||(y1==y0);
    int16_t segH=sec?(y2-y1):(y1-y0); if(segH==0)continue;
    float alpha=(float)i/totalH;
    float beta =(float)(i-(sec?y1-y0:0))/segH;
    int16_t ax=x0+(int16_t)((x2-x0)*alpha);
    int16_t bx=sec?x1+(int16_t)((x2-x1)*beta):x0+(int16_t)((x1-x0)*beta);
    if(ax>bx){int16_t t=ax;ax=bx;bx=t;}
    display.drawHLine(ax,y0+i,bx-ax+1);
  }
}



//  WiFi
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const double LATITUDE  = 0.00;  // Your latitude (e.g. 51.50 for London)
const double LONGITUDE = 0.00;  // Your longitude (e.g. -0.12 for London)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000);

//  EEPROM
#define EEPROM_SZ 48
#define EE_MAG0   0xAB
#define EE_MAG1   0xD1

//  Screens
int  currentScreen  = 0;
const int TOTAL_SCREENS = 11;

//  Preferences
bool    useCelsius = true;
bool    use24Hour  = true;
bool    soundOn    = true;
bool    autoDimOn  = true;
bool    autoCycleOn= true;
uint8_t brightIdx  = 3;
const uint8_t BRIGHT[] = {1, 5, 30, 120, 255};  // [0]=sleep-safe dim
int     settCursor = 0;

//  Sleep (invisible, 5-min inactivity)
bool          sleeping     = false;
unsigned long lastActivity = 0;
const unsigned long SLEEP_MS = 300000UL;

//  Weather
float  temperature = 0;  int  weatherCode = 0;
float  windSpeed   = 0;  int  humidity    = 0;
String sunsetTime  = "";
unsigned long lastWeatherUpdate = 0;
const unsigned long WX_AWAKE = 600000UL;
const unsigned long WX_SLEEP = 3600000UL;

//  Quote
String quoteText = "", quoteAuthor = "";
unsigned long lastQuoteUpdate = 0;
const unsigned long QUOTE_IV = 30000UL;

//  Alarm
bool alarmEnabled=false; int alarmHour=7, alarmMinute=0;
bool alarmRinging=false; unsigned long alarmRingStart=0;
int  alarmCursor=0;

//  Auto-cycle screens 0/1/5
unsigned long lastCycleTime = 0;
const unsigned long CYCLE_IV = 300000UL;
int autoCycleScreens[]={0,1,5}; int autoCycleIdx=0;

//  Timer / Stopwatch
unsigned long holdMoveT=0;
unsigned long swStart=0; bool swRunning=false; unsigned long swElapsed=0;

//  Buttons
const unsigned long LP_MS = 700;
unsigned long lpL=0,lpM=0,lpR=0;
bool bL=false,bM=false,bR=false;
bool llL=false,llM=false,llR=false;
unsigned long lrHoldStart=0; bool lrHolding=false;

//  Eyes reaction
uint8_t eyeBtn=0; unsigned long eyeBtnTime=0;

//  Peek-eyes (screens 0 & 1)
uint8_t  peekPhase=0;   // 0=idle 1=rising 2=showing 3=falling
int      peekWipeH=0;
unsigned long peekTimer=0, nextPeekAt=0;

//  Spotify (USB-Serial bridge)
// ONE-TIME: run get_refresh_token.py once, paste token into SP_REFRESH
const char* SP_ID      = "YOUR_SPOTIFY_CLIENT_ID";
const char* SP_SECRET  = "YOUR_SPOTIFY_CLIENT_SECRET";
const char* SP_REFRESH = "YOUR_SPOTIFY_REFRESH_TOKEN";
String spToken=""; unsigned long spTokenExp=0;
const unsigned long SP_POLL = 3000UL;
SemaphoreHandle_t spMutex;
bool   spConn=false, spPlay=false;
String spTitle="", spArtist="";
float  lpSpin=0.0f;
int    spOff=0;
unsigned long spScrollT=0;
unsigned long spEnteredAt=0; // grace period on entry


// -- Crypto prices --
// Asset 0,1: Kraken free public API (no key)
// Asset 2:   Yahoo Finance (SPY as SPYx proxy, no key)
const char* CR_PAIRS[]  = {"XBTEUR","PAXGUSD","SPY"};
const char* CR_NAMES[]  = {"BTC/EUR","PAXG/USD","SPYx/USD"};
const char* CR_UNITS[]  = {"e","$","$"};
const int   CR_COUNT    = 3;
int    crSel = 0;

struct CrCandle{ float o,h,l,c; };
struct CrAsset {
  CrCandle candles[20];
  int      count;
  float    price;
  bool     loaded;
  unsigned long lastUpd;
};
CrAsset crAssets[3];
const unsigned long CR_IV = 300000UL; // 5 min

//
//  GAMES
//
const int TOTAL_GAMES = 15;
const char* const gameNames[] PROGMEM = {
  "Flappy Bird","Snake","Pong","Breakout","Asteroids",
  "Dino Run","Memory","Shooter","Reaction","Guess",
  "Catch","Dodge","Tunnel","Reflex","Whack-a-Mole"
};
int  gHi[15];
int  selectedGame=0; bool gameOver=true; bool gamePaused=false;
bool inGameOverScreen=false; int gameOverSel=0;
int gameCntdown=0; unsigned long gameCntdownT=0;

//  Flappy
float fbBirdY=32,fbBirdVel=0;
const float FB_GRAV=0.42f,FB_FLAP=-4.6f;
int fbPipeX=130,fbGapY=18; float fbPipeSpd=2.2f;
const int FB_PW=12,FB_BS=5,FB_GAP=27; int fbScore=0;
int fbCnt=3; unsigned long fbCntT=0; bool fbStarted=false;

//  Snake
const int SN_CELL=4,SN_MAX=120;
int snX[SN_MAX],snY[SN_MAX],snLen=3,snFX=64,snFY=32,snDir=0,snNext=0;
unsigned long lastSnakeMove=0; int snSpd=170,snScore=0;

//  Pong
const int PN_PH=16,PN_PX=121;
int pnPY=24; float pnBX=64,pnBY=32,pnBVX=-2.2f,pnBVY=1.3f; int pnScore=0;

//  Breakout
const int BK_COLS=8,BK_ROWS=4,BK_W=15,BK_H=5;
int bkBricks[BK_COLS][BK_ROWS],bkPadX=54;
float bkBX=64,bkBY=50,bkBVX=2.0f,bkBVY=-2.0f;
int bkScore=0,bkLevel=1;

//  Asteroids
float asShipX=64,asShipY=32,asAngle=0,asVX=0,asVY=0;
int asX[8],asY[8],asR[8]; bool asAlive[8];
float asBX[5],asBY[5],asBVX[5],asBVY[5]; bool asBAlive[5];
int asScore=0; unsigned long asLastShot=0,asLastTick=0;

//  Dino (multi-obs)
const int DN_GND=52, DN_MAX=3;
DObs dnO[DN_MAX];
float dnY=DN_GND-10.0f,dnVY=0,dnSpd=3.0f;
bool dnJump=false; int dnScore=0;

//  Memory
int mmSeq[20],mmSeqLen=3,mmSeqIdx=0; bool mmAwaiting=false;
unsigned long mmBlinkT=0; int mmBlinkIdx=0; bool mmDone=true; int mmScore=0;
bool mmGap=false; unsigned long mmGapT=0;

//  Shooter
const int SS_E=6,SS_B=6; int ssShipX=60;
int ssEX[SS_E],ssEY[SS_E]; bool ssEA[SS_E];
int ssBX[SS_B],ssBY[SS_B]; bool ssBA[SS_B];
int ssScore=0; unsigned long ssLastMove=0;

//  Reaction
bool rxActive=false,rxDone=false;
unsigned long rxShowAt=0,rxStart=0,rxResult=0;

//  Guess
int guSecret=50,guGuess=50; bool guDone=false; String guHint=""; int guAtt=0;

//  Catch
int ctPadX=50,ctIX[4],ctIY[4]; bool ctIOn[4];
int ctScore=0,ctLives=3; unsigned long ctSpawnT=0;

//  Dodge
int dgX=60,dgBX[6],dgBY[6],dgBW[6]; bool dgBOn[6];
int dgScore=0; unsigned long dgSpawnT=0; float dgSpd=2.0f;

//  Tunnel
float tnY=32,tnVY=0,tnPhase=0; int tnScore=0; float tnSpd=0.06f;
unsigned long tnTick=0;

//  Reflex
float rfX=10; int rfDir=1; float rfSpd=1.5f;
int rfScore=0,rfRound=0; bool rfWaiting=false; unsigned long rfWaitT=0;

//  Whack
int wkHole=1; bool wkVisible=false;
unsigned long wkShowT=0,wkHideT=0;
int wkScore=0,wkMissed=0; float wkWindow=1200.0f;
int wkBlinkPhase=0; unsigned long wkBlinkT=0; int wkPrevHole=-1;

//
//  FORWARD DECLARATIONS
//
void handleButtons(),handleLeftShort(),handleRightShort(),handleMidShort();
void handleLeftLong(),handleRightLong(),handleMidLong();
void beepShort(),beepLong(),beepSuccess(),beepFail(),beepFlap(),beepAlarm();
void checkAlarm(),drawScreen(),wakeActivity(),applyBrightness();
void drawClockWeather(),drawDetailedWeather(),drawGame(),drawAlarm();
void drawStats(),drawQuote(),drawTimer(),drawSettings(),drawSpotify();
void drawGameOverScreen();
void drawFlappy(),drawSnake(),drawPong(),drawBreakout(),drawAsteroids();
void drawDino(),drawMemory(),drawShooter(),drawReaction(),drawGuess();
void drawCatch(),drawDodge(),drawTunnel(),drawReflex(),drawWhack();
void updateFlappy(),updateSnake(),updatePong(),updateBreakout(),updateAsteroids();
void updateDino(),updateMemory(),updateShooter(),updateReaction();
void updateCatch(),updateDodge(),updateTunnel(),updateReflex(),updateWhack();
void resetGame();
void initAsteroids(),initShooter(),initMemory(),initDino();
void initCatch(),initDodge(),initTunnel(),initReflex(),initWhack();
void handleMemInput(int);
void loadEEPROM(),saveEEPROM(),saveHiScore(int);
String fmtTime(),fmtDate(); float displayTemp(); String weatherCond();
void drawWeatherIcon(int,int,int),fetchWeather(),fetchQuote();
void drawWrappedText(String,int,int,int,int,int);
void drawCenteredText(String,int,uint8_t);
void spControl(const char*),schedulePeek(),updatePeek();
void drawCrypto(),fetchCrypto();
void drawPeekCurtain(),drawFullPeekEyes();
void drawLPRecord(int,int,int,float);
void drawDinoObs(const DObs&);
void spawnDinoObs();

//
//  SETUP
//

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN,SCL_PIN);
  EEPROM.begin(EEPROM_SZ);
  delay(120);
  if(!display.begin()){while(true)delay(100);}
  setDisplayFont(1); // set default font
  display.setDrawColor(1);
  display.clearBuffer(); display.sendBuffer(); delay(100);

  roboEyes.begin(128,64,60);
  roboEyes.setAutoblinker(ON,3,2);
  roboEyes.setIdleMode(ON,2,2);

  pinMode(BTN_LEFT,INPUT_PULLUP);
  pinMode(BTN_MID,INPUT_PULLUP);
  pinMode(BTN_RIGHT,INPUT_PULLUP);
  pinMode(BUZZER_PIN,OUTPUT);
  digitalWrite(BUZZER_PIN,LOW);

  loadEEPROM(); applyBrightness();

  display.setDrawColor(1);
  drawCenteredText(F("DeskMate Mini"),12,2);
  drawCenteredText(F("Smart Display"),32,1);
  drawCenteredText(F("Connecting..."),46,1);
  display.sendBuffer();

  WiFi.begin(ssid,password);
  for(int i=0;i<20&&WiFi.status()!=WL_CONNECTED;i++) delay(500);
  if(WiFi.status()==WL_CONNECTED){
    timeClient.begin(); timeClient.update();
    fetchWeather(); fetchQuote();
    display.clearBuffer();
    drawCenteredText(F("WiFi OK"),28,1); display.sendBuffer(); delay(500);
  }

  spMutex=xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(spotifyTaskFn,"spotify",8192,NULL,1,NULL,0);
  randomSeed(analogRead(34));
  lastActivity=millis(); schedulePeek();
  initMemory(); initShooter(); initDino();
  initCatch(); initDodge(); initTunnel(); initReflex(); initWhack();
  display.clearBuffer(); display.sendBuffer();
}

//
//  LOOP
//
void loop() {
  handleButtons();
  unsigned long now=millis();

  if(WiFi.status()==WL_CONNECTED){
    timeClient.update();
    unsigned long wiv=sleeping?WX_SLEEP:WX_AWAKE;
    if(now-lastWeatherUpdate>wiv) fetchWeather();
    if(!sleeping&&currentScreen==5&&now-lastQuoteUpdate>QUOTE_IV) fetchQuote();
    if(currentScreen==10&&(!crAssets[crSel].loaded||now-crAssets[crSel].lastUpd>CR_IV)) fetchCrypto();
  }
  if(autoCycleOn&&(currentScreen==0||currentScreen==1||currentScreen==5)){
    if(now-lastCycleTime>CYCLE_IV){
      autoCycleIdx=(autoCycleIdx+1)%3;
      currentScreen=autoCycleScreens[autoCycleIdx];
      lastCycleTime=now;
    }
  }
  checkAlarm();

  // Invisible sleep
  if(!alarmRinging){
    bool sh=autoDimOn&&(now-lastActivity>SLEEP_MS);
    if(sh&&!sleeping){sleeping=true;display.setContrast(1);}
    if(!sh&&sleeping){sleeping=false;applyBrightness();}
  }

  if(!sleeping) updatePeek();

  // Game tick
  if(currentScreen==2&&!gameOver&&!gamePaused&&!inGameOverScreen&&gameCntdown==0){
    switch(selectedGame){
      case 0:updateFlappy();break; case 1:updateSnake();break;
      case 2:updatePong();break;   case 3:updateBreakout();break;
      case 4:updateAsteroids();break; case 5:updateDino();break;
      case 6:updateMemory();break; case 7:updateShooter();break;
      case 8:updateReaction();break;
      case 10:updateCatch();break; case 11:updateDodge();break;
      case 12:updateTunnel();break; case 13:updateReflex();break;
      case 14:updateWhack();break;
    }
  }

  // LP spin
  if(currentScreen==9&&spPlay){
    lpSpin+=0.07f; if(lpSpin>6.283f) lpSpin-=6.283f;
  }
  // Title scroll
  if(currentScreen==9&&spPlay&&now-spScrollT>250){
    spScrollT=now;
    int mx=max(0,(int)spTitle.length()-12);
    spOff=(spOff+1)%(mx+8);
  }

  if(gameCntdown>0&&millis()-gameCntdownT>=1000){gameCntdown--;gameCntdownT=millis();}
  drawScreen();
  delay(16);
}

//
//  EEPROM
//
void loadEEPROM(){
  if(EEPROM.read(0)!=EE_MAG0||EEPROM.read(1)!=EE_MAG1){
    for(int i=0;i<15;i++)gHi[i]=0; gHi[8]=9999; saveEEPROM(); return;
  }
  for(int i=0;i<15;i++) gHi[i]=EEPROM.read(2+i*2)|(EEPROM.read(3+i*2)<<8);
  alarmHour=EEPROM.read(32); alarmMinute=EEPROM.read(33);
  alarmEnabled=EEPROM.read(34); useCelsius=EEPROM.read(35);
  use24Hour=EEPROM.read(36); soundOn=EEPROM.read(37);
  brightIdx=EEPROM.read(38); if(brightIdx>4) brightIdx=3;
  autoDimOn=EEPROM.read(39); autoCycleOn=EEPROM.read(40);
}
void saveEEPROM(){
  EEPROM.write(0,EE_MAG0); EEPROM.write(1,EE_MAG1);
  for(int i=0;i<15;i++){EEPROM.write(2+i*2,gHi[i]&0xFF);EEPROM.write(3+i*2,(gHi[i]>>8)&0xFF);}
  EEPROM.write(32,alarmHour);EEPROM.write(33,alarmMinute);
  EEPROM.write(34,alarmEnabled);EEPROM.write(35,useCelsius);
  EEPROM.write(36,use24Hour);EEPROM.write(37,soundOn);
  EEPROM.write(38,brightIdx); EEPROM.write(39,autoDimOn); EEPROM.write(40,autoCycleOn); EEPROM.commit();
}
void saveHiScore(int i){
  EEPROM.write(2+i*2,gHi[i]&0xFF); EEPROM.write(3+i*2,(gHi[i]>>8)&0xFF);
  EEPROM.commit();
}

//
//  BUZZER
//
void beepShort()  {if(soundOn)tone(BUZZER_PIN,2000,50);}
void beepLong()   {if(soundOn)tone(BUZZER_PIN,1500,120);}
void beepSuccess(){if(soundOn){tone(BUZZER_PIN,2500,70);delay(90);tone(BUZZER_PIN,3200,70);}}
void beepFail()   {if(soundOn)tone(BUZZER_PIN,700,250);}
void beepFlap()   {if(soundOn)tone(BUZZER_PIN,1800,30);}
void beepAlarm()  {tone(BUZZER_PIN,2800,200);}  // always rings

//
//  BRIGHTNESS / WAKE
//
void applyBrightness(){display.setContrast(BRIGHT[brightIdx]);}
void wakeActivity(){
  lastActivity=millis();
  if(sleeping){sleeping=false;applyBrightness();}
}

//
//  ALARM
//
void checkAlarm(){
  if(!alarmEnabled||WiFi.status()!=WL_CONNECTED) return;
  int h=timeClient.getHours(),m=timeClient.getMinutes();
  static int lastMin=-1;
  if(h==alarmHour&&m==alarmMinute&&lastMin!=m){
    alarmRinging=true; alarmRingStart=millis(); lastMin=m;
    sleeping=false; applyBrightness();
  }
  if(m!=alarmMinute) lastMin=-1;
  if(alarmRinging){
    if((millis()-alarmRingStart)%900<450) beepAlarm();
    if(millis()-alarmRingStart>60000){alarmRinging=false;alarmEnabled=false;}
  }
}

//
//  SERIAL SPOTIFY BRIDGE
//
bool spGetToken(){
  if(spToken.length()>0&&millis()<spTokenExp) return true;
  WiFiClientSecure cl; cl.setInsecure();
  HTTPClient http;
  http.begin(cl,"https://accounts.spotify.com/api/token");
  http.addHeader("Content-Type","application/x-www-form-urlencoded");
  String cred=base64::encode(String(SP_ID)+":"+SP_SECRET);
  http.addHeader("Authorization","Basic "+cred);
  String body="grant_type=refresh_token&refresh_token=";
  body+=SP_REFRESH;
  int rc=http.POST(body);
  if(rc==200){
    DynamicJsonDocument doc(512); deserializeJson(doc,http.getString());
    spToken=doc["access_token"].as<String>();
    spTokenExp=millis()+(doc["expires_in"].as<unsigned long>()-60)*1000;
    http.end(); return true;
  }
  http.end(); return false;
}

// Runs on core 0 - never blocks the display loop
void spotifyTaskFn(void*){
  while(true){
    if(WiFi.status()==WL_CONNECTED && spGetToken()){
      WiFiClientSecure cl; cl.setInsecure();
      HTTPClient http;
      http.begin(cl,"https://api.spotify.com/v1/me/player?market=from_token");
      http.addHeader("Authorization","Bearer "+spToken);
      int rc=http.GET();
      bool conn=false; bool play=false; String t=""; String a="";
      if(rc==200){
        DynamicJsonDocument doc(2048); deserializeJson(doc,http.getString());
        play=doc["is_playing"].as<bool>();
        t=doc["item"]["name"].as<String>();
        a=doc["item"]["artists"][0]["name"].as<String>();
        conn=true;
      } else if(rc==204){
        conn=true;
      }
      http.end();
      if(xSemaphoreTake(spMutex,pdMS_TO_TICKS(200))==pdTRUE){
        if(t.length()>0&&t!=spTitle){spOff=0;spScrollT=millis()+2000;}
        spConn=conn; spPlay=play;
        if(t.length()>0){spTitle=t;spArtist=a;}
        else if(!conn){spTitle="";spArtist="";}
        xSemaphoreGive(spMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void spControl(const char* action){
  if(WiFi.status()!=WL_CONNECTED||!spGetToken()) return;
  WiFiClientSecure cl; cl.setInsecure();
  HTTPClient http;
  String url="https://api.spotify.com/v1/me/player/";
  String auth="Bearer "+spToken;
  if(strcmp(action,"next")==0){
    url+="next";
    http.begin(cl,url);
    http.addHeader("Authorization",auth);
    http.addHeader("Content-Type","application/json");
    http.POST("{}");
  } else if(strcmp(action,"prev")==0){
    url+="previous";
    http.begin(cl,url);
    http.addHeader("Authorization",auth);
    http.addHeader("Content-Type","application/json");
    http.POST("{}");
  } else {
    if(xSemaphoreTake(spMutex,pdMS_TO_TICKS(200))==pdTRUE){
      url+=(spPlay?"pause":"play");
      spPlay=!spPlay; // optimistic flip
      xSemaphoreGive(spMutex);
    }
    http.begin(cl,url);
    http.addHeader("Authorization",auth);
    http.addHeader("Content-Type","application/json");
    http.PUT("{}");
  }
  http.end();
}

void fetchSpotify(){ /* polling handled by background task */ }

//
//  PEEK EYES
//
void schedulePeek(){nextPeekAt=millis()+random(90000UL,180000UL);}

void updatePeek(){
  if(currentScreen!=0&&currentScreen!=1){
    if(peekPhase){peekPhase=0;peekWipeH=0;schedulePeek();}
    return;
  }
  unsigned long n=millis();
  switch(peekPhase){
    case 0: if(n>=nextPeekAt){peekPhase=1;peekWipeH=0;peekTimer=n;} break;
    case 1: peekWipeH=(n-peekTimer)*64/500;
            if(peekWipeH>=64){peekWipeH=64;peekPhase=2;peekTimer=n;} break;
    case 2: if(n-peekTimer>3000){peekPhase=3;peekTimer=n;peekWipeH=64;} break;
    case 3: peekWipeH=64-(n-peekTimer)*64/400;
            if(peekWipeH<=0){peekWipeH=0;peekPhase=0;schedulePeek();} break;
  }
}

void drawPeekCurtain(){
  const int EY=56;
  int cy=64-peekWipeH;
  display.setDrawColor(0); display.drawBox(0,cy,128,peekWipeH);
  display.setDrawColor(1); display.drawLine(0,cy,127,cy);
  if(cy<EY+8){
    bool blink=(millis()%5000<150);
    if(!blink){
      display.setDrawColor(1); display.drawDisc(38,EY,7);
      display.setDrawColor(1); display.drawDisc(90,EY,7);
      display.setDrawColor(0); display.drawDisc(38,EY-2,3);
      display.setDrawColor(0); display.drawDisc(90,EY-2,3);
      display.setDrawColor(1); display.drawDisc(41,EY-4,1);
      display.setDrawColor(1); display.drawDisc(93,EY-4,1);
    } else {
      display.setDrawColor(1); display.drawHLine(31,EY,15);
      display.setDrawColor(1); display.drawHLine(83,EY,15);
    }
  }
}

void drawFullPeekEyes(){roboEyes.setMood(DEFAULT);roboEyes.update();}

//
//  BUTTONS
//
void handleButtons(){
  bool l=!digitalRead(BTN_LEFT);
  bool m=!digitalRead(BTN_MID);
  bool r=!digitalRead(BTN_RIGHT);
  if(alarmRinging&&(l||m||r)){alarmRinging=false;alarmEnabled=false;delay(200);wakeActivity();return;}
  if(sleeping){if(l||m||r){wakeActivity();delay(200);}return;}
  // Clear stale button state on screen change (prevents action bleed-in)
  static int _prevScr=-1;
  if(currentScreen!=_prevScr){bL=false;bM=false;bR=false;llL=false;llM=false;llR=false;if(currentScreen==9)spEnteredAt=millis();_prevScr=currentScreen;return;}

  // -- Spotify screen: custom button scheme --
  if(currentScreen==9){
    // M+L  prev screen  |  M+R  next screen
    if(m&&l){
      if(!lrHolding){lrHolding=true;lrHoldStart=millis();}
      else if(millis()-lrHoldStart>=400){
        currentScreen=(currentScreen-1+TOTAL_SCREENS)%TOTAL_SCREENS;
        lastCycleTime=millis();beepLong();lrHolding=false;
        bL=false;bM=false;llL=true;llM=true; // suppress release actions
      }
      return;
    }
    if(m&&r){
      if(!lrHolding){lrHolding=true;lrHoldStart=millis();}
      else if(millis()-lrHoldStart>=400){
        currentScreen=(currentScreen+1)%TOTAL_SCREENS;
        lastCycleTime=millis();beepLong();lrHolding=false;
        bR=false;bM=false;llR=true;llM=true;
      }
      return;
    }
    lrHolding=false;
    // Song controls only after 3s grace period
    bool spReady=(millis()-spEnteredAt>3000);
    if(!l&&bL){if(spReady&&spConn)spControl("prev");bL=false;if(spReady)beepShort();wakeActivity();return;}
    if(!m&&bM){if(spReady&&spConn)spControl("pause");bM=false;if(spReady)beepShort();wakeActivity();return;}
    if(!r&&bR){if(spReady&&spConn)spControl("next");bR=false;if(spReady)beepShort();wakeActivity();return;}
    if(l&&!bL){bL=true;wakeActivity();}
    if(!l)bL=false;
    if(m&&!bM){bM=true;wakeActivity();}
    if(!m)bM=false;
    if(r&&!bR){bR=true;wakeActivity();}
    if(!r)bR=false;
    return;
  }

  // -- All other screens --
  // L+R hold 3s exits game
  if(currentScreen==2&&!gameOver&&l&&r){
    if(!lrHolding){lrHolding=true;lrHoldStart=millis();}
    else if(millis()-lrHoldStart>=3000){
      gameOver=true;inGameOverScreen=false;gamePaused=false;
      lrHolding=false;beepLong();return;
    }
  } else lrHolding=false;
  if(l&&!bL){bL=true;lpL=millis();llL=false;handleLeftShort();wakeActivity();}
  if(bL&&l&&!llL&&millis()-lpL>=LP_MS){handleLeftLong();llL=true;wakeActivity();}
  if(!l)bL=false;
  if(m&&!bM){bM=true;lpM=millis();llM=false;handleMidShort();wakeActivity();}
  if(bM&&m&&!llM&&millis()-lpM>=LP_MS){handleMidLong();llM=true;wakeActivity();}
  if(!m)bM=false;
  if(r&&!bR){bR=true;lpR=millis();llR=false;handleRightShort();wakeActivity();}
  if(bR&&r&&!llR&&millis()-lpR>=LP_MS){handleRightLong();llR=true;wakeActivity();}
  if(!r)bR=false;
}

//  helper: dino jump velocity (scales with speed)
inline float dnJumpV(){return -(4.2f+dnSpd*0.18f);}

void handleLeftShort(){
  if(currentScreen==7){eyeBtn=1;eyeBtnTime=millis();}
  if(currentScreen==2){
    if(inGameOverScreen){gameOverSel=0;beepShort();return;}
    if(gameOver){selectedGame=(selectedGame-1+TOTAL_GAMES)%TOTAL_GAMES;beepShort();return;}
    if(!gamePaused) switch(selectedGame){
      case 0:fbBirdVel=FB_FLAP;beepFlap();break;
      case 1:snNext=(snDir+3)%4;beepShort();break;
      case 2:pnPY=max(3,pnPY-5);break;
      case 3:bkPadX=max(3,bkPadX-7);break;
      case 4:asAngle-=0.25f;break;
      case 5:if(!dnJump){dnVY=dnJumpV();dnJump=true;beepShort();}break;
      case 6:if(mmAwaiting)handleMemInput(0);break;
      case 7:ssShipX=max(4,ssShipX-6);break;
      case 8:if(rxActive){rxResult=millis()-rxStart;rxActive=false;rxDone=true;
               if(!gHi[8]||(int)rxResult<gHi[8]){gHi[8]=(int)rxResult;saveHiScore(8);}
               beepSuccess();}
             else if(rxDone){rxDone=false;rxActive=false;rxShowAt=millis()+random(1500,5000);}
             break;
      case 9:guGuess=max(5,guGuess-5);break;
      case 10:ctPadX=max(4,ctPadX-7);break;
      case 11:dgX=max(6,dgX-8);break;
      case 12:tnVY-=0.8f;break;
      case 13:if(!rfWaiting){int d=abs((int)rfX-64);rfScore+=max(0,30-d);
               rfRound++;if(rfRound>=5){if(rfScore>gHi[13]){gHi[13]=rfScore;saveHiScore(13);}
               gameOver=true;inGameOverScreen=true;beepSuccess();}
               else{rfWaiting=true;rfWaitT=millis();}beepShort();}break;
      case 14:if(wkVisible&&wkHole==0){wkScore++;wkVisible=false;
               if(wkScore>gHi[14]){gHi[14]=wkScore;saveHiScore(14);}
               beepSuccess();wkWindow=max(400.0f,wkWindow-30.0f);}
              else if(wkVisible){wkMissed++;if(wkMissed>=3){gameOver=true;inGameOverScreen=true;beepFail();}}
              break;
    } else{gamePaused=false;beepShort();}
  } else if(currentScreen==3){
    if(alarmCursor==0)alarmHour=(alarmHour-1+24)%24;
    else if(alarmCursor==1)alarmMinute=(alarmMinute-5+60)%60;
    beepShort();
  } else if(currentScreen==8){settCursor=(settCursor-1+7)%7;beepShort();}
  else if(currentScreen==10){crSel=(crSel-1+CR_COUNT)%CR_COUNT;crAssets[crSel].lastUpd=0;beepShort();}
}

void handleRightShort(){
  if(currentScreen==7){eyeBtn=3;eyeBtnTime=millis();}
  if(currentScreen==2){
    if(inGameOverScreen){gameOverSel=1;beepShort();return;}
    if(gameOver){selectedGame=(selectedGame+1)%TOTAL_GAMES;beepShort();return;}
    if(!gamePaused) switch(selectedGame){
      case 0:gamePaused=!gamePaused;beepShort();break;
      case 1:snNext=(snDir+1)%4;beepShort();break;
      case 2:pnPY=min(63-PN_PH,pnPY+5);break;
      case 3:bkPadX=min(104,bkPadX+7);break;
      case 4:asAngle+=0.25f;break;
      case 5:if(!dnJump){dnVY=dnJumpV();dnJump=true;beepShort();}break;
      case 6:if(mmAwaiting)handleMemInput(2);break;
      case 7:ssShipX=min(116,ssShipX+6);break;
      case 8:if(rxActive){rxResult=millis()-rxStart;rxActive=false;rxDone=true;
               if(!gHi[8]||(int)rxResult<gHi[8]){gHi[8]=(int)rxResult;saveHiScore(8);}
               beepSuccess();}
             else if(rxDone){rxDone=false;rxActive=false;rxShowAt=millis()+random(1500,5000);}
             break;
      case 9:guGuess=min(100,guGuess+5);break;
      case 10:ctPadX=min(100,ctPadX+7);break;
      case 11:dgX=min(122,dgX+8);break;
      case 12:tnVY+=0.8f;break;
      case 13:if(!rfWaiting){int d=abs((int)rfX-64);rfScore+=max(0,30-d);
               rfRound++;if(rfRound>=5){if(rfScore>gHi[13]){gHi[13]=rfScore;saveHiScore(13);}
               gameOver=true;inGameOverScreen=true;beepSuccess();}
               else{rfWaiting=true;rfWaitT=millis();}beepShort();}break;
      case 14:if(wkVisible&&wkHole==2){wkScore++;wkVisible=false;
               if(wkScore>gHi[14]){gHi[14]=wkScore;saveHiScore(14);}
               beepSuccess();wkWindow=max(400.0f,wkWindow-30.0f);}
              else if(wkVisible){wkMissed++;if(wkMissed>=3){gameOver=true;inGameOverScreen=true;beepFail();}}
              break;
    } else{gamePaused=false;beepShort();}
  } else if(currentScreen==3){
    if(alarmCursor==0)alarmHour=(alarmHour+1)%24;
    else if(alarmCursor==1)alarmMinute=(alarmMinute+5)%60;
    else if(alarmCursor==2){alarmEnabled=!alarmEnabled;saveEEPROM();}
    beepShort();
  } else if(currentScreen==8){settCursor=(settCursor+1)%7;beepShort();}
  else if(currentScreen==10){crSel=(crSel+1)%CR_COUNT;crAssets[crSel].lastUpd=0;beepShort();}
}

void handleMidShort(){
  if(currentScreen==7){eyeBtn=2;eyeBtnTime=millis();}
  if(currentScreen==2){
    if(inGameOverScreen){
      if(gameOverSel==0){resetGame();gameOver=false;inGameOverScreen=false;beepShort();}
      else{inGameOverScreen=false;gameOver=true;beepShort();}
      return;
    }
    if(gameOver){resetGame();gameOver=false;beepShort();return;}
    if(!gamePaused) switch(selectedGame){
      case 0:fbBirdVel=FB_FLAP;beepFlap();break;
      case 1:snNext=3;beepShort();break;
      case 4:asVX+=cosf(asAngle)*0.5f;asVY+=sinf(asAngle)*0.5f;
             if(millis()-asLastShot>180){for(int i=0;i<5;i++)if(!asBAlive[i]){
               asBX[i]=asShipX;asBY[i]=asShipY;
               asBVX[i]=cosf(asAngle)*5.0f;asBVY[i]=sinf(asAngle)*5.0f;
               asBAlive[i]=true;asLastShot=millis();beepShort();break;}}break;
      case 5:if(!dnJump){dnVY=dnJumpV();dnJump=true;beepShort();}break;
      case 6:if(mmAwaiting)handleMemInput(1);break;
      case 7:for(int i=0;i<SS_B;i++)if(!ssBA[i]){ssBX[i]=ssShipX+4;ssBY[i]=46;ssBA[i]=true;break;}
             beepShort();break;
      case 8:if(rxActive){rxResult=millis()-rxStart;rxActive=false;rxDone=true;
               if(!gHi[8]||(int)rxResult<gHi[8]){gHi[8]=(int)rxResult;saveHiScore(8);}
               beepSuccess();}
             else if(rxDone){rxDone=false;rxActive=false;rxShowAt=millis()+random(1500,5000);}
             break;
      case 9:if(!guDone){guAtt++;
               if(guGuess==guSecret){guDone=true;beepSuccess();
                 int s=max(1,100-guAtt*5);if(s>gHi[9]){gHi[9]=s;saveHiScore(9);}
                 gameOver=true;inGameOverScreen=true;}
               else{guHint=guGuess<guSecret?"Higher!":"Lower!";beepFail();}
             }break;
      case 12:tnVY-=1.0f;break;
      case 13:if(!rfWaiting){int d=abs((int)rfX-64);rfScore+=max(0,30-d);
               rfRound++;if(rfRound>=5){if(rfScore>gHi[13]){gHi[13]=rfScore;saveHiScore(13);}
               gameOver=true;inGameOverScreen=true;beepSuccess();}
               else{rfWaiting=true;rfWaitT=millis();}beepShort();}break;
      case 14:if(wkVisible&&wkHole==1){wkScore++;wkVisible=false;
               if(wkScore>gHi[14]){gHi[14]=wkScore;saveHiScore(14);}
               beepSuccess();wkWindow=max(400.0f,wkWindow-30.0f);}
              else if(wkVisible){wkMissed++;if(wkMissed>=3){gameOver=true;inGameOverScreen=true;beepFail();}}
              break;
    }
  } else if(currentScreen==3){alarmCursor=(alarmCursor+1)%3;beepShort();}
  else if(currentScreen==10){crAssets[crSel].loaded=false;crAssets[crSel].lastUpd=0;beepShort();}
  else if(currentScreen==6){
    if(!swRunning){swStart=millis()-swElapsed;swRunning=true;}
    else{swElapsed=millis()-swStart;swRunning=false;}
    beepShort();
  } else if(currentScreen==8){
    switch(settCursor){
      case 0:useCelsius=!useCelsius;saveEEPROM();break;
      case 1:use24Hour=!use24Hour;saveEEPROM();break;
      case 2:brightIdx=(brightIdx+1)%5;applyBrightness();saveEEPROM();break;
      case 3:soundOn=!soundOn;saveEEPROM();break;
      case 4:autoDimOn=!autoDimOn;saveEEPROM();break;
      case 5:autoCycleOn=!autoCycleOn;saveEEPROM();break;
      case 6:for(int i=0;i<15;i++)gHi[i]=0;gHi[8]=9999;saveEEPROM();beepSuccess();break;
    }
    beepShort();
  }
}

void handleLeftLong(){
  if(currentScreen==2&&inGameOverScreen)return;
  currentScreen=(currentScreen-1+TOTAL_SCREENS)%TOTAL_SCREENS;
  if(currentScreen==2){gameOver=true;inGameOverScreen=false;}
  lastCycleTime=millis();beepLong();
}
void handleRightLong(){
  if(currentScreen==2&&inGameOverScreen)return;
  currentScreen=(currentScreen+1)%TOTAL_SCREENS;
  if(currentScreen==2){gameOver=true;inGameOverScreen=false;}
  lastCycleTime=millis();beepLong();
}
void handleMidLong(){
  if(currentScreen==6){swElapsed=0;swRunning=false;beepLong();}
  else beepLong();
}

//
//  GAME INIT
//
void initAsteroids(){
  for(int i=0;i<8;i++){asX[i]=random(0,2)?random(4,44):random(90,124);asY[i]=random(4,60);asR[i]=random(5,12);asAlive[i]=true;}
}
void initDino(){
  dnY=DN_GND-10.0f;dnVY=0;dnJump=false;dnSpd=3.0f;dnScore=0;
  for(int i=0;i<DN_MAX;i++)dnO[i].active=false;
  dnO[0]={132,7,12,true,false};
}
void initMemory(){
  mmSeqLen=3;mmSeqIdx=0;mmAwaiting=false;mmGap=false;
  for(int i=0;i<mmSeqLen;i++)mmSeq[i]=random(0,3);
  mmBlinkT=millis()+600;mmBlinkIdx=0;mmDone=false;mmScore=0;
}
void initShooter(){
  ssShipX=60;ssScore=0;
  for(int i=0;i<SS_E;i++){ssEX[i]=random(128,256);ssEY[i]=random(4,38);ssEA[i]=true;}
  for(int i=0;i<SS_B;i++)ssBA[i]=false;
}
void initCatch(){ctPadX=50;ctScore=0;ctLives=3;ctSpawnT=millis();for(int i=0;i<4;i++)ctIOn[i]=false;}
void initDodge(){dgX=60;dgScore=0;dgSpd=2.0f;dgSpawnT=millis();for(int i=0;i<6;i++)dgBOn[i]=false;}
void initTunnel(){tnY=32;tnVY=0;tnPhase=0;tnScore=0;tnSpd=0.05f;tnTick=millis();}
void initReflex(){rfX=10;rfDir=1;rfSpd=1.5f;rfScore=0;rfRound=0;rfWaiting=false;}
void initWhack(){wkHole=1;wkVisible=false;wkScore=0;wkMissed=0;wkWindow=1200.0f;wkBlinkPhase=0;wkPrevHole=-1;wkShowT=millis()+random(800,2000);}

void resetGame(){
  gamePaused=false;inGameOverScreen=false;gameOverSel=0;
  gameCntdown=3;gameCntdownT=millis();
  switch(selectedGame){
    case 0:fbBirdY=32;fbBirdVel=0;fbPipeX=130;fbGapY=18;fbPipeSpd=2.2f;fbScore=0;
           fbStarted=true;break;
    case 1:snLen=3;snDir=0;snNext=0;snScore=0;snSpd=170;
           for(int i=0;i<3;i++){snX[i]=60-i*SN_CELL;snY[i]=28;}
           snFX=80;snFY=28;lastSnakeMove=0;break;
    case 2:pnPY=24;pnBX=64;pnBY=32;pnBVX=-2.2f;pnBVY=1.3f;pnScore=0;break;
    case 3:bkPadX=54;bkBX=64;bkBY=50;bkBVX=2.0f;bkBVY=-2.0f;bkScore=0;bkLevel=1;
           for(int c=0;c<BK_COLS;c++)for(int r=0;r<BK_ROWS;r++)bkBricks[c][r]=(r<2)?2:1;break;
    case 4:asShipX=64;asShipY=32;asAngle=0;asVX=0;asVY=0;asScore=0;
           initAsteroids();for(int i=0;i<5;i++)asBAlive[i]=false;
           asLastShot=0;asLastTick=millis();break;
    case 5:initDino();break;
    case 6:initMemory();break;
    case 7:initShooter();break;
    case 8:rxActive=false;rxDone=false;rxShowAt=millis()+random(1500,5000);break;
    case 9:guSecret=random(1,21)*5;guGuess=50;guDone=false;guHint="";guAtt=0;break;
    case 10:initCatch();break; case 11:initDodge();break;
    case 12:initTunnel();break; case 13:initReflex();break; case 14:initWhack();break;
  }
}

//
//  MEMORY INPUT
//
void handleMemInput(int btn){
  if(!mmAwaiting)return;
  if(btn==mmSeq[mmSeqIdx]){
    beepShort();mmSeqIdx++;
    if(mmSeqIdx>=mmSeqLen){
      mmScore+=mmSeqLen*10;if(mmScore>gHi[6]){gHi[6]=mmScore;saveHiScore(6);}
      mmSeqLen=min(mmSeqLen+1,19);
      for(int i=0;i<mmSeqLen;i++)mmSeq[i]=random(0,3);
      mmSeqIdx=0;mmAwaiting=false;mmDone=false;
      mmBlinkT=millis()+800;mmBlinkIdx=0;beepSuccess();
    }
  } else{gameOver=true;inGameOverScreen=true;beepFail();}
}

//
//  WEATHER ICON
//
void drawWeatherIcon(int x,int y,int sz){
  int cx=x+sz/2,cy=y+sz/2,r=sz/4;
  if(weatherCode==0){
    display.setDrawColor(1); display.drawDisc(cx,cy,r+1);
    for(int a=0;a<360;a+=45){
      float rad=a*3.14159f/180.0f;
      display.setDrawColor(1); display.drawLine(cx+(int)(cosf(rad)*(r+2)),cy+(int)(sinf(rad)*(r+2)),cx+(int)(cosf(rad)*(r+4)),cy+(int)(sinf(rad)*(r+4)));
    }
  } else if(weatherCode<=2){
    display.setDrawColor(1); display.drawCircle(x+sz*3/4,y+sz/4,sz/6);
    display.setDrawColor(1); display.drawDisc(x+sz/3,cy+sz/6,sz/5);
    display.setDrawColor(1); display.drawDisc(x+sz/2,cy,sz/4);
    display.setDrawColor(1); display.drawDisc(x+sz*2/3,cy+sz/8,sz/5);
    display.setDrawColor(1); display.drawBox(x+sz/5,cy+sz/6,sz*13/20,sz/5);
  } else if(weatherCode==3){
    display.setDrawColor(1); display.drawDisc(cx-sz/6,cy,sz/4);
    display.setDrawColor(1); display.drawDisc(cx+sz/6,cy-sz/8,sz/3);
    display.setDrawColor(1); display.drawDisc(cx+sz/3,cy,sz/5);
    display.setDrawColor(1); display.drawBox(cx-sz/4,cy,sz*3/5,sz/4);
  } else if(weatherCode<=48){
    for(int i=0;i<4;i++){display.setDrawColor(1); display.drawRBox(x+2,y+i*(sz/4)+2,sz-4,sz/6,2);}
  } else if(weatherCode<=82){
    display.setDrawColor(1); display.drawDisc(cx-sz/6,y+sz/3,sz/5);
    display.setDrawColor(1); display.drawDisc(cx+sz/6,y+sz/4,sz/4);
    display.setDrawColor(1); display.drawBox(cx-sz/4,y+sz/3,sz/2+2,sz/5);
    for(int d=0;d<4;d++){display.setDrawColor(1); display.drawLine(x+4+d*6,y+sz*2/3,x+2+d*6,y+sz-2);}
  } else{
    display.setDrawColor(1); display.drawDisc(cx-sz/6,y+sz/3,sz/5);
    display.setDrawColor(1); display.drawDisc(cx+sz/6,y+sz/4,sz/4);
    display.setDrawColor(1); display.drawBox(cx-sz/4,y+sz/3,sz/2+2,sz/5);
    display.setDrawColor(1); display.drawLine(cx+2,y+sz*3/5,cx-4,y+sz*4/5);
    display.setDrawColor(1); display.drawLine(cx-4,y+sz*4/5,cx+2,y+sz*4/5);
    display.setDrawColor(1); display.drawLine(cx+2,y+sz*4/5,cx-4,y+sz-2);
  }
}

//
//  DRAW ROUTER
//
void drawScreen(){
  if(peekPhase==2&&(currentScreen==0||currentScreen==1)){drawFullPeekEyes();return;}
  if(currentScreen==7&&!alarmRinging){
    if(eyeBtn&&millis()-eyeBtnTime<800){
      if(eyeBtn==1)roboEyes.setMood(TIRED);
      else if(eyeBtn==2)roboEyes.setMood(HAPPY);
      else roboEyes.setMood(ANGRY);
    } else if(millis()-eyeBtnTime>=800&&eyeBtn){roboEyes.setMood(DEFAULT);eyeBtn=0;}
    roboEyes.update(); return;
  }
  display.clearBuffer();
  display.setDrawColor(1);
  if(alarmRinging){
    if((millis()/500)%2==0){display.setDrawColor(1); display.drawBox(0,0,128,64);display.setDrawColor(0);}
    setDisplayFont(2);drawCenteredText(F("ALARM!"),20,2);
    display.setDrawColor(1);display.sendBuffer();return;
  }
  switch(currentScreen){
    case 0:drawClockWeather();if(peekPhase==1||peekPhase==3)drawPeekCurtain();break;
    case 1:drawDetailedWeather();if(peekPhase==1||peekPhase==3)drawPeekCurtain();break;
    case 2:drawGame();break;
    case 3:drawAlarm();break;
    case 4:drawStats();break;
    case 5:drawQuote();break;
    case 6:drawTimer();break;
    case 8:drawSettings();break;
    case 9:drawSpotify();break;
    case 10:drawCrypto();break;
  }
  display.sendBuffer();
}

//
//  SCREEN DRAWINGS
//
void drawCenteredText(String s,int y,uint8_t sz){
  setDisplayFont(sz);
  int16_t x1=0,y1=0; uint16_t w=display.getStrWidth(s.c_str()),h=8;
  display.setCursor((128-w)/2,y);display.print(s);
}
// Overload accepting F() strings
void drawCenteredText(const __FlashStringHelper* s,int y,uint8_t sz){
  drawCenteredText(String(s),y,sz);
}

void drawClockWeather(){
  setDisplayFont(1);
  drawCenteredText(fmtDate(),2,0);
  drawCenteredText(fmtTime(),13,3);
  drawWeatherIcon(3,40,22);
  setDisplayFont(2);
  int tv=(int)displayTemp();
  display.setCursor(30,40);display.print(tv);
  int dx=display.getCursorX();
  display.setDrawColor(1); display.drawDisc(dx+2,40,2);
  display.setCursor(dx+6,40);display.print(useCelsius?F("C"):F("F"));
  setDisplayFont(1);
  display.setCursor(30,52);display.print(weatherCond());
  if(alarmEnabled){display.setDrawColor(1); display.drawDisc(121,4,3);display.setDrawColor(0); display.drawDisc(121,4,1);}
}

void drawDetailedWeather(){
  drawCenteredText(F("WEATHER"),0,0);
  drawWeatherIcon(5,10,28);
  setDisplayFont(2);
  int tv=(int)displayTemp();
  display.setCursor(40,11);display.print(tv);
  int dx=display.getCursorX();
  display.setDrawColor(1); display.drawDisc(dx+2,11,2);
  display.setCursor(dx+6,11);display.print(useCelsius?F("C"):F("F"));
  setDisplayFont(1);
  display.setCursor(40,23);display.print(weatherCond());
  display.setCursor(40,33);display.print(String((int)windSpeed)+F(" km/h"));
  display.setCursor(40,43);
  display.print(F("Hum "));
  display.print(humidity);
  display.print(F("%"));
  if (sunsetTime.length()>0) {
  display.setCursor(40,53);
  display.print(F("Set "));
  display.print(sunsetTime);
}
}

void drawGame(){
  if(inGameOverScreen){drawGameOverScreen();return;}
  if(gameOver){
    drawCenteredText(F("GAMES"),1,0);
    int vs=max(0,min(selectedGame-2,TOTAL_GAMES-5));
    for(int i=0;i<5;i++){
      int gi=vs+i; if(gi>=TOTAL_GAMES)break;
      int y=11+i*10; bool sel=(gi==selectedGame);
      if(sel){display.setDrawColor(1);display.drawRBox(2,y-1,124,10,2);display.setDrawColor(0);}
      else display.setDrawColor(1);
      display.setCursor(7,y);
      display.print(gameNames[gi]);
      if(gHi[gi]>0&&!(gi==8&&gHi[gi]==9999)){
        String hi=gi==8?String(gHi[gi])+"ms":String(gHi[gi]);
        uint16_t w=display.getStrWidth(hi.c_str());
        display.setCursor(124-w,y);display.print(hi);
      }
      if(sel)display.setDrawColor(1);
    }
    if(vs>0){display.setCursor(118,12);display.print("^");}
    if(vs+5<TOTAL_GAMES){display.setCursor(118,60);display.print("v");}
    return;
  }
  switch(selectedGame){
    case 0:drawFlappy();break;  case 1:drawSnake();break;
    case 2:drawPong();break;    case 3:drawBreakout();break;
    case 4:drawAsteroids();break; case 5:drawDino();break;
    case 6:drawMemory();break;  case 7:drawShooter();break;
    case 8:drawReaction();break; case 9:drawGuess();break;
    case 10:drawCatch();break;  case 11:drawDodge();break;
    case 12:drawTunnel();break; case 13:drawReflex();break;
    case 14:drawWhack();break;
  }
  if(gameCntdown>0){
    display.setDrawColor(0); display.drawBox(44,16,40,32);
    display.setDrawColor(1); display.drawRFrame(44,16,40,32,4);
    setDisplayFont(3); drawCenteredText(String(gameCntdown),20,3);
  }
}

void drawGameOverScreen(){
  int sc=0;
  switch(selectedGame){
    case 0:sc=fbScore;break;  case 1:sc=snScore;break;  case 2:sc=pnScore;break;
    case 3:sc=bkScore;break;  case 4:sc=asScore;break;  case 5:sc=dnScore;break;
    case 6:sc=mmScore;break;  case 7:sc=ssScore;break;  case 8:sc=(int)rxResult;break;
    case 9:sc=max(0,100-guAtt*5);break;
    case 10:sc=ctScore;break; case 11:sc=dgScore;break; case 12:sc=tnScore;break;
    case 13:sc=rfScore;break; case 14:sc=wkScore;break;
  }
  drawCenteredText(F("GAME OVER"),3,1);
  setDisplayFont(1);
  drawCenteredText("Score: "+String(sc),14,1);
  if(gHi[selectedGame]>0&&!(selectedGame==8&&gHi[selectedGame]==9999))
    drawCenteredText("Best: "+String(gHi[selectedGame]),24,1);
  int rX=14,rY=42,rW=42,rH=14,lX=72,lY=42,lW=42,lH=14;
  if(gameOverSel==0){display.setDrawColor(1); display.drawRBox(rX,rY,rW,rH,3);display.setDrawColor(0);}
  else display.setDrawColor(1); display.drawRFrame(rX,rY,rW,rH,3);
  display.setCursor(rX+6,rY+4);display.print(F("Retry"));
  if(gameOverSel==0)display.setDrawColor(1);
  if(gameOverSel==1){display.setDrawColor(1); display.drawRBox(lX,lY,lW,lH,3);display.setDrawColor(0);}
  else display.setDrawColor(1); display.drawRFrame(lX,lY,lW,lH,3);
  display.setCursor(lX+6,lY+4);display.print(F("Leave"));
  if(gameOverSel==1)display.setDrawColor(1);
}

void drawAlarm(){
  drawCenteredText(F("ALARM"),1,1);
  setDisplayFont(2);
  String t=(alarmHour<10?"0":"")+String(alarmHour)+":"+(alarmMinute<10?"0":"")+String(alarmMinute);
  drawCenteredText(t,13,2);
  setDisplayFont(1);
  if(alarmCursor==0){display.setCursor(32,31);display.print(F("^^"));}
  if(alarmCursor==1){display.setCursor(72,31);display.print(F("^^"));}
  String s=alarmEnabled?"  ON  ":"  OFF  ";
  int16_t x1=0,y1=0; uint16_t w=display.getStrWidth(s.c_str()),h=8;
  int bx=(128-w)/2-3,by=42;
  if(alarmCursor==2){display.setDrawColor(1); display.drawRBox(bx,by,w+6,12,4);display.setDrawColor(0);}
  else display.setDrawColor(1); display.drawRFrame(bx,by,w+6,12,4);
  display.setCursor((128-w)/2,by+2);display.print(s);
  display.setDrawColor(1);
  if(WiFi.status()!=WL_CONNECTED){display.setCursor(18,58);display.print(F("No WiFi"));}
}

void drawStats(){
  drawCenteredText(F("STATS"),1,1);
  bool conn=(WiFi.status()==WL_CONNECTED);
  setDisplayFont(1);
  display.setCursor(4,12);display.print(conn?"WiFi:OK":"WiFi:--");
  if(conn){
    display.setCursor(4,22);display.print(F("RSSI "));display.print(WiFi.RSSI());display.print(F(" dBm"));
    display.setCursor(4,32);display.print(WiFi.localIP().toString());
  }
  display.setCursor(4,42);display.print(String((int)temperature)+F("C  ")+weatherCond());
  unsigned long sc=millis()/1000;
  display.setCursor(4,52);
  display.print(F("Up "));display.print(sc/3600);display.print(F("h "));display.print((sc%3600)/60);display.print(F("m"));
}

void drawQuote(){
  drawCenteredText(F("QUOTE"),1,1);
  if(quoteText.length()==0){drawCenteredText(F("Loading..."),28,1);return;}
  setDisplayFont(1);
  drawWrappedText(quoteText,3,12,122,10,4);
  if(quoteAuthor.length()>0){
    String a="- "+quoteAuthor;
    if(a.length()>21)a=a.substring(0,18)+"...";
    int16_t x1=0,y1=0; uint16_t w=display.getStrWidth(a.c_str()),h=8;
    display.setCursor(126-w,52);display.print(a);
  }
}

void drawTimer(){
  drawCenteredText(F("TIMER"),1,1);
  unsigned long total=swRunning?millis()-swStart:swElapsed;
  char buf[10];sprintf(buf,"%02lu:%02lu:%02lu",total/60000,(total/1000)%60,(total/10)%100);
  drawCenteredText(String(buf),20,2);
  if(swRunning){display.setDrawColor(1); display.drawDisc(122,5,3);}
  else{display.setDrawColor(1); display.drawCircle(122,5,3);}
}

void drawSettings() {
  drawCenteredText(F("SETTINGS"), 1, 0);
  const char* lbl[] = {"Unit","Clock","Bright","Sound","Auto Dim","Auto Cyc","Reset Hi"};
  const char* brlbl[] = {"Min","Low","Med","High","Max"};
  String vals[] = {
    useCelsius?"Celsius":"Fahrenht",
    use24Hour?"24h":"12h",
    String(brlbl[brightIdx]),
    soundOn?"On":"Off",
    autoDimOn?"On":"Off",
    autoCycleOn?"On":"Off",
    "Confirm"
  };
  // show 5 rows at a time, scroll with cursor
  int vs=max(0,min(settCursor-2,2));
  for(int i=0;i<5;i++){
    int ri=vs+i; if(ri>=7)break;
    int y=11+i*10; bool sel=(ri==settCursor);
    if(sel){display.setDrawColor(1);display.drawRBox(2,y-1,124,10,2);display.setDrawColor(0);}
    else display.setDrawColor(1);
    display.setCursor(7,y); display.print(lbl[ri]);
    uint16_t w=display.getStrWidth(vals[ri].c_str());
    display.setCursor(124-w,y); display.print(vals[ri]);
    if(sel)display.setDrawColor(1);
  }
  if(vs>0){display.setCursor(118,12);display.print("^");}
  if(vs+5<7){display.setCursor(118,60);display.print("v");}
}



//  Spotify
void drawLPRecord(int cx,int cy,int r,float spin){

  int left  = cx - r;
  int right = cx + r;

  unsigned long t = millis();

  // animated vertical sound bars
  for(int x = left; x <= right; x += 3){

    float phase = (x - left) * 0.6f;
    int amp = spPlay ? 8 : 3;

    int h = 2 + (int)(amp * sinf(t / 180.0f + phase));

    display.setDrawColor(1); display.drawLine(x,cy - h,x,cy + h);
  }
}


void drawSpotify(){
  setDisplayFont(1);
  display.setCursor(4,1); display.print(F("Player"));

    if(!spConn){
    drawCenteredText(F("No Spotify"),18,1);
    drawCenteredText(F("Check WiFi &"),30,1);
    drawCenteredText(F("refresh token"),42,1);
    return;
  }
  if(spTitle.length()==0){
    drawCenteredText(F("Nothing playing"),28,1);
    return;
  }

  drawLPRecord(22,32,14,lpSpin);

  setDisplayFont(1);
  // Scrolling title
  String dt = spTitle;
  if(spTitle.length() > 11){
    int off = max(0, min(spOff, (int)spTitle.length()-1));
    dt = spTitle.substring(off, min((int)spTitle.length(), off+11));
  }
  display.setCursor(50,19); display.print(dt);       // moved 5px right and 5px down
  String as = spArtist; if(as.length()>13) as = as.substring(0,12) + ".";
  display.setCursor(50,31); display.print(as);      // moved 5px right and 5px down

  // Control bar
  display.setDrawColor(1); display.drawRFrame(2,54,36,10,2);
  display.setDrawColor(1); display.drawRFrame(46,54,36,10,2);
  display.setDrawColor(1); display.drawRFrame(90,54,36,10,2);
  display.setCursor(13,56); display.print(F("|<"));
  display.setCursor(55,56); display.print(spPlay?F("||"):F(">"));
  display.setCursor(101,56); display.print(F(">|"));
}


//
//  GAME DRAWS
//
void drawFlappy(){
  if(!fbStarted){setDisplayFont(3);drawCenteredText(String(fbCnt),24,3);return;}
  int botY=fbGapY+FB_GAP;
  display.setDrawColor(1); display.drawBox(fbPipeX,0,FB_PW,fbGapY);
  display.setDrawColor(1); display.drawBox(fbPipeX-2,fbGapY-4,FB_PW+4,4);
  display.setDrawColor(1); display.drawBox(fbPipeX-2,botY,FB_PW+4,4);
  display.setDrawColor(1); display.drawBox(fbPipeX,botY+4,FB_PW,64-(botY+4));
  int bx=12,by=constrain((int)fbBirdY,0,63-FB_BS);
  display.setDrawColor(1); display.drawRBox(bx,by,FB_BS+2,FB_BS,2);
  display.setDrawColor(0); display.drawBox(bx+FB_BS,by+1,2,2);
  if(fbBirdVel<0)display.setDrawColor(1); display.drawLine(bx,by+FB_BS-1,bx-3,by+FB_BS-3);
  setDisplayFont(2);display.setCursor(54,3);display.print(fbScore);
  if(gHi[0]>0){setDisplayFont(1);display.setCursor(4,4);display.print(F("Hi:"));display.print(gHi[0]);}
  if(gamePaused){
    display.setDrawColor(0); display.drawRBox(36,24,56,16,4);
    display.setDrawColor(1); display.drawRFrame(36,24,56,16,4);
    setDisplayFont(1);drawCenteredText(F("PAUSED"),30,1);
  }
}

void drawSnake(){
  display.setDrawColor(1); display.drawFrame(3,0,122,64);
  display.setDrawColor(1); display.drawBox(snFX,snFY,SN_CELL,SN_CELL);
  display.setDrawColor(0); display.drawLine(snFX+1,snFY+SN_CELL/2,snFX+SN_CELL-2,snFY+SN_CELL/2);
  for(int i=0;i<snLen;i++){display.setDrawColor(1); display.drawBox(snX[i],snY[i],SN_CELL-1,SN_CELL-1);}
  display.setDrawColor(1); display.drawFrame(snX[0],snY[0],SN_CELL,SN_CELL);
  setDisplayFont(1);display.setCursor(5,56);display.print(snScore);
  if(gHi[1]>0){display.setCursor(80,56);display.print(F("Hi:"));display.print(gHi[1]);}
}

void drawPong(){
  display.setDrawColor(1); display.drawBox(4,0,3,64);
  display.setDrawColor(1); display.drawBox(PN_PX,pnPY,4,PN_PH);
  for(int yn=0;yn<64;yn+=8){display.setDrawColor(1); display.drawBox(63,yn,2,4);}
  display.setDrawColor(1); display.drawDisc((int)pnBX,(int)pnBY,2);
  setDisplayFont(2);display.setCursor(48,3);display.print(pnScore);
  if(gHi[2]>0){setDisplayFont(1);display.setCursor(8,4);display.print(F("Hi:"));display.print(gHi[2]);}
}

void drawBreakout(){
  for(int c=0;c<BK_COLS;c++)for(int r=0;r<BK_ROWS;r++){
    if(!bkBricks[c][r])continue;
    int bx=c*16,by=r*6+2;
    if(bkBricks[c][r]==2){display.setDrawColor(1); display.drawBox(bx+1,by,BK_W-1,BK_H);}
    else{display.setDrawColor(1); display.drawFrame(bx+1,by,BK_W-1,BK_H);}
  }
  display.setDrawColor(1); display.drawRBox(bkPadX,58,22,4,2);
  display.setDrawColor(1); display.drawDisc((int)bkBX,(int)bkBY,2);
  setDisplayFont(1);
  display.setCursor(4,54);display.print(F("Sc:"));display.print(bkScore);
  display.setCursor(68,54);display.print(F("Lv:"));display.print(bkLevel);
}

void drawAsteroids(){
  float s=5.0f;
  int x1=(int)(asShipX+cosf(asAngle)*s),y1=(int)(asShipY+sinf(asAngle)*s);
  int x2=(int)(asShipX+cosf(asAngle+2.4f)*s),y2=(int)(asShipY+sinf(asAngle+2.4f)*s);
  int x3=(int)(asShipX+cosf(asAngle-2.4f)*s),y3=(int)(asShipY+sinf(asAngle-2.4f)*s);
  display.setDrawColor(1); display.drawTriangle(x1,y1,x2,y2,x3,y3);
  for(int i=0;i<8;i++){
    if(!asAlive[i])continue;
    display.setDrawColor(1); display.drawCircle(asX[i],asY[i],asR[i]);
    display.setDrawColor(1); display.drawLine(asX[i]-asR[i]/2,asY[i],asX[i]+asR[i]/3,asY[i]-asR[i]/2);
  }
  for(int i=0;i<5;i++)if(asBAlive[i]){display.setDrawColor(1); display.drawLine((int)asBX[i],(int)asBY[i],(int)(asBX[i]-asBVX[i]),(int)(asBY[i]-asBVY[i]));}
  setDisplayFont(1);display.setCursor(4,3);display.print(asScore);
  if(gHi[4]>0){display.setCursor(90,3);display.print(F("Hi:"));display.print(gHi[4]);}
}

void drawDinoObs(const DObs& o){
  int ox=(int)o.x;
  if(!o.bird){
    display.setDrawColor(1); display.drawBox(ox+2,DN_GND-o.h,o.w-4,o.h);
    display.setDrawColor(1); display.drawBox(ox,DN_GND-o.h+3,o.w,4);
    display.setDrawColor(1); display.drawBox(ox,DN_GND-o.h-2,3,6);
    display.setDrawColor(1); display.drawBox(ox+o.w-3,DN_GND-o.h-2,3,6);
  } else {
    display.setDrawColor(1); display.drawBox(ox+1,DN_GND-o.h-2,o.w-2,5);
    display.setDrawColor(1); display.drawLine(ox-3,DN_GND-o.h,ox+2,DN_GND-o.h-4);
    display.setDrawColor(1); display.drawLine(ox+o.w-2,DN_GND-o.h,ox+o.w+3,DN_GND-o.h-4);
  }
}

void drawDino(){
  display.setDrawColor(1); display.drawLine(4,DN_GND,127,DN_GND);
  for(int i=0;i<DN_MAX;i++)if(dnO[i].active)drawDinoObs(dnO[i]);
  int dy=(int)dnY;
  display.setDrawColor(1); display.drawBox(12,dy,10,10);
  display.setDrawColor(1); display.drawBox(18,dy-4,6,6);
  display.setDrawColor(0); display.drawBox(22,dy-3,2,2);
  display.setDrawColor(1); display.drawBox(12,dy+10,3,4);
  display.setDrawColor(1); display.drawBox(17,dy+10,3,4);
  display.setDrawColor(1); display.drawBox(4,dy+3,9,3);
  for(int px=4;px<128;px+=14){int gx=(px+(int)(millis()/80))%124+4;display.setDrawColor(1); display.drawPixel(gx,DN_GND+1);}
  setDisplayFont(1);display.setCursor(4,3);display.print(dnScore);
  if(gHi[5]>0){display.setCursor(80,3);display.print(F("Hi:"));display.print(gHi[5]);}
  // speed bar
  int spb=(int)((dnSpd-3.0f)/5.5f*20.0f);
  display.setDrawColor(1); display.drawFrame(105,1,22,4);
  if(spb>0){display.setDrawColor(1); display.drawBox(106,2,min(spb,20),2);}
}

void drawMemory(){
  drawCenteredText(F("MEMORY"),1,1);
  display.setDrawColor(1); display.drawRFrame(4,12,36,30,3);display.setCursor(16,24);display.print(F("L"));
  display.setDrawColor(1); display.drawRFrame(46,12,36,30,3);display.setCursor(58,24);display.print(F("M"));
  display.setDrawColor(1); display.drawRFrame(88,12,36,30,3);display.setCursor(100,24);display.print(F("R"));
  if(!mmDone&&!mmAwaiting&&!mmGap){
    int btn=mmSeq[mmBlinkIdx],bx=(btn==0)?4:(btn==1)?46:88;
    display.setDrawColor(1); display.drawRBox(bx+1,13,35,29,3);
    display.setDrawColor(0);
    display.setCursor(bx+12,24);display.print(btn==0?F("L"):btn==1?F("M"):F("R"));
    display.setDrawColor(1);
  }
  setDisplayFont(1);
  if(mmAwaiting)drawCenteredText("Your turn! "+String(mmSeqIdx+1)+"/"+String(mmSeqLen),46,1);
  else drawCenteredText("Watch: "+String(mmBlinkIdx+1)+"/"+String(mmSeqLen),46,1);
  display.setCursor(4,56);display.print(F("Sc:"));display.print(mmScore);
  if(gHi[6]>0){display.setCursor(80,56);display.print(F("Hi:"));display.print(gHi[6]);}
}

void drawShooter(){
  display.setDrawColor(1); display.drawLine(4,58,127,58);
  display.setDrawColor(1); u8g2FillTriangle(ssShipX+4,46,ssShipX,57,ssShipX+8,57);
  if((millis()/100)%2){display.setDrawColor(1); display.drawBox(ssShipX+2,58,4,3);}
  for(int i=0;i<SS_E;i++){
    if(!ssEA[i])continue;int ex=ssEX[i],ey=ssEY[i];if(ex<4||ex>127)continue;
    display.setDrawColor(1); display.drawBox(ex+1,ey,6,4);display.setDrawColor(1); display.drawBox(ex,ey+4,8,3);
    display.setDrawColor(1); display.drawBox(ex-1,ey+7,3,2);display.setDrawColor(1); display.drawBox(ex+6,ey+7,3,2);
  }
  for(int i=0;i<SS_B;i++)if(ssBA[i]){display.setDrawColor(1); display.drawBox(ssBX[i],ssBY[i],2,5);}
  setDisplayFont(1);display.setCursor(4,3);display.print(ssScore);
  if(gHi[7]>0){display.setCursor(80,3);display.print(F("Hi:"));display.print(gHi[7]);}
}

void drawReaction(){
  drawCenteredText(F("REACTION"),1,1);
  if(rxDone){
    drawCenteredText(String(rxResult)+" ms",18,2);
    const char* r;
    if(rxResult<200)r="AMAZING!";else if(rxResult<300)r="Great!";else if(rxResult<500)r="Good";else r="Keep going!";
    drawCenteredText(String(r),38,1);
    if(gHi[8]>0&&gHi[8]<9999)drawCenteredText("Best: "+String(gHi[8])+"ms",50,1);
  } else if(!rxActive){
    drawCenteredText(F("Wait for flash..."),26,1);
  } else {
    display.setDrawColor(1); display.drawBox(4,10,120,50);
    display.setDrawColor(0);drawCenteredText(F("PRESS!"),28,2);display.setDrawColor(1);
  }
}

void drawGuess(){
  drawCenteredText(F("GUESS: 5-100"),1,1);
  if(guDone){
    drawCenteredText(F("Correct!"),14,1);
    drawCenteredText("In "+String(guAtt)+" tries",24,1);
    drawCenteredText("Score:"+String(max(0,100-guAtt*5)),36,1);
  } else {
    setDisplayFont(3);drawCenteredText(String(guGuess),14,3);
    setDisplayFont(1);
    if(guHint.length()>0)drawCenteredText(guHint,40,1);
    drawCenteredText("#"+String(guAtt),54,1);
  }
}

void drawCatch(){
  display.setDrawColor(1); display.drawLine(4,60,124,60);
  display.setDrawColor(1); display.drawRBox(ctPadX,58,26,4,2);
  for(int i=0;i<4;i++)if(ctIOn[i]){display.setDrawColor(1); display.drawDisc(ctIX[i],ctIY[i],3);}
  setDisplayFont(1);display.setCursor(4,3);display.print(F("Sc:"));display.print(ctScore);
  for(int l=0;l<ctLives;l++){display.setDrawColor(1); display.drawDisc(110+l*8,5,3);}
}

void drawDodge(){
  display.setDrawColor(1); display.drawLine(4,56,124,56);
  display.setDrawColor(1); display.drawRBox(dgX-4,48,10,8,2);
  for(int i=0;i<6;i++)if(dgBOn[i]){display.setDrawColor(1); display.drawBox(dgBX[i],dgBY[i],dgBW[i],10);}
  setDisplayFont(1);display.setCursor(4,3);display.print(F("Sc:"));display.print(dgScore);
  if(gHi[11]>0){display.setCursor(80,3);display.print(F("Hi:"));display.print(gHi[11]);}
}

void drawTunnel(){
  for(int px=4;px<128;px+=2){
    float ph=tnPhase+(128-px)*tnSpd*2;
    int top=(int)(20+10*sinf(ph));
    int bot=(int)(top+max(16,28-(int)(tnScore/8)));
    display.setDrawColor(1); display.drawPixel(px,top);
    display.setDrawColor(1); display.drawPixel(px,bot);
  }
  display.setDrawColor(1); display.drawDisc(20,(int)tnY,3);
  setDisplayFont(1);display.setCursor(4,2);display.print(tnScore);
  if(gHi[12]>0){display.setCursor(80,2);display.print(F("Hi:"));display.print(gHi[12]);}
}

void drawReflex(){
  drawCenteredText(F("REFLEX STOP"),1,1);
  display.setDrawColor(1); display.drawRFrame(54,25,20,14,3);
  int bx=constrain((int)rfX,4,122);
  display.setDrawColor(1); display.drawBox(bx,23,3,18);
  for(int i=0;i<5;i++){
    if(i<rfRound){display.setDrawColor(1); display.drawDisc(54+i*6,42,2);}
    else{display.setDrawColor(1); display.drawCircle(54+i*6,42,2);}
  }
  setDisplayFont(1);display.setCursor(4,50);display.print(F("Pts:"));display.print(rfScore);
  if(rfWaiting)drawCenteredText(F("Ready..."),54,1);
}

void drawWhack(){
  drawCenteredText(F("WHACK-A-MOLE"),1,1);
  const int hx[]={18,64,110};
  for(int i=0;i<3;i++){
    display.setDrawColor(1); display.drawRFrame(hx[i]-12,24,24,18,4);
    if(wkVisible&&wkHole==i){
      display.setDrawColor(1); display.drawRBox(hx[i]-10,26,20,14,3);
      display.setDrawColor(0); display.drawDisc(hx[i]-3,32,2);
      display.setDrawColor(0); display.drawDisc(hx[i]+3,32,2);
    }
  }
  setDisplayFont(1);
  display.setCursor(14,46);display.print(F("L"));
  display.setCursor(61,46);display.print(F("M"));
  display.setCursor(107,46);display.print(F("R"));
  display.setCursor(4,3);display.print(F("Sc:"));display.print(wkScore);
  for(int m=0;m<3;m++){
    int hcx=100+m*10;
    if(m<3-wkMissed){display.setDrawColor(1); display.drawDisc(hcx,5,3);}
    else{display.setDrawColor(1); display.drawCircle(hcx,5,3);}
  }
}

//
//  GAME UPDATES
//
void updateFlappy(){
  if(!fbStarted){
    if(millis()-fbCntT>=1000){fbCnt--;fbCntT=millis();if(fbCnt<=0){fbStarted=true;fbBirdVel=0;}}
    return;
  }
  fbBirdVel+=FB_GRAV;fbBirdY+=fbBirdVel;
  fbPipeX-=fbPipeSpd;
  if(fbPipeX<-FB_PW){fbPipeX=130;fbGapY=random(6,36);fbScore++;fbPipeSpd=min(4.5f,fbPipeSpd+0.08f);beepSuccess();}
  if(fbBirdY<0||fbBirdY+FB_BS>63)gameOver=true;
  if(!gameOver&&12+FB_BS>fbPipeX&&12<fbPipeX+FB_PW)
    if(fbBirdY<fbGapY||fbBirdY+FB_BS>fbGapY+FB_GAP) gameOver=true;
  if(gameOver){beepFail();if(fbScore>gHi[0]){gHi[0]=fbScore;saveHiScore(0);}inGameOverScreen=true;}
}

void updateSnake(){
  if(millis()-lastSnakeMove<(unsigned long)snSpd)return;
  lastSnakeMove=millis();
  int opp=(snDir+2)%4;if(snNext!=opp)snDir=snNext;
  for(int i=snLen-1;i>0;i--){snX[i]=snX[i-1];snY[i]=snY[i-1];}
  switch(snDir){case 0:snX[0]+=SN_CELL;break;case 1:snY[0]+=SN_CELL;break;case 2:snX[0]-=SN_CELL;break;case 3:snY[0]-=SN_CELL;break;}
  if(snX[0]<4||snX[0]>123-SN_CELL||snY[0]<1||snY[0]>63-SN_CELL)gameOver=true;
  if(abs(snX[0]-snFX)<SN_CELL&&abs(snY[0]-snFY)<SN_CELL){
    if(snLen<SN_MAX-1)snLen++;snScore++;beepSuccess();
    if(snScore>gHi[1]){gHi[1]=snScore;saveHiScore(1);}
    bool ok=false;while(!ok){snFX=random(4,30)*SN_CELL/4*4;snFY=random(1,14)*SN_CELL;ok=true;for(int i=0;i<snLen;i++)if(snX[i]==snFX&&snY[i]==snFY){ok=false;break;}}
    if(snSpd>60)snSpd-=4;
  }
  for(int i=1;i<snLen;i++)if(snX[0]==snX[i]&&snY[0]==snY[i])gameOver=true;
  if(gameOver){beepFail();inGameOverScreen=true;}
}

void updatePong(){
  if(millis()-holdMoveT>60){holdMoveT=millis();if(bL)pnPY=max(3,pnPY-4);if(bR)pnPY=min(63-PN_PH,pnPY+4);}
  pnBX+=pnBVX;pnBY+=pnBVY;
  if(pnBY<=1){pnBY=1;pnBVY=fabsf(pnBVY);beepShort();}
  if(pnBY>=62){pnBY=62;pnBVY=-fabsf(pnBVY);beepShort();}
  if(pnBX<=8){pnBX=8;pnBVX=fabsf(pnBVX);beepShort();}
  if(pnBVX>0&&pnBX>=PN_PX-2&&pnBX<=PN_PX+4&&pnBY>=pnPY&&pnBY<=pnPY+PN_PH){
    float rel=((pnBY-pnPY)/(float)PN_PH)-0.5f;
    pnBVX=-fabsf(pnBVX)-0.1f;pnBVY=rel*4.0f;pnBX=PN_PX-3;
    pnScore++;beepSuccess();if(pnScore>gHi[2]){gHi[2]=pnScore;saveHiScore(2);}
  }
  if(pnBX>=128){beepFail();gameOver=true;inGameOverScreen=true;}
}

void updateBreakout(){
  if(millis()-holdMoveT>60){holdMoveT=millis();if(bL)bkPadX=max(3,bkPadX-6);if(bR)bkPadX=min(104,bkPadX+6);}
  bkBX+=bkBVX;bkBY+=bkBVY;
  if(bkBX<=1){bkBX=1;bkBVX=fabsf(bkBVX);beepShort();}
  if(bkBX>=126){bkBX=126;bkBVX=-fabsf(bkBVX);beepShort();}
  if(bkBY<=1){bkBY=1;bkBVY=fabsf(bkBVY);beepShort();}
  if(bkBVY>0&&bkBY>=55&&bkBY<=60&&bkBX>=bkPadX&&bkBX<=bkPadX+22){
    float rel=((bkBX-bkPadX)/22.0f)-0.5f;bkBVY=-fabsf(bkBVY);bkBVX=rel*4.0f;bkBY=54;beepShort();
  }
  if(bkBY>64){beepFail();gameOver=true;if(bkScore>gHi[3]){gHi[3]=bkScore;saveHiScore(3);}inGameOverScreen=true;return;}
  bool left=false;
  for(int c=0;c<BK_COLS;c++)for(int r=0;r<BK_ROWS;r++){
    if(!bkBricks[c][r])continue;left=true;
    int bx=c*16+1,by=r*6+2;
    if(bkBX>=bx&&bkBX<=bx+BK_W&&bkBY>=by&&bkBY<=by+BK_H){
      bkBricks[c][r]--;bkBVY=-bkBVY;
      bkScore+=(bkBricks[c][r]==0)?2:1;
      if(bkBricks[c][r]>0)beepShort();else beepSuccess();
    }
  }
  if(bkScore>gHi[3]){gHi[3]=bkScore;saveHiScore(3);}
  if(!left){bkLevel++;beepSuccess();delay(200);
    for(int c=0;c<BK_COLS;c++)for(int r=0;r<BK_ROWS;r++)bkBricks[c][r]=(r<2)?2:1;
    float sp=min(4.0f,2.0f+bkLevel*0.3f);bkBVX=bkBVX>0?sp:-sp;bkBVY=bkBVY>0?sp:-sp;
  }
}

void updateAsteroids(){
  asVX*=0.97f;asVY*=0.97f;asShipX+=asVX;asShipY+=asVY;
  if(asShipX<0)asShipX=128;if(asShipX>128)asShipX=0;
  if(asShipY<0)asShipY=64;if(asShipY>64)asShipY=0;
  float sp=sqrtf(asVX*asVX+asVY*asVY);if(sp>4.0f){asVX=asVX/sp*4.0f;asVY=asVY/sp*4.0f;}
  for(int i=0;i<8;i++){
    if(!asAlive[i])continue;asX[i]--;
    if(asX[i]<-asR[i]){asX[i]=130+asR[i];asY[i]=random(4,60);}
    float dx=asShipX-asX[i],dy=asShipY-asY[i];
    if(sqrtf(dx*dx+dy*dy)<asR[i]+4.0f){gameOver=true;beepFail();if(asScore>gHi[4]){gHi[4]=asScore;saveHiScore(4);}inGameOverScreen=true;return;}
  }
  for(int b=0;b<5;b++){
    if(!asBAlive[b])continue;asBX[b]+=asBVX[b];asBY[b]+=asBVY[b];
    if(asBX[b]<0||asBX[b]>128||asBY[b]<0||asBY[b]>64){asBAlive[b]=false;continue;}
    for(int i=0;i<8;i++){
      if(!asAlive[i])continue;float dx=asBX[b]-asX[i],dy=asBY[b]-asY[i];
      if(sqrtf(dx*dx+dy*dy)<asR[i]){asBAlive[b]=false;asX[i]=random(0,2)?-asR[i]-5:133;asY[i]=random(4,60);asScore+=10;beepSuccess();}
    }
  }
  if(millis()-asLastTick>1000){asScore++;asLastTick=millis();}
}

void spawnDinoObs(){
  for(int i=0;i<DN_MAX;i++){
    if(!dnO[i].active){
      bool bird=(dnScore>8&&random(4)==0);
      float mx=132;
      for(int j=0;j<DN_MAX;j++)if(j!=i&&dnO[j].active)mx=max(mx,dnO[j].x+50.0f);
      int8_t w=bird?14:random(6,10), h=bird?8:random(10,18);
      dnO[i]={mx+random(20,40),w,h,true,bird};
      break;
    }
  }
}

void updateDino(){
  float grav=0.28f+dnSpd*0.012f;
  if(dnJump){dnVY+=grav;dnY+=dnVY;if(dnY>=DN_GND-10.0f){dnY=DN_GND-10.0f;dnJump=false;dnVY=0;}}
  int activeCount=0;
  for(int i=0;i<DN_MAX;i++){
    if(!dnO[i].active)continue;
    activeCount++;
    dnO[i].x-=dnSpd;
    if(dnO[i].x<-20){
      dnO[i].active=false;
      dnScore++;if(dnScore>gHi[5]){gHi[5]=dnScore;saveHiScore(5);}
      beepSuccess();
      dnSpd=min(8.5f,3.0f+dnScore*0.14f);
    }
  }
  // Spawn: 1 obs until score 5, 2 until score 15, 3 after
  int maxA=(dnScore>=15)?3:(dnScore>=5)?2:1;
  if(activeCount<maxA){
    float rx=0;for(int i=0;i<DN_MAX;i++)if(dnO[i].active)rx=max(rx,dnO[i].x);
    if(rx<80||activeCount==0) spawnDinoObs();
  }
  // Collision
  int dy=(int)dnY;
  for(int i=0;i<DN_MAX;i++){
    if(!dnO[i].active)continue;
    int ox=(int)dnO[i].x;
    if(!dnO[i].bird){
      if(ox<24&&ox+dnO[i].w>12&&dy+10>DN_GND-dnO[i].h){gameOver=true;beepFail();inGameOverScreen=true;return;}
    } else {
      int by=DN_GND-dnO[i].h-2;
      if(ox<24&&ox+dnO[i].w>12&&dy+10>by&&dy<by+dnO[i].h){gameOver=true;beepFail();inGameOverScreen=true;return;}
    }
  }
}

void updateMemory(){
  if(!mmAwaiting&&!mmDone){
    if(mmGap){if(millis()>mmGapT){mmGap=false;mmBlinkT=millis()+700;}}
    else if(millis()>mmBlinkT){
      int prev=mmBlinkIdx; mmBlinkIdx++;
      if(mmBlinkIdx>=mmSeqLen){mmDone=true;mmAwaiting=true;mmSeqIdx=0;}
      else if(mmSeq[mmBlinkIdx]==mmSeq[prev]){mmGap=true;mmGapT=millis()+250;if(soundOn)tone(BUZZER_PIN,1600,120);}
      else mmBlinkT=millis()+700;
    }
  }
}

void updateReaction(){
  if(rxDone)return;
  if(!rxActive&&millis()>rxShowAt){rxActive=true;rxStart=millis();}
}

void updateShooter(){
  if(millis()-holdMoveT>60){holdMoveT=millis();if(bL)ssShipX=max(4,ssShipX-5);if(bR)ssShipX=min(116,ssShipX+5);}
  if(millis()-ssLastMove>50){
    ssLastMove=millis();
    for(int i=0;i<SS_E;i++){if(!ssEA[i])continue;ssEX[i]-=2;
      if(ssEX[i]<-10){ssEX[i]=random(128,200);ssEY[i]=random(4,38);}
      if(ssEX[i]<ssShipX+9&&ssEX[i]+8>ssShipX&&ssEY[i]+9>46&&ssEY[i]<58){gameOver=true;beepFail();if(ssScore>gHi[7]){gHi[7]=ssScore;saveHiScore(7);}inGameOverScreen=true;return;}
    }
  }
  for(int i=0;i<SS_B;i++){if(!ssBA[i])continue;ssBY[i]-=5;if(ssBY[i]<0){ssBA[i]=false;continue;}
    for(int j=0;j<SS_E;j++){if(!ssEA[j])continue;
      if(ssBX[i]>=ssEX[j]&&ssBX[i]<=ssEX[j]+8&&ssBY[i]>=ssEY[j]&&ssBY[i]<=ssEY[j]+9){
        ssBA[i]=false;ssEX[j]=random(128,220);ssEY[j]=random(4,38);ssScore+=10;beepSuccess();
        if(ssScore>gHi[7]){gHi[7]=ssScore;saveHiScore(7);}break;
      }
    }
  }
}

void updateCatch(){
  if(millis()-holdMoveT>60){holdMoveT=millis();if(bL)ctPadX=max(4,ctPadX-6);if(bR)ctPadX=min(100,ctPadX+6);}
  if(millis()-ctSpawnT>max(400UL,1200UL-(unsigned long)(ctScore*30))&&ctScore<40){
    ctSpawnT=millis();
    for(int i=0;i<4;i++)if(!ctIOn[i]){ctIX[i]=random(8,120);ctIY[i]=4;ctIOn[i]=true;break;}
  }
  for(int i=0;i<4;i++){
    if(!ctIOn[i])continue; ctIY[i]+=2;
    if(ctIY[i]>58){ctIOn[i]=false;ctLives--;beepFail();
      if(ctLives<=0){gameOver=true;inGameOverScreen=true;if(ctScore>gHi[10]){gHi[10]=ctScore;saveHiScore(10);}}}
    else if(ctIY[i]>=55&&abs(ctIX[i]-ctPadX-13)<16){ctIOn[i]=false;ctScore++;beepSuccess();if(ctScore>gHi[10]){gHi[10]=ctScore;saveHiScore(10);}}
  }
}

void updateDodge(){
  if(millis()-holdMoveT>60){holdMoveT=millis();if(bL)dgX=max(6,dgX-7);if(bR)dgX=min(122,dgX+7);}
  if(millis()-dgSpawnT>max(300UL,1000UL-(unsigned long)(dgScore*20))){
    dgSpawnT=millis();
    for(int i=0;i<6;i++)if(!dgBOn[i]){dgBX[i]=random(8,100);dgBY[i]=-12;dgBW[i]=random(12,30);dgBOn[i]=true;break;}
  }
  for(int i=0;i<6;i++){
    if(!dgBOn[i])continue; dgBY[i]+=(int)dgSpd;
    if(dgBY[i]>57){dgBOn[i]=false;dgScore++;beepSuccess();if(dgScore>gHi[11]){gHi[11]=dgScore;saveHiScore(11);}dgSpd=min(5.0f,2.0f+dgScore*0.05f);}
    if(dgBX[i]<dgX+6&&dgBX[i]+dgBW[i]>dgX-4&&dgBY[i]+10>48&&dgBY[i]<56){gameOver=true;beepFail();if(dgScore>gHi[11]){gHi[11]=dgScore;saveHiScore(11);}inGameOverScreen=true;return;}
  }
}

void updateTunnel(){
  tnVY+=0.15f;tnVY=constrain(tnVY,-3.5f,3.5f);tnY+=tnVY;tnPhase+=tnSpd;
  tnSpd=min(0.12f,0.05f+tnScore*0.002f);
  float ph=tnPhase+(128-20)*tnSpd*2;
  int top=(int)(20+10*sinf(ph));
  int bot=(int)(top+max(16,28-(int)(tnScore/8)));
  if((int)tnY<=top+3||(int)tnY>=bot-3){gameOver=true;beepFail();if(tnScore>gHi[12]){gHi[12]=tnScore;saveHiScore(12);}inGameOverScreen=true;return;}
  if(millis()-tnTick>800){tnScore++;tnTick=millis();}
}

void updateReflex(){
  if(rfWaiting){if(millis()-rfWaitT>1000){rfWaiting=false;rfX=random(0,2)?10:118;rfDir=rfX<64?1:-1;rfSpd+=0.3f;}return;}
  rfX+=rfDir*rfSpd;
  if(rfX>=122){rfX=122;rfDir=-1;}if(rfX<=4){rfX=4;rfDir=1;}
}

void updateWhack(){
  if(wkBlinkPhase==1){if(millis()>wkBlinkT){wkVisible=false;wkBlinkPhase=2;wkBlinkT=millis()+180;}}
  else if(wkBlinkPhase==2){if(millis()>wkBlinkT){wkVisible=true;wkBlinkPhase=0;wkHideT=millis()+(unsigned long)wkWindow;}}
  else if(!wkVisible){
    if(millis()>wkShowT){
      int nh=random(0,3); wkHole=nh; wkVisible=true;
      if(nh==wkPrevHole){wkBlinkPhase=1;wkBlinkT=millis()+130;if(soundOn)tone(BUZZER_PIN,1900,100);}
      else{wkBlinkPhase=0;wkHideT=millis()+(unsigned long)wkWindow;}
      wkPrevHole=nh;
    }
  } else {
    if(millis()>wkHideT){wkVisible=false;wkMissed++;beepFail();
      wkShowT=millis()+random(400,1200);
      if(wkMissed>=3){gameOver=true;inGameOverScreen=true;if(wkScore>gHi[14]){gHi[14]=wkScore;saveHiScore(14);}}
    }
  }
}

//
//  UTILITY
//
String fmtTime(){
  int h=timeClient.getHours(),m=timeClient.getMinutes();
  if(!use24Hour){if(h==0)h=12;else if(h>12)h-=12;}
  return String(h)+":"+(m<10?"0":"")+String(m);
}
String fmtDate(){
  time_t raw=timeClient.getEpochTime();struct tm*ti=localtime(&raw);
  const char*days[]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  const char*mons[]={"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
  return String(days[ti->tm_wday])+", "+String(mons[ti->tm_mon])+" "+String(ti->tm_mday);
}
float displayTemp(){return useCelsius?temperature:temperature*9.0f/5.0f+32.0f;}
String weatherCond(){
  if(weatherCode==0)return F("Clear");
  if(weatherCode==1)return F("Mostly Clear");
  if(weatherCode==2)return F("Partly Cloudy");
  if(weatherCode==3)return F("Overcast");
  if(weatherCode<=48)return F("Foggy");
  if(weatherCode<=82)return F("Rainy");
  if(weatherCode<=99)return F("Stormy");
  return F("Unknown");
}
void drawWrappedText(String s,int x,int y,int mw,int lh,int ml){
  int lines=0;
  while(s.length()>0&&lines<ml){
    int len=1; uint16_t w;
    w=display.getStrWidth(s.substring(0,len+1).c_str());
    while(len<(int)s.length()&&w<=mw){len++;w=display.getStrWidth(s.substring(0,len+1).c_str());}
    int br=len;if(len<(int)s.length())for(int k=len;k>0;k--)if(s[k]==' '||s[k]=='-'){br=k;break;}
    display.setCursor(x,y+lines*lh);display.print(s.substring(0,br));
    s=s.substring(br);s.trim();lines++;
  }
}

void fetchWeather(){
  if(WiFi.status()!=WL_CONNECTED)return;
  HTTPClient http;
  String url=F("https://api.open-meteo.com/v1/forecast?latitude=");
  url+=String(LATITUDE,2);url+=F("&longitude=");url+=String(LONGITUDE,2);
  url+=F("&current=temperature_2m,relative_humidity_2m,weather_code,wind_speed_10m&daily=sunset&timezone=auto");
  http.begin(url);
  if(http.GET()==200){
    DynamicJsonDocument doc(2048);deserializeJson(doc,http.getString());
    JsonObject cur=doc[F("current")];
    temperature=cur[F("temperature_2m")].as<float>();
    weatherCode=cur[F("weather_code")].as<int>();
    windSpeed=cur[F("wind_speed_10m")].as<float>();
    humidity=cur[F("relative_humidity_2m")].as<int>();
    JsonObject daily=doc[F("daily")];
    if(daily.containsKey(F("sunset"))){
      const char*ss=daily[F("sunset")][0];
      if(ss){String sv=String(ss);int t=sv.indexOf('T');if(t>=0)sunsetTime=sv.substring(t+1,t+6);}
    }
    lastWeatherUpdate=millis();
  }
  http.end();
}

void fetchCrypto(){
  if(WiFi.status()!=WL_CONNECTED)return;
  CrAsset& A=crAssets[crSel];
  WiFiClientSecure cl; cl.setInsecure();
  HTTPClient http;
  http.setTimeout(12000);

  if(crSel<2){
    // -- Kraken OHLC  240min candles, last 20 periods (~3.3 days) --
    // 240min gives small JSON (~720 bytes per candle * 20 = 14KB max)
    // Use 'since' = now - 21*4h so we get exactly ~21 rows back
    unsigned long ep=timeClient.getEpochTime();
    if(ep<1000000000UL) ep=1735689600UL;
    unsigned long since=ep - 21UL*14400UL;
    String url="https://api.kraken.com/0/public/OHLC?pair=";
    url+=CR_PAIRS[crSel];
    url+="&interval=240&since=";
    url+=String(since);
    http.begin(cl,url);
    int rc=http.GET();
    if(rc==200){
      // Read full body first  getStream() cuts off on ESP32+TLS
      String body=http.getString();
      DynamicJsonDocument doc(16384);
      DeserializationError err=deserializeJson(doc,body);
      if(!err && doc["error"].as<JsonArray>().size()==0){
        JsonArray arr;
        for(JsonPair kv:doc["result"].as<JsonObject>()){
          if(strcmp(kv.key().c_str(),"last")!=0){arr=kv.value().as<JsonArray>();break;}
        }
        if(!arr.isNull()&&arr.size()>0){
          int total=(int)arr.size();
          int start=max(0,total-20);
          A.count=0;
          for(int i=start;i<total&&A.count<20;i++){
            JsonArray c=arr[i];
            // Kraken values are quoted strings: "95000.00000"
            A.candles[A.count].o=atof(c[1]|"0");
            A.candles[A.count].h=atof(c[2]|"0");
            A.candles[A.count].l=atof(c[3]|"0");
            A.candles[A.count].c=atof(c[4]|"0");
            A.count++;
          }
          if(A.count>0) A.price=A.candles[A.count-1].c;
          A.loaded=true; A.lastUpd=millis();
        }
      }
    }
    http.end();
  } else {
    // -- Yahoo Finance (SPY  1h candles, last 3 days) --
    http.begin(cl,"https://query1.finance.yahoo.com/v8/finance/chart/SPY?interval=60m&range=3d");
    http.addHeader("User-Agent","Mozilla/5.0");
    int rc=http.GET();
    if(rc==200){
      String body=http.getString();
      DynamicJsonDocument doc(9216);
      if(!deserializeJson(doc,body)){
        JsonObject result=doc["chart"]["result"][0];
        float curP=result["meta"]["regularMarketPrice"]|0.0f;
        JsonArray opens =result["indicators"]["quote"][0]["open"].as<JsonArray>();
        JsonArray highs =result["indicators"]["quote"][0]["high"].as<JsonArray>();
        JsonArray lows  =result["indicators"]["quote"][0]["low"].as<JsonArray>();
        JsonArray closes=result["indicators"]["quote"][0]["close"].as<JsonArray>();
        if(closes.size()>0){
          int total=(int)closes.size();
          int start=max(0,total-20);
          A.count=0;
          for(int i=start;i<total&&A.count<20;i++){
            float o=opens[i]|0.0f,h=highs[i]|0.0f,l=lows[i]|0.0f,c=closes[i]|0.0f;
            if(c==0.0f) continue; // skip null candles
            A.candles[A.count]={o>0?o:c, h>0?h:c, l>0?l:c, c};
            A.count++;
          }
          A.price=(curP>0)?curP:(A.count>0?A.candles[A.count-1].c:0);
          A.loaded=true; A.lastUpd=millis();
        }
      }
    }
    http.end();
  }
}

void drawCrypto(){
  CrAsset& A=crAssets[crSel];
  setDisplayFont(1);
  display.setCursor(2,1); display.print(CR_NAMES[crSel]);
  if(A.price>0){
    String ps;
    if(A.price>=10000) ps=String((int)A.price)+CR_UNITS[crSel];
    else if(A.price>=100) ps=String(A.price,1)+CR_UNITS[crSel];
    else ps=String(A.price,2)+CR_UNITS[crSel];
    uint16_t pw=display.getStrWidth(ps.c_str());
    display.setCursor(126-pw,1); display.print(ps);
  }
  display.setDrawColor(1); display.drawHLine(0,10,128);

  if(!A.loaded){
    drawCenteredText(F("Loading..."),30,1);
    setDisplayFont(0);
    display.setCursor(2,54); display.print(F("L/R switch  M refresh"));
    return;
  }
  if(A.count<2){drawCenteredText(F("No data"),30,1);return;}

  const int CHART_Y=12, CHART_H=51, CHART_X=0;
  const int CW=4, GAP=2;
  int n=min(A.count,20);
  int totalW=n*(CW+GAP)-GAP;
  int xOff=CHART_X+(128-totalW)/2;

  float pmin=A.candles[A.count-n].l;
  float pmax=A.candles[A.count-n].h;
  for(int i=A.count-n;i<A.count;i++){
    if(A.candles[i].l<pmin)pmin=A.candles[i].l;
    if(A.candles[i].h>pmax)pmax=A.candles[i].h;
  }
  float rng=pmax-pmin; if(rng<0.01f)rng=0.01f;

  for(int ci=0;ci<n;ci++){
    int idx=A.count-n+ci;
    CrCandle& cd=A.candles[idx];
    int x0=xOff+ci*(CW+GAP);
    int xm=x0+CW/2; // wick center x

    #define PY(p) (CHART_Y+CHART_H-1-(int)(((p)-pmin)/rng*(CHART_H-1)+0.5f))
    int yH=PY(cd.h), yL=PY(cd.l);
    int yO=PY(cd.o), yC=PY(cd.c);
    #undef PY

    // Wick
    display.setDrawColor(1);
    display.drawLine(xm,yH,xm,yL);

    // Body
    int by=min(yO,yC); int bh=abs(yC-yO); if(bh<1)bh=1;
    if(cd.c>=cd.o){
      // Bullish (close >= open): hollow candle
      display.setDrawColor(1); display.drawFrame(x0,by,CW,bh);
    } else {
      // Bearish: filled candle
      display.setDrawColor(1); display.drawBox(x0,by,CW,bh);
    }
  }

  // Price scale: show pmin and pmax on right edge
  display.setDrawColor(1);
  setDisplayFont(0);
  // Format price labels (no decimals for large values)
  String smax=pmax>=1000?String((int)pmax):(pmax>=10?String(pmax,1):String(pmax,2));
  String smin=pmin>=1000?String((int)pmin):(pmin>=10?String(pmin,1):String(pmin,2));
  display.setCursor(128-display.getStrWidth(smax.c_str()),12); display.print(smax);
  display.setCursor(128-display.getStrWidth(smin.c_str()),56); display.print(smin);

  // Nav hint + age
  setDisplayFont(0);
  if(A.lastUpd>0){
    unsigned long age=(millis()-A.lastUpd)/1000;
    String ag=age<60?String(age)+"s":String(age/60)+"m";
    display.setCursor(2,57); display.print(ag);
  }
}

void fetchQuote(){
  if(WiFi.status()!=WL_CONNECTED)return;
  HTTPClient http;http.begin(F("https://zenquotes.io/api/random"));
  if(http.GET()==200){
    DynamicJsonDocument doc(1024);deserializeJson(doc,http.getString());
    JsonObject obj=doc[0];quoteText=obj["q"].as<String>();quoteAuthor=obj["a"].as<String>();
    lastQuoteUpdate=millis();
  }
  http.end();
}


