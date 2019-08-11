/**
https://docs.m5stack.com/#/en/quick_start/m5stickc/m5stickc_quick_start

INSTALL:
  dev-embedded/arduino-1.8.7, dev-python/pyserial-3.2.1
  # emerge -av arduino pyserial

Arduino IDE:
  File->Peferences->Settings
    Additional Boards Manager URLs: https://dl.espressif.com/dl/package_esp32_index.json
  Tools->Board:->Boards Manager...
    esp32 by Espressif System version 1.0.2
      https://github.com/espressif/arduino-esp32
  Sketch->Include Library->Manage Libraries...
    M5StickC by M5StickC Version 0.0.7
      https://github.com/m5stack/M5StickC


https://github.com/m5stack/M5StickC/tree/master/src
https://github.com/espressif/arduino-esp32/tree/master/libraries
*/

#include <M5StickC.h>
//#include <EEPROM.h>  
//#include <Preferences.h>  // https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences/src
//#include <Ticker.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <time.h>


const char *apSSID = "M5StickC";
const char *apPassword = "";
WebServer server(80);
const IPAddress apAddr(192, 168, 4, 1);
const IPAddress gwAddr(192, 168, 4, 1);
const IPAddress subnet(255, 255, 255, 0);

const char *ntpServer[] = {"ntp.jst.mfeed.ad.jp", "ntp.nict.jp", "jp.pool.ntp.org"};
const char *timeZone = "JST-9";
WiFiClient client;
IPAddress addr;

RTC_DateTypeDef rtcDate;
RTC_TimeTypeDef rtcTime;
const char *weekday[] = {"Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"};

typedef struct { int16_t x; int16_t y; int16_t z; } accelerometer_t;
typedef struct { int16_t x; int16_t y; int16_t z; } gyroscope_t;
//typedef struct { float x; float y; float z; } accelerometer_t;
//typedef struct { float x; float y; float z; } gyroscope_t;
typedef struct { accelerometer_t accl; gyroscope_t gyro; } imu_t;

typedef struct { float x; float y; float z; } correction_value_t;
typedef struct { correction_value_t accl; correction_value_t gyro; } imu_correct_t;

typedef struct {
  float roll;
  float pitch;
  float yaw;
  float eulerAngles[3];
  float quaternion[4];
} attitude_t;

#define IMU_SAMPLES 5
imu_t imu6a[IMU_SAMPLES];
imu_t imu6;
const imu_correct_t imu6c = {{-0.01, -0.09, -0.11}, {-1.0, 0.0, 8.0}};
attitude_t attitude = {0,0,0, {0,0,0}, {0,0,0,0}};

typedef enum {
  STATE_WATCH_HMS,
  STATE_WATCH_HM,
  STATE_IMU,
  STATE_WIFI_STA,
  STATE_WIFI_AP,
  STATE_LOW_BATTERY
} state_t;
state_t state = STATE_WATCH_HMS;

// 7 ~ 15
#define BRIGHTNESS_FULL 15
#define BRIGHTNESS_HIGH 10
#define BRIGHTNESS_LOW  8

uint8_t sleepCount = 0;

bool isFlash = false;
uint8_t flashCount = 0;

long startTime = 0;
long loopTime = 0;
long elapsedTime = 0;

uint8_t counter = 128;
bool isHandling = true;


//void drawTask(void *arg) { while(1){ vTaskDelay(100 / portTICK_RATE_MS); } }

/**
 * put your setup code here, to run once:
 */
void setup() {
  M5.begin();
  M5.IMU.Init();

  pinMode(M5_BUTTON_HOME, INPUT);
  pinMode(M5_BUTTON_RST, INPUT);
  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, HIGH);  // LOW=on, HIGH=off

  M5.Axp.ScreenBreath(BRIGHTNESS_HIGH);
  M5.Lcd.setRotation(3);

  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
  M5.Lcd.fillScreen(TFT_BLACK);

  // HOME + RST
  if(digitalRead(M5_BUTTON_HOME) == LOW && digitalRead(M5_BUTTON_RST) == LOW){
    digitalWrite(M5_LED, LOW);
    while(digitalRead(M5_BUTTON_HOME) == LOW || digitalRead(M5_BUTTON_RST) == LOW);
    digitalWrite(M5_LED, HIGH);
    M5.Lcd.println("RESET Preferences!");
    delay(1000);
    M5.Lcd.fillScreen(TFT_BLACK);
  }

  // figlet "M5"; figlet -f small "Stick C"
  //  __  __ ____  
  // |  \/  | ___| 
  // | |\/| |___ \ 
  // | |  | |___) |
  // |_|  |_|____/ 
  //  ___ _   _    _      ___ 
  // / __| |_(_)__| |__  / __|
  // \__ \  _| / _| / / | (__ 
  // |___/\__|_\__|_\_\  \___|
  M5.Lcd.printf("  __  __ ____  \n |  \\/  | ___| \n | |\\/| |___ \\ \n | |  | |___) |\n |_|  |_|____/ \n");
  M5.Lcd.printf("  ___ _   _    _      ___ \n / __| |_(_)__| |__  / __|\n \\__ \\  _| / _| / / | (__ \n |___/\\__|_\\__|_\\_\\  \\___|");
  delay(3000);

  //setCpuFrequencyMhz(10);  //クロック下げてWiFiを使用するとフリーズ?

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
  esp_sleep_enable_timer_wakeup(1000000);  // microseconds
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, 0);  // M5_BUTTON_HOME

  M5.Rtc.GetData(&rtcDate);
  M5.Rtc.GetTime(&rtcTime);
  startTime = millis();
  loopTime = startTime;
  elapsedTime = 0;

  changeState(STATE_WATCH_HMS);

  //xTaskCreatePinnedToCore(drawTask, "drawTask", 2048, NULL, 1, NULL, 1);
}

/**
 * put your main code here, to run repeatedly:
 */
void loop() {
  M5.update();
  uint8_t btn = 0x00;
  btn = 0x01 * M5.BtnA.wasReleased() + 0x10 * M5.BtnB.wasReleased();

  loopTime = millis();

  // HOME + RST
  if(digitalRead(M5_BUTTON_HOME) == LOW && digitalRead(M5_BUTTON_RST) == LOW){
    digitalWrite(M5_LED, LOW);
    while(digitalRead(M5_BUTTON_HOME) == LOW || digitalRead(M5_BUTTON_RST) == LOW);
    digitalWrite(M5_LED, HIGH);
    switch(state){
      case STATE_WATCH_HM:
        changeState(STATE_WIFI_STA);
        setRtcByWifiNtp();
        break;
      case STATE_WATCH_HMS:
        changeState(STATE_WIFI_AP);
        setRtcByWifiAp();
        break;
      default: break;
    }
  }

  if(digitalRead(M5_BUTTON_HOME) == LOW){
    flashCount = 2;
    isFlash = true;
    M5.Axp.ScreenBreath(BRIGHTNESS_FULL);
    while(digitalRead(M5_BUTTON_HOME) == LOW);
  }

  if(digitalRead(M5_BUTTON_RST) == LOW){
    switch(state){
      case STATE_WATCH_HM: changeState(STATE_WATCH_HMS); break;
      case STATE_WATCH_HMS: changeState(STATE_IMU); break;
      case STATE_IMU: changeState(STATE_WATCH_HM); break;
      default: break;
    }
    while(digitalRead(M5_BUTTON_RST) == LOW);
  }

  // PowerSwitch: 0x01=long-press(1s), 0x02=press
  if(M5.Axp.GetBtnPress() == 0x02){
    esp_restart();
  }

  if(isFlash){
    flashCount++;
    if(flashCount > 5){
      flashCount = 0;
      isFlash = false;
      M5.Axp.ScreenBreath(BRIGHTNESS_LOW);
    }
  }

  if(M5.Axp.GetWarningLeve()){
    changeState(STATE_LOW_BATTERY);
  }

  if(state == STATE_IMU){
    updateIMU();
    printIMU();
  }

  elapsedTime = loopTime - startTime;
  if(elapsedTime >= 1000){
    int sec = (int)(elapsedTime / 1000);
    elapsedTime -= sec * 1000;
    rtcTime.Seconds += sec;
    if(rtcTime.Seconds > 59){
      rtcTime.Seconds -= 60;
      rtcTime.Minutes += 1;
      if(rtcTime.Minutes > 59){
        M5.Rtc.GetData(&rtcDate);
        M5.Rtc.GetTime(&rtcTime);
      }
    }

    switch(state){
      case STATE_WATCH_HMS: printDateHMS(); break;
      case STATE_WATCH_HM: printDateHM(); break;
      //case STATE_IMU: printIMU(); break;
      //case STATE_WIFI_AP: break;
      //case STATE_WIFI_STA: break;
      case STATE_LOW_BATTERY:
        sleepCount++;
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(0, 16);
        M5.Lcd.print(" ! WARNING ! ");
        M5.Lcd.setCursor(0, 48);
        M5.Lcd.print(" LOW BATTERY ");
        if(sleepCount > 10){
          sleepCount = 0;
          changeState(STATE_WATCH_HMS);
          M5.Axp.SetSleep();
        }
        break;
      default: break;
    }

    startTime = loopTime;
  }

  esp_light_sleep_start();
}


void printBattery() {
  M5.Lcd.setCursor(0, 72);
  M5.Lcd.setTextSize(1);
  double vbat = M5.Axp.GetVbatData() * 1.1 / 1000;  // バッテリー電圧 step:1.1mV
  //double batp = (M5.Axp.GetVbatData() - 3.3) / 0.89 * 100  //3.3 ~ 4.19V ?
  double batp = (M5.Axp.GetVbatData() - 2690) * 0.09;  // https://twitter.com/lovyan03/status/1112728410835419141
  double batt = -144.7 + M5.Axp.GetTempData() * 0.1;  // バッテリー温度
  int charge = M5.Axp.GetIchargeData() * 0.5;  // 充電電流mA
  if(charge == 0){
    charge = -(M5.Axp.GetIdischargeData() * 0.5);  // 放電電流mA
  }
  M5.Lcd.printf(" %3.0f%%(%4.2fV) %4.1fC %+4imA ", batp, vbat, batt, charge);
}

void printDateHMS() {
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("%04d-%02d-%02d %s", rtcDate.Year, rtcDate.Month, rtcDate.Date, weekday[rtcDate.WeekDay]);
  M5.Lcd.setCursor(4, 36);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("%02d:%02d:%02d", rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds);
  counter++;
  if(counter > 5){
    counter = 0;
    printBattery();
  }
}

void printDateHM() {
  counter++;
  if(counter > 10){
    counter = 0;
    M5.Lcd.setCursor(0, 8);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("%04d-%02d-%02d %s", rtcDate.Year, rtcDate.Month, rtcDate.Date, weekday[rtcDate.WeekDay]);
    M5.Lcd.setCursor(4, 24);
    M5.Lcd.setTextSize(5);
    M5.Lcd.printf("%02d:%02d", rtcTime.Hours, rtcTime.Minutes);
    printBattery();
  }
}

void updateIMU() {
  M5.IMU.getGyroAdc(&imu6.gyro.x, &imu6.gyro.y, &imu6.gyro.z);
  M5.IMU.getAccelAdc(&imu6.accl.x, &imu6.accl.y, &imu6.accl.z);
  //M5.IMU.getGyroData(&imu6.gyro.x, &imu6.gyro.y, &imu6.gyro.z);
  //M5.IMU.getAccelData(&imu6.accl.x, &imu6.accl.y, &imu6.accl.z);

  //for(int i = IMU_SAMPLES - 1; i > 0; i--){ imu6a[i] = imu6a[i - 1]; }
  //imu6a[0] = imu6;
  for(int i = 0; i < IMU_SAMPLES - 1; i++){ imu6a[i] = imu6a[i + 1]; }
  imu6a[IMU_SAMPLES - 1] = imu6;

  imu6 = {{0,0,0}, {0,0,0}};
  for(int i = 0; i < IMU_SAMPLES; i++){
    imu6 = { {
      imu6.accl.x + imu6a[i].accl.x,
      imu6.accl.y + imu6a[i].accl.y,
      imu6.accl.z + imu6a[i].accl.z
    }, {
      imu6.gyro.x + imu6a[i].gyro.x,
      imu6.gyro.y + imu6a[i].gyro.y,
      imu6.gyro.z + imu6a[i].gyro.z
    } };
  }
  imu6 = { {
    imu6.accl.x / IMU_SAMPLES,
    imu6.accl.y / IMU_SAMPLES,
    imu6.accl.z / IMU_SAMPLES
  }, {
    imu6.gyro.x / IMU_SAMPLES,
    imu6.gyro.y / IMU_SAMPLES,
    imu6.gyro.z / IMU_SAMPLES
  } };

  float ax = ((float)imu6.accl.x) * M5.IMU.aRes + imu6c.accl.x;
  float ay = ((float)imu6.accl.y) * M5.IMU.aRes + imu6c.accl.y;
  float az = ((float)imu6.accl.z) * M5.IMU.aRes + imu6c.accl.z;
  int gx = (int)(((float)imu6.gyro.x) * M5.IMU.gRes + imu6c.gyro.x);
  int gy = (int)(((float)imu6.gyro.y) * M5.IMU.gRes + imu6c.gyro.y);
  int gz = (int)(((float)imu6.gyro.z) * M5.IMU.gRes + imu6c.gyro.z);
  float roll = atan2(ay, -ax);
  float pitch = -atan2(az, sqrt(ax*ax + ay*ay));
  float yaw = 2 * PI;

  attitude = {roll,pitch,yaw, {0,0,0}, {0,0,0,0}};  // roll, pitch, yaw, eulerAngles[3], Quaternion[4]
}

/**
 * Rotation Matrix
 *   sx,sy: source X,Y
 *   rad: rotation angle(radian)
 *   dx,dy: destination X,Y
 *   ox,oy: origin X,Y  (0,0)
 */
void rotate(int sx, int sy, float rad, int *dx, int *dy) {
  float sinT = sin(rad);
  float cosT = cos(rad);
  *dx = (int)(sx * cosT - sy * sinT);
  *dy = (int)(sx * sinT + sy * cosT);
}
void rotate(int sx, int sy, float rad, int *dx, int *dy, int ox, int oy) {
  float sinT = sin(rad);
  float cosT = cos(rad);
  int x = sx - ox;
  int y = sy - oy;
  *dx = (int)(x * cosT - y * sinT) + ox;
  *dy = (int)(x * sinT + y * cosT) + oy;
}

// (80,40)を原点として(sx,sy)から(dx,dy),(-sx,sy)から(-dx,dy)への線分をrad回転させて描画
// x:0~160 y:0~80 | x:-80~80 y:-40~40
void drawLines(int sx, int sy, int dx, int dy, float rad) {
  int rsx,rsy, rdx,rdy;
  rotate(sx,sy, rad, &rsx,&rsy);
  rotate(dx,dy, rad, &rdx,&rdy);
  M5.Lcd.drawLine(rsx + 80,rsy + 40, rdx + 80,rdy + 40, TFT_GREEN);
  rotate(-sx,sy, rad, &rsx,&rsy);
  rotate(-dx,dy, rad, &rdx,&rdy);
  M5.Lcd.drawLine(rsx + 80,rsy + 40, rdx + 80,rdy + 40, TFT_GREEN);
}

void printIMU() {
  M5.Lcd.fillRect(0,0, 160,80, TFT_BLACK);
  M5.Lcd.drawCircle(80,40, 68, TFT_GREENYELLOW);
  //M5.Lcd.drawTriangle(10,70, 80,40, 150,70, TFT_GREENYELLOW);

  // Center
  M5.Lcd.drawCircle(80,40, 2, TFT_GREEN);
  M5.Lcd.drawLine(70,40, 90,40, TFT_GREEN);
  M5.Lcd.drawLine(80,35, 80,40, TFT_GREEN);

  int sx = 15, dx = 35;
  int deg = attitude.pitch * RAD_TO_DEG;
  int y = (deg * 3) % 15;
  int n;
  for(int i = 0; i < 7; i++){
    n = y - 45 + i * 15;
    if(
      (i <= 3 && -20 + (i*5) < deg && deg <= -15 + (i*5)) ||
      (i >= 3 && 0 + (i-3)*5 <= deg && deg < 5 + (i-3)*5)
    ){
      drawLines(sx,n, dx+20,n, attitude.roll);  // horizon
    }else{
      drawLines(sx,n, dx,n, attitude.roll);  // per 5deg
    }
  }

  M5.Lcd.setCursor(59, 0);
  M5.Lcd.printf("Yaw:%3.0f", attitude.yaw * RAD_TO_DEG);  // 0 ~ 360
  M5.Lcd.setCursor(0, 36);
  M5.Lcd.printf("Pitch:%3.0f", attitude.pitch * RAD_TO_DEG);  // -90 ~ 90
  M5.Lcd.setCursor(53, 64);
  M5.Lcd.printf("Roll:%4.0f", attitude.roll * RAD_TO_DEG);  // -180 ~ 180

  printBattery();
}


void setRtcDate(uint16_t y, uint8_t m, uint8_t d, uint8_t w) {
  rtcDate.Year = y;
  rtcDate.Month = m;
  rtcDate.Date = d;
  rtcDate.WeekDay = w;
  M5.Rtc.SetData(&rtcDate);
}

void setRtcTime(uint8_t h, uint8_t m, uint8_t s) {
  rtcTime.Hours = h;
  rtcTime.Minutes = m;
  rtcTime.Seconds = s;
  M5.Rtc.SetTime(&rtcTime);
  startTime = millis();
  loopTime = startTime;
  elapsedTime = 0;
}


void changeState(state_t s) {
  state = s;
  flashCount = 0;
  if(! isFlash){
    isFlash = true;
    M5.Axp.ScreenBreath(BRIGHTNESS_HIGH);
  }
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0,0, 1);
  counter = 128;

  if(state == STATE_IMU){
    M5.Lcd.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
    esp_sleep_enable_timer_wakeup( 125000);
  }else{
    M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
    esp_sleep_enable_timer_wakeup(1000000);
  }
}


/**
 * Connect to AUTH_OPEN WiFi and set RTC
 */
void setRtcByWifiNtp() {
  M5.Lcd.setCursor(0, 8);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println(" #### WiFi STA ####");

  WiFi.mode(WIFI_STA);  //WIFI_AP, WIFI_AP_STA
  WiFi.disconnect();
  delay(250);
  bool isUpdated = false;
  int n = WiFi.scanNetworks();
  if(n == 0){
    M5.Lcd.println("AP not found!");
  }else{
    M5.Lcd.printf("APs: %d\n", n);
    for(int i = 0; i < n; i++){
      if(WiFi.encryptionType(i) != WIFI_AUTH_OPEN){ continue; }
      char *pass = "";
      String s = WiFi.SSID(i);
      M5.Lcd.setCursor(0, 24);
      M5.Lcd.printf("AP: %d %.16s\n", WiFi.RSSI(i), s.c_str());  // RSSI dBm
      WiFi.begin(s.c_str(), pass);
      bool isTimeout = false;
      int timeout = 0;
      while(WiFi.status() != WL_CONNECTED){
        timeout++;
        if(timeout > 50){
          timeout = 0;
          isTimeout = true;
          break;
        }
        delay(100);
      }
      if(isTimeout){ continue; }
      addr = WiFi.localIP();
      M5.Lcd.printf("IP: %d.%d.%d.%d\n", addr[0], addr[1], addr[2], addr[3]);
      //configTime(9 * 3600, 0, ntpServer[0], ntpServer[1], ntpServer[2]);
      configTzTime(timeZone, ntpServer[0], ntpServer[1], ntpServer[2]);
      struct tm timeInfo;
      timeout = 0;
      while(! getLocalTime(&timeInfo)){
        timeout++;
        if(timeout > 50){
          timeout = 0;
          isTimeout = true;
          break;
        }
        delay(100);
      }
      if(isTimeout){ continue; }
      setRtcDate(timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_wday);
      setRtcTime(timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
      isUpdated = true;
      break;
    }
  }
  if(isUpdated){
    M5.Lcd.setTextSize(2);
    M5.Lcd.println(" RTC Updated ");
  }else{
    M5.Lcd.println("AUTH_OPEN AP not found!");
  }
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  delay(3000);
  changeState(STATE_WATCH_HMS);
}

/**
 * Access the web server on WiFi-AP from the browser and set RTC
 */
void setRtcByWifiAp() {
  M5.Lcd.setCursor(0, 8);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println(" #### WiFi AP ####");

  WiFi.softAPConfig(apAddr, gwAddr, subnet);
  WiFi.softAP(apSSID, apPassword);  // (const char* ssid, const char* passphrase = NULL, int channel = 1, int ssid_hidden = 0)
  WiFi.mode(WIFI_AP);
  addr = WiFi.softAPIP();
  M5.Lcd.printf("AP: %s\n", apSSID);
  M5.Lcd.printf("IP: %d.%d.%d.%d\n", addr[0], addr[1], addr[2], addr[3]);
  M5.Lcd.print("waiting.");

  server.on("/", []() {
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1' />";
    HTTPMethod m = server.method();
    switch(m){
      case HTTP_GET:
        M5.Lcd.print("*");
        html += "<script>function p(n){ return n.toString().padStart(2, '0'); } window.addEventListener('load', function(e){ let d = new Date();";
        html += "document.f.i.value = `${d.getFullYear()}${p(d.getMonth()+1)}${p(d.getDate())}${d.getDay()}${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;";
        html += "if(location.search === ''){ document.f.submit(); } });</script>";
        html += "</head><body><form method='POST' name='f'><input type='text' name='i' value='209912314235959' /><input type='submit'></form></body></html>";
        server.send(200, "text/html", html);
        break;
      case HTTP_POST:
        String reqParam = server.arg("i");
        if(reqParam.length() == 15){  // i=209912314235959
          M5.Lcd.println("@");
          setRtcDate(reqParam.substring(0, 4).toInt(), reqParam.substring(4, 6).toInt(), reqParam.substring(6, 8).toInt(), reqParam.substring(8, 9).toInt());
          setRtcTime(reqParam.substring(9, 11).toInt(), reqParam.substring(11, 13).toInt(), reqParam.substring(13, 15).toInt());
          html += "</head><body><p>RTC Updated</p>" + reqParam + "</body></html>";
          server.send(200, "text/html", html);
          M5.Lcd.setTextSize(2);
          M5.Lcd.println(" RTC Updated ");
          isHandling = false;
        }else{ M5.Lcd.print("!"); }
        break;
      //default: M5.Lcd.print("?"); break;  // jump to case label [-fpermissive]
    }
  });
  server.onNotFound([]() { server.send(404, "text/plain", "404 Not Found"); });
  server.begin();

  int timeout = 0;
  isHandling = true;
  while(isHandling){  // "isHandling" sets 'true' to 'false' in "server.on(...)" called from "server.handleClient()"
    timeout++;
    if(timeout % 30 == 0){ M5.Lcd.print("."); }
    if(timeout > 300){ M5.Lcd.println("Timeout!"); break; }
    server.handleClient();
    delay(100);
  }
  server.stop();
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  delay(3000);
  changeState(STATE_WATCH_HMS);
}
