/*
 * XY6020L Monitor/Controller for ESP32
 * v7: external RUN/STOP button on GPIO 25, cosmetics (degC, spacing)
 */

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT_S 15
#define WIFI_SSID "WLAN_2.4G_A8CC"
#define WIFI_PASS "DPDPC54BA8CC"

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3 * 3600;
const int   daylightOffset_sec = 0;

// ===================================================================
// PINOUT
// ===================================================================
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4
#define RXD2     16
#define TXD2     17
#define ENC_CLK  27
#define ENC_DT   14
#define ENC_SW   26
#define TOUCH_PIN 13
#define BTN_PIN   25     // external RUN/STOP button (WEIPENG, COM->GND, NO->GPIO25)

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
WebServer server(80);

#define COLOR_BG         0x0000
#define COLOR_PANEL      0x10A2
#define COLOR_BORDER     0x23DF
#define COLOR_VOLT       0xFFE0
#define COLOR_AMP        0x07FF
#define COLOR_CYAN       0x07FF
#define COLOR_WATT       0xFFFF
#define COLOR_SET        0xFDA0
#define COLOR_TEMP       0x07E0
#define COLOR_CV         0x07E0
#define COLOR_CC         0xF800
#define COLOR_TEXT_DIM   0x9CD6

float vOut = 0.0f, iOut = 0.0f, pOut = 0.0f;
float vSet = 0.0f, iSet = 0.0f, temp = 0.0f;
float ahCount = 0.0f, whCount = 0.0f;
uint32_t workSeconds = 0;
uint32_t lastTick = 0;
bool isLinkOk = false;
bool isOutputOn = false;
bool isCCMode = false;

uint8_t currentScreen = 0;
bool redrawScreenFlag = true;
float powerHistory[30] = {0};
unsigned long lastGraphUpdate = 0;
uint8_t editMode = 0;
float vStep = 0.10f, iStep = 0.10f;
int lastClkState = HIGH;
unsigned long lastEncRotate = 0;
bool pendingWrite = false;
uint16_t pendingReg = 0, pendingVal = 0;
unsigned long lastEncoderMove = 0;
bool lastTouchState = LOW;
unsigned long lastTouchDebounce = 0;

// --- ENCODER BUTTON ISR (GPIO 26) ---
volatile bool          btnIsrState      = HIGH;
volatile unsigned long btnLastEdgeMs    = 0;
volatile unsigned long btnPressStartMs  = 0;
volatile unsigned long btnPressEndMs    = 0;
volatile bool          btnPressedEvent  = false;
volatile bool          btnReleasedEvent = false;

void IRAM_ATTR btnISR() {
  unsigned long now = millis();
  if (now - btnLastEdgeMs < 25) return;
  bool cur = digitalRead(ENC_SW);
  if (cur == btnIsrState) return;
  btnLastEdgeMs = now;
  btnIsrState = cur;
  if (cur == LOW) { btnPressStartMs = now; btnPressedEvent = true; }
  else            { btnPressEndMs   = now; btnReleasedEvent = true; }
}

// --- EXTERNAL BUTTON ISR (GPIO 25) ---
volatile bool          btnExtState      = HIGH;
volatile unsigned long btnExtLastEdgeMs = 0;
volatile bool          btnExtPressed    = false;  // set on press edge

void IRAM_ATTR btnExtISR() {
  unsigned long now = millis();
  if (now - btnExtLastEdgeMs < 30) return;  // 30 ms debounce
  bool cur = digitalRead(BTN_PIN);
  if (cur == btnExtState) return;
  btnExtLastEdgeMs = now;
  btnExtState = cur;
  if (cur == LOW) {
    btnExtPressed = true;   // register press event (only on press, not release)
  }
}

#define RX_BUF_SIZE 64
uint8_t rxBuffer[RX_BUF_SIZE];
uint8_t rxLen = 0;
unsigned long lastGoodResponse = 0;
bool newFrame = false;
enum ModbusState { MB_IDLE, MB_WAIT_RESPONSE };
ModbusState mbState = MB_IDLE;
unsigned long mbSendTimestamp = 0;
uint8_t mbExpectedBytes = 0;

bool wasConnected = false;
unsigned long lastWiFiAttempt = 0;

void updateScreen1_Values();
void updateScreen2_Values();
void updateScreen3_Values();
void drawInfoScreen();
void redrawCurrentScreen();

void drawWiFiIcon(int x, int y, bool connected) {
  uint16_t activeColor = COLOR_VOLT;
  uint16_t inactiveColor = COLOR_PANEL;
  tft.fillRect(x,      y + 9, 3, 3,  activeColor);
  tft.fillRect(x + 4,  y + 6, 3, 6,  connected ? activeColor : inactiveColor);
  tft.fillRect(x + 8,  y + 3, 3, 9,  connected ? activeColor : inactiveColor);
  tft.fillRect(x + 12, y,     3, 12, connected ? activeColor : inactiveColor);
}

const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>XY6020L Web Console</title>
<style>
  body{font-family:Arial,sans-serif;background:#121212;color:#fff;text-align:center;margin:0;padding:15px;}
  .card{background:#1e1e1e;padding:15px;margin:10px auto;max-width:400px;border-radius:10px;}
  .val{font-size:28px;font-weight:bold;margin:5px 0;}
  .v{color:#ffe000;} .i{color:#07ffef;} .p{color:#ffffff;} .stat{color:#07e000;font-size:20px;font-weight:bold;}
  .badge{padding:3px 8px;border-radius:4px;font-size:14px;font-weight:bold;display:inline-block;margin-left:5px;}
  .cv{background:#07e000;color:#000;} .cc{background:#f80000;color:#fff;} .off{background:#555;color:#fff;}
  button{padding:10px 20px;font-size:16px;font-weight:bold;border:none;border-radius:5px;cursor:pointer;margin:5px;}
  .btn-on{background:#07e000;color:#000;width:100%;} .btn-off{background:#f80000;color:#fff;width:100%;}
  .btn-rst{background:#ff9800;color:#000;}
  input{width:70px;padding:6px;font-size:16px;text-align:center;border-radius:5px;border:none;}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;text-align:left;margin-top:10px;}
  .grid div{background:#2a2a2a;padding:8px;border-radius:5px;}
</style></head>
<body>
  <h2>XY6020L Web Console</h2>
  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center;">
      <span>Status: <span id="mode" class="badge cv">CV</span></span>
      <span>Temp: <span id="t">0.0</span> &deg;C</span>
    </div>
    <div>Tensio: <span id="v" class="val v">0.00</span> V</div>
    <div>Fluxus: <span id="i" class="val i">0.00</span> A</div>
    <div>Potentia: <span id="p" class="val p">0.0</span> W</div>
  </div>
  <div class="card">
    <h3>Statistica</h3>
    <div class="grid">
      <div>Capacitas: <br><span id="ah" class="stat">0.000</span> Ah</div>
      <div>Energia: <br><span id="wh" class="stat">0.00</span> Wh</div>
    </div>
    <div style="margin-top:10px;">Tempus: <b id="time">00:00:00</b></div>
    <p><button class="btn-rst" onclick="resetStats()">Reset Ah / Wh / Time</button></p>
  </div>
  <div class="card">
    <h3>Administratio</h3>
    <p>Uset: <input type="text" id="vset"> V <button onclick="setVal('v')">SET</button></p>
    <p>Iset: <input type="text" id="iset"> A <button onclick="setVal('i')">SET</button></p>
    <p><button id="pwr" onclick="togglePower()">---</button></p>
  </div>
<script>
let outputState = false;
function fmtTime(s){
  let h = Math.floor(s/3600), m = Math.floor((s%3600)/60), sec = s%60;
  return (h<10?'0':'')+h+':'+(m<10?'0':'')+m+':'+(sec<10?'0':'')+sec;
}
function update(){
  fetch('/data').then(r=>r.json()).then(d=>{
    document.getElementById('v').innerText = d.v.toFixed(2);
    document.getElementById('i').innerText = d.i.toFixed(2);
    document.getElementById('p').innerText = d.p.toFixed(1);
    document.getElementById('t').innerText = d.t.toFixed(1);
    document.getElementById('ah').innerText = d.ah.toFixed(3);
    document.getElementById('wh').innerText = d.wh.toFixed(2);
    document.getElementById('time').innerText = fmtTime(d.sec);
    let modeEl = document.getElementById('mode');
    if(!d.out){ modeEl.innerText = "OFF"; modeEl.className = "badge off"; }
    else if(d.cc){ modeEl.innerText = "CC"; modeEl.className = "badge cc"; }
    else { modeEl.innerText = "CV"; modeEl.className = "badge cv"; }
    if(!document.activeElement.id.includes('set')){
      document.getElementById('vset').value = d.vs.toFixed(2);
      document.getElementById('iset').value = d.is.toFixed(2);
    }
    outputState = d.out;
    let btn = document.getElementById('pwr');
    btn.innerText = outputState ? "OFF (RUNNING)" : "ON (STOPPED)";
    btn.className = outputState ? "btn-off" : "btn-on";
  });
}
function togglePower(){ fetch('/set?out=' + (outputState ? 0 : 1)); }
function resetStats(){ fetch('/reset'); }
function setVal(type){
  let val = document.getElementById(type + 'set').value.replace(',', '.');
  fetch('/set?' + type + '=' + val);
}
setInterval(update, 500);
</script></body></html>
)rawliteral";// ===================================================================
// MODBUS FUNCTIONS
// ===================================================================
uint16_t calculateCRC(const uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; }
      else                     { crc >>= 1; }
    }
  }
  return crc;
}

void queueSetpointWrite(uint16_t reg, float value) {
  pendingReg = reg;
  pendingVal = (uint16_t)(value * 100.0f);
  pendingWrite = true;
  lastEncoderMove = millis();
}

void writeModbusRegister(uint16_t regAddr, uint16_t regValue) {
  uint8_t msg[8];
  msg[0] = 0x01; msg[1] = 0x06;
  msg[2] = highByte(regAddr); msg[3] = lowByte(regAddr);
  msg[4] = highByte(regValue); msg[5] = lowByte(regValue);
  uint16_t crc = calculateCRC(msg, 6);
  msg[6] = lowByte(crc); msg[7] = highByte(crc);
  while (Serial2.available()) Serial2.read();
  Serial2.write(msg, 8);
  mbState = MB_IDLE;
}

void requestModbusData(uint8_t slaveAddr, uint16_t startReg, uint16_t numRegs) {
  uint8_t msg[8];
  msg[0] = slaveAddr; msg[1] = 0x03;
  msg[2] = highByte(startReg); msg[3] = lowByte(startReg);
  msg[4] = highByte(numRegs);  msg[5] = lowByte(numRegs);
  uint16_t crc = calculateCRC(msg, 6);
  msg[6] = lowByte(crc); msg[7] = highByte(crc);
  while (Serial2.available()) Serial2.read();
  Serial2.write(msg, 8);
  rxLen = 0;
  mbExpectedBytes = 5 + (numRegs * 2);
  mbState = MB_WAIT_RESPONSE;
  mbSendTimestamp = millis();
}

void processModbusStream() {
  while (Serial2.available()) {
    if (rxLen < RX_BUF_SIZE) rxBuffer[rxLen++] = Serial2.read();
    else                     Serial2.read();
  }
  if (mbState == MB_WAIT_RESPONSE) {
    if (rxLen >= mbExpectedBytes) {
      uint16_t crcRec  = rxBuffer[mbExpectedBytes - 2] | (rxBuffer[mbExpectedBytes - 1] << 8);
      uint16_t crcCalc = calculateCRC(rxBuffer, mbExpectedBytes - 2);
      if (rxBuffer[0] == 0x01 && rxBuffer[1] == 0x03 && crcRec == crcCalc) {
        if (rxLen >= 43) {
          if (!(pendingWrite && pendingReg == 0x0000))
            vSet = ((rxBuffer[3] << 8) | rxBuffer[4]) / 100.0f;
          if (!(pendingWrite && pendingReg == 0x0001))
            iSet = ((rxBuffer[5] << 8) | rxBuffer[6]) / 100.0f;
          vOut = ((rxBuffer[7]  << 8) | rxBuffer[8])  / 100.0f;
          iOut = ((rxBuffer[9]  << 8) | rxBuffer[10]) / 100.0f;
          pOut = vOut * iOut;
          temp = ((rxBuffer[13] << 8) | rxBuffer[14]) / 100.0f;
          uint16_t reg13 = (rxBuffer[29] << 8) | rxBuffer[30];
          isCCMode = (reg13 == 1);
          isLinkOk = true;
          lastGoodResponse = millis();
          newFrame = true;
        }
      }
      mbState = MB_IDLE;
    } else if (millis() - mbSendTimestamp > 100) {
      mbState = MB_IDLE;
    }
  }
}

// ===================================================================
// WEBSERVER
// ===================================================================
void setupWebServer() {
  server.on("/", []() { server.send_P(200, "text/html", HTML_PAGE); });
  server.on("/data", []() {
    char json[256];
    snprintf(json, sizeof(json),
      "{\"v\":%.2f,\"i\":%.2f,\"p\":%.1f,\"t\":%.1f,"
      "\"vs\":%.2f,\"is\":%.2f,\"out\":%d,\"cc\":%d,"
      "\"ah\":%.3f,\"wh\":%.2f,\"sec\":%u}",
      vOut, iOut, pOut, temp, vSet, iSet,
      isOutputOn ? 1 : 0, isCCMode ? 1 : 0,
      ahCount, whCount, (unsigned)workSeconds);
    server.send(200, "application/json", json);
  });
  server.on("/reset", []() {
    ahCount = 0.0f; whCount = 0.0f; workSeconds = 0;
    redrawScreenFlag = true;
    server.send(200, "text/plain", "OK");
  });
  server.on("/set", []() {
    if (server.hasArg("v")) { vSet = server.arg("v").toFloat(); queueSetpointWrite(0x0000, vSet); }
    if (server.hasArg("i")) { iSet = server.arg("i").toFloat(); queueSetpointWrite(0x0001, iSet); }
    if (server.hasArg("out")) {
      isOutputOn = (server.arg("out").toInt() == 1);
      writeModbusRegister(0x0012, isOutputOn ? 0x0001 : 0x0000);
      redrawScreenFlag = true;
    }
    server.send(200, "text/plain", "OK");
  });
  server.begin();
}

void handleWiFiManager() {
  bool isConnected = (WiFi.status() == WL_CONNECTED);
  if (isConnected) {
    if (!wasConnected) { wasConnected = true; redrawScreenFlag = true; }
    server.handleClient();
    ArduinoOTA.handle();
  } else {
    if (wasConnected) { wasConnected = false; redrawScreenFlag = true; }
    if (millis() - lastWiFiAttempt > 10000) {
      lastWiFiAttempt = millis();
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }
}

// ===================================================================
// SCREEN 1 - MAIN
// Left:  x=2..192  (w=190)
// Right: x=210..318 (w=108)
// ===================================================================
void drawScreen1_Layout() {
  tft.fillScreen(COLOR_BG);

  tft.drawRoundRect(2, 2, 190, 158, 6, COLOR_BORDER);
  tft.drawFastHLine(3, 54,  188, COLOR_BORDER);
  tft.drawFastHLine(3, 106, 188, COLOR_BORDER);

  tft.fillCircle(16, 26, 11, COLOR_VOLT);
  tft.setFont();
  tft.setTextColor(COLOR_BG); tft.setTextSize(2); tft.setCursor(11, 19); tft.print("V");

  tft.fillCircle(16, 76, 11, COLOR_AMP);
  tft.setCursor(11, 69); tft.print("A");

  tft.fillCircle(16, 126, 11, COLOR_WATT);
  tft.setCursor(11, 119); tft.print("W");

  tft.drawRoundRect(2, 163, 190, 75, 6, COLOR_BORDER);
  tft.setTextColor(COLOR_TEXT_DIM); tft.setTextSize(2);
  tft.setCursor(10, 168); tft.print("Cap:");
  tft.setCursor(10, 190); tft.print("Ene:");
  tft.setCursor(10, 212); tft.print("Time:");

  // Right column
  tft.fillRoundRect(210, 2, 108, 58, 4, COLOR_PANEL);
  tft.drawRoundRect(210, 2, 108, 58, 4, COLOR_BORDER);

  tft.fillRoundRect(210, 64, 108, 58, 4, COLOR_PANEL);
  tft.drawRoundRect(210, 64, 108, 58, 4, COLOR_BORDER);

  tft.fillRoundRect(210, 126, 108, 26, 4, COLOR_CV);
  tft.drawRoundRect(210, 126, 108, 26, 4, COLOR_BORDER);

  tft.fillRoundRect(210, 156, 108, 82, 6, COLOR_CC);
  tft.drawRoundRect(210, 156, 108, 82, 6, COLOR_WATT);

  redrawScreenFlag = true;
}

void updateScreen1_Values() {
  tft.setTextWrap(false);
  char buf[20];
  tft.setFont();

  static float lastV = -1, lastI = -1, lastP = -1;
  static float lastVSt = -1, lastISt = -1, lastTmp = -1;
  static float lastAh = -1, lastWh = -1;
  static uint32_t lastSec = 999999;
  static uint8_t lastEditMode = 99;
  static bool lastWifiState = false;
  static int lastDrawnState = -1;
  static bool lastLink = true;

  // ---- V ----
  if (vOut != lastV || redrawScreenFlag) {
    lastV = vOut;
    tft.fillRect(28, 15, 90, 24, COLOR_BG);
    tft.setTextColor(COLOR_VOLT); tft.setTextSize(3);
    tft.setCursor(28, 15);
    snprintf(buf, sizeof(buf), "%5.2f", vOut);
    tft.print(buf);
  }

  // ---- A ----
  if (iOut != lastI || redrawScreenFlag) {
    lastI = iOut;
    tft.fillRect(28, 65, 90, 24, COLOR_BG);
    tft.setTextColor(COLOR_AMP); tft.setTextSize(3);
    tft.setCursor(28, 65);
    snprintf(buf, sizeof(buf), "%5.2f", iOut);
    tft.print(buf);
  }

  // ---- W ----
  if (pOut != lastP || redrawScreenFlag) {
    lastP = pOut;
    tft.fillRect(28, 115, 90, 24, COLOR_BG);
    tft.setTextColor(COLOR_WATT); tft.setTextSize(3);
    tft.setCursor(28, 115);
    if (pOut < 100.0f) snprintf(buf, sizeof(buf), "%5.1f", pOut);
    else               snprintf(buf, sizeof(buf), "%5.0f", pOut);
    tft.print(buf);
  }

  // ---- Temperature with °C ----
  if (temp != lastTmp || redrawScreenFlag) {
    lastTmp = temp;
    tft.fillRect(118, 115, 72, 24, COLOR_BG);
    tft.setTextColor(COLOR_TEMP);
    tft.setTextSize(2);
    tft.setCursor(120, 123);
    // Print temp as "24.0" then degree symbol then "C"
    snprintf(buf, sizeof(buf), "%4.1f", temp);
    tft.print(buf);
    tft.write(247);   // degree symbol in Adafruit GFX built-in font
    tft.print("C");
  }

  // ---- Uset ----
  if (vSet != lastVSt || editMode != lastEditMode || redrawScreenFlag) {
    lastVSt = vSet;

    uint16_t bgColor  = (editMode == 0) ? COLOR_SET : COLOR_PANEL;
    uint16_t txtColor = (editMode == 0) ? COLOR_BG  : COLOR_VOLT;

    tft.fillRoundRect(210, 2, 108, 58, 4, bgColor);
    tft.drawRoundRect(210, 2, 108, 58, 4, COLOR_BORDER);

    if (editMode == 0) {
      tft.fillTriangle(200, 33, 200, 21, 206, 27, COLOR_SET);
    } else {
      tft.fillRect(198, 2, 12, 58, COLOR_BG);
    }

    tft.setTextColor(txtColor);
    tft.setTextSize(3);
    tft.setCursor(214, 15);
    snprintf(buf, sizeof(buf), "%5.2f", vSet);
    tft.print(buf);
  }

  // ---- Iset ----
  if (iSet != lastISt || editMode != lastEditMode || redrawScreenFlag) {
    lastISt = iSet;

    uint16_t bgColor  = (editMode == 1) ? COLOR_SET : COLOR_PANEL;
    uint16_t txtColor = (editMode == 1) ? COLOR_BG  : COLOR_VOLT;

    tft.fillRoundRect(210, 64, 108, 58, 4, bgColor);
    tft.drawRoundRect(210, 64, 108, 58, 4, COLOR_BORDER);

    if (editMode == 1) {
      tft.fillTriangle(200, 95, 200, 83, 206, 89, COLOR_SET);
    } else {
      tft.fillRect(198, 64, 12, 58, COLOR_BG);
    }

    tft.setTextColor(txtColor);
    tft.setTextSize(3);
    tft.setCursor(214, 77);
    snprintf(buf, sizeof(buf), "%5.2f", iSet);
    tft.print(buf);

    lastEditMode = editMode;
  }

  // ---- Stats ----
  bool currentWifiState = (WiFi.status() == WL_CONNECTED);

  if (ahCount != lastAh || whCount != lastWh || workSeconds != lastSec
      || currentWifiState != lastWifiState || redrawScreenFlag) {
    lastAh = ahCount;
    lastWh = whCount;
    lastSec = workSeconds;
    lastWifiState = currentWifiState;

    tft.setTextSize(2);
    tft.fillRect(60, 166, 128, 66, COLOR_BG);

    tft.setTextColor(COLOR_WATT);
    tft.setCursor(60, 168); tft.print(ahCount, 3); tft.print(" Ah");
    tft.setCursor(60, 190); tft.print(whCount, 2); tft.print(" Wh");

    uint32_t h = workSeconds / 3600;
    uint32_t m = (workSeconds % 3600) / 60;
    uint32_t s = workSeconds % 60;
    tft.setCursor(62, 212);
    if (h < 10) tft.print("0");
    tft.print(h); tft.print(":");
    if (m < 10) tft.print("0");
    tft.print(m); tft.print(":");
    if (s < 10) tft.print("0");
    tft.print(s);

    drawWiFiIcon(165, 190, currentWifiState);
  }

  // ---- LINK ----
  if (isLinkOk != lastLink || redrawScreenFlag) {
    lastLink = isLinkOk;
    if (isLinkOk) {
      tft.fillRoundRect(210, 126, 108, 26, 4, COLOR_CV);
      tft.drawRoundRect(210, 126, 108, 26, 4, COLOR_BORDER);
      tft.setTextColor(COLOR_BG); tft.setTextSize(2);
      tft.setCursor(244, 132); tft.print("LINK");
    } else {
      tft.fillRoundRect(210, 126, 108, 26, 4, COLOR_CC);
      tft.drawRoundRect(210, 126, 108, 26, 4, COLOR_BORDER);
      tft.setTextColor(COLOR_WATT); tft.setTextSize(2);
      tft.setCursor(248, 132); tft.print("ERR");
    }
  }

  // ---- RUN/STOP ----
  if (lastDrawnState != (int)isOutputOn || redrawScreenFlag) {
    lastDrawnState = (int)isOutputOn;
    tft.fillRoundRect(210, 156, 108, 82, 6, isOutputOn ? COLOR_CV : COLOR_CC);
    tft.drawRoundRect(210, 156, 108, 82, 6, COLOR_WATT);
    tft.setTextColor(COLOR_BG);
    tft.setTextSize(3);
    tft.setCursor(isOutputOn ? 228 : 226, 185);
    tft.print(isOutputOn ? "RUN " : "STOP");
  }
}// ===================================================================
// SCREEN 2 - ANALYTICS
// ===================================================================
void drawScreen2_Layout() {
  tft.fillScreen(COLOR_BG);
  tft.setFont();
  tft.drawRoundRect(2, 2, 316, 30, 4, COLOR_BORDER);
  tft.setTextColor(COLOR_VOLT); tft.setTextSize(2); tft.setCursor(10, 9); tft.print("ANALYTICS & LOAD");
  tft.setTextColor(COLOR_WATT, COLOR_PANEL); tft.setCursor(280, 9); tft.print("P2");

  tft.drawRoundRect(2, 36, 316, 65, 6, COLOR_BORDER);
  tft.setTextColor(COLOR_TEXT_DIM); tft.setTextSize(1); tft.setCursor(10, 43); tft.print("LOAD RESISTANCE (Ohms):");

  tft.drawRoundRect(2, 105, 316, 60, 6, COLOR_BORDER);
  tft.setTextColor(COLOR_TEXT_DIM); tft.setTextSize(1); tft.setCursor(10, 112); tft.print("CURRENT LOAD LEVEL:");
  tft.drawRect(15, 135, 290, 18, COLOR_BORDER);

  tft.drawRoundRect(2, 170, 316, 68, 6, COLOR_BORDER);
  tft.setTextColor(COLOR_TEXT_DIM); tft.setTextSize(1); tft.setCursor(10, 175); tft.print("NETWORK & STATUS:");
}

void updateScreen2_Values() {
  tft.setTextWrap(false);
  tft.setFont();

  float resistance = (iOut > 0.005f) ? (vOut / iOut) : 0.0f;
  tft.setTextColor(COLOR_AMP, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(15, 65);
  if (resistance > 999.9f || iOut <= 0.005f) {
    tft.print("OPEN CIR  ");
  } else {
    char rb[20];
    snprintf(rb, sizeof(rb), "%.2f Ohm   ", resistance);
    tft.print(rb);
  }

  int barWidth = 0;
  if (iSet > 0.01f) {
    barWidth = (int)((iOut / iSet) * 286.0f);
    if (barWidth > 286) barWidth = 286;
    if (barWidth < 0)   barWidth = 0;
  }
  tft.fillRect(17, 137, barWidth, 14, COLOR_AMP);
  tft.fillRect(17 + barWidth, 137, 286 - barWidth, 14, COLOR_BG);

  tft.setTextColor(COLOR_WATT, COLOR_BG);
  tft.setTextSize(1);
  tft.setCursor(20, 155);
  tft.print("Out: "); tft.print(iOut, 2);
  tft.print("A / Limit: "); tft.print(iSet, 2);
  tft.print("A     ");

  tft.setCursor(10, 190);
  tft.setTextSize(2);
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(COLOR_VOLT, COLOR_BG);
    tft.print("IP: "); tft.print(WiFi.localIP().toString()); tft.print("   ");
  } else {
    tft.setTextColor(COLOR_CC, COLOR_BG);
    tft.print("WiFi: DISCONNECTED ");
  }

  tft.setCursor(10, 212);
  tft.setTextColor(COLOR_WATT, COLOR_BG);
  tft.print(isOutputOn ? "OUT: RUN " : "OUT: STOP");
  tft.setCursor(180, 212);
  tft.print(isOutputOn ? (isCCMode ? "CC MODE" : "CV MODE") : "OFF    ");
}

// ===================================================================
// SCREEN 3 - GRAPH
// ===================================================================
void drawScreen3_Layout() {
  tft.fillScreen(COLOR_BG);
  tft.setFont();

  tft.drawRoundRect(2, 2, 316, 30, 4, COLOR_BORDER);
  tft.setTextColor(COLOR_VOLT); tft.setTextSize(2);
  tft.setCursor(10, 9); tft.print("POWER HISTORY");
  tft.setTextColor(COLOR_WATT, COLOR_PANEL); tft.setCursor(280, 9); tft.print("P3");

  tft.drawRoundRect(2, 36, 316, 200, 6, COLOR_BORDER);
  tft.drawFastHLine(10, 130, 300, COLOR_BORDER);
  tft.drawFastHLine(10, 200, 300, COLOR_BORDER);

  tft.setTextColor(COLOR_TEXT_DIM); tft.setTextSize(1);
  tft.setCursor(10, 220); tft.print("Time -> (1 point / 3 sec)");

  redrawScreenFlag = true;
}

void updateScreen3_Values() {
  tft.fillRect(5, 45, 310, 185, COLOR_BG);

  tft.drawFastHLine(10, 130, 300, COLOR_BORDER);
  tft.drawFastHLine(10, 200, 300, COLOR_BORDER);

  tft.setTextColor(COLOR_TEXT_DIM); tft.setTextSize(1);
  tft.setCursor(10, 220); tft.print("Time -> (1 point / 3 sec)");

  float maxP = 10.0f;
  for (int i = 0; i < 30; i++) if (powerHistory[i] > maxP) maxP = powerHistory[i];

  float scaleMax = 100.0f;
  if      (maxP <= 10.0f)  scaleMax = 10.0f;
  else if (maxP <= 50.0f)  scaleMax = 50.0f;
  else if (maxP <= 100.0f) scaleMax = 100.0f;
  else if (maxP <= 250.0f) scaleMax = 250.0f;
  else if (maxP <= 500.0f) scaleMax = 500.0f;
  else                     scaleMax = 1000.0f;

  tft.setCursor(275, 48);  tft.print((int)scaleMax); tft.print("W");
  tft.setCursor(275, 133); tft.print((int)(scaleMax / 2.0f)); tft.print("W");

  for (int i = 0; i < 29; i++) {
    int x1 = 15 + (i * 10);
    int x2 = 15 + ((i + 1) * 10);

    int y1 = 200 - (int)((powerHistory[i]     / scaleMax) * 150.0f);
    int y2 = 200 - (int)((powerHistory[i + 1] / scaleMax) * 150.0f);

    if (y1 < 45)  y1 = 45;  if (y1 > 200) y1 = 200;
    if (y2 < 45)  y2 = 45;  if (y2 > 200) y2 = 200;

    tft.drawLine(x1, y1, x2, y2, COLOR_WATT);
  }
}

// ===================================================================
// SCREEN 4 - CLOCK / INFO
// ===================================================================
void drawInfoScreen() {
  struct tm timeinfo;

  if (redrawScreenFlag) {
    tft.fillScreen(COLOR_BG);

    tft.drawRoundRect(2, 2, 316, 30, 4, COLOR_BORDER);
    tft.setTextColor(COLOR_VOLT); tft.setTextSize(2);
    tft.setCursor(10, 9); tft.print("INFO & CLOCK");
    tft.setTextColor(COLOR_WATT, COLOR_PANEL); tft.setCursor(280, 9); tft.print("P4");

    tft.drawRoundRect(2, 36, 316, 200, 6, COLOR_BORDER);
  }

  if (!getLocalTime(&timeinfo)) {
    tft.setTextColor(COLOR_CC, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(40, 90);
    tft.print("Syncing Time...  ");
    return;
  }

  char timeStr[10];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  tft.setTextColor(COLOR_CYAN, COLOR_BG);
  tft.setTextSize(3);
  tft.setCursor(55, 48);
  tft.print(timeStr);

  char dateStr[15];
  snprintf(dateStr, sizeof(dateStr), "%02d.%02d.%d",
           timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
  tft.setTextColor(COLOR_WATT, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(95, 92);
  tft.print(dateStr);

  static bool lastOutputState = true;
  static bool lastCCMode = true;
  if (lastOutputState != isOutputOn || lastCCMode != isCCMode) {
    tft.fillRect(15, 118, 290, 18, COLOR_BG);
    lastOutputState = isOutputOn;
    lastCCMode = isCCMode;
  }
  tft.setCursor(15, 118);
  tft.setTextSize(2);
  if (!isOutputOn) {
    tft.setTextColor(COLOR_CC, COLOR_BG); tft.print("OUT: OFF        ");
  } else if (isCCMode) {
    tft.setTextColor(COLOR_CC, COLOR_BG); tft.print("OUT: RUN (CC)   ");
  } else {
    tft.setTextColor(COLOR_CV, COLOR_BG); tft.print("OUT: RUN (CV)   ");
  }

  static float oldAh = -1;
  static float oldWh = -1;
  static uint32_t oldSec = 999999;

  if (ahCount != oldAh || whCount != oldWh || workSeconds != oldSec) {
    tft.fillRect(15, 142, 290, 36, COLOR_BG);

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setCursor(15, 142); tft.print("Capacity:");
    tft.setTextSize(2);
    tft.setTextColor(COLOR_VOLT, COLOR_BG);
    tft.setCursor(15, 156); tft.print(ahCount, 3); tft.print("Ah");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setCursor(120, 142); tft.print("Energy:");
    tft.setTextSize(2);
    tft.setTextColor(COLOR_WATT, COLOR_BG);
    tft.setCursor(120, 156); tft.print(whCount, 2); tft.print("Wh");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.setCursor(215, 142); tft.print("Time:");

    uint32_t h = workSeconds / 3600;
    uint32_t m = (workSeconds % 3600) / 60;
    uint32_t s = workSeconds % 60;
    char timeBuf[10];
    snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u:%02u", h, m, s);

    tft.setTextSize(2);
    tft.setTextColor(COLOR_CYAN, COLOR_BG);
    tft.setCursor(215, 156);
    tft.print(timeBuf);

    oldAh = ahCount; oldWh = whCount; oldSec = workSeconds;
  }

  static int lastWifiStatus = -1;
  int currentWifiStatus = WiFi.status();
  if (currentWifiStatus != lastWifiStatus) {
    tft.fillRect(15, 198, 290, 20, COLOR_BG);
    lastWifiStatus = currentWifiStatus;
  }

  tft.setCursor(15, 198);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft.print("WiFi: ");
  if (currentWifiStatus == WL_CONNECTED) {
    tft.setTextColor(COLOR_VOLT, COLOR_BG);
    tft.print(WIFI_SSID);
    tft.setTextColor(COLOR_CV, COLOR_BG);
    tft.print(" (OK)");
  } else {
    tft.setTextColor(COLOR_CC, COLOR_BG);
    tft.print("DISCONNECTED         ");
  }
}

void redrawCurrentScreen() {
  if (currentScreen == 0) {
    drawScreen1_Layout();
    updateScreen1_Values();
  } else if (currentScreen == 1) {
    drawScreen2_Layout();
    updateScreen2_Values();
  } else if (currentScreen == 2) {
    drawScreen3_Layout();
    updateScreen3_Values();
  } else if (currentScreen == 3) {
    redrawScreenFlag = true;
    drawInfoScreen();
  }
}

// ===================================================================
// ENCODER ROTATION
// ===================================================================
void handleEncoder() {
  int currentClkState = digitalRead(ENC_CLK);
  if (currentClkState != lastClkState && currentClkState == LOW) {
    if (millis() - lastEncRotate >= 3) {
      lastEncRotate = millis();

      if (digitalRead(ENC_DT) != currentClkState) {
        if (editMode == 0) {
          vSet += vStep; if (vSet > 60.0f) vSet = 60.0f;
          queueSetpointWrite(0x0000, vSet);
        } else {
          iSet += iStep; if (iSet > 20.0f) iSet = 20.0f;
          queueSetpointWrite(0x0001, iSet);
        }
      } else {
        if (editMode == 0) {
          vSet -= vStep; if (vSet < 0.0f) vSet = 0.0f;
          queueSetpointWrite(0x0000, vSet);
        } else {
          iSet -= iStep; if (iSet < 0.0f) iSet = 0.0f;
          queueSetpointWrite(0x0001, iSet);
        }
      }
    }
  }
  lastClkState = currentClkState;
}

// ===================================================================
// ENCODER BUTTON EVENTS (from ISR)
// ===================================================================
void handleButtonEvents() {
  if (btnPressedEvent) {
    btnPressedEvent = false;
  }

  if (btnReleasedEvent) {
    btnReleasedEvent = false;
    unsigned long duration = btnPressEndMs - btnPressStartMs;

    if (duration >= 600) {
      isOutputOn = !isOutputOn;
      writeModbusRegister(0x0012, isOutputOn ? 0x0001 : 0x0000);
      redrawScreenFlag = true;
      Serial.printf("[ENC] LONG %lu ms -> output %s\n", duration, isOutputOn ? "ON" : "OFF");
    } else if (duration > 50) {
      editMode = (editMode == 0) ? 1 : 0;
      redrawScreenFlag = true;
      Serial.printf("[ENC] SHORT %lu ms -> editMode %d\n", duration, editMode);
    }
  }
}

// ===================================================================
// EXTERNAL BUTTON EVENTS (GPIO 25)
// ===================================================================
void handleExternalButton() {
  if (btnExtPressed) {
    btnExtPressed = false;
    isOutputOn = !isOutputOn;
    writeModbusRegister(0x0012, isOutputOn ? 0x0001 : 0x0000);
    redrawScreenFlag = true;
    Serial.printf("[EXT] click -> output %s\n", isOutputOn ? "ON" : "OFF");
  }
}

// ===================================================================
// TOUCH BUTTON
// ===================================================================
void handleTouchButton() {
  bool currentTouch = digitalRead(TOUCH_PIN);
  if (currentTouch != lastTouchState) lastTouchDebounce = millis();

  if ((millis() - lastTouchDebounce) > 60) {
    static bool touchTriggered = false;
    if (currentTouch == HIGH && !touchTriggered) {
      touchTriggered = true;
      currentScreen = (currentScreen + 1) % 4;
      redrawScreenFlag = true;
      redrawCurrentScreen();
    } else if (currentTouch == LOW) {
      touchTriggered = false;
    }
  }
  lastTouchState = currentTouch;
}

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT,  INPUT_PULLUP);
  pinMode(ENC_SW,  INPUT_PULLUP);
  pinMode(TOUCH_PIN, INPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);

  lastClkState = digitalRead(ENC_CLK);
  btnIsrState  = digitalRead(ENC_SW);
  btnExtState  = digitalRead(BTN_PIN);

  attachInterrupt(digitalPinToInterrupt(ENC_SW),  btnISR,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnExtISR, CHANGE);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.invertDisplay(false);

  drawScreen1_Layout();
  updateScreen1_Values();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  setupWebServer();
  ArduinoOTA.begin();

  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
}

// ===================================================================
// LOOP
// ===================================================================
void loop() {
  esp_task_wdt_reset();

  handleButtonEvents();
  handleExternalButton();

  handleWiFiManager();
  handleEncoder();
  handleTouchButton();
  processModbusStream();

  if (pendingWrite && (millis() - lastEncoderMove > 150)) {
    writeModbusRegister(pendingReg, pendingVal);
    pendingWrite = false;
  }

  static unsigned long lastModbusPoll = 0;
  if (mbState == MB_IDLE && (millis() - lastModbusPoll > 250)) {
    lastModbusPoll = millis();
    requestModbusData(0x01, 0x0000, 20);
  }

  if (millis() - lastGoodResponse > 1500) {
    isLinkOk = false;
  }

  if (newFrame) {
    newFrame = false;
    if (currentScreen == 0) {
      updateScreen1_Values();
    } else if (currentScreen == 1) {
      updateScreen2_Values();
    } else if (currentScreen == 2 && (millis() - lastGraphUpdate > 3000)) {
      lastGraphUpdate = millis();
      for (int i = 0; i < 29; i++) powerHistory[i] = powerHistory[i + 1];
      powerHistory[29] = pOut;
      updateScreen3_Values();
    }
  }

  if (currentScreen == 3) {
    static unsigned long lastClockRefresh = 0;
    if (millis() - lastClockRefresh > 1000) {
      lastClockRefresh = millis();
      redrawScreenFlag = false;
      drawInfoScreen();
    }
  }

  if (isLinkOk && isOutputOn) {
    if (millis() - lastTick >= 1000) {
      lastTick = millis();
      workSeconds++;
      ahCount += (iOut / 3600.0f);
      whCount += (pOut / 3600.0f);
    }
  }

  static uint8_t redrawFrames = 0;
  if (redrawScreenFlag) {
    redrawFrames++;
    if (redrawFrames > 3) {
      redrawScreenFlag = false;
      redrawFrames = 0;
    }
  } else {
    redrawFrames = 0;
  }

  delay(1);
}
