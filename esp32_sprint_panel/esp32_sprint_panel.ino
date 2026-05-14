#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

const char* ssid = "STM_UART_TEST";
const char* password = "12345678";

WebServer server(80);

#define ESP_RX2 27
#define ESP_TX2 17

typedef enum
{
  PANEL_MODE_FREE = 0,
  PANEL_MODE_TEST = 1,
  PANEL_MODE_STAGED = 2
} PanelMode_t;

static PanelMode_t panelMode = PANEL_MODE_FREE;

/* Edge-triggered staged UART (V:/Q:) — only on stage change, min 1000 ms between sends */
static int lastAppliedStage = -1;
static int lastAppliedSpeed = -1;
static int lastAppliedTorque = -1;
static unsigned long lastStagedCommandMs = 0;

String uartLine = "";

bool testRunning = false;
bool testFinished = false;

float startMeter = 0.0;
float currentMeter = 0.0;
float previousMeter = 0.0;
bool havePreviousMeter = false;
float trainingStartMeter = 0.0;
bool trainingStartValid = false;
String lastYonLabel = "—";
String lastFazLabel = "IDLE";

#define METER_DIR_EPS 0.002f

float safeParkMeter = 0.0f;
bool safeParkValid = false;
bool haveLiveMeter = false;

float distanceDone = 0.0;
float targetDistance = 10.0;

float liveRealSpeedMs = 0.0;
int liveTorque = 0;
int liveSetSpeed = 0;
float liveMaxHizMs = 0.0;

float testMaxSpeed = 0.0;
bool resultSavedForCurrentTest = false;

float avgSpeed = 0.0;

unsigned long testStartMs = 0;
unsigned long testElapsedMs = 0;

#define LOG_SIZE 20
float logTime[LOG_SIZE];
float logMeter[LOG_SIZE];
float logSpeed[LOG_SIZE];
int logCount = 0;

#define HISTORY_LEN 5

typedef struct
{
  uint32_t no;
  float distance;
  float timeSec;
  float avgSpeed;
  float maxSpeed;
  int torque;
  int setSpeed;
} SessionHistoryEntry_t;

static SessionHistoryEntry_t sessionHistory[HISTORY_LEN];
static int sessionHistoryCount = 0;
static uint32_t sessionRunCounter = 0;

static void sessionHistoryPush(uint32_t no,
                               float distance,
                               float timeSec,
                               float avgSp,
                               float maxSp,
                               int torque,
                               int setSp)
{
  for (int i = HISTORY_LEN - 1; i > 0; i--)
  {
    sessionHistory[i] = sessionHistory[i - 1];
  }

  sessionHistory[0].no = no;
  sessionHistory[0].distance = distance;
  sessionHistory[0].timeSec = timeSec;
  sessionHistory[0].avgSpeed = avgSp;
  sessionHistory[0].maxSpeed = maxSp;
  sessionHistory[0].torque = torque;
  sessionHistory[0].setSpeed = setSp;

  if (sessionHistoryCount < HISTORY_LEN)
    sessionHistoryCount++;
}

static String buildHistoryTableHtml()
{
  String html = "";

  for (int i = 0; i < sessionHistoryCount; i++)
  {
    html += "<tr><td>";
    html += String(i + 1);
    html += ". Çalışma</td><td>";
    html += String(sessionHistory[i].no);
    html += "</td><td>";
    html += String(sessionHistory[i].distance, 2);
    html += "</td><td>";
    html += String(sessionHistory[i].timeSec, 2);
    html += "</td><td>";
    html += String(sessionHistory[i].avgSpeed, 2);
    html += "</td><td>";
    html += String(sessionHistory[i].maxSpeed, 2);
    html += "</td><td>";
    html += String(sessionHistory[i].torque);
    html += "</td><td>";
    html += String(sessionHistory[i].setSpeed);
    html += "</td></tr>";
  }

  return html;
}

static void stagedTargetFromMetre(float metre, int* stageIdx, int* outSpd, int* outTrq)
{
  static const int speeds[] = { 100, 200, 300, 400, 500 };
  static const int torqs[] = { 5, 5, 10, 10, 15 };

  float x = fabsf(metre);
  if (x >= 10.0f)
    x = 9.999f;

  int idx = (int)(x / 2.0f);
  if (idx < 0)
    idx = 0;
  if (idx > 4)
    idx = 4;

  *stageIdx = idx;
  *outSpd = speeds[idx];
  *outTrq = torqs[idx];
}

static void updateTrainingDirectionAndPhase(float m, float dm)
{
  if (!trainingStartValid)
  {
    lastYonLabel = "—";
    lastFazLabel = "IDLE";
    return;
  }

  if (fabsf(dm) < METER_DIR_EPS)
  {
    lastYonLabel = "Sabit";
    lastFazLabel = "IDLE";
    return;
  }

  bool uzaklasiyor = false;
  if (m > trainingStartMeter && dm > 0.0f)
    uzaklasiyor = true;
  else if (m < trainingStartMeter && dm < 0.0f)
    uzaklasiyor = true;

  if (uzaklasiyor)
  {
    lastYonLabel = "Uzaklaşıyor";
    lastFazLabel = "RESIST";
  }
  else
  {
    lastYonLabel = "Yaklaşıyor";
    lastFazLabel = "ASSIST";
  }
}

static void sendStmChar(char c)
{
  Serial2.write((uint8_t)c);
  Serial2.flush();
  Serial.printf("UART TX -> %c\n", c);
}

static void maybeApplyStagedAfterLive(void)
{
  if (panelMode != PANEL_MODE_STAGED)
    return;

  int idx = 0;
  int spd = 0;
  int trq = 0;

  stagedTargetFromMetre(currentMeter, &idx, &spd, &trq);

  Serial.printf("WEB: STAGED LIVE metre=%.2f stage=%d last=%d\n",
                (double)currentMeter, idx, lastAppliedStage);

  if (idx == lastAppliedStage)
  {
    Serial.printf("WEB: STAGED SKIP SAME stage=%d\n", idx);
    return;
  }

  unsigned long now = millis();
  if (lastAppliedStage != -1 && (now - lastStagedCommandMs) < 1000UL)
  {
    Serial.println("WEB: STAGED SKIP INTERVAL");
    return;
  }

  char spdChar = '1' + idx;  // 0->'1', 1->'2', 2->'3', 3->'4', 4->'5'
  Serial.printf("WEB: STAGED SEND SPEED CHAR=%c speed=%d\n", spdChar, spd);
  sendStmChar(spdChar);

  lastAppliedStage = idx;
  lastAppliedSpeed = spd;
  lastAppliedTorque = trq;
  lastStagedCommandMs = now;

  Serial.printf("WEB: STAGED APPLY stage=%d speed=%d torque=%d\n", idx, spd, trq);
}

static bool parseLivePayload(const String& line,
                             float* outMetre,
                             float* outHiz,
                             int* outTork,
                             int* outSetHiz,
                             float* outMaxHiz)
{
  if (!line.startsWith("LIVE,"))
    return false;

  String rest = line.substring(5);
  rest.trim();

  int c1 = rest.indexOf(',');
  int c2 = rest.indexOf(',', c1 + 1);
  int c3 = rest.indexOf(',', c2 + 1);
  int c4 = rest.indexOf(',', c3 + 1);

  if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0)
    return false;

  *outMetre = rest.substring(0, c1).toFloat();
  *outHiz = rest.substring(c1 + 1, c2).toFloat();
  *outTork = rest.substring(c2 + 1, c3).toInt();
  *outSetHiz = rest.substring(c3 + 1, c4).toInt();
  *outMaxHiz = rest.substring(c4 + 1).toFloat();

  return true;
}

void addLogPoint(float t, float m, float s)
{
  if (!testRunning) return;

  if (logCount < LOG_SIZE)
  {
    logTime[logCount] = t;
    logMeter[logCount] = m;
    logSpeed[logCount] = s;
    logCount++;
  }
  else
  {
    for (int i = 1; i < LOG_SIZE; i++)
    {
      logTime[i - 1] = logTime[i];
      logMeter[i - 1] = logMeter[i];
      logSpeed[i - 1] = logSpeed[i];
    }

    logTime[LOG_SIZE - 1] = t;
    logMeter[LOG_SIZE - 1] = m;
    logSpeed[LOG_SIZE - 1] = s;
  }
}

void finishTestAt10m()
{
  if (logCount >= 2)
  {
    float t2 = logTime[logCount - 1];
    float m2 = logMeter[logCount - 1];

    float t1 = logTime[logCount - 2];
    float m1 = logMeter[logCount - 2];

    if (m2 != m1)
    {
      float ratio = (targetDistance - m1) / (m2 - m1);
      if (ratio < 0.0) ratio = 0.0;
      if (ratio > 1.0) ratio = 1.0;

      float correctedTime = t1 + ratio * (t2 - t1);

      distanceDone = targetDistance;
      testElapsedMs = (unsigned long)(correctedTime * 1000.0);

      if (correctedTime > 0.1)
        avgSpeed = targetDistance / correctedTime;

      logTime[logCount - 1] = correctedTime;
      logMeter[logCount - 1] = targetDistance;
      logSpeed[logCount - 1] = liveRealSpeedMs;
    }
    else
    {
      distanceDone = targetDistance;
    }
  }
  else
  {
    distanceDone = targetDistance;
  }

  testRunning = false;
  testFinished = true;

  if (!resultSavedForCurrentTest)
  {
    resultSavedForCurrentTest = true;
    sessionRunCounter++;
    float tsec = testElapsedMs / 1000.0;
    sessionHistoryPush(sessionRunCounter,
                       distanceDone,
                       tsec,
                       avgSpeed,
                       testMaxSpeed,
                       liveTorque,
                       liveSetSpeed);
  }
}

void updateTestValues()
{
  if (testRunning)
  {
    testElapsedMs = millis() - testStartMs;
    distanceDone = abs(currentMeter - startMeter);

    float elapsedSec = testElapsedMs / 1000.0;

    if (elapsedSec > 0.1)
      avgSpeed = distanceDone / elapsedSec;

    if (distanceDone >= targetDistance)
    {
      finishTestAt10m();
    }
  }
}

String makeChartData()
{
  String data = "";

  for (int i = 0; i < logCount; i++)
  {
    if (i > 0) data += ";";

    data += String(logTime[i], 2);
    data += ",";
    data += String(logMeter[i], 2);
    data += ",";
    data += String(logSpeed[i], 2);
  }

  return data;
}

void handleRoot()
{
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">

  <style>
    * { box-sizing: border-box; }
    body {
      font-family: system-ui, -apple-system, Segoe UI, Arial, sans-serif;
      background: #111827;
      color: #f9fafb;
      margin: 0;
      padding: 1.25rem;
      text-align: center;
    }
    .shell {
      max-width: 960px;
      margin: 0 auto;
    }
    .panel {
      background: #1f2937;
      border-radius: 16px;
      padding: 1.5rem 1.25rem;
      box-shadow: 0 12px 40px rgba(0,0,0,0.45);
      border: 1px solid #374151;
    }
    h1 { margin: 0 0 0.35rem; font-size: 1.5rem; letter-spacing: 0.02em; }
    .sub { color: #9ca3af; margin: 0 0 1rem; font-size: 0.95rem; }

    .mode-row {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
      gap: 0.6rem;
      margin-bottom: 1rem;
      text-align: left;
    }
    .mode-card {
      background: #111827;
      border: 2px solid #374151;
      border-radius: 12px;
      padding: 0.75rem 0.85rem;
      cursor: pointer;
      user-select: none;
      transition: border-color 0.15s, box-shadow 0.15s;
    }
    .mode-card:hover { border-color: #6b7280; }
    .mode-card.active {
      border-color: #22c55e;
      box-shadow: 0 0 0 1px rgba(34,197,94,0.35);
    }
    .mode-card h4 {
      margin: 0 0 0.25rem;
      font-size: 0.95rem;
      color: #e5e7eb;
    }
    .mode-card p {
      margin: 0;
      font-size: 0.72rem;
      color: #9ca3af;
      line-height: 1.35;
    }

    .btn-row { margin: 0.35rem 0; display: flex; flex-wrap: wrap; justify-content: center; gap: 0.5rem; }
    button {
      min-width: 160px;
      padding: 0.85rem 1rem;
      font-size: 1rem;
      font-weight: 600;
      border: none;
      border-radius: 10px;
      color: #fff;
      cursor: pointer;
      transition: transform 0.08s ease, filter 0.15s;
    }
    button:hover { filter: brightness(1.08); }
    button:active { transform: scale(0.98); }
    .start { background: #16a34a; }
    .stop { background: #dc2626; }
    .plus { background: #2563eb; }
    .minus { background: #9333ea; }
    .test { background: #f59e0b; color: #111827; }
    .origin { background: #0d9488; }
    .parkcal { background: #0369a1; }

    #status {
      margin: 1rem 0 0.25rem;
      font-size: 1.1rem;
      color: #22c55e;
      min-height: 1.5rem;
    }

    .dash-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      gap: 0.75rem;
      margin: 1.25rem 0;
      text-align: left;
    }
    .dash-card {
      background: #111827;
      border: 1px solid #374151;
      border-radius: 12px;
      padding: 0.85rem 1rem;
    }
    .dash-card h3 {
      margin: 0 0 0.35rem;
      font-size: 0.72rem;
      font-weight: 600;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      color: #9ca3af;
    }
    .dash-card .val {
      font-size: 1.35rem;
      font-weight: 700;
      color: #22c55e;
      line-height: 1.2;
      word-break: break-all;
    }

    .section {
      margin-top: 1rem;
      background: #111827;
      border-radius: 12px;
      padding: 1rem;
      border: 1px solid #374151;
      text-align: left;
    }
    .section h2 {
      margin: 0 0 0.65rem;
      font-size: 1rem;
      color: #e5e7eb;
    }
    .smallValue { font-size: 0.95rem; color: #e5e7eb; margin: 0.35rem 0; }

    table { width: 100%; border-collapse: collapse; font-size: 0.85rem; margin-top: 0.5rem; }
    th, td { border: 1px solid #374151; padding: 0.45rem 0.5rem; }
    th { color: #22c55e; text-align: left; }

    canvas {
      width: 100%;
      max-width: 100%;
      height: 220px;
      background: #0b1220;
      border-radius: 10px;
      margin-top: 0.5rem;
      display: block;
    }

    .staged-hidden { display: none; }
  </style>
</head>

<body>
<div class="shell">
<div class="panel">

<h1>STM32 PLC CONTROL</h1>
<p class="sub">ESP32 → STM32 → PLC</p>

<div class="mode-row" id="modeRow">
  <div class="mode-card" data-mode="0" role="button" tabindex="0">
    <h4>Serbest Mod</h4>
    <p>Hız ve tork elle; mevcut davranış.</p>
  </div>
  <div class="mode-card" data-mode="1" role="button" tabindex="0">
    <h4>10m Test Modu</h4>
    <p>10 m test akışı (TEST BAŞLAT ile).</p>
  </div>
  <div class="mode-card" data-mode="2" role="button" tabindex="0">
    <h4>Kademeli Mod</h4>
    <p>Mesafeye göre hedef setpoint (sadece gösterim).</p>
  </div>
</div>

<div class="section" style="margin-top:0;">
  <h2>Çalışma modu</h2>
  <div class="smallValue">Aktif: <strong id="activeModeLabel">Serbest Mod</strong></div>
</div>

<div id="stagedPanel" class="section staged-hidden">
  <h2>Kademeli plan (gösterim)</h2>
  <div class="smallValue">Anlık metre: <span id="stagedMetre">—</span> m</div>
  <div class="smallValue">Aşama: <span id="stagedStageIdx">—</span> · Mesafe aralığı: <span id="stagedRange">—</span> m</div>
  <div class="smallValue">Hedef set hız: <span id="stagedTgtSpd">—</span> · Hedef tork: <span id="stagedTgtTrq">—</span></div>
  <p style="font-size:0.78rem;color:#9ca3af;margin:0.5rem 0 0;">Aşama değişince STM32’a <strong>V:</strong> / <strong>Q:</strong> gönderilir (en az 1 sn aralık); her LIVE’da değil.</p>
  <p style="font-size:0.78rem;color:#9ca3af;margin:0.35rem 0 0;">Kademeli mod şu an gösterim modudur.</p>
  <h2 style="margin-top:1rem;font-size:0.95rem;">Kademe tablosu</h2>
  <table>
    <thead>
      <tr>
        <th>Mesafe</th>
        <th>Hedef hız</th>
        <th>Hedef tork</th>
      </tr>
    </thead>
    <tbody>
      <tr><td>0 – 2 m</td><td>100</td><td>5</td></tr>
      <tr><td>2 – 4 m</td><td>200</td><td>5</td></tr>
      <tr><td>4 – 6 m</td><td>300</td><td>10</td></tr>
      <tr><td>6 – 8 m</td><td>400</td><td>10</td></tr>
      <tr><td>8 – 10 m</td><td>500</td><td>15</td></tr>
    </tbody>
  </table>
</div>

<div class="btn-row">
  <button class="origin" onclick="sendCmd('/training_start')">Başlangıç Noktasını Belirle</button>
</div>
<div class="btn-row">
  <button class="parkcal" onclick="sendCmd('/calibrate_park')">Güvenli Park Noktasını Kalibre Et</button>
</div>
<div class="btn-row">
  <button class="start" onclick="sendCmd('/r')">START</button>
  <button class="stop" onclick="sendCmd('/s')">STOP</button>
</div>
<div class="btn-row">
  <button class="plus" onclick="sendCmd('/tp')">TORK +</button>
  <button class="minus" onclick="sendCmd('/tm')">TORK -</button>
</div>
<div class="btn-row">
  <button class="plus" onclick="sendCmd('/hp')">HIZ +</button>
  <button class="minus" onclick="sendCmd('/hm')">HIZ -</button>
</div>
<div class="btn-row">
  <button class="test" onclick="sendCmd('/teststart')">10m TEST BAŞLAT</button>
  <button class="stop" onclick="sendCmd('/testreset')">TEST SIFIRLA</button>
</div>

<div id="status">HAZIR</div>

<div class="dash-grid">
  <div class="dash-card"><h3>Anlık Metre</h3><div id="meter" class="val">—</div></div>
  <div class="dash-card"><h3>Gerçek Hız</h3><div id="speed" class="val">—</div></div>
  <div class="dash-card"><h3>Set Hız</h3><div id="setSpeed" class="val">—</div></div>
  <div class="dash-card"><h3>Tork</h3><div id="torque" class="val">—</div></div>
  <div class="dash-card"><h3>Max Hız</h3><div id="maxspeed" class="val">—</div></div>
  <div class="dash-card"><h3>Başlangıç Metre</h3><div id="trainingStart" class="val">—</div></div>
  <div class="dash-card"><h3>Yön</h3><div id="yon" class="val">—</div></div>
  <div class="dash-card"><h3>Faz</h3><div id="faz" class="val">—</div></div>
  <div class="dash-card"><h3>Kalibre Park Metresi</h3><div id="safePark" class="val">—</div></div>
  <div class="dash-card"><h3>Park Durumu</h3><div id="parkStatus" class="val">—</div></div>
</div>

<div class="section">
  <h2>10 Metre Test</h2>
  <div class="smallValue">Durum: <span id="testState">—</span></div>
  <div class="smallValue">Gidilen: <span id="dist">—</span> m</div>
  <div class="smallValue">Süre: <span id="time">—</span> sn</div>
  <div class="smallValue">Ortalama Hız: <span id="avg">—</span> m/sn</div>
</div>

<div class="section">
  <h2>Çalışma geçmişi (son 5)</h2>
  <table>
    <thead>
      <tr>
        <th></th>
        <th>No</th>
        <th>Mesafe (m)</th>
        <th>Süre (sn)</th>
        <th>Ort. hız</th>
        <th>Max hız</th>
        <th>Tork</th>
        <th>Set hız</th>
      </tr>
    </thead>
    <tbody id="historyTable">
    </tbody>
  </table>
</div>

<div class="section">
  <h2>Hız - Zaman Grafiği</h2>
  <canvas id="speedChart" width="520" height="220"></canvas>
</div>

<div class="section">
  <h2>Mesafe - Zaman Grafiği</h2>
  <canvas id="meterChart" width="520" height="220"></canvas>
</div>

<div class="section">
  <h2>Hız - Zaman / Mesafe Tablosu</h2>
  <table>
    <thead>
      <tr>
        <th>Süre</th>
        <th>Mesafe</th>
        <th>Hız</th>
      </tr>
    </thead>
    <tbody id="logTable">
    </tbody>
  </table>
</div>

</div>
</div>

<script>
const MODE_LABELS = { "0": "Serbest Mod", "1": "10m Test Modu", "2": "Kademeli Mod" };

let lastHistoryHtml = "";
let lastChartRedrawMs = 0;
const CHART_REDRAW_MS = 2000;

let uiMode = "0";

let trainingStartValue = null;
let safeParkValue = null;
let previousUiMeter = null;

function applyModeUi(modeKey, statusText)
{
  document.getElementById("status").innerHTML = statusText;
  uiMode = modeKey;
  document.getElementById("activeModeLabel").innerHTML = MODE_LABELS[modeKey] || modeKey;
  setModeHighlight(modeKey);
  var sp = document.getElementById("stagedPanel");
  if (modeKey === "2")
    sp.classList.remove("staged-hidden");
  else
    sp.classList.add("staged-hidden");
}

function requestModeFromEsp(modeKey)
{
  var paths = { "0": "/mode/free", "1": "/mode/test", "2": "/mode/staged" };
  var path = paths[modeKey];
  if (!path)
    return;

  fetch(path, { method: "GET", cache: "no-store" })
  .then(function(r) {
    if (!r.ok)
      throw new Error("HTTP " + r.status);
    return r.text();
  })
  .then(function(data) {
    applyModeUi(modeKey, data);
  })
  .catch(function() {
    document.getElementById("status").innerHTML = "MODE: istek basarisiz (" + path + ")";
  });
}

function selectModeFree()
{
  requestModeFromEsp("0");
}

function selectModeTest()
{
  requestModeFromEsp("1");
}

function selectModeStaged()
{
  requestModeFromEsp("2");
}

function bindModeCards()
{
  document.querySelectorAll("#modeRow .mode-card[data-mode]").forEach(function(card) {
    card.addEventListener("click", function(ev) {
      ev.preventDefault();
      var m = card.getAttribute("data-mode");
      if (m === "0")
        selectModeFree();
      else if (m === "1")
        selectModeTest();
      else if (m === "2")
        selectModeStaged();
    });
    card.addEventListener("keydown", function(ev) {
      if (ev.key === "Enter" || ev.key === " ")
      {
        ev.preventDefault();
        card.click();
      }
    });
  });
}

function computeStagedFromMetre(metreStr)
{
  let m = Math.abs(parseFloat(metreStr));
  if (isNaN(m)) m = 0;
  if (m >= 10.0) m = 9.999;
  let idx = Math.floor(m / 2.0);
  if (idx < 0) idx = 0;
  if (idx > 4) idx = 4;
  const bounds = [ 0, 2, 4, 6, 8, 10 ];
  const speeds = [ 100, 200, 300, 400, 500 ];
  const torqs = [ 5, 5, 10, 10, 15 ];
  return {
    idx: idx,
    lo: bounds[idx],
    hi: bounds[idx + 1],
    spd: speeds[idx],
    trq: torqs[idx]
  };
}

function parseCalibrationMeter(data, expectedStart)
{
  var s = (data || "").trim();
  if (s.indexOf(expectedStart) !== 0)
    return null;
  var colon = s.indexOf(":");
  if (colon < 0)
    return null;
  var v = parseFloat(s.substring(colon + 1).trim());
  if (isNaN(v))
    return null;
  return v;
}

function sendCmd(url)
{
  fetch(url, { method: "GET", cache: "no-store" })
  .then(function(r) { return r.text(); })
  .then(function(data) {
    document.getElementById("status").innerHTML = data;
    if (url === "/training_start")
    {
      var tm = parseCalibrationMeter(data, "BASLANGIC NOKTASI KAYDEDILDI");
      if (tm !== null)
      {
        trainingStartValue = tm;
        document.getElementById("trainingStart").innerHTML = tm.toFixed(2) + " m";
      }
    }
    else if (url === "/calibrate_park")
    {
      var pm = parseCalibrationMeter(data, "GUVENLI PARK NOKTASI KAYDEDILDI");
      if (pm !== null)
      {
        safeParkValue = pm;
        document.getElementById("safePark").innerHTML = pm.toFixed(2) + " m";
      }
    }
  })
  .catch(function() {});
}

function setModeHighlight(modeId)
{
  document.querySelectorAll(".mode-card").forEach(function(el) {
    el.classList.toggle("active", el.getAttribute("data-mode") === String(modeId));
  });
}

function parseChartData(raw)
{
  let arr = [];

  if(!raw || raw.length < 3) return arr;

  let rows = raw.split(';');

  for(let r of rows)
  {
    let p = r.split(',');
    if(p.length >= 3)
    {
      arr.push({
        t: parseFloat(p[0]),
        m: parseFloat(p[1]),
        s: parseFloat(p[2])
      });
    }
  }

  return arr;
}

function drawChart(canvasId, points, field, unit)
{
  let canvas = document.getElementById(canvasId);
  let ctx = canvas.getContext('2d');

  let w = canvas.width;
  let h = canvas.height;

  ctx.clearRect(0, 0, w, h);

  ctx.fillStyle = "#0b1220";
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = "#374151";
  ctx.lineWidth = 1;

  ctx.beginPath();
  ctx.moveTo(40, 10);
  ctx.lineTo(40, h - 30);
  ctx.lineTo(w - 10, h - 30);
  ctx.stroke();

  if(points.length < 2)
  {
    ctx.fillStyle = "#e5e7eb";
    ctx.font = "18px Arial";
    ctx.fillText("Veri bekleniyor", 70, 110);
    return;
  }

  let maxT = points[points.length - 1].t;
  let maxY = 0.1;

  for(let p of points)
  {
    if(p[field] > maxY) maxY = p[field];
  }

  ctx.strokeStyle = "#22c55e";
  ctx.lineWidth = 3;
  ctx.beginPath();

  for(let i = 0; i < points.length; i++)
  {
    let x = 40 + (points[i].t / maxT) * (w - 60);
    let y = (h - 30) - (points[i][field] / maxY) * (h - 50);

    if(i == 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }

  ctx.stroke();

  ctx.fillStyle = "#22c55e";
  ctx.font = "16px Arial";
  ctx.fillText("Max: " + maxY.toFixed(2) + " " + unit, 55, 25);
  ctx.fillText("t/sn", w - 50, h - 8);
}

function updateLive()
{
  fetch('/live')
  .then(response => response.text())
  .then(data => {
    let p = data.split('|');
    if (p.length < 4)
      return;

    let values = p[0].split(',');
    if (values.length < 9)
      return;

    document.getElementById("meter").innerHTML = values[0] + " m";
    document.getElementById("speed").innerHTML = values[1] + " m/sn";
    document.getElementById("setSpeed").innerHTML = values[3];
    document.getElementById("torque").innerHTML = values[2];
    document.getElementById("maxspeed").innerHTML = values[4] + " m/sn";

    document.getElementById("trainingStart").innerHTML =
      (trainingStartValue === null) ? "—" : (trainingStartValue.toFixed(2) + " m");
    document.getElementById("safePark").innerHTML =
      (safeParkValue === null) ? "—" : (safeParkValue.toFixed(2) + " m");

    var meter = parseFloat(values[0]);
    if (!isNaN(meter))
    {
      var dm = (previousUiMeter === null) ? 0 : (meter - previousUiMeter);
      previousUiMeter = meter;

      var yon = "—";
      var faz = "IDLE";
      if (trainingStartValue === null)
      {
        yon = "—";
        faz = "IDLE";
      }
      else if (Math.abs(dm) < 0.002)
      {
        yon = "Sabit";
        faz = "IDLE";
      }
      else
      {
        var uzaklasiyor = (meter > trainingStartValue && dm > 0) ||
          (meter < trainingStartValue && dm < 0);
        if (uzaklasiyor)
        {
          yon = "Uzaklaşıyor";
          faz = "RESIST";
        }
        else
        {
          yon = "Yaklaşıyor";
          faz = "ASSIST";
        }
      }
      document.getElementById("yon").innerHTML = yon;
      document.getElementById("faz").innerHTML = faz;

      var parkStatus;
      if (safeParkValue === null)
        parkStatus = "KALİBRE EDİLMEDİ";
      else if (meter <= safeParkValue)
        parkStatus = "PARKTA";
      else
        parkStatus = "PARK DIŞI";
      document.getElementById("parkStatus").innerHTML = parkStatus;
    }

    document.getElementById("testState").innerHTML = values[5];
    document.getElementById("dist").innerHTML = values[6];
    document.getElementById("time").innerHTML = values[7];
    document.getElementById("avg").innerHTML = values[8];

    document.getElementById("logTable").innerHTML = p[1] || '';

    let histChunk = p[3] || '';
    if (histChunk !== lastHistoryHtml)
    {
      document.getElementById("historyTable").innerHTML = histChunk;
      lastHistoryHtml = histChunk;
    }

    let testActive = (values[5] === "ÇALIŞIYOR");
    let nowMs = Date.now();
    if (testActive || (nowMs - lastChartRedrawMs >= CHART_REDRAW_MS))
    {
      lastChartRedrawMs = nowMs;
      let chartPoints = parseChartData(p[2] || '');
      drawChart("speedChart", chartPoints, "s", "m/sn");
      drawChart("meterChart", chartPoints, "m", "m");
    }

    if (uiMode === "2")
    {
      let st = computeStagedFromMetre(values[0]);
      document.getElementById("stagedMetre").innerHTML = values[0];
      document.getElementById("stagedStageIdx").innerHTML = String(st.idx);
      document.getElementById("stagedRange").innerHTML = String(st.lo) + " – " + String(st.hi);
      document.getElementById("stagedTgtSpd").innerHTML = String(st.spd);
      document.getElementById("stagedTgtTrq").innerHTML = String(st.trq);
    }
  })
  .catch(function() {});
}

bindModeCards();
setInterval(updateLive, 1000);
</script>

</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleLive()
{
  updateTestValues();

  String state = "BEKLİYOR";

  if (testRunning)
    state = "ÇALIŞIYOR";
  else if (testFinished)
    state = "TAMAMLANDI";

  float elapsedSec = testElapsedMs / 1000.0;

  String values =
    String(currentMeter, 2) + "," +
    String(liveRealSpeedMs, 2) + "," +
    String(liveTorque) + "," +
    String(liveSetSpeed) + "," +
    String(testMaxSpeed, 2) + "," +
    state + "," +
    String(distanceDone, 2) + "," +
    String(elapsedSec, 2) + "," +
    String(avgSpeed, 2);

  String table = "";

  for (int i = 0; i < logCount; i++)
  {
    table += "<tr><td>";
    table += String(logTime[i], 2);
    table += " sn</td><td>";
    table += String(logMeter[i], 2);
    table += " m</td><td>";
    table += String(logSpeed[i], 2);
    table += " m/sn</td></tr>";
  }

  String chartData = makeChartData();
  String histHtml = buildHistoryTableHtml();

  server.send(200, "text/plain",
              values + "|" + table + "|" + chartData + "|" + histHtml);
}

void handleModeFree()
{
  Serial.println("WEB: MODE FREE");
  panelMode = PANEL_MODE_FREE;
  lastAppliedStage = -1;
  lastAppliedSpeed = -1;
  lastAppliedTorque = -1;
  lastStagedCommandMs = 0;
  server.send(200, "text/plain", "MOD: SERBEST");
}

void handleModeTest()
{
  Serial.println("WEB: MODE TEST");
  panelMode = PANEL_MODE_TEST;
  lastAppliedStage = -1;
  lastAppliedSpeed = -1;
  lastAppliedTorque = -1;
  lastStagedCommandMs = 0;
  server.send(200, "text/plain", "MOD: 10m TEST");
}

void handleModeStaged()
{
  Serial.println("WEB: MODE STAGED");
  panelMode = PANEL_MODE_STAGED;
  lastAppliedStage = -1;
  lastAppliedSpeed = -1;
  lastAppliedTorque = -1;
  lastStagedCommandMs = 0;
  Serial.println("WEB: STAGED MODE SELECTED (edge V/Q on stage change)");
  server.send(200, "text/plain", "MOD: KADEMELI");
}

void handleTestStart()
{
  startMeter = currentMeter;

  distanceDone = 0.0;
  avgSpeed = 0.0;
  testElapsedMs = 0;
  logCount = 0;

  testMaxSpeed = 0.0;
  resultSavedForCurrentTest = false;

  testRunning = true;
  testFinished = false;
  testStartMs = millis();

  server.send(200, "text/plain", "10m TEST BASLADI");
}

void handleTrainingStartSet()
{
  trainingStartMeter = currentMeter;
  trainingStartValid = true;
  Serial.printf("WEB: TRAINING START METRE=%.3f\n", (double)trainingStartMeter);
  String msg = "BASLANGIC NOKTASI KAYDEDILDI: ";
  msg += String(trainingStartMeter, 2);
  server.send(200, "text/plain", msg);
}

void handleCalibratePark()
{
  safeParkMeter = currentMeter;
  safeParkValid = true;
  Serial.printf("WEB: PARK CALIBRATED meter=%.3f\n", (double)safeParkMeter);
  String msg = "GUVENLI PARK NOKTASI KAYDEDILDI: ";
  msg += String(safeParkMeter, 2);
  server.send(200, "text/plain", msg);
}

void handlePark()
{
  Serial.println("WEB: PARK REQUEST");
  server.send(200, "text/plain", "PARK (GOSTERIM)");
}

void handleTestReset()
{
  testRunning = false;
  testFinished = false;

  distanceDone = 0.0;
  avgSpeed = 0.0;
  testElapsedMs = 0;
  logCount = 0;

  resultSavedForCurrentTest = false;

  server.send(200, "text/plain", "TEST SIFIRLANDI");
}

void handleR()
{
  Serial.println("WEB: START");
  sendStmChar('R');
  server.send(200, "text/plain", "START GONDERILDI");
}

void handleS()
{
  Serial.println("WEB: STOP");
  sendStmChar('S');
  server.send(200, "text/plain", "STOP GONDERILDI");
}

void handleTorquePlus()
{
  Serial.println("WEB: TORQUE +");
  sendStmChar('A');
  server.send(200, "text/plain", "TORK ARTIRILDI");
}

void handleTorqueMinus()
{
  Serial.println("WEB: TORQUE -");
  sendStmChar('Z');
  server.send(200, "text/plain", "TORK AZALTILDI");
}

void handleSpeedPlus()
{
  Serial.println("WEB: SPEED +");
  sendStmChar('K');
  server.send(200, "text/plain", "HIZ ARTIRILDI");
}

void handleSpeedMinus()
{
  Serial.println("WEB: SPEED -");
  sendStmChar('M');
  server.send(200, "text/plain", "HIZ AZALTILDI");
}

void setup()
{
  Serial.begin(115200);

  Serial2.begin(115200, SERIAL_8N1, ESP_RX2, ESP_TX2);

  WiFi.softAP(ssid, password);

  Serial.println("ESP32 AP basladi");
  Serial.print("WiFi adi: ");
  Serial.println(ssid);
  Serial.print("IP adresi: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/live", handleLive);
  server.on("/teststart", handleTestStart);
  server.on("/testreset", handleTestReset);
  server.on("/training_start", handleTrainingStartSet);
  server.on("/calibrate_park", handleCalibratePark);
  server.on("/park", handlePark);

  server.on("/mode/free", handleModeFree);
  server.on("/mode/test", handleModeTest);
  server.on("/mode/staged", handleModeStaged);

  server.on("/r", handleR);
  server.on("/s", handleS);

  server.on("/tp", handleTorquePlus);
  server.on("/tm", handleTorqueMinus);

  server.on("/hp", handleSpeedPlus);
  server.on("/hm", handleSpeedMinus);

  server.begin();

  Serial.println("WEB SERVER BASLADI");
}

void loop()
{
  server.handleClient();

  while (Serial2.available())
  {
    char c = Serial2.read();
    // Serial.write(c);

    if (c == '\r' || c == '\n')
    {
      uartLine.trim();

      if (uartLine.length() > 0)
      {
        if (uartLine.startsWith("LIVE,"))
        {
          float m = 0.0f;
          float hiz = 0.0f;
          int tork = 0;
          int setHiz = 0;
          float maxHiz = 0.0f;

          if (parseLivePayload(uartLine, &m, &hiz, &tork, &setHiz, &maxHiz))
          {
            haveLiveMeter = true;

            if (!havePreviousMeter)
            {
              previousMeter = m;
              havePreviousMeter = true;
              currentMeter = m;
              lastYonLabel = "Sabit";
              lastFazLabel = "IDLE";
            }
            else
            {
              float dm = m - previousMeter;
              previousMeter = m;
              currentMeter = m;
              updateTrainingDirectionAndPhase(m, dm);
            }

            liveRealSpeedMs = hiz;
            liveTorque = tork;
            liveSetSpeed = setHiz;
            liveMaxHizMs = maxHiz;

            maybeApplyStagedAfterLive();

            if (testRunning)
            {
              if (liveRealSpeedMs > testMaxSpeed)
                testMaxSpeed = liveRealSpeedMs;

              updateTestValues();

              float elapsedSec = testElapsedMs / 1000.0;
              addLogPoint(elapsedSec, distanceDone, liveRealSpeedMs);
            }
          }
        }
      }

      uartLine = "";
    }
    else
    {
      if (uartLine.length() < 128)
        uartLine += c;
      else
        uartLine = "";
    }
  }
}
