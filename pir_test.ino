/* ============================================================
   PIR人体红外传感器检测台  v1.1 (SSE推送)
   ------------------------------------------------------------
   用途: 独立测试HC-SR501, 网页实时查看触发状态/波形/事件日志
   接线: PIR OUT -> GPIO16(RX2丝印), VCC->5V(VIN), GND->GND
         指示LED: 阳极经220Ω->GPIO4, 阴极->GND
         OLED: SDA->D21, SCL->D22, VCC->3.3V(禁接5V), GND->GND
   传输: SSE长连接, 边沿50ms检测即推, 端到端延迟约50ms
   注意: 上电预热约60s, 期间输出可能误跳; OLED上电秒亮
   ============================================================ */
#include <WiFi.h>
#include <WebServer.h>

// ---------------- 引脚 ----------------
#define PIN_PIR 16   // PIR输出, 高电平=检测到人
#define PIN_LED 4    // 指示LED: 有人=亮

// ---------------- OLED ----------------
#define USE_OLED 1     // 1=启用OLED(需安装Adafruit_SSD1306库)
#if USE_OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
#endif

// ---------------- WiFi (AP自建热点) ----------------
const char* AP_SSID = "PIR-TestBoard";
const char* AP_PASS = "12345678";   // WPA2密码至少8位

// ---------------- 采样/记录 ----------------
const unsigned long EDGE_MS = 50;                  // 边沿检测周期, 决定触发延迟下限
const unsigned long WAVE_MS = 250;                 // 波形采样周期, 96格=24s窗口
const uint8_t HIST_N = 96;
const uint8_t LOG_N = 10;

bool pirState = false;                            // 当前: true=有人
unsigned long stateSince = 0;                     // 当前状态起始时刻
unsigned long triggerCount = 0;                   // 累计触发次数(进入有人计一次)
unsigned long lastHighMs = 0;                     // 上次"有人"持续时长
unsigned long lastLowMs = 0;                      // 上次"无人"持续时长
uint8_t hist[HIST_N];                             // 波形环形缓冲 0/1
uint8_t histIdx = 0;
String logs[LOG_N];                               // 事件日志, 新的在前
uint8_t logCount = 0;

WebServer server(80);

void pushLog(const String& s) {
  if (logCount < LOG_N) logCount++;
  for (uint8_t i = logCount - 1; i > 0; i--) logs[i] = logs[i - 1];
  logs[0] = s;
}

// ---------------- 边沿记录 ----------------
// now=true: 无人->有人(触发); now=false: 有人->无人(恢复)
void recordEdge(bool now, unsigned long m) {
  unsigned long dur = m - stateSince;
  char buf[56];
  if (now) {                                       // 进入有人
    lastLowMs = dur;                              // 刚结束的"无人"持续了dur
    triggerCount++;
    snprintf(buf, sizeof(buf), "[+%6.1fs] 触发 → 有人", m / 1000.0f);
  } else {                                        // 离开有人
    lastHighMs = dur;                             // 刚结束的"有人"持续了dur
    snprintf(buf, sizeof(buf), "[+%6.1fs] 恢复 → 无人 (保持%.1fs)", m / 1000.0f, dur / 1000.0f);
  }
  pirState = now;
  stateSince = m;
  pushLog(String(buf));
  Serial.println(buf);
  digitalWrite(PIN_LED, pirState ? HIGH : LOW);
}

// ---------------- 采样任务 (SSE handler与主循环共用) ----------------
bool sampleEdge() {
  static unsigned long lastEdge = 0;
  if (millis() - lastEdge < EDGE_MS) return false;
  lastEdge = millis();
  bool now = (digitalRead(PIN_PIR) == HIGH);
  if (now == pirState) return false;
  recordEdge(now, millis());
  return true;
}
bool sampleWave() {
  static unsigned long lastWave = 0;
  if (millis() - lastWave < WAVE_MS) return false;
  lastWave = millis();
  hist[histIdx] = pirState ? 1 : 0;
  histIdx = (histIdx + 1) % HIST_N;
  return true;
}

// ---------------- OLED 本地显示 ----------------
#if USE_OLED
void oledInit() {
  Wire.begin(21, 22);                          // I2C: SDA=D21, SCL=D22
  if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] 已启用 (0x3C)");
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("Booting...");
    oled.display();
  } else {
    Serial.println("[OLED] 初始化失败, 检查接线/地址");
  }
}

void oledRefresh() {
  static unsigned long lastOled = 0;                // 200ms门控: 主循环与SSE循环共用
  if (millis() - lastOled < 200) return;
  lastOled = millis();
  char buf[26];
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  // 行1: AP热点IP
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(WiFi.softAPIP().toString());
  // 行2-3: 状态大字
  oled.setTextSize(3);
  oled.setCursor(0, 14);
  if (pirState) {
    oled.print("HUMAN!");
  } else {
    oled.print("CLEAR");
  }
  // 行4: 触发次数 + 当前状态持续时长
  oled.setTextSize(1);
  oled.setCursor(0, 41);
  snprintf(buf, sizeof(buf), "CNT:%lu  CUR:%.1fs", triggerCount, (millis() - stateSince) / 1000.0f);
  oled.print(buf);
  // 行5: 上次有人/无人持续 + 客户端数
  oled.setCursor(0, 53);
  snprintf(buf, sizeof(buf), "H%.1fs L%.1fs  AP:%d",
           lastHighMs / 1000.0f, lastLowMs / 1000.0f, WiFi.softAPgetStationNum());
  oled.print(buf);
  oled.display();
}
#endif

// ---------------- 状态JSON ----------------
String jsonStatus() {
  String j;
  j.reserve(384);
  j = "{";
  j += "\"pir\":" + String(pirState ? "true" : "false") + ",";
  j += "\"cnt\":" + String(triggerCount) + ",";
  j += "\"cur\":" + String((millis() - stateSince) / 1000.0f, 1) + ",";
  j += "\"lh\":" + String(lastHighMs / 1000.0f, 1) + ",";
  j += "\"ll\":" + String(lastLowMs / 1000.0f, 1) + ",";
  j += "\"up\":" + String(millis() / 1000) + ",";
  j += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  j += "\"log\":[";
  for (uint8_t i = 0; i < logCount; i++) { if (i) j += ","; j += "\"" + logs[i] + "\""; }
  j += "],\"his\":[";
  for (uint8_t i = 0; i < HIST_N; i++) { if (i) j += ","; j += String(hist[(histIdx + i) % HIST_N]); }
  j += "]}";
  return j;
}

// ---------------- SSE 长连接推送 (裸写socket, 不经WebServer组帧) ----------------
void handleEvents() {
  WiFiClient client = server.client();
  client.setNoDelay(true);                          // 关Nagle, 避免小包攒200ms
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
               "Cache-Control: no-cache\r\nConnection: keep-alive\r\n"
               "Access-Control-Allow-Origin: *\r\n\r\n");
  client.print("data: " + jsonStatus() + "\n\n");   // 连接建立立即推一帧
  while (client.connected()) {
    bool push = sampleEdge();                        // 边沿: 立即推送, ~50ms到达
    if (sampleWave()) push = true;                   // 波形心跳: 250ms推送
    if (push) client.print("data: " + jsonStatus() + "\n\n");
#if USE_OLED
    oledRefresh();                                   // SSE占用主循环期间OLED照常刷新
#endif
    delay(1);                                        // 让出WiFi/TCP栈
  }
  client.stop();
}

// ---------------- 轮询兜底 (SSE不可用时网页自动切换到此接口) ----------------
void handleData() { server.send(200, "application/json", jsonStatus()); }

static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PIR 人体红外检测台</title>
<style>
:root{--bg:#060a10;--panel:#0c1320;--panel2:#0a101b;--line:#1c2940;--cyan:#2dd4e8;--green:#34d399;
--amber:#fbbf24;--red:#f87171;--text:#dbe7f4;--dim:#5b6b82;--mono:Consolas,'Courier New',monospace}
*{margin:0;padding:0;box-sizing:border-box}
body{background:var(--bg);color:var(--text);font-family:var(--mono);min-height:100vh;
background-image:linear-gradient(rgba(45,212,232,.03) 1px,transparent 1px),linear-gradient(90deg,rgba(45,212,232,.03) 1px,transparent 1px);
background-size:28px 28px}
body::after{content:'';position:fixed;inset:0;pointer-events:none;background:repeating-linear-gradient(0deg,rgba(0,0,0,.12) 0 1px,transparent 1px 3px)}
.wrap{max-width:640px;margin:0 auto;padding:18px 14px 40px}
header{display:flex;justify-content:space-between;align-items:center;padding:10px 0 16px;border-bottom:1px solid var(--line);margin-bottom:16px}
.brand h1{font-size:15px;font-weight:600;letter-spacing:2px}
.brand small{display:block;font-size:10px;color:var(--dim);letter-spacing:3px;font-weight:400}
#badge{padding:7px 16px;font-size:14px;letter-spacing:2px;border:1px solid;border-radius:2px;position:relative}
#badge::before,#badge::after{content:'';position:absolute;width:6px;height:6px;border:1px solid currentColor}
#badge::before{top:-1px;left:-1px;border-right:none;border-bottom:none}
#badge::after{bottom:-1px;right:-1px;border-left:none;border-top:none}
.st0{color:var(--green);border-color:var(--green)}
.st1{color:var(--red);border-color:var(--red);animation:bl .6s infinite}
@keyframes bl{50%{opacity:.3}}
.card{background:var(--panel);border:1px solid var(--line);padding:14px;margin-bottom:10px;position:relative}
.card::before{content:'';position:absolute;top:-1px;left:-1px;width:7px;height:7px;border-top:1px solid var(--cyan);border-left:1px solid var(--cyan)}
.card h3{font-size:11px;color:var(--dim);letter-spacing:3px;font-weight:400;margin-bottom:12px;display:flex;justify-content:space-between}
.big{font-size:38px;line-height:1.1;text-shadow:0 0 14px rgba(45,212,232,.4)}
.big.ok{color:var(--green);text-shadow:0 0 14px rgba(52,211,153,.4)}
.big.hit{color:var(--red);text-shadow:0 0 14px rgba(248,113,113,.5);animation:bl .6s infinite}
.big small{font-size:13px;color:var(--dim);text-shadow:none;margin-left:6px}
#warm{margin-top:10px;font-size:12px;color:var(--amber);display:none}
.row{display:flex;justify-content:space-between;align-items:baseline;padding:7px 0;border-bottom:1px dashed rgba(28,41,64,.8);font-size:13px}
.row:last-child{border:none}
.row b{font-weight:400;color:var(--dim)}
.row .v{color:var(--text)} .row .v.hot{color:var(--red)} .row .v.money{color:var(--cyan);font-size:17px}
.box{display:flex;gap:2px;height:26px;align-items:flex-end}
.box i{flex:1;background:#13202f;border:1px solid var(--line)}
.box i.hit{background:rgba(248,113,113,.8);border-color:var(--red);box-shadow:0 0 6px rgba(248,113,113,.5)}
.legend{display:flex;justify-content:space-between;font-size:11px;color:var(--dim);margin-top:6px}
#log div{padding:6px 0;border-bottom:1px dashed rgba(28,41,64,.8);font-size:12px;color:var(--dim)}
#log div:last-child{border:none}
#log div.h{color:var(--red)}
#log .empty{color:var(--dim);opacity:.6}
footer{margin-top:16px;padding-top:12px;border-top:1px solid var(--line);display:flex;justify-content:space-between;font-size:10.5px;color:var(--dim);letter-spacing:1px}
.dot{display:inline-block;width:6px;height:6px;border-radius:50%;background:var(--green);margin-right:5px;animation:pl 2s infinite}
@keyframes pl{50%{opacity:.4}}
.reveal{opacity:0;transform:translateY(8px);animation:rv .5s forwards}
@keyframes rv{to{opacity:1;transform:none}}
</style></head><body><div class="wrap">
<header class="reveal">
  <div class="brand"><h1>PIR 人体红外检测台<small>HC-SR501 TESTBENCH</small></h1></div>
  <div id="badge" class="st0">无人</div>
</header>
<div class="card reveal" style="animation-delay:.08s">
  <h3>实时状态 <span id="curTxt">0.0 s</span></h3>
  <div class="big ok" id="big">无人<small>NO HUMAN</small></div>
  <div id="warm">⏳ HC-SR501 上电预热约 60s，期间输出可能乱跳，属正常现象</div>
</div>
<div class="card reveal" style="animation-delay:.14s">
  <h3>输出波形 <span id="wtag">近 24 秒</span></h3>
  <div class="box" id="box"></div>
  <div class="legend"><span>无人</span><span>每格 250ms</span><span>有人</span></div>
</div>
<div class="card reveal" style="animation-delay:.2s">
  <h3>统计</h3>
  <div class="row"><b>当前电平</b><span class="v" id="lvl">LOW (0)</span></div>
  <div class="row"><b>累计触发次数</b><span class="v money" id="cnt">0 次</span></div>
  <div class="row"><b>上次有人持续</b><span class="v" id="lh">-- s</span></div>
  <div class="row"><b>上次无人持续</b><span class="v" id="ll">-- s</span></div>
  <div class="row"><b>开机时长 / WiFi信号</b><span class="v" id="up">--</span></div>
</div>
<div class="card reveal" style="animation-delay:.26s">
  <h3>事件日志 <span>最新在上 · 最多10条</span></h3>
  <div id="log"><div class="empty">暂无事件</div></div>
</div>
<footer class="reveal" style="animation-delay:.3s">
  <span><i class="dot" id="conn"></i><span id="connTxt">已连接</span></span>
  <span id="ipTxt">--</span><span>PIR TEST v1.1 / SSE</span>
</footer>
</div>
<script>
const $=id=>document.getElementById(id);
const box=$('box');
for(let i=0;i<96;i++){box.appendChild(document.createElement('i'))}
function render(d){
  const hit=d.pir;
  const b=$('badge');b.className=hit?'st1':'st0';b.textContent=hit?'有人':'无人';
  const bg=$('big');
  bg.className='big '+(hit?'hit':'ok');
  bg.innerHTML=hit?'⚠ 有人<small>HUMAN</small>':'无人<small>NO HUMAN</small>';
  $('curTxt').textContent='保持 '+d.cur.toFixed(1)+' s';
  $('warm').style.display=d.up<60?'block':'none';
  const bars=box.children;
  for(let i=0;i<96;i++){bars[i].className=d.his[i]?'hit':''}
  $('lvl').textContent=hit?'HIGH (1)':'LOW (0)';
  $('cnt').textContent=d.cnt+' 次';
  $('lh').textContent=d.lh>0?d.lh.toFixed(1)+' s':'-- s';
  $('ll').textContent=d.ll>0?d.ll.toFixed(1)+' s':'-- s';
  $('up').textContent=d.up+' s / '+d.rssi+' dBm';
  const lg=$('log');
  if(d.log.length){lg.innerHTML=d.log.map(s=>'<div class="'+(s.includes('有人')?'h':'')+'">'+s+'</div>').join('')}
  if(!$('ipTxt').textContent.includes('.'))$('ipTxt').textContent=location.host||'--';
}
if(location.protocol.startsWith('http'))$('ipTxt').textContent=location.host;
let es=null,esFails=0,gotAny=false,pollTimer=null;
function startPoll(){
  if(pollTimer)return;
  const tick=async()=>{
    try{const r=await fetch('/data',{cache:'no-store'});render(await r.json());gotAny=true;
      $('connTxt').textContent='已连接(轮询)';$('conn').style.background='var(--green)';}
    catch(e){$('connTxt').textContent='连接中断';$('conn').style.background='var(--red)';}
    pollTimer=setTimeout(tick,300);
  };
  tick();
}
function startSSE(){
  es=new EventSource('/events');
  es.onopen=()=>{esFails=0;$('connTxt').textContent='已连接';$('conn').style.background='var(--green)';};
  es.onmessage=e=>{gotAny=true;try{render(JSON.parse(e.data));}catch(_){}};
  es.onerror=()=>{
    $('connTxt').textContent='重连中';$('conn').style.background='var(--amber)';
    if(!gotAny&&++esFails>=2){es.close();es=null;startPoll();}
  };
}
startSSE();
</script></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// ---------------- 初始化 ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

#if USE_OLED
  oledInit();
#endif

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);            // 关闭modem sleep, 保持射频常醒
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[AP] 热点 %s 已开启, IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/events", handleEvents);
  server.enableCORS(true);
  server.begin();
  Serial.println("========================================");
  Serial.println("  PIR 人体红外传感器检测台 v1.1 (SSE)");
  Serial.printf("  热点: %s (密码 %s)\n", AP_SSID, AP_PASS);
  Serial.printf("  网页: http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("  手机连此热点后用浏览器打开上面网址");
  Serial.println("  提示: 上电预热约60s后再开始测试");
  Serial.println("========================================");
}

void loop() {
  server.handleClient();          // SSE连接时在此阻塞, 采样由handleEvents内部完成
  sampleEdge();                  // 无SSE客户端时保持采样, 让LED独立可用
  sampleWave();
  static unsigned long lastChk = 0;
  static uint8_t lastStations = 99;
  if (millis() - lastChk >= 3000) {
    lastChk = millis();
    uint8_t n = WiFi.softAPgetStationNum();
    if (n != lastStations) {
      lastStations = n;
      Serial.printf("[AP] 当前连接客户端: %d\n", n);
    }
  }
#if USE_OLED
  oledRefresh();
#endif
}
