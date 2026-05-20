/**
 * CAN Bus Logger - ESP32-S3 N16R8  (Full-Potential Build)
 *
 *  Hardware:
 *    Dual Xtensa LX7 cores @ 240 MHz
 *    8 MB OPI PSRAM  -> 300 K-frame ring buffer  (~6.0 MB / 8 MB)
 *    16 MB Flash
 *    Native USB-OTG  -> simultaneous SLCAN output (no CP2102 needed)
 *    TWAI (CAN) peripheral  -> 500 kbps, listen-only
 *
 *  Architecture:
 *    Core 1  Priority 10  canTask   - TWAI alert-driven (sleeps when bus idle)
 *    Core 0  Priority  5  webTask   - WiFi AP + HTTP server
 *    (Arduino loop task deleted after setup)
 *
 *  Web UI  ->  connect to "CAN-Logger" WiFi, open 192.168.4.1
 *    Live terminal    200 ms poll, up to 300 frames/burst
 *    FPS / buffer % / drop counter in real time
 *    CAN-ID filter
 *    Download CSV (full PSRAM buffer, us timestamps)
 *    Reset buffer
 *
 *  Native USB SLCAN: appears as CDC serial at any baud
 *
 *  Arduino IDE settings:
 *    Board   : ESP32S3 Dev Module
 *    PSRAM   : OPI PSRAM
 *    USB Mode: USB-OTG (TinyUSB)
 *    CPU Freq: 240 MHz
 *
 *  CAN transceiver: SN65HVD230
 *    TX -> GPIO16,  RX -> GPIO17
 */

#include "driver/twai.h"
#include "esp_timer.h"
#include <WiFi.h>
#include <WebServer.h>

// -- Native USB CDC toggle ---------------------------------------------------
// Set 0 if you want plain UART Serial (e.g. debugging without host USB)
#define USE_NATIVE_USB  1

#if USE_NATIVE_USB
  #include "USB.h"
  #include "USBCDC.h"
  USBCDC USBSerial;
  #define SLCAN_PORT  USBSerial
#else
  #define SLCAN_PORT  Serial
#endif

// -- CAN Transceiver Pins ----------------------------------------------------
#define TX_PIN   GPIO_NUM_16
#define RX_PIN   GPIO_NUM_17

// -- WiFi AP -----------------------------------------------------------------
#define WIFI_SSID  "CAN-Logger"
#define WIFI_PASS  "12345678"

// -- Frame Buffer ------------------------------------------------------------
#define MAX_FRAMES   300000UL
#define API_BATCH    300
#define TWAI_Q_LEN   128

struct LogFrame {
  uint32_t ts_us;
  uint32_t id;
  uint8_t  dlc;
  uint8_t  data[8];
  uint8_t  extd;
};

static LogFrame* rx_buffer = nullptr;
static volatile uint32_t write_count  = 0;
static volatile uint32_t drop_count   = 0;
static volatile uint32_t fps_current  = 0;

WebServer server(80);

// -- Embedded Web UI (PROGMEM) -----------------------------------------------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CAN Logger ESP32-S3</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#07070e;color:#c8ffc8;font-family:'Courier New',monospace;height:100vh;display:flex;flex-direction:column;padding:8px;gap:5px}
h2{color:#4af;font-size:12px;letter-spacing:3px}
#bar{display:flex;gap:5px;align-items:center;flex-wrap:wrap}
button{background:#111;color:#4af;border:1px solid #4af;padding:3px 9px;cursor:pointer;font-family:inherit;font-size:11px;border-radius:2px;transition:.15s}
button:hover{background:#4af;color:#000}
#pbtn{color:#fa4;border-color:#fa4}
#pbtn:hover,#pbtn.act:hover{background:#fa4;color:#000}
#pbtn.act{color:#4af;border-color:#4af}
#pbtn.act:hover{background:#4af;color:#000}
#rbtn{color:#f55;border-color:#f55}
#rbtn:hover{background:#f55;color:#000}
#flt{background:#111;color:#c8ffc8;border:1px solid #333;padding:3px 7px;font-family:inherit;font-size:11px;width:130px;border-radius:2px}
#flt:focus{outline:none;border-color:#4af}
#dot{font-size:11px;color:#888;margin-left:auto}
#sbar{display:flex;gap:14px;font-size:10px;color:#555;padding:3px 0 3px;border-bottom:1px solid #111;align-items:center}
#sbar b{color:#4fc}
#sbar .warn{color:#fa4}
#bwrap{flex:1;max-width:150px;height:5px;background:#1a1a2e;border-radius:3px}
#bfill{height:100%;border-radius:3px;transition:width .6s,background .6s;background:#4fc}
#term{flex:1;background:#040410;border:1px solid #1a1a2e;overflow-y:auto;padding:5px;font-size:11px;line-height:1.55}
.hdr{color:#2a2a3a}
.row{display:flex}
.ts{color:#4a4a6a;min-width:96px;font-size:10px;padding-top:1px}
.sid{color:#4af;min-width:66px;font-weight:bold}
.eid{color:#f84;min-width:66px;font-weight:bold}
.byt{color:#4fc}
</style>
</head><body>
<h2>CAN BUS LOGGER - ESP32-S3 N16R8</h2>
<div id="bar">
  <button id="pbtn" onclick="togglePause()">Pause</button>
  <button onclick="clearView()">Clear</button>
  <button onclick="location.href='/download'">Download CSV</button>
  <button id="rbtn" onclick="doReset()">Reset Buffer</button>
  <input id="flt" type="text" placeholder="Filter CAN ID (e.g. 7E8)" oninput="filtChange()">
  <span id="dot">Connecting...</span>
</div>
<div id="sbar">
  <span>FPS <b id="s-fps">-</b></span>
  <span>Frames <b id="s-tot">-</b></span>
  <span>Drops <b id="s-drp" class="warn">0</b></span>
  <span>Buffer <b id="s-pct">-</b>%</span>
  <div id="bwrap"><div id="bfill" style="width:0"></div></div>
</div>
<div id="term"><span class="hdr">-- ts_us -------- CAN_ID -- data bytes (hex) --</span></div>
<script>
const POLL_MS=200,STATS_MS=800,MAX_ROWS=4000;
let paused=false,lastIdx=0,flt='',rowCount=0;
const term=document.getElementById('term');
function togglePause(){paused=!paused;const b=document.getElementById('pbtn');b.textContent=paused?'Resume':'Pause';b.className=paused?'act':'';}
function clearView(){term.innerHTML='<span class="hdr">-- ts_us -------- CAN_ID -- data bytes (hex) --</span>';rowCount=0;}
function filtChange(){flt=document.getElementById('flt').value.trim().toUpperCase();}
async function doReset(){if(!confirm('Wipe PSRAM buffer? All logged frames lost.'))return;await fetch('/api/reset',{method:'POST'});lastIdx=0;clearView();}
async function pollFrames(){if(paused)return;try{const r=await fetch('/api/frames?since='+lastIdx,{signal:AbortSignal.timeout(350)});if(!r.ok)return;const d=await r.json();document.getElementById('dot').textContent='Live';if(!d.frames||!d.frames.length)return;lastIdx=d.next;let html='';for(const f of d.frames){if(flt&&!f.id.includes(flt))continue;html+='<div class="row"><span class="ts">'+f.ts+'</span><span class="'+(f.e?'eid':'sid')+'">'+f.id+'</span><span class="byt">'+f.b+'</span></div>';rowCount++;}term.insertAdjacentHTML('beforeend',html);while(rowCount>MAX_ROWS){const rows=term.getElementsByClassName('row');if(rows.length){rows[0].remove();rowCount--;}else break;}term.scrollTop=term.scrollHeight;}catch(e){document.getElementById('dot').textContent='ERR: '+e.message;}}
async function pollStats(){try{const r=await fetch('/api/stats',{signal:AbortSignal.timeout(300)});if(!r.ok)return;const d=await r.json();document.getElementById('s-fps').textContent=d.fps;document.getElementById('s-tot').textContent=d.total.toLocaleString();document.getElementById('s-drp').textContent=d.drops;document.getElementById('s-pct').textContent=d.pct;const fill=document.getElementById('bfill');fill.style.width=d.pct+'%';fill.style.background=d.pct>90?'#f55':d.pct>70?'#fa4':'#4fc';}catch(e){}}
setInterval(pollFrames,POLL_MS);
setInterval(pollStats,STATS_MS);
</script>
</body></html>
)rawliteral";

// -- Atomic helpers ----------------------------------------------------------
static inline uint32_t atomic_read(volatile uint32_t* p){return __atomic_load_n(p,__ATOMIC_ACQUIRE);}
static inline void atomic_inc(volatile uint32_t* p){__atomic_fetch_add(p,1,__ATOMIC_RELEASE);}
static inline void atomic_write(volatile uint32_t* p,uint32_t v){__atomic_store_n(p,v,__ATOMIC_RELEASE);}

// -- HTTP Handlers -----------------------------------------------------------
void handleRoot(){server.send_P(200,"text/html",INDEX_HTML);}

void handleFrames(){
  uint32_t wc=atomic_read(&write_count),since=0;
  if(server.hasArg("since"))since=(uint32_t)server.arg("since").toInt();
  if(since>wc)since=wc;
  if(wc-since>MAX_FRAMES)since=wc-MAX_FRAMES;
  uint32_t avail=wc-since;
  uint32_t count=(avail>API_BATCH)?API_BATCH:avail;
  String json;json.reserve(count*70+64);
  json="{\"total\":";json+=wc;json+=",\"next\":";json+=(since+count);json+=",\"frames\":["; 
  char tmp[24];
  for(uint32_t i=0;i<count;i++){
    const LogFrame&f=rx_buffer[(since+i)%MAX_FRAMES];
    if(i)json+=',';
    json+="{\"ts\":";snprintf(tmp,sizeof(tmp),"%lu",(unsigned long)f.ts_us);json+=tmp;
    json+=",\"id\":\"";if(f.extd)snprintf(tmp,sizeof(tmp),"%08lX",(unsigned long)f.id);else snprintf(tmp,sizeof(tmp),"%03lX",(unsigned long)f.id);json+=tmp;
    json+="\",\"e\":";json+=(f.extd?1:0);json+=",\"b\":\"";
    for(int b=0;b<f.dlc;b++){if(b)json+=' ';snprintf(tmp,sizeof(tmp),"%02X",f.data[b]);json+=tmp;}
    json+="\"}";
  }
  json+="]}";
  server.send(200,"application/json",json);
}

void handleStats(){
  uint32_t wc=atomic_read(&write_count),dc=atomic_read(&drop_count),fps=atomic_read(&fps_current);
  uint32_t filled=(wc>MAX_FRAMES)?MAX_FRAMES:wc;
  uint8_t pct=(uint8_t)((filled*100ULL)/MAX_FRAMES);
  char buf[80];
  snprintf(buf,sizeof(buf),"{\"fps\":%lu,\"total\":%lu,\"drops\":%lu,\"pct\":%u}",(unsigned long)fps,(unsigned long)wc,(unsigned long)dc,pct);
  server.send(200,"application/json",buf);
}

void handleReset(){atomic_write(&write_count,0);atomic_write(&drop_count,0);atomic_write(&fps_current,0);server.send(200,"application/json","{\"ok\":1}");}

void handleDownload(){
  uint32_t wc=atomic_read(&write_count),start=(wc>MAX_FRAMES)?(wc-MAX_FRAMES):0;
  server.sendHeader("Content-Disposition","attachment; filename=\"can_log.csv\"");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"text/csv","");
  server.sendContent("timestamp_us,can_id,dlc,d0,d1,d2,d3,d4,d5,d6,d7\r\n");
  char line[96];
  for(uint32_t i=start;i<wc;i++){
    const LogFrame&f=rx_buffer[i%MAX_FRAMES];
    char idbuf[10];
    if(f.extd)snprintf(idbuf,sizeof(idbuf),"%08lX",(unsigned long)f.id);else snprintf(idbuf,sizeof(idbuf),"%03lX",(unsigned long)f.id);
    snprintf(line,sizeof(line),"%lu,%s,%u",(unsigned long)f.ts_us,idbuf,f.dlc);
    server.sendContent(String(line));
    for(int b=0;b<8;b++){if(b<f.dlc)snprintf(line,sizeof(line),",%02X",f.data[b]);else snprintf(line,sizeof(line),",");server.sendContent(String(line));}
    server.sendContent("\r\n");
  }
  server.sendContent("");
}

// -- Core 1 - CAN Ingestion Task (Priority 10) --------------------------------
void canTask(void*){
  uint32_t fps_accum=0,fps_t=millis();
  char slcan[36];
  while(true){
    uint32_t alerts=0;
    twai_read_alerts(&alerts,portMAX_DELAY);
    if(alerts&TWAI_ALERT_RX_DATA){
      twai_message_t msg;
      while(twai_receive(&msg,0)==ESP_OK){
        uint32_t idx=atomic_read(&write_count)%MAX_FRAMES;
        rx_buffer[idx].ts_us=(uint32_t)esp_timer_get_time();
        rx_buffer[idx].id=msg.identifier;
        rx_buffer[idx].dlc=msg.data_length_code;
        rx_buffer[idx].extd=msg.extd?1:0;
        memcpy(rx_buffer[idx].data,msg.data,msg.data_length_code);
        atomic_inc(&write_count);
        fps_accum++;
        int n;
        if(msg.extd)n=snprintf(slcan,sizeof(slcan),"T%08lX%u",(unsigned long)msg.identifier,msg.data_length_code);
        else n=snprintf(slcan,sizeof(slcan),"t%03lX%u",(unsigned long)msg.identifier,msg.data_length_code);
        for(int b=0;b<msg.data_length_code&&n<(int)sizeof(slcan)-3;b++)n+=snprintf(slcan+n,sizeof(slcan)-n,"%02X",msg.data[b]);
        slcan[n++]='\r';
        SLCAN_PORT.write((uint8_t*)slcan,n);
      }
    }
    if(alerts&TWAI_ALERT_RX_QUEUE_FULL)atomic_inc(&drop_count);
    uint32_t now=millis();
    if(now-fps_t>=1000){atomic_write(&fps_current,fps_accum);fps_accum=0;fps_t=now;}
  }
}

// -- Core 0 - Web Server Task (Priority 5) ------------------------------------
void webTask(void*){while(true){server.handleClient();vTaskDelay(1);}}

// -- Setup -------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
#if USE_NATIVE_USB
  USB.begin();
  USBSerial.begin(0);
#endif
  if(psramInit())rx_buffer=(LogFrame*)ps_malloc(MAX_FRAMES*sizeof(LogFrame));
  if(!rx_buffer){Serial.println("[WARN] PSRAM alloc failed - heap fallback (1000 frames)");rx_buffer=(LogFrame*)malloc(1000*sizeof(LogFrame));}
  else{Serial.printf("[OK] PSRAM buffer  %lu frames  %lu B/frame  %.2f MB\n",MAX_FRAMES,(unsigned long)sizeof(LogFrame),(float)(MAX_FRAMES*sizeof(LogFrame))/1048576.0f);}
  WiFi.softAP(WIFI_SSID,WIFI_PASS);
  Serial.printf("[OK] AP: %-16s  IP: %s\n",WIFI_SSID,WiFi.softAPIP().toString().c_str());
  server.on("/",HTTP_GET,handleRoot);
  server.on("/api/frames",HTTP_GET,handleFrames);
  server.on("/api/stats",HTTP_GET,handleStats);
  server.on("/api/reset",HTTP_POST,handleReset);
  server.on("/download",HTTP_GET,handleDownload);
  server.begin();
  Serial.println("[OK] HTTP :80");
  twai_general_config_t g=TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN,RX_PIN,TWAI_MODE_LISTEN_ONLY);
  twai_timing_config_t t=TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f=TWAI_FILTER_CONFIG_ACCEPT_ALL();
  g.rx_queue_len=TWAI_Q_LEN;
  if(twai_driver_install(&g,&t,&f)!=ESP_OK){Serial.println("[ERR] TWAI install failed");while(true)vTaskDelay(1000);}
  twai_reconfigure_alerts(TWAI_ALERT_RX_DATA|TWAI_ALERT_RX_QUEUE_FULL,NULL);
  twai_start();
  Serial.println("[OK] TWAI 500kbps listen-only, alerts enabled");
  xTaskCreatePinnedToCore(canTask,"CAN",4096,NULL,10,NULL,1);
  xTaskCreatePinnedToCore(webTask,"WEB",8192,NULL,5,NULL,0);
  Serial.println("[OK] Tasks running");
}

void loop(){vTaskDelete(NULL);}
