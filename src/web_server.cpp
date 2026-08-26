#include "web_server.h"
#include <math.h>
#include "arm.h"
#include "config.h"
#include "joint_model.h"
#include "nvs_store.h"
#include "wifi_manager.h"

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>6-Axis Arm Controller</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;background:#0f172a;color:#f8fafc;min-height:100vh;padding:16px;display:flex;flex-direction:column;align-items:center}
.header{text-align:center;margin-bottom:14px}
.header h1{font-size:1.4rem;color:#38bdf8}
.header p{font-size:.8rem;color:#94a3b8}
.tabs{display:flex;gap:6px;background:#1e293b;padding:6px;border-radius:10px;margin-bottom:16px;flex-wrap:wrap;justify-content:center;width:100%;max-width:900px}
.tab-btn{background:transparent;border:none;color:#94a3b8;padding:8px 14px;border-radius:8px;font-size:.85rem;font-weight:600;cursor:pointer}
.tab-btn.active{background:#38bdf8;color:#0f172a}
.tab-btn.disabled{opacity:.35;cursor:not-allowed}
.tab-pane{display:none;width:100%;max-width:900px}
.tab-pane.active{display:block}
.card{background:#1e293b;border-radius:14px;padding:18px;box-shadow:0 8px 20px rgba(0,0,0,.35);margin-bottom:14px}
.card h2{font-size:1.05rem;color:#38bdf8;margin-bottom:12px;border-bottom:1px solid #334155;padding-bottom:8px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
.badge{padding:3px 10px;border-radius:12px;font-size:.7rem;text-transform:uppercase;font-weight:700}
.b-idle{background:#1e3a8a;color:#93c5fd}.b-run{background:#166534;color:#4ade80}
.b-fault{background:#7f1d1d;color:#fca5a5}.b-warn{background:#78350f;color:#fcd34d}
.btn{border:none;border-radius:8px;padding:10px 14px;font-weight:600;font-size:.9rem;cursor:pointer;transition:transform .08s}
.btn:active{transform:scale(.97)}
.primary{background:#2563eb;color:#fff}.warn{background:#d97706;color:#fff}
.danger{background:#dc2626;color:#fff}.ghost{background:#334155;color:#cbd5e1}
.ok{background:#059669;color:#fff}
.grid-joints{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px}
.jcard{background:#0f172a;border:1px solid #334155;border-radius:10px;padding:12px}
.jname{font-weight:700;color:#cbd5e1;display:flex;justify-content:space-between;align-items:center;margin-bottom:6px}
.jdeg{font-size:1.5rem;color:#38bdf8;font-variant-numeric:tabular-nums}
.jenc{font-size:.72rem;color:#94a3b8}
.jflags{font-size:.68rem;margin-top:4px}
.f-h{color:#4ade80}.f-r{color:#93c5fd}.f-d{color:#fca5a5}.f-e{color:#fcd34d}
.jbtns{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:8px}
.stepsel{display:flex;gap:4px;margin-top:8px;width:200px}
.stepbtn{flex:1;background:#334155;color:#cbd5e1;border:none;padding:4px;border-radius:6px;font-size:.75rem;cursor:pointer;font-weight:600}
.stepbtn.active{background:#38bdf8;color:#0f172a}
.stat-line{display:flex;justify-content:space-between;font-size:.85rem;color:#94a3b8;margin:4px 0}
input[type=text],input[type=password]{width:100%;background:#0f172a;border:1px solid #334155;color:#f8fafc;font-size:1rem;padding:9px 12px;border-radius:8px;outline:none;margin-bottom:8px}
label{display:block;font-size:.78rem;font-weight:600;color:#94a3b8;margin-bottom:4px;text-transform:uppercase;letter-spacing:.04em}
#estop{position:fixed;bottom:16px;right:16px;z-index:99;padding:18px 30px;font-size:1.15rem;box-shadow:0 6px 24px rgba(220,38,38,.5);animation:estopPulse 2.2s ease-in-out infinite}
@keyframes estopPulse{0%,100%{box-shadow:0 6px 24px rgba(220,38,38,.5)}50%{box-shadow:0 6px 36px rgba(220,38,38,.9)}}
@media(prefers-reduced-motion:reduce){#estop{animation:none}}
.muted{color:#64748b;font-size:.78rem}
.btn[disabled]{opacity:.4;cursor:not-allowed;transform:none}
.tab-btn:focus-visible{outline:2px solid #38bdf8}
.home-track{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}
.hchip{background:#334155;color:#94a3b8;padding:3px 12px;border-radius:11px;font-size:.72rem;font-weight:700}
.hchip.on{background:#d97706;color:#fff}
.hchip.done{background:#166534;color:#4ade80}
#toast{position:fixed;bottom:16px;left:16px;z-index:98;background:#0f172a;border:1px solid #334155;border-left-width:4px;border-radius:8px;padding:10px 14px;font-size:.84rem;color:#cbd5e1;display:none;max-width:70vw;box-shadow:0 6px 20px rgba(0,0,0,.4)}
.t-ok{border-color:#059669}.t-warn{border-color:#d97706}.t-err{border-color:#dc2626}
.offline{color:#fca5a5!important}
</style>
</head>
<body>
<div class="header"><h1>6-Axis Robotic Arm</h1><p id="sub">connecting...</p></div>

<div class="tabs">
  <button class="tab-btn active" data-t="dash">Dashboard</button>
  <button class="tab-btn" data-t="joints">Joints</button>
  <button class="tab-btn" data-t="home">Homing</button>
  <button class="tab-btn" data-t="wifi">WiFi</button>
  <button class="tab-btn" data-t="cart">Cartesian</button>
  <button class="tab-btn" data-t="draw">Draw</button>
</div>

<div id="dash" class="tab-pane active">
  <div class="card"><h2>System Status</h2>
    <div class="stat-line"><span>Mode</span><span id="mode" class="badge b-idle">idle</span></div>
    <div class="stat-line"><span>Homed joints</span><span id="homedN">-/6</span></div>
    <div class="stat-line"><span>WiFi</span><span id="wifiInfo">-</span></div>
    <div class="stat-line"><span>Endstops</span><span id="esInfo" class="muted">-</span></div>
    <div class="stat-line"><span>Homing</span><span id="homProg">idle</span></div>
    <div class="home-track"><span class="hchip" id="hc0">J1</span><span class="hchip" id="hc1">J2</span><span class="hchip" id="hc2">J3</span><span class="hchip" id="hc3">J4</span></div>
  </div>
  <div class="card"><h2>Quick Actions</h2>
    <div class="row">
      <button class="btn primary need-idle" onclick="api('/api/home/all')">HOME ALL (J1-J4)</button>
      <button class="btn warn" onclick="api('/api/stop')">STOP ALL</button>
      <button class="btn ghost" onclick="clearFault()">CLEAR FAULT</button>
    </div>
    <p class="muted" style="margin-top:8px">HOME ALL chạy tuần tự J1→J2→J3→J4. J5/J6 dùng Set-Home thủ công ở tab Homing.</p>
  </div>
</div>

<div id="joints" class="tab-pane">
  <div class="card"><h2>Manual Joint Control</h2>
    <div class="row" style="margin-bottom:10px"><span style="font-size:.8rem;color:#94a3b8">Bước jog:</span>
      <div class="stepsel" id="stepSel"></div>
    </div>
    <div class="grid-joints" id="jointGrid"></div>
  </div>
</div>

<div id="home" class="tab-pane">
  <div class="card"><h2>Automatic Homing (TMC J1-J4)</h2>
    <div class="row">
      <button class="btn primary need-idle" onclick="api('/api/home/all')">HOME ALL</button>
      <span class="muted">từng khớp:</span>
      <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=0')">Home J1</button>
      <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=1')">Home J2</button>
      <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=2')">Home J3</button>
      <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=3')">Home J4</button>
    </div>
    <p class="muted" style="margin-top:8px">J1/J2: min-stop → về GIỮA hành trình. J3/J4: min-stop/stall + lùi 2°.</p>
  </div>
  <div class="card"><h2>Manual Set-Home & Calibration</h2>
    <p class="muted" style="margin-bottom:10px">Đưa khớp về vị trí mong muốn rồi nhấn Set Home. Dữ liệu lưu NVS — giữ nguyên sau khi tắt nguồn.</p>
    <div class="grid-joints" id="setHomeGrid"></div>
  </div>
</div>

<div id="wifi" class="tab-pane">
  <div class="card"><h2>Kết nối hiện tại</h2>
    <div class="stat-line"><span>Chế độ</span><span id="wfMode" class="badge b-info">-</span></div>
    <div class="stat-line"><span>IP</span><span id="wfIp">-</span></div>
    <div class="stat-line"><span>SSID</span><span id="wfSsidNow">-</span></div>
    <div class="stat-line"><span>RSSI</span><span id="wfRssi">-</span></div>
  </div>
  <div class="card"><h2>WiFi Provisioning</h2>
    <label>SSID</label><input type="text" id="wfSsid" placeholder="ten wifi nha">
    <label>Password</label><input type="password" id="wfPass" placeholder="mat khau">
    <div class="row" style="margin-top:8px">
      <button class="btn ok" onclick="saveWifi()">SAVE &amp; REBOOT</button>
      <span class="muted">Lưu vào NVS — không cần flash lại khi đổi mạng.</span>
    </div>
  </div>
</div>

<div id="cart" class="tab-pane">
  <div class="card"><h2>Move TCP (bút hướng xuống)</h2>
    <p class="muted" style="margin-bottom:8px">Yêu cầu đã HOME J1-J4. Vị trí hiện tại:
      <span id="poseNow" style="color:#38bdf8;font-weight:700">-</span></p>
    <div class="row">
      <input type="text" id="mvX" inputmode="decimal" placeholder="X (mm)" style="width:90px">
      <input type="text" id="mvY" inputmode="decimal" placeholder="Y (mm)" style="width:90px">
      <input type="text" id="mvZ" inputmode="decimal" placeholder="Z (mm)" style="width:90px">
      <input type="text" id="mvFeed" inputmode="decimal" placeholder="feed mm/s" value="30" style="width:90px">
      <button class="btn primary need-idle" onclick="moveTo()">MOVE</button>
    </div>
    <p class="muted" style="margin-top:6px">Home TCP = (146, 0, 365). Giấy vẽ đặt dưới bút, Z nhỏ hơn khi hạ.</p>
  </div>
</div>

<div id="draw" class="tab-pane">
  <div class="card"><h2>Draw Shape (pen on paper)</h2>
    <canvas id="cv" width="420" height="300"
      style="background:#0f172a;border:1px solid #334155;border-radius:8px;display:block;margin-bottom:10px"></canvas>
    <label>Hình</label>
    <select id="dwShape" style="background:#0f172a;color:#f8fafc;border:1px solid #334155;padding:8px;border-radius:8px;margin-bottom:8px">
      <option value="line">Line</option>
      <option value="circle">Circle</option>
    </select>
    <div class="row">
      <input type="text" id="dwA1" inputmode="decimal" placeholder="x1 / cx" style="width:90px">
      <input type="text" id="dwA2" inputmode="decimal" placeholder="y1 / cy" style="width:90px">
      <input type="text" id="dwA3" inputmode="decimal" placeholder="x2 / r"   style="width:90px">
      <input type="text" id="dwA4" inputmode="decimal" placeholder="y2"       style="width:90px">
      <input type="text" id="dwZ"  inputmode="decimal" placeholder="z giấy"   style="width:90px">
      <input type="text" id="dwFeed" inputmode="decimal" placeholder="feed"   style="width:80px" value="20">
    </div>
    <div class="row" style="margin-top:10px">
      <button class="btn ok need-idle" onclick="startDraw()">START DRAW</button>
      <button class="btn danger" onclick="api('/api/stop')">ABORT</button>
      <button class="btn ghost" onclick="previewShape()">PREVIEW</button>
      <span class="muted">Preview: hình chiếu từ trên xuống (đơn vị mm, gốc = trục J1).</span>
    </div>
  </div>
</div>

<button id="estop" class="btn danger" onclick="api('/api/stop')">&#9888; E-STOP</button>
<div id="toast"></div>

<script>
let stepSize=1.0, pollTimer=null, failN=0, toastTimer=null;
const AXES=["J1 Base","J2 Shoulder","J3 Elbow","J4 WristPan","J5 Tilt","J6 Roll"];

function toast(msg,cls){
  const t=document.getElementById('toast');
  t.textContent=msg;
  t.className=cls?('t-'+cls):'';
  t.style.display='block';
  clearTimeout(toastTimer);
  toastTimer=setTimeout(()=>{t.style.display='none';},2600);
}
function api(url){
  return fetch(url).then(r=>r.text()).then(t=>{
    toast((t==='OK'?'\u2713 ':'')+t,t==='OK'?'ok':'warn');
    return t;
  }).catch(()=>toast('Lỗi mạng / mất kết nối','err'));
}
function post(url,body){
  return fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body})
    .then(r=>r.text())
    .then(t=>toast((t==='OK'?'\u2713 ':'')+t,t==='OK'?'ok':'warn'))
    .catch(()=>toast('Lỗi mạng / mất kết nối','err'));
}
function clearFault(){ post('/api/jog','fault_clear=1'); }

document.querySelectorAll('.tab-btn[data-t]').forEach(b=>{
  b.onclick=()=>{
    document.querySelectorAll('.tab-btn').forEach(x=>x.classList.remove('active'));
    document.querySelectorAll('.tab-pane').forEach(x=>x.classList.remove('active'));
    b.classList.add('active');
    document.getElementById(b.dataset.t).classList.add('active');
  };
});

[0.5,1,5,15].forEach(s=>{
  const b=document.createElement('button');
  b.className='stepbtn'+(s===stepSize?' active':''); b.textContent=s+'\u00B0';
  b.onclick=()=>{stepSize=s;document.querySelectorAll('.stepbtn').forEach(x=>x.classList.remove('active'));b.classList.add('active');};
  document.getElementById('stepSel').appendChild(b);
});

function buildCards(){
  const g=document.getElementById('jointGrid'); g.innerHTML='';
  const sg=document.getElementById('setHomeGrid'); sg.innerHTML='';
  for(let i=0;i<6;i++){
    g.insertAdjacentHTML('beforeend',
     `<div class="jcard">
        <div class="jname"><span>${AXES[i]}</span><span class="jdeg" id="jd${i}">--</span></div>
        <div class="jenc" id="je${i}">encoder: --</div>
        <div class="jflags" id="jf${i}"></div>
        <div class="jbtns">
          <button class="btn danger" onclick="jog(${i},-${stepSize})">&#8630;</button>
          <button class="btn primary" onclick="jog(${i},${stepSize})">&#8634;</button>
        </div>
      </div>`);
    sg.insertAdjacentHTML('beforeend',
     `<div class="jcard">
        <div class="jname"><span>${AXES[i]}</span></div>
        <div class="jbtns">
          <button class="btn ok need-idle" onclick="api('/api/sethome?axis=${i}')">Set Home</button>
          <button class="btn warn" onclick="if(confirm('Xóa calib J${i+1}? Sẽ mất vị trí đã lưu trong NVS.'))api('/api/clearcalib?axis=${i}')">Clear Calib</button>
        </div>
      </div>`);
  }
}
buildCards();

function jog(axis,dir){
  post('/api/jog',`axis=${axis}&deg=${dir*stepSize}`);
}

function saveWifi(){
  const s=document.getElementById('wfSsid').value.trim(), p=document.getElementById('wfPass').value;
  if(!s){toast('Nhập SSID','warn');return;}
  post('/api/wifi',`ssid=${encodeURIComponent(s)}&pass=${encodeURIComponent(p)}`)
    .then(()=>{document.getElementById('sub').innerText='Đã lưu. Đang restart...';});
}

function moveTo(){
  const b=`x=${document.getElementById('mvX').value||0}&y=${document.getElementById('mvY').value||0}&z=${document.getElementById('mvZ').value||0}&feed=${document.getElementById('mvFeed').value||30}`;
  post('/api/move',b);
}

function drawParams(){
  const sh=document.getElementById('dwShape').value;
  const v=id=>parseFloat(document.getElementById(id).value)||0;
  let b;
  if(sh==='line'){
    b=`shape=line&x1=${v('dwA1')}&y1=${v('dwA2')}&x2=${v('dwA3')}&y2=${v('dwA4')}`;
  }else{
    b=`shape=circle&cx=${v('dwA1')}&cy=${v('dwA2')}&r=${v('dwA3')}`;
  }
  b+=`&z=${v('dwZ')}&feed=${document.getElementById('dwFeed').value||20}`;
  return b;
}
function startDraw(){
  post('/api/draw',drawParams());
  previewShape();
}
function previewShape(){
  const sh=document.getElementById('dwShape').value;
  const v=id=>parseFloat(document.getElementById(id).value)||0;
  const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
  ctx.clearRect(0,0,cv.width,cv.height);
  const sc=0.9,cxp=cv.width/2,cyp=cv.height*0.92;
  // grid 50mm
  ctx.strokeStyle='#1e293b';
  for(let gx=-300;gx<=300;gx+=50){
    ctx.beginPath();ctx.moveTo(cxp+gx*sc,0);ctx.lineTo(cxp+gx*sc,cv.height);ctx.stroke();
    ctx.beginPath();ctx.moveTo(0,cyp-gx*sc);ctx.lineTo(cv.width,cyp-gx*sc);ctx.stroke();
  }
  ctx.strokeStyle='#38bdf8';ctx.lineWidth=2;ctx.beginPath();
  if(sh==='line'){
    ctx.moveTo(cxp+v('dwA1')*sc,cyp-v('dwA2')*sc);
    ctx.lineTo(cxp+v('dwA3')*sc,cyp-v('dwA4')*sc);
  }else{
    ctx.arc(cxp+v('dwA1')*sc,cyp-v('dwA2')*sc,v('dwA3')*sc,0,Math.PI*2);
  }
  ctx.stroke();
  // TCP hiện tại
  if(lastPose){ctx.fillStyle='#f8fafc';ctx.beginPath();
    ctx.arc(cxp+lastPose.x*sc,cyp-lastPose.y*sc,4,0,Math.PI*2);ctx.fill();}
  // marker home (146,0)
  ctx.fillStyle='#4ade80';ctx.beginPath();
  ctx.arc(cxp+146*sc,cyp,3,0,Math.PI*2);ctx.fill();
  ctx.fillStyle='#64748b';ctx.font='10px sans-serif';
  ctx.fillText('home',cxp+146*sc+6,cyp+3);
}
let lastPose=null;

function updateUI(d){
  const modeEl=document.getElementById('mode');
  modeEl.innerText=d.mode;
  modeEl.className='badge '+(d.mode==='fault'?'b-fault':(d.mode==='idle'?'b-idle':'b-run'));
  document.getElementById('sub').innerText=`${d.wifi.ssid||''} | ${d.wifi.ip}`;
  let hn=0;d.joints.forEach(j=>hn+=j.homed?1:0);
  document.getElementById('homedN').innerText=`${hn}/6`;
  document.getElementById('wifiInfo').innerText=`${d.wifi.mode.toUpperCase()} ${d.wifi.rssi?('RSSI '+d.wifi.rssi):''}`;
  document.getElementById('wfMode').innerText=d.wifi.mode.toUpperCase();
  document.getElementById('wfIp').innerText=d.wifi.ip;
  document.getElementById('wfSsidNow').innerText=d.wifi.ssid||'(AP)';
  document.getElementById('wfRssi').innerText=d.wifi.rssi?('RSSI '+d.wifi.rssi+' dBm'):'-';
  // khóa nút hành động khi arm đang bận hoặc FAULT
  document.querySelectorAll('.need-idle').forEach(b=>{b.disabled=(d.busy||d.mode==='fault');});
  const esParts=[];
  d.endstops.forEach((e,i)=>{
    if(e.min&&e.min.pressed)esParts.push(`J${i+1}MIN`);
    if(e.max&&e.max.pressed)esParts.push(`J${i+1}MAX`);
  });
  const esEl=document.getElementById('esInfo');
  esEl.innerText=esParts.length?esParts.join(', '):'clear';
  esEl.className=esParts.length?'badge b-fault':'muted';
  const h=d.homing;
  const hcs=[document.getElementById('hc0'),document.getElementById('hc1'),
             document.getElementById('hc2'),document.getElementById('hc3')];
  if(h&&h.active){
    hcs.forEach((c,i)=>{c.className='hchip'+(i<h.axis-1?' done':(i===h.axis-1?' on':''));});
    document.getElementById('homProg').innerText=`J${h.axis}: ${h.phase}`;
  }else{
    hcs.forEach(c=>c.className='hchip');
    document.getElementById('homProg').innerText=h?(h.lastOk?'hoàn tất':'lỗi lần trước'):'';
  }
  if(d.pose){
    lastPose=d.pose;
    document.getElementById('poseNow').innerText=`(${d.pose.x.toFixed(1)}, ${d.pose.y.toFixed(1)}, ${d.pose.z.toFixed(1)})`;
  }

  d.joints.forEach((j,i)=>{
    document.getElementById(`jd${i}`).innerText=j.deg.toFixed(1)+'\u00B0';
    document.getElementById(`je${i}`).innerText='encoder: '+(j.encOK?j.encDeg.toFixed(1)+'\u00B0':'MAT KET NOI');
    const f=[];
    f.push(`<span class="${j.homed?'f-h':'f-e'}">${j.homed?(j.restored?'HOMED(nvs)':'HOMED'):'CHUA HOME'}</span>`);
    if(j.drift)f.push('<span class="f-d">DRIFT!</span>');
    if(!j.encOK)f.push('<span class="f-e">ENC ERR</span>');
    document.getElementById(`jf${i}`).innerHTML=f.join(' \u00B7 ');
  });
}

function setOnline(on){
  const s=document.getElementById('sub');
  s.classList.toggle('offline',!on);
  if(!on)s.innerText='\u26A0 MẤT KẾT NỐI — đang thử lại...';
}
function pollOnce(){
  fetch('/api/status').then(r=>r.json())
    .then(d=>{failN=0;setOnline(true);updateUI(d);})
    .catch(()=>{if(++failN>=3)setOnline(false);});
}
setInterval(pollOnce,300);
pollOnce();
</script>
</body>
</html>
)rawliteral";

namespace {
WebServer* srv = nullptr;
ArmController* armPtr = nullptr;
WifiManager* wifiPtr = nullptr;
JointModel* jointsPtr = nullptr;
NvsStore* nvsPtr = nullptr;

void sendJson(int code, const String& body) { srv->send(code, "application/json", body); }

void handleRoot() { srv->send_P(200, "text/html", INDEX_HTML); }

void handleStatus() {
    if (armPtr == nullptr || wifiPtr == nullptr) {
        sendJson(500, "{\"error\":\"not ready\"}");
        return;
    }
    sendJson(200, armPtr->statusJson());
}

void handleJog() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }

    if (srv->hasArg("fault_clear")) {
        ArmCommand c;
        c.type = ArmCommand::CLEAR_FAULT;
        srv->send(200, "text/plain", armPtr->submit(c) ? "OK" : "busy");
        return;
    }

    const int axis = srv->arg("axis").toInt();
    const float deg = srv->arg("deg").toFloat();
    if (axis < 0 || axis >= NUM_MOTORS || deg == 0.0f ||
        fabsf(deg) > 45.0f) { // giới hạn an toàn mỗi lệnh jog
        srv->send(400, "text/plain", "bad args");
        return;
    }
    ArmCommand c;
    c.type = ArmCommand::JOG_REL;
    c.axis = static_cast<uint8_t>(axis);
    c.value = deg;
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleStop() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    ArmCommand c;
    c.type = ArmCommand::STOP_ALL;
    // STOP luôn được chấp nhận: đợi tối đa 50ms cho slot queue
    const bool ok = armPtr->submit(c, 50);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "queue full");
}

void handleMove() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    const float x = srv->arg("x").toFloat();
    const float y = srv->arg("y").toFloat();
    const float z = srv->arg("z").toFloat();
    if (z < -20.0f || z > 450.0f) { srv->send(400, "text/plain", "z out of range"); return; }
    ArmCommand c;
    c.type = ArmCommand::MOVE_CART;
    c.p[0] = x; c.p[1] = y; c.p[2] = z;
    c.p[5] = srv->hasArg("feed") ? srv->arg("feed").toFloat() : 30.0f;
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    srv->send(armPtr->submit(c, 20) ? 200 : 503, "text/plain", "OK");
}

void handleDraw() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    const String shape = srv->arg("shape");
    ArmCommand c;
    if (shape == "line") {
        c.type = ArmCommand::DRAW_LINE;
        c.p[0] = srv->arg("x1").toFloat();
        c.p[1] = srv->arg("y1").toFloat();
        c.p[2] = srv->arg("x2").toFloat();
        c.p[3] = srv->arg("y2").toFloat();
        c.p[4] = srv->arg("z").toFloat();
    } else if (shape == "circle") {
        c.type = ArmCommand::DRAW_CIRCLE;
        c.p[0] = srv->arg("cx").toFloat();
        c.p[1] = srv->arg("cy").toFloat();
        c.p[2] = srv->arg("z").toFloat();
        c.p[3] = srv->arg("r").toFloat();
        if (c.p[3] < 5.0f || c.p[3] > 250.0f) {
            srv->send(400, "text/plain", "r out of range");
            return;
        }
    } else {
        srv->send(400, "text/plain", "shape: line|circle");
        return;
    }
    c.p[5] = srv->hasArg("feed") ? srv->arg("feed").toFloat() : DRAW_FEED_MM_S;
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    srv->send(armPtr->submit(c, 20) ? 200 : 503, "text/plain", "OK");
}

void handleHomeAll() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::HOME_ALL;
    srv->send(armPtr->submit(c, 20) ? 200 : 503, "text/plain", "OK");
}

void handleHomeAxis() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    const int axis = srv->arg("axis").toInt();
    if (axis < 0 || axis >= 4) { srv->send(400, "text/plain", "axis 0..3 only"); return; }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::HOME_AXIS;
    c.axis = static_cast<uint8_t>(axis);
    srv->send(armPtr->submit(c, 20) ? 200 : 503, "text/plain", "OK");
}

void handleSetHome() {
    if (jointsPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    const int axis = srv->arg("axis").toInt();
    if (axis < 0 || axis >= NUM_MOTORS) { srv->send(400, "text/plain", "bad axis"); return; }
    if (armPtr != nullptr && armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::SET_HOME;
    c.axis = static_cast<uint8_t>(axis);
    srv->send(armPtr->submit(c, 20) ? 200 : 503, "text/plain", "OK");
}

void handleClearCalib() {
    if (jointsPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    const int axis = srv->arg("axis").toInt();
    if (axis < 0 || axis >= NUM_MOTORS) { srv->send(400, "text/plain", "bad axis"); return; }
    jointsPtr->forgetHome(static_cast<uint8_t>(axis));
    srv->send(200, "text/plain", "OK");
}

void handleWifiSave() {
    if (wifiPtr == nullptr || nvsPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    String ssid = srv->arg("ssid");
    String pass = srv->arg("pass");
    ssid.trim();
    if (ssid.isEmpty() || ssid.length() > 32 || pass.length() > 64) {
        srv->send(400, "text/plain", "bad ssid/pass");
        return;
    }
    if (!wifiPtr->provision(ssid, pass)) {
        srv->send(500, "text/plain", "nvs save failed");
        return;
    }
    srv->send(200, "application/json", "{\"saved\":true,\"reboot\":true}");
    Serial.println("[WEB] WiFi creds moi da luu — restart sau 1s...");
    delay(1000);
    ESP.restart();
}

} // namespace

void webBegin(WebServer& server, ArmController* arm, WifiManager* wifi,
              JointModel* joints, NvsStore* nvs) {
    srv = &server;
    armPtr = arm;
    wifiPtr = wifi;
    jointsPtr = joints;
    nvsPtr = nvs;

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleStatus);
    server.on("/api/jog", HTTP_POST, handleJog);
    server.on("/api/stop", HTTP_GET, handleStop);
    server.on("/api/move", HTTP_POST, handleMove);
    server.on("/api/draw", HTTP_POST, handleDraw);
    server.on("/api/home/all", HTTP_GET, handleHomeAll);
    server.on("/api/home/axis", HTTP_GET, handleHomeAxis);
    server.on("/api/sethome", HTTP_GET, handleSetHome);
    server.on("/api/clearcalib", HTTP_GET, handleClearCalib);
    server.on("/api/wifi", HTTP_POST, handleWifiSave);

    server.begin();
    Serial.printf("[WEB] HTTP server on port %u\n", WEB_SERVER_PORT);
}
