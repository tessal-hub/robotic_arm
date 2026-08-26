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
:root{
  /* Colors - Primary */
  --color-primary: #38bdf8;
  --color-primary-deep: #0ea5e9;
  --color-primary-muted: #7dd3fc;
  /* Colors - Semantic Status */
  --color-success: #059669;
  --color-success-bg: #0c1a17;
  --color-success-text: #6ee7b7;
  --color-warning: #d97706;
  --color-warning-bg: #1c1917;
  --color-warning-text: #fcd34d;
  --color-danger: #dc2626;
  --color-danger-bg: #7f1d1d;
  --color-danger-text: #fca5a5;
  --color-info: #2563eb;
  --color-info-bg: #1e3a8a;
  --color-info-text: #93c5fd;
  /* Colors - Neutral */
  --color-bg: #0f172a;
  --color-surface: #1e293b;
  --color-surface-elevated: #0f172a;
  --color-border: #334155;
  --color-border-strong: #475569;
  /* Colors - Text */
  --color-text-primary: #f8fafc;
  --color-text-secondary: #cbd5e1;
  --color-text-muted: #94a3b8;
  --color-text-dim: #64748b;
  /* Colors - Special */
  --color-focus-ring: #38bdf8;
  --color-estop-glow: rgba(220,38,38,0.5);
  --color-estop-glow-strong: rgba(220,38,38,0.9);
  --color-shadow-card: rgba(0,0,0,0.35);
  --color-shadow-toast: rgba(0,0,0,0.4);
  /* Spacing */
  --space-xs: 4px;
  --space-sm: 6px;
  --space-md: 8px;
  --space-lg: 10px;
  --space-xl: 12px;
  --space-2xl: 14px;
  --space-3xl: 16px;
  --space-4xl: 18px;
  --space-gutter: 12px;
  --space-card-padding: 18px;
  --space-card-gap: 14px;
  --space-tab-gap: 6px;
  --space-page-padding: 16px;
  --space-max-width: 900px;
  /* Border Radius */
  --radius-xs: 6px;
  --radius-sm: 8px;
  --radius-md: 10px;
  --radius-lg: 14px;
  --radius-pill: 11px;
  --radius-badge: 12px;
  /* Typography */
  --font-sans: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
  --font-mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  --fs-display: clamp(1.25rem, 4vw, 1.5rem);
  --fs-headline: 1.05rem;
  --fs-title: 0.92rem;
  --fs-body: 0.85rem;
  --fs-label: 0.78rem;
  --fs-mono: 0.78rem;
  --fs-tabular: 1.5rem;
  --fw-regular: 400;
  --fw-medium: 500;
  --fw-semibold: 600;
  --fw-bold: 700;
  --lh-tight: 1.2;
  --lh-normal: 1.3;
  --lh-relaxed: 1.4;
  --lh-loose: 1.55;
  --ls-label: 0.04em;
  /* Transitions */
  --transition-fast: 80ms;
  --transition-normal: 150ms;
  --ease-out: cubic-bezier(0.16, 1, 0.3, 1);
  --ease-spring: cubic-bezier(0.23, 1, 0.32, 1);
  /* Shadows */
  --shadow-card: 0 8px 20px var(--color-shadow-card);
  --shadow-toast: 0 6px 20px var(--color-shadow-toast);
  --shadow-estop: 0 6px 24px var(--color-estop-glow);
  --shadow-estop-strong: 0 6px 36px var(--color-estop-glow-strong);
  /* Z-index */
  --z-toast: 98;
  --z-estop: 99;
}

*{box-sizing:border-box;margin:0;padding:0}
body{
  font-family:var(--font-sans);
  background:var(--color-bg);
  color:var(--color-text-primary);
  min-height:100vh;
  padding:var(--space-page-padding);
  display:flex;
  flex-direction:column;
  align-items:center;
}
.header{text-align:center;margin-bottom:14px}
.header h1{
  font-size:var(--fs-display);
  font-weight:var(--fw-bold);
  color:var(--color-primary);
  line-height:var(--lh-tight);
}
.header p{font-size:0.8rem;color:var(--color-text-muted);line-height:var(--lh-normal)}
.tabs{
  display:flex;gap:var(--space-tab-gap);
  background:var(--color-surface);
  padding:var(--space-sm);
  border-radius:var(--radius-md);
  margin-bottom:16px;
  flex-wrap:wrap;
  justify-content:center;
  width:100%;
  max-width:var(--space-max-width);
}
.tab-btn{
  background:transparent;
  border:none;
  color:var(--color-text-muted);
  padding:var(--space-md) var(--space-lg);
  border-radius:var(--radius-sm);
  font-size:0.85rem;
  font-weight:var(--fw-semibold);
  cursor:pointer;
  transition:background var(--transition-normal) var(--ease-out),color var(--transition-normal) var(--ease-out);
}
.tab-btn:hover{
  background:var(--color-border);
  color:var(--color-text-secondary);
}
.tab-btn.active{
  background:var(--color-primary);
  color:var(--color-bg);
}
.tab-btn.disabled{
  opacity:.35;
  cursor:not-allowed;
}
.tab-btn:focus-visible{
  outline:2px solid var(--color-focus-ring);
  outline-offset:2px;
}
.tab-pane{display:none;width:100%;max-width:var(--space-max-width)}
.tab-pane.active{display:block}
.card{
  background:var(--color-surface);
  border-radius:var(--radius-lg);
  padding:var(--space-card-padding);
  box-shadow:var(--shadow-card);
  margin-bottom:var(--space-card-gap);
}
.card h2{
  font-size:var(--fs-headline);
  font-weight:var(--fw-semibold);
  color:var(--color-primary);
  margin-bottom:12px;
  border-bottom:1px solid var(--color-border);
  padding-bottom:var(--space-md);
}
.row{
  display:flex;
  gap:var(--space-md);
  flex-wrap:wrap;
  align-items:center;
}
.badge{
  padding:3px 10px;
  border-radius:var(--radius-badge);
  font-size:var(--fs-label);
  font-weight:var(--fw-bold);
  text-transform:uppercase;
  letter-spacing:var(--ls-label);
}
.b-idle{background:var(--color-info-bg);color:var(--color-info-text)}
.b-run{background:var(--color-success-bg);color:var(--color-success-text)}
.b-fault{background:var(--color-danger-bg);color:var(--color-danger-text)}
.b-warn{background:var(--color-warning-bg);color:var(--color-warning-text)}
.btn{
  border:none;
  border-radius:var(--radius-sm);
  padding:var(--space-md) var(--space-lg);
  font-weight:var(--fw-semibold);
  font-size:var(--fs-body);
  cursor:pointer;
  transition:transform var(--transition-fast) var(--ease-spring),background var(--transition-normal) var(--ease-out);
}
.btn:hover:not(:disabled){background:var(--color-primary-deep)}
.btn:active:not(:disabled){transform:scale(0.97)}
.btn:focus-visible{outline:2px solid var(--color-focus-ring);outline-offset:2px}
.btn[disabled]{opacity:.4;cursor:not-allowed;transform:none}
.primary{background:var(--color-info);color:var(--color-text-primary)}
.warn{background:var(--color-warning);color:var(--color-text-primary)}
.danger{background:var(--color-danger);color:var(--color-text-primary)}
.ghost{background:var(--color-border);color:var(--color-text-secondary)}
.ok{background:var(--color-success);color:var(--color-text-primary)}
.grid-joints{
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(260px,1fr));
  gap:var(--space-gutter);
}
.jcard{
  background:var(--color-bg);
  border:1px solid var(--color-border);
  border-radius:var(--radius-md);
  padding:12px;
}
.jname{
  font-weight:var(--fw-bold);
  color:var(--color-text-secondary);
  display:flex;
  justify-content:space-between;
  align-items:center;
  margin-bottom:6px;
}
.jdeg{
  font-size:var(--fs-tabular);
  color:var(--color-primary);
  font-variant-numeric:tabular-nums;
  font-family:var(--font-mono);
}
.jenc{
  font-size:0.72rem;
  color:var(--color-text-muted);
  font-family:var(--font-sans);
}
.jflags{font-size:0.68rem;margin-top:4px;font-family:var(--font-sans)}
.f-h{color:#4ade80}.f-r{color:#93c5fd}.f-d{color:#fca5a5}.f-e{color:#fcd34d}
.jbtns{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:var(--space-xs);
  margin-top:var(--space-md);
}
.stepsel{
  display:flex;
  gap:var(--space-xs);
  margin-top:var(--space-md);
  width:200px;
}
.stepbtn{
  flex:1;
  background:var(--color-border);
  color:var(--color-text-secondary);
  border:none;
  padding:4px var(--space-xs);
  border-radius:var(--radius-xs);
  font-size:0.75rem;
  font-weight:var(--fw-semibold);
  cursor:pointer;
  transition:background var(--transition-normal) var(--ease-out),color var(--transition-normal) var(--ease-out);
}
.stepbtn:hover{background:var(--color-border-strong)}
.stepbtn.active{background:var(--color-primary);color:var(--color-bg)}
.stepbtn:focus-visible{outline:2px solid var(--color-focus-ring);outline-offset:2px}
.stat-line{
  display:flex;
  justify-content:space-between;
  font-size:var(--fs-body);
  color:var(--color-text-muted);
  margin:4px 0;
}
label{
  display:block;
  font-size:var(--fs-label);
  font-weight:var(--fw-semibold);
  color:var(--color-text-muted);
  margin-bottom:4px;
  text-transform:uppercase;
  letter-spacing:var(--ls-label);
}
input[type=text],input[type=password],select{
  width:100%;
  background:var(--color-bg);
  border:1px solid var(--color-border);
  color:var(--color-text-primary);
  font-size:1rem;
  padding:9px 12px;
  border-radius:var(--radius-sm);
  outline:none;
  margin-bottom:8px;
  font-family:var(--font-sans);
  transition:border-color var(--transition-normal) var(--ease-out);
}
input[type=text]:focus,input[type=password]:focus,select:focus{
  border-color:var(--color-focus-ring);
}
input::placeholder{color:var(--color-text-dim)}
select{appearance:none;background-image:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='16' height='16' viewBox='0 0 24 24' fill='none' stroke='%2394a3b8' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'%3E%3Cpolyline points='6 9 12 15 18 9'%3E%3C/polyline%3E%3C/svg%3E");background-repeat:no-repeat;background-position:right 12px center;padding-right:36px}
#estop{
  position:fixed;
  bottom:16px;
  right:16px;
  z-index:var(--z-estop);
  padding:18px 30px;
  font-size:1.15rem;
  font-weight:var(--fw-semibold);
  box-shadow:var(--shadow-estop);
  animation:estopPulse 2.2s ease-in-out infinite;
}
@keyframes estopPulse{
  0%,100%{box-shadow:var(--shadow-estop)}
  50%{box-shadow:var(--shadow-estop-strong)}
}
@media(prefers-reduced-motion:reduce){
  #estop{animation:none}
  *{transition-duration:0.01ms !important;animation-duration:0.01ms !important}
}
.muted{color:var(--color-text-dim);font-size:var(--fs-label)}
.need-idle:disabled{opacity:.4;cursor:not-allowed;transform:none}
.home-track{display:flex;gap:var(--space-xs);margin-top:var(--space-md);flex-wrap:wrap}
.hchip{
  background:var(--color-border);
  color:var(--color-text-muted);
  padding:3px 12px;
  border-radius:var(--radius-pill);
  font-size:0.72rem;
  font-weight:var(--fw-bold);
}
.hchip.on{background:var(--color-warning);color:var(--color-text-primary)}
.hchip.done{background:var(--color-success-bg);color:var(--color-success-text)}
#toast{
  position:fixed;
  bottom:16px;
  left:16px;
  z-index:var(--z-toast);
  background:var(--color-bg);
  border:1px solid var(--color-border);
  border-left-width:4px;
  border-radius:var(--radius-sm);
  padding:10px 14px;
  font-size:0.84rem;
  color:var(--color-text-secondary);
  display:none;
  max-width:70vw;
  box-shadow:var(--shadow-toast);
  animation:toastIn 0.15s var(--ease-out);
}
@keyframes toastIn{
  from{opacity:0;transform:translateY(8px)}
  to{opacity:1;transform:translateY(0)}
}
.t-ok{border-left-color:var(--color-success)}
.t-warn{border-left-color:var(--color-warning)}
.t-err{border-left-color:var(--color-danger)}
.offline{color:var(--color-danger-text)!important}
.step-label{display:flex;align-items:center;gap:var(--space-md);font-size:0.8rem;color:var(--color-text-muted)}
.btn-loading{position:relative;color:transparent}
.btn-loading::after{content:"";position:absolute;width:16px;height:16px;top:50%;left:50%;margin:-8px 0 0 -8px;border:2px solid currentColor;border-right-color:transparent;border-radius:50%;animation:spin 0.6s linear infinite}
@keyframes spin{to{transform:rotate(360deg)}}
</style>
</head>
<body>
<div class="header"><h1>6-Axis Robotic Arm</h1><p id="sub">Đang kết nối...</p></div>

<div class="tabs" role="tablist" aria-label="Điều khiển cánh tay">
  <button class="tab-btn active" data-t="dash" role="tab" aria-selected="true" aria-controls="dash">Dashboard</button>
  <button class="tab-btn" data-t="joints" role="tab" aria-selected="false" aria-controls="joints">Joints</button>
  <button class="tab-btn" data-t="home" role="tab" aria-selected="false" aria-controls="home">Homing</button>
  <button class="tab-btn" data-t="wifi" role="tab" aria-selected="false" aria-controls="wifi">WiFi</button>
  <button class="tab-btn" data-t="cart" role="tab" aria-selected="false" aria-controls="cart">Cartesian</button>
  <button class="tab-btn" data-t="draw" role="tab" aria-selected="false" aria-controls="draw">Draw</button>
</div>

<div id="dash" class="tab-pane active" role="tabpanel" aria-labelledby="dash-tab">
  <div class="card"><h2>Trạng thái hệ thống</h2>
    <div class="stat-line"><span>Chế độ</span><span id="mode" class="badge b-idle">idle</span></div>
    <div class="stat-line"><span>Khớp đã home</span><span id="homedN">-/6</span></div>
    <div class="stat-line"><span>WiFi</span><span id="wifiInfo">-</span></div>
    <div class="stat-line"><span>Endstops</span><span id="esInfo" class="muted">-</span></div>
    <div class="stat-line"><span>Homing</span><span id="homProg">idle</span></div>
    <div class="home-track" aria-label="Tiến độ homing"><span class="hchip" id="hc0">J1</span><span class="hchip" id="hc1">J2</span><span class="hchip" id="hc2">J3</span><span class="hchip" id="hc3">J4</span></div>
  </div>
  <div class="card"><h2>Thao tác nhanh</h2>
    <div class="row">
      <button class="btn primary need-idle" onclick="api('/api/home/all')" aria-label="Home all joints J1-J4">HOME ALL (J1–J4)</button>
      <button class="btn warn" onclick="api('/api/stop')" aria-label="Emergency stop all motion">STOP ALL</button>
      <button class="btn ghost" onclick="clearFault()" aria-label="Clear fault state">CLEAR FAULT</button>
    </div>
    <p class="muted" style="margin-top:8px">HOME ALL chạy tuần tự J1→J2→J3→J4. J5/J6 dùng Set-Home thủ công ở tab Homing.</p>
  </div>
</div>

<div id="joints" class="tab-pane" role="tabpanel" aria-labelledby="joints-tab">
  <div class="card"><h2>Điều khiển khớp thủ công</h2>
    <div class="step-label" style="margin-bottom:10px"><span>Bước jog:</span>
      <div class="stepsel" id="stepSel" role="group" aria-label="Chọn bước jog"></div>
    </div>
    <div class="grid-joints" id="jointGrid" role="list" aria-label="6 khớp điều khiển"></div>
  </div>
</div>

<div id="home" class="tab-pane" role="tabpanel" aria-labelledby="home-tab">
  <div class="card"><h2>Homing tự động (TMC J1–J4)</h2>
    <div class="row" style="flex-wrap:wrap;gap:8px">
      <button class="btn primary need-idle" onclick="api('/api/home/all')" aria-label="Home all joints sequentially">HOME ALL</button>
      <span class="muted">Từng khớp:</span>
      <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=0')" aria-label="Home joint 1">Home J1</button>
      <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=1')" aria-label="Home joint 2">Home J2</button>
      <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=2')" aria-label="Home joint 3">Home J3</button>
      <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=3')" aria-label="Home joint 4">Home J4</button>
    </div>
    <p class="muted" style="margin-top:8px">J1/J2: min-stop → về GIỮA hành trình. J3/J4: min-stop/stall + lùi 2°.</p>
  </div>
  <div class="card"><h2>Set-Home thủ công & Calibration</h2>
    <p class="muted" style="margin-bottom:10px">Đưa khớp về vị trí mong muốn rồi nhấn Set Home. Dữ liệu lưu NVS — giữ nguyên sau khi tắt nguồn.</p>
    <div class="grid-joints" id="setHomeGrid" role="list" aria-label="Set home và xóa calib"></div>
  </div>
</div>

<div id="wifi" class="tab-pane" role="tabpanel" aria-labelledby="wifi-tab">
  <div class="card"><h2>Kết nối hiện tại</h2>
    <div class="stat-line"><span>Chế độ</span><span id="wfMode" class="badge b-info">-</span></div>
    <div class="stat-line"><span>IP</span><span id="wfIp">-</span></div>
    <div class="stat-line"><span>SSID</span><span id="wfSsidNow">-</span></div>
    <div class="stat-line"><span>RSSI</span><span id="wfRssi">-</span></div>
  </div>
  <div class="card"><h2>Cấu hình WiFi</h2>
    <label for="wfSsid">SSID</label>
    <input type="text" id="wfSsid" placeholder="tên wifi nhà" autocomplete="off">
    <label for="wfPass">Mật khẩu</label>
    <input type="password" id="wfPass" placeholder="mật khẩu" autocomplete="current-password">
    <div class="row" style="margin-top:8px">
      <button class="btn ok" onclick="saveWifi()" aria-label="Lưu WiFi và khởi động lại">SAVE & REBOOT</button>
      <span class="muted">Lưu vào NVS — không cần flash lại khi đổi mạng.</span>
    </div>
  </div>
</div>

<div id="cart" class="tab-pane" role="tabpanel" aria-labelledby="cart-tab">
  <div class="card"><h2>Di chuyển TCP (bút hướng xuống)</h2>
    <p class="muted" style="margin-bottom:8px">Yêu cầu đã HOME J1–J4. Vị trí hiện tại:
      <span id="poseNow" style="color:var(--color-primary);font-weight:var(--fw-bold)">-</span></p>
    <div class="row" style="flex-wrap:wrap;gap:8px">
      <input type="text" id="mvX" inputmode="decimal" placeholder="X (mm)" style="width:90px" aria-label="X coordinate">
      <input type="text" id="mvY" inputmode="decimal" placeholder="Y (mm)" style="width:90px" aria-label="Y coordinate">
      <input type="text" id="mvZ" inputmode="decimal" placeholder="Z (mm)" style="width:90px" aria-label="Z coordinate">
      <input type="text" id="mvFeed" inputmode="decimal" placeholder="feed mm/s" value="30" style="width:90px" aria-label="Feed rate">
      <button class="btn primary need-idle" onclick="moveTo()" aria-label="Move to position">MOVE</button>
    </div>
    <p class="muted" style="margin-top:6px">Home TCP = (146, 0, 365). Giấy vẽ đặt dưới bút, Z nhỏ hơn khi hạ.</p>
  </div>
</div>

<div id="draw" class="tab-pane" role="tabpanel" aria-labelledby="draw-tab">
  <div class="card"><h2>Vẽ hình (bút trên giấy)</h2>
    <canvas id="cv" width="420" height="300" aria-label="Preview đường vẽ từ trên xuống" style="background:var(--color-bg);border:1px solid var(--color-border);border-radius:var(--radius-sm);display:block;margin-bottom:10px"></canvas>
    <label for="dwShape">Hình</label>
    <select id="dwShape" aria-label="Chọn hình vẽ">
      <option value="line">Line</option>
      <option value="circle">Circle</option>
    </select>
    <div class="row" style="flex-wrap:wrap;gap:8px">
      <input type="text" id="dwA1" inputmode="decimal" placeholder="x1 / cx" style="width:90px" aria-label="X1 / Center X">
      <input type="text" id="dwA2" inputmode="decimal" placeholder="y1 / cy" style="width:90px" aria-label="Y1 / Center Y">
      <input type="text" id="dwA3" inputmode="decimal" placeholder="x2 / r"   style="width:90px" aria-label="X2 / Radius">
      <input type="text" id="dwA4" inputmode="decimal" placeholder="y2"       style="width:90px" aria-label="Y2">
      <input type="text" id="dwZ"  inputmode="decimal" placeholder="z giấy"   style="width:90px" aria-label="Paper Z">
      <input type="text" id="dwFeed" inputmode="decimal" placeholder="feed"   style="width:80px" value="20" aria-label="Feed rate">
    </div>
    <div class="row" style="margin-top:10px;flex-wrap:wrap;gap:8px">
      <button class="btn ok need-idle" onclick="startDraw()" aria-label="Bắt đầu vẽ">START DRAW</button>
      <button class="btn danger" onclick="api('/api/stop')" aria-label="Dừng vẽ ngay">ABORT</button>
      <button class="btn ghost" onclick="previewShape()" aria-label="Xem trước đường vẽ">PREVIEW</button>
      <span class="muted">Preview: hình chiếu từ trên xuống (đơn vị mm, gốc = trục J1).</span>
    </div>
  </div>
</div>

<button id="estop" class="btn danger" onclick="api('/api/stop')" aria-label="Emergency stop — dừng khẩn cấp">&#9888; E-STOP</button>
<div id="toast" role="status" aria-live="polite" aria-atomic="true"></div>

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
    toast((t==='OK'?'✓ ':'')+t,t==='OK'?'ok':'warn');
    return t;
  }).catch(()=>toast('Lỗi mạng / mất kết nối','err'));
}
function post(url,body){
  return fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body})
    .then(r=>r.text())
    .then(t=>toast((t==='OK'?'✓ ':'')+t,t==='OK'?'ok':'warn'))
    .catch(()=>toast('Lỗi mạng / mất kết nối','err'));
}
function clearFault(){ post('/api/jog','fault_clear=1'); }

document.querySelectorAll('.tab-btn[data-t]').forEach(b=>{
  b.onclick=()=>{
    document.querySelectorAll('.tab-btn').forEach(x=>{
      x.classList.remove('active');
      x.setAttribute('aria-selected','false');
    });
    document.querySelectorAll('.tab-pane').forEach(x=>x.classList.remove('active'));
    b.classList.add('active');
    b.setAttribute('aria-selected','true');
    document.getElementById(b.dataset.t).classList.add('active');
  };
});

[0.5,1,5,15].forEach(s=>{
  const b=document.createElement('button');
  b.className='stepbtn'+(s===stepSize?' active':'');
  b.textContent=s+'°';
  b.setAttribute('role','radio');
  b.setAttribute('aria-checked',s===stepSize);
  b.onclick=()=>{
    stepSize=s;
    document.querySelectorAll('.stepbtn').forEach(x=>{
      x.classList.remove('active');
      x.setAttribute('aria-checked','false');
    });
    b.classList.add('active');
    b.setAttribute('aria-checked','true');
  };
  document.getElementById('stepSel').appendChild(b);
});

function buildCards(){
  const g=document.getElementById('jointGrid'); g.innerHTML='';
  const sg=document.getElementById('setHomeGrid'); sg.innerHTML='';
  for(let i=0;i<6;i++){
    g.insertAdjacentHTML('beforeend',
     `<div class="jcard" role="listitem">
        <div class="jname"><span>${AXES[i]}</span><span class="jdeg" id="jd${i}">--</span></div>
        <div class="jenc" id="je${i}">encoder: --</div>
        <div class="jflags" id="jf${i}"></div>
        <div class="jbtns">
          <button class="btn danger" onclick="jog(${i},-${stepSize})" aria-label="Jog ${AXES[i]} negative">&#8630;</button>
          <button class="btn primary" onclick="jog(${i},${stepSize})" aria-label="Jog ${AXES[i]} positive">&#8634;</button>
        </div>
      </div>`);
    sg.insertAdjacentHTML('beforeend',
     `<div class="jcard" role="listitem">
        <div class="jname"><span>${AXES[i]}</span></div>
        <div class="jbtns">
          <button class="btn ok need-idle" onclick="api('/api/sethome?axis=${i}')" aria-label="Set home for ${AXES[i]}">Set Home</button>
          <button class="btn warn" onclick="if(confirm('Xóa calib J${i+1}? Sẽ mất vị trí đã lưu trong NVS.'))api('/api/clearcalib?axis=${i}')" aria-label="Clear calibration for ${AXES[i]}">Clear Calib</button>
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
  const btn=event.target;
  btn.classList.add('btn-loading');
  btn.disabled=true;
  post('/api/wifi',`ssid=${encodeURIComponent(s)}&pass=${encodeURIComponent(p)}`)
    .then(()=>{
      document.getElementById('sub').innerText='Đã lưu. Đang restart...';
      btn.classList.remove('btn-loading');
      btn.disabled=false;
    });
}

function moveTo(){
  const btn=event.target;
  btn.classList.add('btn-loading');
  btn.disabled=true;
  const b=`x=${document.getElementById('mvX').value||0}&y=${document.getElementById('mvY').value||0}&z=${document.getElementById('mvZ').value||0}&feed=${document.getElementById('mvFeed').value||30}`;
  post('/api/move',b).finally(()=>{
    btn.classList.remove('btn-loading');
    btn.disabled=false;
  });
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
  const btn=event.target;
  btn.classList.add('btn-loading');
  btn.disabled=true;
  post('/api/draw',drawParams()).finally(()=>{
    btn.classList.remove('btn-loading');
    btn.disabled=false;
  });
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
  ctx.strokeStyle='var(--color-primary)';ctx.lineWidth=2;ctx.beginPath();
  if(sh==='line'){
    ctx.moveTo(cxp+v('dwA1')*sc,cyp-v('dwA2')*sc);
    ctx.lineTo(cxp+v('dwA3')*sc,cyp-v('dwA4')*sc);
  }else{
    ctx.arc(cxp+v('dwA1')*sc,cyp-v('dwA2')*sc,v('dwA3')*sc,0,Math.PI*2);
  }
  ctx.stroke();
  // TCP hiện tại
  if(lastPose){ctx.fillStyle='var(--color-text-primary)';ctx.beginPath();
    ctx.arc(cxp+lastPose.x*sc,cyp-lastPose.y*sc,4,0,Math.PI*2);ctx.fill();}
  // marker home (146,0)
  ctx.fillStyle='#4ade80';ctx.beginPath();
  ctx.arc(cxp+146*sc,cyp,3,0,Math.PI*2);ctx.fill();
  ctx.fillStyle='var(--color-text-dim)';ctx.font='10px sans-serif';
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
    document.getElementById(`jd${i}`).innerText=j.deg.toFixed(1)+'°';
    document.getElementById(`je${i}`).innerText='encoder: '+(j.encOK?j.encDeg.toFixed(1)+'°':'MẤT KẾT NỐI');
    const f=[];
    f.push(`<span class="${j.homed?'f-h':'f-e'}">${j.homed?(j.restored?'HOMED (NVS)':'HOMED'):'CHƯA HOME'}</span>`);
    if(j.drift)f.push('<span class="f-d">DRIFT!</span>');
    if(!j.encOK)f.push('<span class="f-e">ENC ERR</span>');
    document.getElementById(`jf${i}`).innerHTML=f.join(' · ');
  });
}

function setOnline(on){
  const s=document.getElementById('sub');
  s.classList.toggle('offline',!on);
  if(!on)s.innerText='⚠ MẤT KẾT NỐI — đang thử lại...';
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
