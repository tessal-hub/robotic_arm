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
<title>6-Axis Robotic Arm Controller &amp; 3D Digital Clone</title>
<style>
:root{
  --color-primary: #38bdf8;
  --color-primary-deep: #0ea5e9;
  --color-primary-muted: #7dd3fc;
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
  --color-bg: #0b0f19;
  --color-surface: #111827;
  --color-surface-elevated: #1e293b;
  --color-border: #1f2937;
  --color-border-strong: #334155;
  --color-text-primary: #f8fafc;
  --color-text-secondary: #cbd5e1;
  --color-text-muted: #94a3b8;
  --color-text-dim: #64748b;
  --color-focus-ring: #38bdf8;
  --color-estop-glow: rgba(220,38,38,0.5);
  --color-estop-glow-strong: rgba(220,38,38,0.9);
  --color-shadow-card: rgba(0,0,0,0.45);
  --color-shadow-toast: rgba(0,0,0,0.5);
  --space-xs: 4px;
  --space-sm: 6px;
  --space-md: 8px;
  --space-lg: 10px;
  --space-xl: 12px;
  --space-2xl: 14px;
  --space-card-padding: 16px;
  --space-card-gap: 14px;
  --space-tab-gap: 6px;
  --space-page-padding: 14px;
  --space-max-width: 1200px;
  --radius-xs: 4px;
  --radius-sm: 8px;
  --radius-md: 10px;
  --radius-lg: 14px;
  --radius-pill: 11px;
  --radius-badge: 12px;
  --font-sans: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
  --font-mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  --fs-display: clamp(1.2rem, 3.5vw, 1.5rem);
  --fs-headline: 1.0rem;
  --fs-body: 0.85rem;
  --fs-label: 0.76rem;
  --fs-tabular: 1.4rem;
  --fw-regular: 400;
  --fw-medium: 500;
  --fw-semibold: 600;
  --fw-bold: 700;
  --transition-fast: 80ms;
  --transition-normal: 150ms;
  --ease-out: cubic-bezier(0.16, 1, 0.3, 1);
  --ease-spring: cubic-bezier(0.23, 1, 0.32, 1);
  --shadow-card: 0 8px 24px var(--color-shadow-card);
  --shadow-toast: 0 6px 20px var(--shadow-toast);
  --shadow-estop: 0 6px 24px var(--color-estop-glow);
  --shadow-estop-strong: 0 6px 36px var(--color-estop-glow-strong);
  --z-toast: 98;
  --z-estop: 99;
}

*{box-sizing:border-box;margin:0;padding:0}
::selection{background:var(--color-primary);color:var(--color-bg)}
html{color-scheme:dark}
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
:focus-visible{outline:2px solid var(--color-focus-ring);outline-offset:2px}
.app{width:100%;max-width:var(--space-max-width)}

/* ---- Header ---- */
.header{
  display:flex;align-items:center;justify-content:space-between;gap:var(--space-xl);
  padding:var(--space-md) 0 var(--space-xl);
  flex-wrap:wrap;
}
.brand{display:flex;align-items:baseline;gap:var(--space-xl);flex-wrap:wrap}
.brand h1{font-size:var(--fs-display);font-weight:var(--fw-bold);color:var(--color-text-primary)}
.brand h1 b{color:var(--color-primary);font-weight:var(--fw-bold)}
.brand .ver{font-size:var(--fs-label);color:var(--color-text-dim);font-family:var(--font-mono);background:var(--color-surface-elevated);padding:2px 8px;border-radius:4px}
.connline{display:flex;align-items:center;gap:var(--space-md);color:var(--color-text-muted);font-size:var(--fs-body);flex-wrap:wrap}
.connline .dot{width:8px;height:8px;border-radius:50%;background:var(--color-success)}
.connline.offline .dot{background:var(--color-danger)}
.connline.offline{color:var(--color-danger-text)}

/* ---- Tabs ---- */
.tabs{
  display:flex;gap:var(--space-tab-gap);
  background:var(--color-surface);
  padding:var(--space-sm);
  border-radius:var(--radius-md);
  margin-bottom:14px;
  flex-wrap:wrap;
  justify-content:center;
  width:100%;
  border:1px solid var(--color-border);
}
.tab-btn{
  background:transparent;
  border:none;
  color:var(--color-text-muted);
  padding:var(--space-md) var(--space-lg);
  border-radius:var(--radius-sm);
  font-size:0.84rem;
  font-weight:var(--fw-semibold);
  cursor:pointer;
  transition:background var(--transition-normal) var(--ease-out),color var(--transition-normal) var(--ease-out);
}
.tab-btn:hover{background:var(--color-surface-elevated);color:var(--color-text-secondary)}
.tab-btn.active{background:var(--color-primary);color:var(--color-bg);font-weight:var(--fw-bold)}
.tab-btn.disabled{opacity:.35;cursor:not-allowed}
.tab-pane{display:none}
.tab-pane.active{display:block}

/* ---- Dashboard grid ---- */
.dash-grid{display:grid;grid-template-columns:1.2fr .8fr;gap:var(--space-card-gap)}
@media(max-width:880px){.dash-grid{grid-template-columns:1fr}}

/* ---- Cards ---- */
.card{
  background:var(--color-surface);
  border-radius:var(--radius-lg);
  padding:var(--space-card-padding);
  box-shadow:var(--shadow-card);
  margin-bottom:var(--space-card-gap);
  border:1px solid var(--color-border);
}
.card-head{
  display:flex;align-items:center;justify-content:space-between;gap:var(--space-md);
  margin-bottom:var(--space-xl);
  padding-bottom:var(--space-md);
  border-bottom:1px solid var(--color-border);
}
.card-head h2{font-size:var(--fs-headline);font-weight:var(--fw-semibold);color:var(--color-primary)}
.card-head .meta{font-size:var(--fs-label);color:var(--color-text-dim)}

/* ---- 3D Viewport & Simulation Box ---- */
.sim-viewport-box{
  position:relative;
  background:#090d16;
  border:1px solid var(--color-border);
  border-radius:var(--radius-sm);
  overflow:hidden;
}
.sim-canvas{
  display:block;width:100%;height:380px;
  background:#090d16;cursor:grab;
}
.sim-canvas:active{cursor:grabbing}
.sim-overlay-bar{
  position:absolute;top:8px;left:8px;right:8px;
  display:flex;justify-content:space-between;align-items:center;
  pointer-events:none;flex-wrap:wrap;gap:6px;z-index:2;
}
.sim-pill-group{
  display:flex;gap:4px;
  background:rgba(17,24,39,0.85);backdrop-filter:blur(6px);
  padding:3px;border-radius:var(--radius-sm);
  border:1px solid var(--color-border-strong);
  pointer-events:auto;
}
.sim-pill-btn{
  background:transparent;border:none;color:var(--color-text-muted);
  padding:4px 9px;border-radius:var(--radius-xs);
  font-size:0.72rem;font-weight:var(--fw-semibold);cursor:pointer;
  transition:all var(--transition-fast);
}
.sim-pill-btn:hover{background:var(--color-surface-elevated);color:var(--color-text-primary)}
.sim-pill-btn.active{background:var(--color-primary);color:var(--color-bg);font-weight:var(--fw-bold)}
.sim-pill-btn.src-live.active{background:#10b981;color:#0b0f19}
.sim-pill-btn.src-sim.active{background:#f59e0b;color:#0b0f19}

.sim-hud-box{
  background:#090d16;border:1px solid var(--color-border);border-radius:var(--radius-sm);
  padding:8px 12px;font-family:var(--font-mono);font-size:0.74rem;color:#f8fafc;
  line-height:1.45;white-space:pre-wrap;margin-top:8px;
}

/* ---- Controls Grid ---- */
.sim-ctrl-grid{display:grid;grid-template-columns:1fr 1fr;gap:var(--space-card-gap)}
@media(max-width:768px){.sim-ctrl-grid{grid-template-columns:1fr}}

.sim-slider-row{
  display:grid;grid-template-columns:85px 1fr 50px;align-items:center;gap:8px;
  margin-bottom:6px;font-size:0.78rem;
}
.sim-slider-row label{color:var(--color-text-secondary);font-weight:var(--fw-medium);margin-bottom:0;text-transform:none;letter-spacing:normal}
.sim-slider-row input[type="range"]{accent-color:var(--color-primary);width:100%;margin:0}
.sim-slider-row span.val{color:var(--color-primary);font-family:var(--font-mono);font-weight:var(--fw-bold);text-align:right}

.preset-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(95px,1fr));gap:6px;margin-top:8px}

/* ---- Mode readout ---- */
.mode-block{display:flex;flex-direction:column;gap:var(--space-sm);margin-bottom:var(--space-xl)}
.mode-label{font-size:var(--fs-label);font-weight:var(--fw-semibold);color:var(--color-text-dim);text-transform:uppercase;letter-spacing:0.04em}
.mode-line{display:flex;align-items:center;gap:var(--space-md);flex-wrap:wrap}
.mode-word{font-size:2.0rem;font-weight:var(--fw-bold);line-height:1;color:var(--color-text-primary);font-variant-numeric:tabular-nums}
.badge{
  padding:3px 10px;border-radius:var(--radius-badge);
  font-size:var(--fs-label);font-weight:var(--fw-bold);
  text-transform:uppercase;letter-spacing:0.04em;
  white-space:nowrap;
}
.b-idle{background:var(--color-info-bg);color:var(--color-info-text)}
.b-run{background:var(--color-success-bg);color:var(--color-success-text)}
.b-fault{background:var(--color-danger-bg);color:var(--color-danger-text)}
.b-warn{background:var(--color-warning-bg);color:var(--color-warning-text)}
body .mode-word.fault{color:var(--color-danger-text)}
body .mode-word.run{color:var(--color-success-text)}

/* ---- Stat lines ---- */
.stat-line{display:flex;justify-content:space-between;font-size:var(--fs-body);color:var(--color-text-muted);margin:4px 0;gap:var(--space-lg)}
.stat-line .k{color:var(--color-text-muted);font-variant-numeric:tabular-nums}
.stat-line .v{color:var(--color-text-primary);font-variant-numeric:tabular-nums;font-weight:var(--fw-medium);text-align:right}
.stat-line .v.muted{color:var(--color-text-muted)}
.stat-line .v.off{color:var(--color-text-dim)}
.stat-line.danger .v{color:var(--color-danger-text)}

/* ---- Home progress ---- */
.home-track{display:flex;gap:var(--space-xs);margin-top:var(--space-xl);flex-wrap:wrap}
.hchip{
  background:var(--color-border);color:var(--color-text-muted);
  padding:3px 12px;border-radius:var(--radius-pill);
  font-size:0.72rem;font-weight:var(--fw-bold);
}
.hchip.on{background:var(--color-warning);color:var(--color-text-primary)}
.hchip.done{background:var(--color-success-bg);color:var(--color-success-text)}

/* ---- Buttons ---- */
.btn{
  border:none;border-radius:var(--radius-sm);
  padding:var(--space-md) var(--space-lg);
  font-weight:var(--fw-semibold);font-size:var(--fs-body);
  cursor:pointer;
  transition:transform var(--transition-fast) var(--ease-spring),background var(--transition-normal) var(--ease-out),opacity var(--transition-normal);
}
.btn:hover:not(:disabled){filter:brightness(1.1)}
.btn:active:not(:disabled){transform:scale(0.97)}
.btn[disabled],.btn:disabled{opacity:.4;cursor:not-allowed;transform:none;filter:none}
.primary{background:var(--color-info);color:var(--color-text-primary)}
.warn{background:var(--color-warning);color:var(--color-text-primary)}
.danger{background:var(--color-danger);color:var(--color-text-primary)}
.ghost{background:var(--color-surface-elevated);color:var(--color-text-secondary);border:1px solid var(--color-border)}
.ok{background:var(--color-success);color:var(--color-text-primary)}
.row{display:flex;gap:var(--space-md);flex-wrap:wrap;align-items:center}
.btn-loading{position:relative;color:transparent!important}
.btn-loading::after{content:"";position:absolute;width:16px;height:16px;top:50%;left:50%;margin:-8px 0 0 -8px;border:2px solid currentColor;border-right-color:transparent;border-radius:50%;animation:spin 0.6s linear infinite}
@keyframes spin{to{transform:rotate(360deg)}}

/* ---- Joints ---- */
.grid-joints{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:var(--space-card-gap)}
.jcard{
  background:var(--color-surface-elevated);
  border:1px solid var(--color-border);
  border-radius:var(--radius-md);
  padding:12px;
}
.jname{font-weight:var(--fw-bold);color:var(--color-text-secondary);display:flex;justify-content:space-between;align-items:center;margin-bottom:6px}
.jdeg{font-size:var(--fs-tabular);color:var(--color-primary);font-variant-numeric:tabular-nums;font-family:var(--font-mono)}
.jenc{font-size:0.72rem;color:var(--color-text-muted)}
.jflags{font-size:0.68rem;margin-top:4px;display:flex;gap:var(--space-sm);flex-wrap:wrap}
.f-h{color:#4ade80}.f-r{color:#93c5fd}.f-d{color:#fca5a5}.f-e{color:#fcd34d}
.jctrl{margin-top:var(--space-lg)}
.jbtns{display:grid;grid-template-columns:1fr 1fr;gap:var(--space-xs)}
.jbtns .btn{padding:9px var(--space-lg);font-size:1.0rem}
.stepsel{display:flex;gap:var(--space-xs);margin-top:var(--space-lg);width:200px}
.stepbtn{
  flex:1;background:var(--color-surface-elevated);color:var(--color-text-secondary);
  border:1px solid var(--color-border);padding:4px var(--space-xs);border-radius:var(--radius-xs);
  font-size:0.75rem;font-weight:var(--fw-semibold);cursor:pointer;
  transition:background var(--transition-normal) var(--ease-out),color var(--transition-normal) var(--ease-out);
}
.stepbtn:hover{background:var(--color-border-strong)}
.stepbtn.active{background:var(--color-primary);color:var(--color-bg);font-weight:var(--fw-bold)}
.step-label{display:flex;align-items:center;gap:var(--space-md);font-size:0.8rem;color:var(--color-text-muted);margin-bottom:var(--space-lg)}
.step-label .lbl{white-space:nowrap}

/* ---- Forms ---- */
label{display:block;font-size:var(--fs-label);font-weight:var(--fw-semibold);color:var(--color-text-muted);margin-bottom:4px;text-transform:uppercase;letter-spacing:0.04em}
input[type=text],input[type=password],input[type=number],select{
  width:100%;background:var(--color-surface-elevated);border:1px solid var(--color-border);
  color:var(--color-text-primary);font-size:0.95rem;padding:8px 12px;border-radius:var(--radius-sm);
  outline:none;margin-bottom:var(--space-md);font-family:var(--font-sans);
  transition:border-color var(--transition-normal) var(--ease-out);
}
input:focus,select:focus{border-color:var(--color-focus-ring)}
input::placeholder{color:var(--color-text-dim)}
select{appearance:none;background-image:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='16' height='16' viewBox='0 0 24 24' fill='none' stroke='%2394a3b8' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'%3E%3Cpolyline points='6 9 12 15 18 9'%3E%3C/polyline%3E%3C/svg%3E");background-repeat:no-repeat;background-position:right 12px center;padding-right:36px}

/* ---- E-STOP ---- */
#estop{
  position:fixed;bottom:16px;right:16px;z-index:var(--z-estop);
  padding:16px 26px;font-size:1.1rem;font-weight:var(--fw-semibold);
  box-shadow:var(--shadow-estop);animation:estopPulse 2.2s ease-in-out infinite;
}
@keyframes estopPulse{0%,100%{box-shadow:var(--shadow-estop)}50%{box-shadow:var(--shadow-estop-strong)}}
@media(prefers-reduced-motion:reduce){
  #estop{animation:none}
  *{transition-duration:0.01ms!important;animation-duration:0.01ms!important}
}

/* ---- Toast ---- */
#toast{
  position:fixed;bottom:16px;left:16px;z-index:var(--z-toast);
  background:var(--color-surface);border:1px solid var(--color-border);
  border-left-width:4px;border-radius:var(--radius-sm);padding:10px 14px;
  font-size:0.84rem;color:var(--color-text-secondary);display:none;max-width:70vw;
  box-shadow:var(--shadow-toast);animation:toastIn 0.15s var(--ease-out);
}
@keyframes toastIn{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:translateY(0)}}
.t-ok{border-left-color:var(--color-success)}
.t-warn{border-left-color:var(--color-warning)}
.t-err{border-left-color:var(--color-danger)}

.muted{color:var(--color-text-dim);font-size:var(--fs-label)}
.off{color:var(--color-text-dim)}
.mono{font-family:var(--font-mono)}
.divider{height:1px;background:var(--color-border);margin:var(--space-xl) 0}

::-webkit-scrollbar{width:8px;height:8px}
::-webkit-scrollbar-thumb{background:var(--color-border-strong);border-radius:4px}
::-webkit-scrollbar-track{background:transparent}
</style>
</head>
<body>
<div class="app">
  <header class="header">
    <div class="brand"><h1>6-Axis <b>Arm</b></h1><span class="ver">NEMA-6AXIS DIGITAL CLONE</span></div>
    <div class="connline" id="conn"><span class="dot"></span><span id="sub">Đang kết nối...</span></div>
  </header>

  <nav class="tabs" role="tablist" aria-label="Điều khiển cánh tay">
    <button class="tab-btn active" data-t="dash" role="tab" aria-selected="true">Dashboard</button>
    <button class="tab-btn" data-t="sim" role="tab" aria-selected="false">3D Simulation</button>
    <button class="tab-btn" data-t="joints" role="tab" aria-selected="false">Joints</button>
    <button class="tab-btn" data-t="home" role="tab" aria-selected="false">Homing</button>
    <button class="tab-btn" data-t="cart" role="tab" aria-selected="false">Cartesian</button>
    <button class="tab-btn" data-t="draw" role="tab" aria-selected="false">Draw</button>
    <button class="tab-btn" data-t="wifi" role="tab" aria-selected="false">WiFi</button>
  </nav>

  <!-- ============ DASHBOARD ============ -->
  <div id="dash" class="tab-pane active" role="tabpanel">
    <div class="dash-grid">
      <div class="card">
        <div class="card-head">
          <h2>Mô hình cánh tay 3D (Thời gian thực)</h2>
          <span class="meta mono" id="poseNow">--</span>
        </div>
        <div class="sim-viewport-box">
          <div class="sim-overlay-bar">
            <div class="sim-pill-group">
              <button class="sim-pill-btn active" onclick="setDashView('3d', this)">3D Orbit</button>
              <button class="sim-pill-btn" onclick="setDashView('side', this)">Side (X-Z)</button>
              <button class="sim-pill-btn" onclick="setDashView('top', this)">Top (X-Y)</button>
            </div>
            <div class="sim-pill-group">
              <button class="sim-pill-btn" onclick="resetDashCam()">Reset View</button>
            </div>
          </div>
          <canvas id="poseCanvas" class="sim-canvas" width="640" height="380"></canvas>
        </div>
        <div class="sim-hud-box" id="dashHud">TELEMETRY: Đang tải...</div>
      </div>

      <div class="card">
        <div class="card-head"><h2>Trạng thái hệ thống</h2><span class="meta">300 ms</span></div>
        <div class="mode-block">
          <span class="mode-label">Chế độ hoạt động</span>
          <div class="mode-line"><span class="mode-word" id="modeWord">--</span><span id="mode" class="badge b-idle">idle</span></div>
        </div>
        <div class="stat-line"><span class="k">WiFi</span><span class="v" id="wifiInfo">-</span></div>
        <div class="stat-line"><span class="k">Khớp đã home</span><span class="v" id="homedN">-/6</span></div>
        <div class="stat-line" id="esRow"><span class="k">Endstops</span><span class="v" id="esInfo" class="muted">-</span></div>
        <div class="stat-line"><span class="k">Homing</span><span class="v muted" id="homProg">idle</span></div>
        <div class="home-track" aria-label="Tiến độ homing"><span class="hchip" id="hc0">J1</span><span class="hchip" id="hc1">J2</span><span class="hchip" id="hc2">J3</span><span class="hchip" id="hc3">J4</span></div>
        <div class="divider"></div>
        <div class="row">
          <button class="btn primary need-idle" onclick="api('/api/home/all')">HOME ALL</button>
          <button class="btn warn" onclick="api('/api/stop')">STOP ALL</button>
          <button class="btn ghost" onclick="clearFault()">CLEAR FAULT</button>
        </div>
        <p class="muted" style="margin-top:10px">HOME ALL chạy tuần tự J1→J2→J3→J4. J5/J6 dùng Set-Home thủ công ở tab Homing.</p>
      </div>
    </div>
  </div>

  <!-- ============ 3D SIMULATION STUDIO ============ -->
  <div id="sim" class="tab-pane" role="tabpanel">
    <div class="card">
      <div class="card-head">
        <h2>3D Digital Clone Studio (Exact Craig MDH Kinematics)</h2>
        <span class="meta" id="simSourceLabel">[CHẾ ĐỘ MÔ PHỎNG]</span>
      </div>

      <div class="sim-viewport-box">
        <div class="sim-overlay-bar">
          <div class="sim-pill-group">
            <button class="sim-pill-btn active" onclick="setSimView('3d', this)">3D Orbit</button>
            <button class="sim-pill-btn" onclick="setSimView('side', this)">Side (X-Z)</button>
            <button class="sim-pill-btn" onclick="setSimView('top', this)">Top (X-Y)</button>
          </div>
          <div class="sim-pill-group">
            <button class="sim-pill-btn src-live" id="btnSrcLive" onclick="setSimSource('live')">🔴 Live Robot</button>
            <button class="sim-pill-btn src-sim active" id="btnSrcSim" onclick="setSimSource('sim')">🟢 Interactive Sim</button>
            <button class="sim-pill-btn" onclick="resetSimCam()">Reset Cam</button>
          </div>
        </div>
        <canvas id="simCanvas" class="sim-canvas" width="680" height="400"></canvas>
      </div>

      <div class="sim-hud-box" id="simHud">TELEMETRY HUD | Đang tính toán động học...</div>
    </div>

    <div class="sim-ctrl-grid">
      <!-- Card A: Cartesian Target -->
      <div class="card">
        <div class="card-head">
          <h2>1. Tọa độ Cartesian IK (mm)</h2>
          <span class="badge b-run" id="simIkBadge">IK OK</span>
        </div>
        <div class="sim-slider-row">
          <label>Target X:</label>
          <input type="range" id="simX" min="-250" max="250" step="1" value="160" oninput="onSimCartChange()">
          <span class="val" id="simXVal">160 mm</span>
        </div>
        <div class="sim-slider-row">
          <label>Target Y:</label>
          <input type="range" id="simY" min="-250" max="250" step="1" value="0" oninput="onSimCartChange()">
          <span class="val" id="simYVal">0 mm</span>
        </div>
        <div class="sim-slider-row">
          <label>Target Z:</label>
          <input type="range" id="simZ" min="-10" max="350" step="1" value="10" oninput="onSimCartChange()">
          <span class="val" id="simZVal">10 mm</span>
        </div>
        <div class="row" style="margin-top:10px">
          <button class="btn primary need-idle" onclick="sendSimPoseToRobot()">⚡ NẠP VỊ TRÍ XUỐNG ROBOT</button>
        </div>
      </div>

      <!-- Card B: Joint Angles -->
      <div class="card">
        <div class="card-head">
          <h2>2. Góc khớp Joint (Độ)</h2>
          <span class="meta">Đồng bộ 2 chiều</span>
        </div>
        <div id="simJointSliders"></div>
      </div>

      <!-- Card C: Presets -->
      <div class="card">
        <div class="card-head"><h2>3. Tư thế mẫu (Presets)</h2></div>
        <div class="preset-grid">
          <button class="btn ghost" onclick="applySimPreset('home')">Home (0°)</button>
          <button class="btn ghost" onclick="applySimPreset('draw')">Draw Ready</button>
          <button class="btn ghost" onclick="applySimPreset('reach_fwd')">Reach +X</button>
          <button class="btn ghost" onclick="applySimPreset('reach_back')">Reach -X</button>
          <button class="btn ghost" onclick="applySimPreset('fold')">Folded</button>
        </div>
      </div>

      <!-- Card D: Trajectory Animator -->
      <div class="card">
        <div class="card-head"><h2>4. Mô phỏng vẽ (Trajectory Animator)</h2></div>
        <div class="row" style="margin-bottom:8px">
          <button class="btn ghost active" id="btnSimLine" onclick="setSimPathType('line')">Draw Line</button>
          <button class="btn ghost" id="btnSimCirc" onclick="setSimPathType('circle')">Draw Circle</button>
          <button class="btn primary" id="btnSimPlay" onclick="toggleSimPlay()">▶ Play</button>
        </div>
        <div class="sim-slider-row">
          <label>Scrub:</label>
          <input type="range" id="simScrub" min="0" max="100" step="1" value="0" oninput="onSimScrubChange(this.value)">
          <span class="val" id="simScrubVal">0%</span>
        </div>
      </div>
    </div>
  </div>

  <!-- ============ JOINTS ============ -->
  <div id="joints" class="tab-pane" role="tabpanel">
    <div class="card">
      <div class="card-head"><h2>Điều khiển khớp thủ công</h2><span class="meta">jog tương đối theo bước</span></div>
      <div class="step-label"><span class="lbl">Bước jog:</span><div class="stepsel" id="stepSel" role="group"></div><span class="muted" id="stepVal">1.0°</span></div>
      <div class="grid-joints" id="jointGrid" role="list"></div>
    </div>
  </div>

  <!-- ============ HOMING ============ -->
  <div id="home" class="tab-pane" role="tabpanel">
    <div class="card">
      <div class="card-head"><h2>Homing tự động (TMC J1–J4)</h2></div>
      <div class="row">
        <button class="btn primary need-idle" onclick="api('/api/home/all')">HOME ALL</button>
        <span class="muted">Từng khớp:</span>
        <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=0')">Home J1</button>
        <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=1')">Home J2</button>
        <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=2')">Home J3</button>
        <button class="btn ghost need-idle" onclick="api('/api/home/axis?axis=3')">Home J4</button>
      </div>
      <p class="muted" style="margin-top:10px">J1/J2: min-stop → về GIỮA hành trình. J3/J4: min-stop/stall + lùi 2°.</p>
    </div>
    <div class="card">
      <div class="card-head"><h2>Set-Home &amp; Calibration</h2><span class="meta">lưu NVS</span></div>
      <div class="grid-joints" id="setHomeGrid" role="list"></div>
    </div>
  </div>

  <!-- ============ CARTESIAN ============ -->
  <div id="cart" class="tab-pane" role="tabpanel">
    <div class="card">
      <div class="card-head"><h2>Di chuyển TCP (bút hướng xuống)</h2><span class="meta mono" id="poseNow2">--</span></div>
      <p class="muted" style="margin-bottom:12px">Yêu cầu đã HOME J1–J4. Home TCP = (146, 0, 365). Giấy vẽ đặt dưới bút, Z nhỏ hơn khi hạ.</p>
      <div class="row">
        <input type="number" id="mvX" step="any" placeholder="X mm" value="160">
        <input type="number" id="mvY" step="any" placeholder="Y mm" value="0">
        <input type="number" id="mvZ" step="any" placeholder="Z mm" value="10">
        <input type="number" id="mvFeed" step="any" placeholder="feed mm/s" value="30">
        <button class="btn primary need-idle" onclick="moveTo()">MOVE</button>
      </div>
    </div>
  </div>

  <!-- ============ DRAW ============ -->
  <div id="draw" class="tab-pane" role="tabpanel">
    <div class="card">
      <div class="card-head"><h2>Vẽ hình (bút trên giấy)</h2></div>
      <canvas id="cv" width="420" height="260" style="background:#090d16;border:1px solid var(--color-border);border-radius:var(--radius-sm);display:block;margin-bottom:12px;max-width:100%;height:auto"></canvas>
      <label for="dwShape">Hình</label>
      <select id="dwShape">
        <option value="line">Line</option>
        <option value="circle">Circle</option>
      </select>
      <div class="row" style="margin-bottom:10px">
        <input type="number" step="any" id="dwA1" placeholder="x1 / cx" value="100">
        <input type="number" step="any" id="dwA2" placeholder="y1 / cy" value="-70">
        <input type="number" step="any" id="dwA3" placeholder="x2 / r" value="180">
        <input type="number" step="any" id="dwA4" placeholder="y2" value="70">
        <input type="number" step="any" id="dwZ" placeholder="z giấy" value="10">
        <input type="number" step="any" id="dwFeed" placeholder="feed" value="20">
      </div>
      <div class="row">
        <button class="btn ok need-idle" onclick="startDraw()">START DRAW</button>
        <button class="btn danger" onclick="api('/api/stop')">ABORT</button>
        <button class="btn ghost" onclick="previewShape()">PREVIEW</button>
      </div>
    </div>
  </div>

  <!-- ============ WIFI ============ -->
  <div id="wifi" class="tab-pane" role="tabpanel">
    <div class="dash-grid">
      <div class="card">
        <div class="card-head"><h2>Kết nối hiện tại</h2></div>
        <div class="stat-line"><span class="k">Chế độ</span><span class="v" id="wfMode" class="badge b-info">-</span></div>
        <div class="stat-line"><span class="k">IP</span><span class="v mono" id="wfIp">-</span></div>
        <div class="stat-line"><span class="k">SSID</span><span class="v" id="wfSsidNow">-</span></div>
        <div class="stat-line"><span class="k">RSSI</span><span class="v" id="wfRssi">-</span></div>
      </div>
      <div class="card">
        <div class="card-head"><h2>Cấu hình WiFi</h2></div>
        <label for="wfSsid">SSID</label>
        <input type="text" id="wfSsid" placeholder="tên wifi" autocomplete="off">
        <label for="wfPass">Mật khẩu</label>
        <input type="password" id="wfPass" placeholder="mật khẩu" autocomplete="current-password">
        <div class="row" style="margin-top:8px">
          <button class="btn ok" onclick="saveWifi()">SAVE &amp; REBOOT</button>
        </div>
      </div>
    </div>
  </div>
</div>

<button id="estop" class="btn danger" onclick="api('/api/stop')">&#9888; E-STOP</button>
<div id="toast" role="status"></div>

<script>
/* =============================================================================
   1. HARDWARE CONSTANTS & CRAIG MODIFIED DH PARAMETERS (src/config.h)
============================================================================= */
const D1 = 139.0, A2 = 138.0, A3 = 88.0, D4 = 126.0, D_TOOL = 20.0;
const L_FORE = Math.hypot(A3, D4); 
const DELTA_WRIST = Math.atan2(D4, A3) * 180.0 / Math.PI; 
const DELTA_RAD = Math.atan2(D4, A3);

const JOINT_LIMITS = [
  [-90.0, 90.0],
  [-90.0, 90.0],
  [0.0, 90.0],
  [-180.0, 180.0],
  [-120.0, 120.0],
  [-360.0, 360.0]
];
const AXES = ["J1 Base Yaw", "J2 Shoulder", "J3 Elbow", "J4 Wrist Pan", "J5 Wrist Tilt", "J6 Tool Roll"];

/* =============================================================================
   2. MATRIX MATH & FORWARD KINEMATICS
============================================================================= */
function deg2rad(d){ return d * Math.PI / 180.0; }
function rad2deg(r){ return r * 180.0 / Math.PI; }

function mat4Id(){
  return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
}
function mat4Mul(a, b){
  const out = new Array(16);
  for(let r=0; r<4; r++){
    for(let c=0; c<4; c++){
      out[r*4 + c] = a[r*4 + 0]*b[0*4 + c] +
                     a[r*4 + 1]*b[1*4 + c] +
                     a[r*4 + 2]*b[2*4 + c] +
                     a[r*4 + 3]*b[3*4 + c];
    }
  }
  return out;
}

function craigMDH(a, alphaDeg, d, thetaDeg){
  const th = deg2rad(thetaDeg), al = deg2rad(alphaDeg);
  const ct = Math.cos(th), st = Math.sin(th);
  const ca = Math.cos(al), sa = Math.sin(al);
  return [
    ct,       -st,      0,    a,
    st * ca,   ct * ca, -sa, -d * sa,
    st * sa,   ct * sa,  ca,  d * ca,
    0,         0,        0,   1
  ];
}

function forwardKinematics(enc){
  const th = [
    enc[0],
    enc[1] - 90.0, 
    enc[2],
    enc[3],
    enc[4],
    enc[5]
  ];

  let T = mat4Id();
  T = mat4Mul(T, craigMDH(0.0, 0.0, D1, th[0]));
  const p_sh = [T[3], T[7], T[11]];

  T = mat4Mul(T, craigMDH(0.0, -90.0, 0.0, th[1]));

  T = mat4Mul(T, craigMDH(A2, 0.0, 0.0, th[2]));
  const p_el = [T[3], T[7], T[11]];

  const p_bend = [
    T[0]*A3 + T[3],
    T[4]*A3 + T[7],
    T[8]*A3 + T[11]
  ];

  T = mat4Mul(T, craigMDH(A3, -90.0, D4, th[3]));
  const p_wr = [T[3], T[7], T[11]];

  T = mat4Mul(T, craigMDH(0.0, 90.0, 0.0, th[4]));

  T = mat4Mul(T, craigMDH(0.0, -90.0, 0.0, th[5]));
  const p_tcp = [
    T[2]*D_TOOL + T[3],
    T[6]*D_TOOL + T[7],
    T[10]*D_TOOL + T[11]
  ];

  return {
    base: [0, 0, 0],
    shoulder: p_sh,
    elbow: p_el,
    fore_bend: p_bend,
    wrist: p_wr,
    tcp: p_tcp
  };
}

/* =============================================================================
   3. CLOSED-FORM PEN-DOWN IK SOLVER
============================================================================= */
function ikPenDown(tx, ty, tz){
  const cx = tx, cy = ty, cz = tz + D_TOOL;
  const t1 = Math.atan2(cy, cx);
  const r = Math.hypot(cx, cy);
  const h = cz - D1;
  const dist = Math.hypot(r, h);
  const maxR = A2 + L_FORE, minR = Math.abs(A2 - L_FORE);

  if(dist > maxR) return { ok: false, reason: "Vượt tầm" };
  if(dist < minR) return { ok: false, reason: "Vùng chết" };

  let cb = (A2*A2 + dist*dist - L_FORE*L_FORE) / (2.0 * A2 * dist);
  cb = Math.max(-1.0, Math.min(1.0, cb));
  const beta = Math.acos(cb);
  const gamma = Math.atan2(h, r);
  let best = null, bestJ3 = 999;

  for(const s of [1, -1]){
    const phi2 = gamma + s * beta;
    const t2 = -phi2;
    const rRem = r - A2 * Math.cos(phi2);
    const hRem = h - A2 * Math.sin(phi2);
    const phiPsi = Math.atan2(hRem, rRem);
    const q23 = -phiPsi - DELTA_RAD;
    const t3 = q23 - t2;

    const e1 = rad2deg(t1);
    const e2 = rad2deg(t2) + 90.0;
    const e3 = rad2deg(t3);
    const e5 = -rad2deg(q23);

    if(e1 < JOINT_LIMITS[0][0] || e1 > JOINT_LIMITS[0][1]) continue;
    if(e2 < JOINT_LIMITS[1][0] || e2 > JOINT_LIMITS[1][1]) continue;
    if(e3 < JOINT_LIMITS[2][0] || e3 > JOINT_LIMITS[2][1]) continue;
    if(e5 < JOINT_LIMITS[4][0] || e5 > JOINT_LIMITS[4][1]) continue;

    if(!best || Math.abs(e3) < bestJ3){
      best = [e1, e2, e3, 0.0, e5, 0.0];
      bestJ3 = Math.abs(e3);
    }
  }

  if(!best) return { ok: false, reason: "Ngoài giới hạn" };
  return { ok: true, angles: best };
}

/* =============================================================================
   4. FLAW & SAFETY DETECTOR
============================================================================= */
function auditPose(angles, lm){
  const flaws = [], warns = [];
  for(let i=0; i<6; i++){
    if(angles[i] < JOINT_LIMITS[i][0] - 0.1 || angles[i] > JOINT_LIMITS[i][1] + 0.1) flaws.push(`J${i+1} Limit`);
  }
  return { flaws, warns };
}

/* =============================================================================
   5. 3D & MULTI-VIEW CANVAS ENGINE
============================================================================= */
class Canvas3DRenderer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.viewMode = '3d';
    this.cam = { yaw: -45, pitch: 25, zoom: 1.15, panX: 0, panY: 0 };
    this.isDragging = false;
    this.lastX = 0;
    this.lastY = 0;
    this.initEvents();
  }

  initEvents() {
    const cv = this.canvas;
    cv.addEventListener('mousedown', e => {
      this.isDragging = true;
      this.lastX = e.clientX;
      this.lastY = e.clientY;
    });
    window.addEventListener('mousemove', e => {
      if(!this.isDragging) return;
      const dx = e.clientX - this.lastX, dy = e.clientY - this.lastY;
      this.lastX = e.clientX; this.lastY = e.clientY;
      if(this.viewMode === '3d') { this.cam.yaw += dx * 0.7; this.cam.pitch = Math.max(-85, Math.min(85, this.cam.pitch - dy * 0.7)); }
      else { this.cam.panX += dx; this.cam.panY += dy; }
      this.renderCurrent();
    });
    window.addEventListener('mouseup', () => { this.isDragging = false; });
    cv.addEventListener('wheel', e => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.08 : 0.92;
      this.cam.zoom = Math.max(0.4, Math.min(3.5, this.cam.zoom * factor));
      this.renderCurrent();
    }, { passive: false });
  }

  project(p) {
    const W = this.canvas.width, H = this.canvas.height;
    if(this.viewMode === 'side') return { u: W/2 + this.cam.panX + p[0]*this.cam.zoom, v: H*0.82 + this.cam.panY - p[2]*this.cam.zoom };
    if(this.viewMode === 'top') return { u: W/2 + this.cam.panX + p[0]*this.cam.zoom, v: H/2 + this.cam.panY - p[1]*this.cam.zoom };
    const ry = deg2rad(this.cam.yaw), rp = deg2rad(this.cam.pitch);
    const x1 = p[0]*Math.cos(ry) - p[1]*Math.sin(ry), y1 = p[0]*Math.sin(ry) + p[1]*Math.cos(ry), z1 = p[2];
    const y2 = y1*Math.cos(rp) - z1*Math.sin(rp), z2 = y1*Math.sin(rp) + z1*Math.cos(rp);
    return { u: W/2 + this.cam.panX + x1*this.cam.zoom, v: H*0.72 + this.cam.panY - z2*this.cam.zoom };
  }

  render(landmarks, pathPts) {
    this.lastLm = landmarks; this.lastPath = pathPts;
    const ctx = this.ctx, W = this.canvas.width, H = this.canvas.height;
    ctx.clearRect(0, 0, W, H);

    // 1. Grid & Floor
    if(this.viewMode === '3d'){
      ctx.strokeStyle = '#1e293b'; ctx.lineWidth = 1;
      for(let g = -250; g <= 250; g += 50){
        const p1 = this.project([g, -250, 0]), p2 = this.project([g, 250, 0]);
        ctx.beginPath(); ctx.moveTo(p1.u, p1.v); ctx.lineTo(p2.u, p2.v); ctx.stroke();
        const q1 = this.project([-250, g, 0]), q2 = this.project([250, g, 0]);
        ctx.beginPath(); ctx.moveTo(q1.u, q1.v); ctx.lineTo(q2.u, q2.v); ctx.stroke();
      }
    } else if(this.viewMode === 'side'){
      ctx.strokeStyle = '#1e293b'; ctx.lineWidth = 1;
      for(let g = -100; g <= 300; g += 50){
        const p1 = this.project([g, 0, -20]), p2 = this.project([g, 0, 400]);
        ctx.beginPath(); ctx.moveTo(p1.u, p1.v); ctx.lineTo(p2.u, p2.v); ctx.stroke();
      }
      for(let z = 0; z <= 400; z += 50){
        const p1 = this.project([-120, 0, z]), p2 = this.project([300, 0, z]);
        ctx.beginPath(); ctx.moveTo(p1.u, p1.v); ctx.lineTo(p2.u, p2.v); ctx.stroke();
      }
      const t1 = this.project([-120, 0, 0]), t2 = this.project([300, 0, 0]);
      ctx.strokeStyle = '#06b6d4'; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(t1.u, t1.v); ctx.lineTo(t2.u, t2.v); ctx.stroke();
    } else if(this.viewMode === 'top'){
      ctx.strokeStyle = '#1e293b'; ctx.lineWidth = 1;
      for(let g = -250; g <= 250; g += 50){
        const p1 = this.project([g, -250, 0]), p2 = this.project([g, 250, 0]);
        ctx.beginPath(); ctx.moveTo(p1.u, p1.v); ctx.lineTo(p2.u, p2.v); ctx.stroke();
        const q1 = this.project([-250, g, 0]), q2 = this.project([250, g, 0]);
        ctx.beginPath(); ctx.moveTo(q1.u, q1.v); ctx.lineTo(q2.u, q2.v); ctx.stroke();
      }
      const cMax = [];
      for(let a=0; a<=Math.PI*2+0.1; a+=0.2){
        cMax.push(this.project([(A2+L_FORE)*Math.cos(a), (A2+L_FORE)*Math.sin(a), 0]));
      }
      ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 1.2; ctx.setLineDash([4, 4]);
      ctx.beginPath(); cMax.forEach((p, i) => i === 0 ? ctx.moveTo(p.u, p.v) : ctx.lineTo(p.u, p.v)); ctx.stroke();
      ctx.setLineDash([]);
    }

    if(!landmarks) return;

    // 2. Trajectory Waypoints
    if(pathPts && pathPts.length > 0){
      ctx.strokeStyle = '#f43f5e'; ctx.lineWidth = 2.5;
      ctx.beginPath();
      let first = true;
      for(const wp of pathPts){
        if(wp.drawing){
          const p = this.project([wp.x, wp.y, wp.z]);
          if(first){ ctx.moveTo(p.u, p.v); first = false; } else { ctx.lineTo(p.u, p.v); }
        }
      }
      ctx.stroke();
    }

    // 3. Robot Arm Skeleton
    const pBase = this.project(landmarks.base), pSh = this.project(landmarks.shoulder), pEl = this.project(landmarks.elbow), pBend = this.project(landmarks.fore_bend), pWr = this.project(landmarks.wrist), pTcp = this.project(landmarks.tcp);
    ctx.strokeStyle = '#64748b'; ctx.lineWidth = 7; ctx.lineCap = 'round'; ctx.beginPath(); ctx.moveTo(pBase.u, pBase.v); ctx.lineTo(pSh.u, pSh.v); ctx.stroke();
    ctx.strokeStyle = '#0ea5e9'; ctx.lineWidth = 5.5; ctx.beginPath(); ctx.moveTo(pSh.u, pSh.v); ctx.lineTo(pEl.u, pEl.v); ctx.stroke();
    ctx.strokeStyle = '#10b981'; ctx.lineWidth = 4.5; ctx.beginPath(); ctx.moveTo(pEl.u, pEl.v); ctx.lineTo(pBend.u, pBend.v); ctx.stroke();
    ctx.strokeStyle = '#059669'; ctx.lineWidth = 4.5; ctx.beginPath(); ctx.moveTo(pBend.u, pBend.v); ctx.lineTo(pWr.u, pWr.v); ctx.stroke();
    ctx.strokeStyle = '#f43f5e'; ctx.lineWidth = 3.5; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(pWr.u, pWr.v); ctx.lineTo(pTcp.u, pTcp.v); ctx.stroke();
    ctx.setLineDash([]);

    // 4. Joint Markers
    const joints = [pSh, pEl, pBend, pWr];
    ctx.fillStyle = '#f59e0b'; ctx.strokeStyle = '#0f172a'; ctx.lineWidth = 1.5;
    for(const j of joints){
      ctx.beginPath(); ctx.arc(j.u, j.v, 4.5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }
    // TCP Pen Tip
    ctx.fillStyle = '#f43f5e'; ctx.beginPath(); ctx.arc(pTcp.u, pTcp.v, 5.5, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = 'rgba(244,63,94,0.3)'; ctx.beginPath(); ctx.arc(pTcp.u, pTcp.v, 11.0, 0, Math.PI * 2); ctx.fill();
  }

  renderCurrent() { this.render(this.lastLm, this.lastPath); }
}

/* =============================================================================
   6. STATE & APP CONTROLLER
============================================================================= */
let dashRenderer, simRenderer;
let simAngles = [0, 0, 0, 0, 0, 0];
let simSource = 'sim';
let simPathType = 'line';
let simWaypoints = [];
let simPlayTimer = null;
let simScrubIdx = 0;
let stepSize = 1.0, pollTimer = null, failN = 0, toastTimer = null, lastPose = null;

function initSimulationUI() {
  dashRenderer = new Canvas3DRenderer('poseCanvas');
  simRenderer = new Canvas3DRenderer('simCanvas');

  const jContainer = document.getElementById('simJointSliders');
  if(jContainer){
    jContainer.innerHTML = '';
    for(let i=0; i<6; i++){
      const lim = JOINT_LIMITS[i];
      jContainer.insertAdjacentHTML('beforeend', `
        <div class="sim-slider-row">
          <label>J${i+1} (${AXES[i].split(' ')[1]}):</label>
          <input type="range" id="simJ${i}" min="${lim[0]}" max="${lim[1]}" step="0.5" value="${simAngles[i]}" oninput="onSimJointChange(${i}, this.value)">
          <span class="val" id="simJ${i}Val">${simAngles[i].toFixed(1)}°</span>
        </div>
      `);
    }
  }

  generateSimPath();
  updateSimFromAngles(simAngles);
}

function setDashView(v, btn){
  dashRenderer.viewMode = v;
  if(btn){
    btn.parentElement.querySelectorAll('.sim-pill-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
  }
  dashRenderer.renderCurrent();
}
function resetDashCam(){
  dashRenderer.cam = { yaw: -45, pitch: 25, zoom: 1.15, panX: 0, panY: 0 };
  dashRenderer.renderCurrent();
}

function setSimView(v, btn){
  simRenderer.viewMode = v;
  if(btn){
    btn.parentElement.querySelectorAll('.sim-pill-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
  }
  simRenderer.renderCurrent();
}
function resetSimCam(){
  simRenderer.cam = { yaw: -45, pitch: 25, zoom: 1.15, panX: 0, panY: 0 };
  simRenderer.renderCurrent();
}

function setSimSource(src){
  simSource = src;
  document.getElementById('btnSrcLive').classList.toggle('active', src === 'live');
  document.getElementById('btnSrcSim').classList.toggle('active', src === 'sim');
  document.getElementById('simSourceLabel').textContent = src === 'live' ? '[GƯƠNG ROBOT THẬT (LIVE)]' : '[CHẾ ĐỘ MÔ PHỎNG]';
  if(src === 'live' && window.lastRobotAngles){
    updateSimFromAngles(window.lastRobotAngles);
  }
}

function updateSimFromAngles(angles){
  simAngles = [...angles];
  const lm = forwardKinematics(simAngles);
  const audit = auditPose(simAngles, lm);

  for(let i=0; i<6; i++){
    const slider = document.getElementById(`simJ${i}`);
    if(slider) slider.value = simAngles[i];
    const valText = document.getElementById(`simJ${i}Val`);
    if(valText) valText.textContent = simAngles[i].toFixed(1) + '°';
  }

  const elX = document.getElementById('simX'), elY = document.getElementById('simY'), elZ = document.getElementById('simZ');
  if(elX) elX.value = Math.round(lm.tcp[0]);
  if(elY) elY.value = Math.round(lm.tcp[1]);
  if(elZ) elZ.value = Math.round(lm.tcp[2]);
  const elXV = document.getElementById('simXVal'), elYV = document.getElementById('simYVal'), elZV = document.getElementById('simZVal');
  if(elXV) elXV.textContent = lm.tcp[0].toFixed(1) + ' mm';
  if(elYV) elYV.textContent = lm.tcp[1].toFixed(1) + ' mm';
  if(elZV) elZV.textContent = lm.tcp[2].toFixed(1) + ' mm';

  simRenderer.render(lm, simWaypoints);

  const statusBadge = audit.flaws.length === 0 && audit.warns.length === 0 ? '[OK: NORMAL]' :
                     (audit.flaws.length > 0 ? '[CRITICAL FAULT]' : '[WARNING]');
  let hudText = `TELEMETRY HUD | ${statusBadge}\n` +
                `TCP Position: X=${lm.tcp[0].toFixed(1)} mm  Y=${lm.tcp[1].toFixed(1)} mm  Z=${lm.tcp[2].toFixed(1)} mm  |  Wrist: X=${lm.wrist[0].toFixed(1)} Y=${lm.wrist[1].toFixed(1)} Z=${lm.wrist[2].toFixed(1)} mm\n` +
                `Joint Angles: J1:${simAngles[0].toFixed(1)}° | J2:${simAngles[1].toFixed(1)}° | J3:${simAngles[2].toFixed(1)}° | J4:${simAngles[3].toFixed(1)}° | J5:${simAngles[4].toFixed(1)}° | J6:${simAngles[5].toFixed(1)}°\n`;
  if(audit.flaws.length > 0) hudText += `FLAWS: ${audit.flaws.join(' ; ')}`;
  else if(audit.warns.length > 0) hudText += `WARNINGS: ${audit.warns.join(' ; ')}`;
  else hudText += `STATUS: Kinematics hợp lệ, an toàn không va chạm.`;

  const hudEl = document.getElementById('simHud');
  if(hudEl) hudEl.textContent = hudText;
}

function onSimJointChange(axis, val){
  if(simSource === 'live') setSimSource('sim');
  simAngles[axis] = parseFloat(val);
  updateSimFromAngles(simAngles);
}

function onSimCartChange(){
  if(simSource === 'live') setSimSource('sim');
  const x = parseFloat(document.getElementById('simX').value);
  const y = parseFloat(document.getElementById('simY').value);
  const z = parseFloat(document.getElementById('simZ').value);
  document.getElementById('simXVal').textContent = x.toFixed(1) + ' mm';
  document.getElementById('simYVal').textContent = y.toFixed(1) + ' mm';
  document.getElementById('simZVal').textContent = z.toFixed(1) + ' mm';

  const res = ikPenDown(x, y, z);
  const badge = document.getElementById('simIkBadge');
  if(res.ok){
    if(badge){ badge.textContent = 'IK OK'; badge.className = 'badge b-run'; }
    updateSimFromAngles(res.angles);
  } else {
    if(badge){ badge.textContent = 'OUT OF REACH'; badge.className = 'badge b-fault'; }
  }
}

function applySimPreset(type){
  setSimSource('sim');
  if(type === 'home'){
    updateSimFromAngles([0, 0, 0, 0, 0, 0]);
  } else if(type === 'draw'){
    const ik = ikPenDown(160, 0, 10);
    if(ik.ok) updateSimFromAngles(ik.angles);
  } else if(type === 'reach_fwd'){
    updateSimFromAngles([0, 90, 0, 0, -DELTA_WRIST, 0]);
  } else if(type === 'reach_back'){
    updateSimFromAngles([0, -90, 0, 0, -DELTA_WRIST, 0]);
  } else if(type === 'fold'){
    updateSimFromAngles([0, 45, 90, 0, -45, 0]);
  }
}

function generateSimPath(){
  simWaypoints = [];
  if(simPathType === 'circle'){
    const cx = 140, cy = 0, r = 45, z = 10;
    for(let a=0; a<=Math.PI*2; a+=0.15){
      simWaypoints.push({ x: cx + r*Math.cos(a), y: cy + r*Math.sin(a), z: z, drawing: true });
    }
  } else {
    const x1 = 100, y1 = -70, x2 = 180, y2 = 70, z = 10;
    for(let s=0; s<=1.0; s+=0.04){
      simWaypoints.push({ x: x1 + s*(x2-x1), y: y1 + s*(y2-y1), z: z, drawing: true });
    }
  }
  const scrubEl = document.getElementById('simScrub');
  if(scrubEl) scrubEl.max = Math.max(1, simWaypoints.length - 1);
}

function setSimPathType(type){
  simPathType = type;
  const bLine = document.getElementById('btnSimLine'), bCirc = document.getElementById('btnSimCirc');
  if(bLine) bLine.classList.toggle('active', type === 'line');
  if(bCirc) bCirc.classList.toggle('active', type === 'circle');
  generateSimPath();
  onSimScrubChange(0);
}

function onSimScrubChange(val){
  const idx = parseInt(val);
  simScrubIdx = idx;
  const scrubVal = document.getElementById('simScrubVal');
  if(scrubVal) scrubVal.textContent = `${Math.round(idx / Math.max(1, simWaypoints.length-1) * 100)}%`;
  if(simWaypoints[idx]){
    const wp = simWaypoints[idx];
    const ik = ikPenDown(wp.x, wp.y, wp.z);
    if(ik.ok) updateSimFromAngles(ik.angles);
  }
}

function toggleSimPlay(){
  const btn = document.getElementById('btnSimPlay');
  if(simPlayTimer){
    clearInterval(simPlayTimer);
    simPlayTimer = null;
    if(btn) btn.textContent = '▶ Play';
  } else {
    if(btn) btn.textContent = '⏸ Pause';
    simPlayTimer = setInterval(() => {
      simScrubIdx = (simScrubIdx + 1) % simWaypoints.length;
      const scrubEl = document.getElementById('simScrub');
      if(scrubEl) scrubEl.value = simScrubIdx;
      onSimScrubChange(simScrubIdx);
    }, 45);
  }
}

function sendSimPoseToRobot(){
  const lm = forwardKinematics(simAngles);
  post('/api/move', `x=${lm.tcp[0].toFixed(2)}&y=${lm.tcp[1].toFixed(2)}&z=${lm.tcp[2].toFixed(2)}&feed=30`);
}

/* =============================================================================
   7. STANDARD WEB APP HANDLERS & STATUS POLLER
============================================================================= */
function toast(msg, cls){
  const t = document.getElementById('toast');
  if(!t) return;
  t.textContent = msg;
  t.className = cls ? ('t-' + cls) : '';
  t.style.display = 'block';
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.style.display = 'none'; }, 2600);
}
function api(url){
  return fetch(url).then(r => r.text()).then(t => {
    toast((t === 'OK' ? '✓ ' : '') + t, t === 'OK' ? 'ok' : 'warn');
    return t;
  }).catch(() => toast('Lỗi mạng / mất kết nối', 'err'));
}
function post(url, body){
  return fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body })
    .then(r => r.text())
    .then(t => toast((t === 'OK' ? '✓ ' : '') + t, t === 'OK' ? 'ok' : 'warn'))
    .catch(() => toast('Lỗi mạng / mất kết nối', 'err'));
}
function clearFault(){ post('/api/jog', 'fault_clear=1'); }

document.querySelectorAll('.tab-btn[data-t]').forEach(b => {
  b.onclick = () => {
    document.querySelectorAll('.tab-btn').forEach(x => { x.classList.remove('active'); x.setAttribute('aria-selected', 'false'); });
    document.querySelectorAll('.tab-pane').forEach(x => x.classList.remove('active'));
    b.classList.add('active'); b.setAttribute('aria-selected', 'true');
    const target = document.getElementById(b.dataset.t);
    if(target) target.classList.add('active');
    if(b.dataset.t === 'sim' && simRenderer) simRenderer.renderCurrent();
    if(b.dataset.t === 'dash' && dashRenderer) dashRenderer.renderCurrent();
  };
});

[0.5, 1, 5, 15].forEach(s => {
  const b = document.createElement('button');
  b.className = 'stepbtn' + (s === stepSize ? ' active' : '');
  b.textContent = s + '°';
  b.onclick = () => {
    stepSize = s;
    document.querySelectorAll('.stepbtn').forEach(x => x.classList.remove('active'));
    b.classList.add('active');
    const sv = document.getElementById('stepVal'); if(sv) sv.textContent = s.toFixed(1).replace('.0','') + '°';
  };
  const stepSel = document.getElementById('stepSel');
  if(stepSel) stepSel.appendChild(b);
});

function buildCards(){
  const g = document.getElementById('jointGrid');
  const sg = document.getElementById('setHomeGrid');
  if(g) g.innerHTML = '';
  if(sg) sg.innerHTML = '';
  for(let i=0; i<6; i++){
    if(g) g.insertAdjacentHTML('beforeend',
     `<div class="jcard">
        <div class="jname"><span>${AXES[i]}</span><span class="jdeg" id="jd${i}">--</span></div>
        <div class="jenc" id="je${i}">encoder: --</div>
        <div class="jflags" id="jf${i}"></div>
        <div class="jctrl">
          <div class="jbtns">
            <button class="btn danger" onclick="jog(${i},-1)">&#8630; &minus;</button>
            <button class="btn primary" onclick="jog(${i},1)">+ &#8634;</button>
          </div>
        </div>
      </div>`);
    if(sg) sg.insertAdjacentHTML('beforeend',
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

function jog(axis, dir){ post('/api/jog', `axis=${axis}&deg=${dir * stepSize}`); }
function moveTo(){
  const b = `x=${document.getElementById('mvX').value||0}&y=${document.getElementById('mvY').value||0}&z=${document.getElementById('mvZ').value||0}&feed=${document.getElementById('mvFeed').value||30}`;
  post('/api/move', b);
}
function saveWifi(){
  const s = document.getElementById('wfSsid').value.trim(), p = document.getElementById('wfPass').value;
  if(!s){ toast('Nhập SSID', 'warn'); return; }
  post('/api/wifi', `ssid=${encodeURIComponent(s)}&pass=${encodeURIComponent(p)}`)
    .then(() => { document.getElementById('sub').textContent = 'Đã lưu. Đang restart...'; });
}
function startDraw(){
  const sh = document.getElementById('dwShape').value;
  const v = id => parseFloat(document.getElementById(id).value) || 0;
  let b = sh === 'line' ? `shape=line&x1=${v('dwA1')}&y1=${v('dwA2')}&x2=${v('dwA3')}&y2=${v('dwA4')}` : `shape=circle&cx=${v('dwA1')}&cy=${v('dwA2')}&r=${v('dwA3')}`;
  b += `&z=${v('dwZ')}&feed=${document.getElementById('dwFeed').value||20}`;
  post('/api/draw', b);
}
function previewShape(){
  const cv = document.getElementById('cv');
  if(!cv) return;
  const ctx = cv.getContext('2d'), W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);
  const sh = document.getElementById('dwShape').value;
  const v = id => parseFloat(document.getElementById(id).value) || 0;
  const ox = W/2, oy = H/2, sc = 0.8;
  ctx.strokeStyle = '#334155'; ctx.strokeRect(0, 0, W, H);
  ctx.strokeStyle = '#f43f5e'; ctx.lineWidth = 2;
  if(sh === 'line'){
    ctx.beginPath();
    ctx.moveTo(ox + v('dwA1')*sc, oy - v('dwA2')*sc);
    ctx.lineTo(ox + v('dwA3')*sc, oy - v('dwA4')*sc);
    ctx.stroke();
  } else {
    ctx.beginPath();
    ctx.arc(ox + v('dwA1')*sc, oy - v('dwA2')*sc, v('dwA3')*sc, 0, Math.PI*2);
    ctx.stroke();
  }
}

function updateUI(d){
  const m = d.mode || 'idle';
  const modeWord = document.getElementById('modeWord');
  const modeBadge = document.getElementById('mode');
  const map = { idle: { w: 'IDLE', c: 'b-idle' }, homing: { w: 'HOMING', c: 'b-run' }, jog: { w: 'JOG', c: 'b-run' }, cart: { w: 'CART', c: 'b-run' }, draw: { w: 'DRAW', c: 'b-run' }, fault: { w: 'FAULT', c: 'b-fault' } };
  const mm = map[m] || { w: m.toUpperCase(), c: 'b-idle' };
  if(modeWord) { modeWord.textContent = mm.w; modeWord.className = 'mode-word ' + (m === 'fault' ? 'fault' : (m !== 'idle' ? 'run' : '')); }
  if(modeBadge) { modeBadge.textContent = m; modeBadge.className = 'badge ' + mm.c; }

  const subEl = document.getElementById('sub');
  if(subEl) subEl.textContent = `${d.wifi.ssid || '(AP)'} · ${d.wifi.ip}`;
  let hn = 0; d.joints.forEach(j => hn += j.homed ? 1 : 0);
  const homedEl = document.getElementById('homedN');
  if(homedEl) homedEl.textContent = `${hn}/6`;
  const wifiInfo = document.getElementById('wifiInfo');
  if(wifiInfo) wifiInfo.textContent = `${(d.wifi.mode||'').toUpperCase()}${d.wifi.rssi ? (' · ' + d.wifi.rssi + ' dBm') : ''}`;

  document.querySelectorAll('.need-idle').forEach(b => { b.disabled = (d.busy || m === 'fault'); });

  const robotAngles = d.joints.map(j => j.deg);
  window.lastRobotAngles = robotAngles;

  // Render Dashboard 3D
  const lmLive = forwardKinematics(robotAngles);
  if(dashRenderer) dashRenderer.render(lmLive, null);
  const hudDash = `TCP: X=${lmLive.tcp[0].toFixed(1)} Y=${lmLive.tcp[1].toFixed(1)} Z=${lmLive.tcp[2].toFixed(1)} mm | Wrist: (${lmLive.wrist[0].toFixed(1)}, ${lmLive.wrist[1].toFixed(1)}, ${lmLive.wrist[2].toFixed(1)})`;
  const dashHudEl = document.getElementById('dashHud');
  if(dashHudEl) dashHudEl.textContent = hudDash;

  if(d.pose){
    const s = `(${d.pose.x.toFixed(1)}, ${d.pose.y.toFixed(1)}, ${d.pose.z.toFixed(1)})`;
    const p1 = document.getElementById('poseNow'); if(p1) p1.textContent = s;
    const p2 = document.getElementById('poseNow2'); if(p2) p2.textContent = s;
  }

  if(simSource === 'live'){
    updateSimFromAngles(robotAngles);
  }

  d.joints.forEach((j, i) => {
    const jd = document.getElementById('jd' + i); if(jd) jd.textContent = j.deg.toFixed(1) + '°';
    const je = document.getElementById('je' + i); if(je) je.textContent = 'encoder: ' + (j.encOK ? j.encDeg.toFixed(1) + '°' : 'MẤT KẾT NỐI');
    const jf = document.getElementById('jf' + i); if(jf){
      const f = [];
      f.push(`<span class="${j.homed ? 'f-h' : 'f-e'}">${j.homed ? (j.restored ? 'HOMED (NVS)' : 'HOMED') : 'CHƯA HOME'}</span>`);
      if(j.drift) f.push('<span class="f-d">DRIFT!</span>');
      if(!j.encOK) f.push('<span class="f-e">ENC ERR</span>');
      jf.innerHTML = f.join(' · ');
    }
  });
}

function setOnline(on){
  const c = document.getElementById('conn');
  if(c) c.classList.toggle('offline', !on);
  const s = document.getElementById('sub');
  if(!on && s) s.textContent = 'MẤT KẾT NỐI — đang thử lại...';
}
function pollOnce(){
  fetch('/api/status').then(r => r.json())
    .then(d => { failN = 0; setOnline(true); updateUI(d); })
    .catch(() => { if(++failN >= 3) setOnline(false); });
}

initSimulationUI();
setInterval(pollOnce, 300);
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
    if (z < -15.0f || z > 435.0f) { srv->send(400, "text/plain", "z out of range"); return; }
    ArmCommand c;
    c.type = ArmCommand::MOVE_CART;
    c.p[0] = x; c.p[1] = y; c.p[2] = z;
    c.p[5] = srv->hasArg("feed") ? srv->arg("feed").toFloat() : 30.0f;
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
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
        if (c.p[4] < -15.0f || c.p[4] > 435.0f) {
            srv->send(400, "text/plain", "z out of range");
            return;
        }
    } else if (shape == "circle") {
        c.type = ArmCommand::DRAW_CIRCLE;
        c.p[0] = srv->arg("cx").toFloat();
        c.p[1] = srv->arg("cy").toFloat();
        c.p[2] = srv->arg("z").toFloat();
        c.p[3] = srv->arg("r").toFloat();
        if (c.p[2] < -15.0f || c.p[2] > 435.0f) {
            srv->send(400, "text/plain", "z out of range");
            return;
        }
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
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleHomeAll() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::HOME_ALL;
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleHomeAxis() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    const int axis = srv->arg("axis").toInt();
    if (axis < 0 || axis >= 4) { srv->send(400, "text/plain", "axis 0..3 only"); return; }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::HOME_AXIS;
    c.axis = static_cast<uint8_t>(axis);
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleSetHome() {
    if (jointsPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    const int axis = srv->arg("axis").toInt();
    if (axis < 0 || axis >= NUM_MOTORS) { srv->send(400, "text/plain", "bad axis"); return; }
    if (armPtr != nullptr && armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::SET_HOME;
    c.axis = static_cast<uint8_t>(axis);
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
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
