#include "web_server.h"
#include <math.h>
#include "arm.h"
#include "config.h"
#include "joint_model.h"
#include "nvs_store.h"
#include "wifi_manager.h"
#include "work_plane.h"

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, viewport-fit=cover">
<title>6-Axis Robot Arm Controller &amp; Digital Twin</title>
<style>
:root {
  --bg: #090d16;
  --surface: #111827;
  --surface-sub: #162032;
  --surface-elevated: #1f293d;
  --border: #1f2a3d;
  --border-strong: #334155;
  --text-main: #f8fafc;
  --text-muted: #94a3b8;
  --text-dim: #64748b;
  --primary: #38bdf8;
  --primary-hover: #0ea5e9;
  --success: #10b981;
  --success-bg: rgba(16,185,129,0.12);
  --success-text: #6ee7b7;
  --danger: #ef4444;
  --danger-bg: rgba(239,68,68,0.14);
  --danger-text: #fca5a5;
  --warning: #f59e0b;
  --warning-bg: rgba(245,158,11,0.14);
  --warning-text: #fcd34d;
  --radius-xs: 4px;
  --radius-sm: 6px;
  --radius-md: 10px;
  --radius-lg: 14px;
  --radius-pill: 9999px;
  --font-sans: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
  --font-mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
::selection { background: var(--primary); color: var(--bg); }
html { color-scheme: dark; }
body {
  font-family: var(--font-sans);
  background: var(--bg);
  color: var(--text-main);
  min-height: 100vh;
  padding: 14px;
  padding-bottom: 84px;
  display: flex;
  flex-direction: column;
  align-items: center;
}
:focus-visible { outline: 2px solid var(--primary); outline-offset: 2px; }
.app { width: 100%; max-width: 1200px; }

/* ---- Header ---- */
.header {
  display: flex; align-items: center; justify-content: space-between; gap: 14px;
  padding: 6px 0 16px; flex-wrap: wrap; border-bottom: 1px solid var(--border);
  margin-bottom: 14px;
}
.brand { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
.brand h1 { font-size: 1.25rem; font-weight: 700; color: var(--text-main); letter-spacing: -0.02em; }
.brand h1 b { color: var(--primary); }
.brand .ver { font-size: 0.72rem; color: var(--text-muted); font-family: var(--font-mono); background: var(--surface-elevated); padding: 3px 8px; border-radius: var(--radius-xs); border: 1px solid var(--border); }
.conn-pill {
  display: flex; align-items: center; gap: 8px; font-size: 0.82rem; font-family: var(--font-mono);
  background: var(--surface); padding: 5px 12px; border-radius: var(--radius-pill); border: 1px solid var(--border);
}
.conn-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--success); }
.conn-pill.offline .conn-dot { background: var(--danger); }
.conn-pill.offline { color: var(--danger-text); border-color: var(--danger); }

/* ---- Main Navigation Tabs ---- */
.nav-tabs {
  display: flex; gap: 6px; background: var(--surface);
  padding: 5px; border-radius: var(--radius-md); margin-bottom: 16px;
  border: 1px solid var(--border); flex-wrap: wrap; justify-content: center;
}
.nav-tab {
  background: transparent; border: none; color: var(--text-muted);
  padding: 8px 16px; border-radius: var(--radius-sm); font-size: 0.86rem;
  font-weight: 600; cursor: pointer; min-height: 38px;
  display: inline-flex; align-items: center; gap: 6px;
  transition: background 150ms ease, color 150ms ease;
}
.nav-tab:hover { background: var(--surface-elevated); color: var(--text-main); }
.nav-tab.active { background: var(--primary); color: var(--bg); font-weight: 700; }
.tab-pane { display: none; }
.tab-pane.active { display: block; }

/* ---- Layout Grids ---- */
.grid-2col { display: grid; grid-template-columns: 1.25fr 0.75fr; gap: 14px; }
@media (max-width: 900px) { .grid-2col { grid-template-columns: 1fr; } }
.grid-cards-3 { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 12px; }

/* ---- Cards ---- */
.card {
  background: var(--surface); border-radius: var(--radius-lg);
  padding: 16px; border: 1px solid var(--border); margin-bottom: 14px;
}
.card-head {
  display: flex; align-items: center; justify-content: space-between; gap: 10px;
  margin-bottom: 12px; padding-bottom: 8px; border-bottom: 1px solid var(--border);
}
.card-head h2 { font-size: 0.95rem; font-weight: 600; color: var(--primary); display: flex; align-items: center; gap: 6px; }
.card-head .meta { font-size: 0.76rem; color: var(--text-muted); font-family: var(--font-mono); }

/* ---- 3D Viewport Box ---- */
.viewport-box {
  position: relative; background: #070a10; border: 1px solid var(--border);
  border-radius: var(--radius-md); overflow: hidden; margin-bottom: 10px;
}
.viewport-canvas { display: block; width: 100%; height: 380px; cursor: grab; }
.viewport-canvas:active { cursor: grabbing; }
.viewport-toolbar {
  position: absolute; top: 8px; left: 8px; right: 8px;
  display: flex; justify-content: space-between; align-items: center;
  pointer-events: none; gap: 6px; flex-wrap: wrap; z-index: 2;
}
.toolbar-group {
  display: flex; gap: 4px; background: rgba(17,24,39,0.88); backdrop-filter: blur(8px);
  padding: 3px; border-radius: var(--radius-sm); border: 1px solid var(--border-strong);
  pointer-events: auto;
}
.tool-btn {
  background: transparent; border: none; color: var(--text-muted);
  padding: 4px 10px; border-radius: var(--radius-xs); font-size: 0.74rem;
  font-weight: 600; cursor: pointer; min-height: 28px;
  display: inline-flex; align-items: center; justify-content: center;
  transition: all 120ms ease;
}
.tool-btn:hover { background: var(--surface-elevated); color: var(--text-main); }
.tool-btn.active { background: var(--primary); color: var(--bg); font-weight: 700; }
.tool-btn.live-badge.active { background: var(--success); color: var(--bg); }
.tool-btn.sim-badge.active { background: var(--warning); color: var(--bg); }

.hud-pill-bar {
  display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 8px;
  background: var(--surface-sub); padding: 10px 12px; border-radius: var(--radius-md);
  border: 1px solid var(--border); font-family: var(--font-mono); font-size: 0.78rem;
}
.hud-item { display: flex; flex-direction: column; gap: 2px; }
.hud-item .lbl { color: var(--text-dim); font-size: 0.68rem; text-transform: uppercase; font-family: var(--font-sans); }
.hud-item .val { color: var(--text-main); font-weight: 600; }
.hud-item .val.accent { color: var(--primary); }

/* ---- Status & Mode ---- */
.status-hero {
  display: flex; align-items: center; justify-content: space-between;
  background: var(--surface-sub); padding: 14px 16px; border-radius: var(--radius-md);
  border: 1px solid var(--border); margin-bottom: 12px; flex-wrap: wrap; gap: 10px;
}
.mode-title { font-size: 0.72rem; color: var(--text-dim); text-transform: uppercase; letter-spacing: 0.04em; }
.mode-text { font-size: 1.6rem; font-weight: 800; font-family: var(--font-mono); }
.mode-text.fault { color: var(--danger-text); }
.mode-text.run { color: var(--success-text); }
.badge {
  padding: 4px 10px; border-radius: var(--radius-pill); font-size: 0.72rem;
  font-weight: 700; text-transform: uppercase; letter-spacing: 0.04em;
}
.b-idle { background: rgba(56,189,248,0.14); color: var(--primary); border: 1px solid rgba(56,189,248,0.3); }
.b-run { background: var(--success-bg); color: var(--success-text); border: 1px solid rgba(16,185,129,0.3); }
.b-fault { background: var(--danger-bg); color: var(--danger-text); border: 1px solid rgba(239,68,68,0.3); }
.b-warn { background: var(--warning-bg); color: var(--warning-text); border: 1px solid rgba(245,158,11,0.3); }

/* ---- Stat Rows ---- */
.stat-row { display: flex; justify-content: space-between; align-items: center; font-size: 0.82rem; padding: 6px 0; border-bottom: 1px solid rgba(255,255,255,0.04); }
.stat-row:last-child { border-bottom: none; }
.stat-row .k { color: var(--text-muted); }
.stat-row .v { color: var(--text-main); font-family: var(--font-mono); font-weight: 600; text-align: right; }

/* ---- Homing Chips ---- */
.home-track { display: flex; gap: 6px; margin: 10px 0 14px; flex-wrap: wrap; }
.hchip {
  background: var(--surface-elevated); color: var(--text-dim); padding: 4px 12px;
  border-radius: var(--radius-pill); font-size: 0.72rem; font-weight: 700; font-family: var(--font-mono);
  border: 1px solid var(--border);
}
.hchip.active { background: var(--warning-bg); color: var(--warning-text); border-color: var(--warning); }
.hchip.done { background: var(--success-bg); color: var(--success-text); border-color: var(--success); }

/* ---- Buttons ---- */
.btn {
  border: none; border-radius: var(--radius-sm); padding: 9px 16px;
  font-weight: 600; font-size: 0.84rem; cursor: pointer; min-height: 38px;
  display: inline-flex; align-items: center; justify-content: center; gap: 6px;
  transition: background 150ms ease, opacity 150ms ease, transform 80ms ease;
}
.btn:active:not(:disabled) { transform: scale(0.97); }
.btn:disabled { opacity: 0.38; cursor: not-allowed; }
.btn-primary { background: var(--primary); color: var(--bg); }
.btn-primary:hover:not(:disabled) { background: var(--primary-hover); }
.btn-success { background: var(--success); color: #041f18; font-weight: 700; }
.btn-danger { background: var(--danger); color: #fff; }
.btn-warning { background: var(--warning); color: #261601; font-weight: 700; }
.btn-ghost { background: var(--surface-elevated); color: var(--text-main); border: 1px solid var(--border-strong); }
.btn-ghost:hover:not(:disabled) { background: var(--border-strong); }
.btn-block { width: 100%; }
.btn-row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin-top: 10px; }

/* ---- Jog Module Cards ---- */
.jog-header-bar {
  display: flex; align-items: center; justify-content: space-between;
  background: var(--surface); padding: 10px 14px; border-radius: var(--radius-md);
  border: 1px solid var(--border); margin-bottom: 12px; flex-wrap: wrap; gap: 10px;
}
.step-selector { display: flex; gap: 4px; background: var(--surface-sub); padding: 3px; border-radius: var(--radius-sm); border: 1px solid var(--border); }
.step-btn {
  background: transparent; border: none; color: var(--text-muted);
  padding: 5px 12px; border-radius: var(--radius-xs); font-size: 0.78rem; font-weight: 600; cursor: pointer; min-height: 30px;
}
.step-btn.active { background: var(--primary); color: var(--bg); font-weight: 700; }

.jcard {
  background: var(--surface-sub); border: 1px solid var(--border);
  border-radius: var(--radius-md); padding: 12px; display: flex; flex-direction: column; gap: 8px;
}
.jcard-top { display: flex; justify-content: space-between; align-items: baseline; }
.jcard-name { font-weight: 700; font-size: 0.88rem; color: var(--text-main); }
.jcard-deg { font-size: 1.25rem; font-family: var(--font-mono); font-weight: 700; color: var(--primary); }
.jcard-enc { font-size: 0.72rem; color: var(--text-muted); font-family: var(--font-mono); }
.jcard-flags { display: flex; gap: 6px; font-size: 0.68rem; font-weight: 600; flex-wrap: wrap; }
.flag-homed { color: var(--success-text); }
.flag-unhomed { color: var(--text-dim); }
.flag-drift { color: var(--danger-text); background: var(--danger-bg); padding: 1px 4px; border-radius: 3px; }
.flag-encerr { color: var(--warning-text); background: var(--warning-bg); padding: 1px 4px; border-radius: 3px; }

.jog-controls { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-top: 4px; }
.jog-btn {
  min-height: 42px; font-size: 1.1rem; font-weight: 700; border-radius: var(--radius-sm);
  background: var(--surface-elevated); border: 1px solid var(--border-strong); color: var(--text-main);
  cursor: pointer; display: flex; align-items: center; justify-content: center;
  transition: all 100ms ease;
}
.jog-btn:hover:not(:disabled) { border-color: var(--primary); color: var(--primary); }
.jog-btn:active:not(:disabled) { transform: scale(0.96); }
.jog-calib-row { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-top: 2px; }
.jog-calib-row .btn { min-height: 30px; font-size: 0.72rem; padding: 4px 8px; }

/* ---- Cartesian & Draw Studio ---- */
.mode-toggle-bar {
  display: flex; gap: 6px; background: var(--surface-sub);
  padding: 4px; border-radius: var(--radius-md); border: 1px solid var(--border); margin-bottom: 12px;
}
.mode-toggle-btn {
  flex: 1; background: transparent; border: none; color: var(--text-muted);
  padding: 8px 12px; border-radius: var(--radius-sm); font-size: 0.84rem; font-weight: 600;
  cursor: pointer; text-align: center;
}
.mode-toggle-btn.active { background: var(--surface-elevated); color: var(--primary); font-weight: 700; border: 1px solid var(--border-strong); }

.slider-row {
  display: grid; grid-template-columns: 85px 1fr 65px; align-items: center; gap: 8px;
  margin-bottom: 8px; font-size: 0.8rem;
}
.slider-row label { color: var(--text-muted); font-weight: 600; }
.slider-row input[type="range"] { accent-color: var(--primary); width: 100%; cursor: pointer; }
.slider-row .val { color: var(--primary); font-family: var(--font-mono); font-weight: 700; text-align: right; }

.input-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(80px, 1fr)); gap: 8px; margin-bottom: 10px; }
.input-field { display: flex; flex-direction: column; gap: 3px; }
.input-field label { font-size: 0.70rem; color: var(--text-dim); text-transform: uppercase; font-weight: 600; }
.input-field input, .input-field select {
  width: 100%; background: var(--surface-elevated); border: 1px solid var(--border-strong);
  color: var(--text-main); font-size: 0.88rem; padding: 7px 10px; border-radius: var(--radius-sm);
  outline: none; font-family: var(--font-mono);
}
.input-field input:focus, .input-field select:focus { border-color: var(--primary); }

.preset-pills { display: flex; gap: 6px; flex-wrap: wrap; margin-bottom: 12px; }
.preset-pill {
  background: var(--surface-elevated); border: 1px solid var(--border); color: var(--text-muted);
  padding: 5px 10px; border-radius: var(--radius-pill); font-size: 0.74rem; font-weight: 600; cursor: pointer;
}
.preset-pill:hover { color: var(--text-main); border-color: var(--border-strong); }

/* ---- Floating E-STOP & Toast ---- */
#estop {
  position: fixed; bottom: 18px; right: 18px; z-index: 99;
  padding: 14px 24px; font-size: 1.05rem; font-weight: 800; border-radius: var(--radius-pill);
  box-shadow: 0 4px 20px rgba(239,68,68,0.5); letter-spacing: 0.04em;
}
#toast {
  position: fixed; bottom: 18px; left: 18px; z-index: 98;
  background: var(--surface-elevated); border: 1px solid var(--border-strong);
  border-radius: var(--radius-md); padding: 10px 18px; font-size: 0.84rem;
  color: var(--text-main); display: none; max-width: 80vw;
  box-shadow: 0 8px 24px rgba(0,0,0,0.5);
}
#toast.t-ok { border-color: var(--success); background: #0c1a17; color: var(--success-text); }
#toast.t-warn { border-color: var(--warning); background: #1c1917; color: var(--warning-text); }
#toast.t-err { border-color: var(--danger); background: #7f1d1d; color: var(--danger-text); }

@media (pointer: coarse) {
  .nav-tab { min-height: 44px; padding: 10px 18px; font-size: 0.9rem; }
  .btn { min-height: 44px; }
  .jog-btn { min-height: 48px; }
}
@media (prefers-reduced-motion: reduce) {
  * { transition-duration: 0.01ms !important; animation-duration: 0.01ms !important; }
}
</style>
</head>
<body>
<main class="app" id="main-content">
  <!-- Header -->
  <header class="header">
    <div class="brand">
      <h1>6-Axis <b>Arm Controller</b></h1>
      <span class="ver">NEMA-6AXIS DIGITAL TWIN</span>
    </div>
    <div class="conn-pill" id="connPill">
      <span class="conn-dot"></span>
      <span id="connLabel">Đang kết nối...</span>
    </div>
  </header>

  <!-- 4 Core Navigation Tabs -->
  <nav class="nav-tabs" role="tablist" aria-label="Menu điều khiển cánh tay robot">
    <button id="tab-dash" class="nav-tab active" data-t="pane-dash" role="tab" aria-selected="true" aria-controls="pane-dash">📊 Dashboard</button>
    <button id="tab-jog" class="nav-tab" data-t="pane-jog" role="tab" aria-selected="false" aria-controls="pane-jog">🕹️ Jog &amp; Calib</button>
    <button id="tab-motion" class="nav-tab" data-t="pane-motion" role="tab" aria-selected="false" aria-controls="pane-motion">🎯 Cartesian &amp; Draw Studio</button>
    <button id="tab-settings" class="nav-tab" data-t="pane-settings" role="tab" aria-selected="false" aria-controls="pane-settings">⚙️ Settings &amp; WiFi</button>
  </nav>

  <!-- =========================================================================
       TAB 1: DASHBOARD (Live Twin & System Health)
  ========================================================================== -->
  <div id="pane-dash" class="tab-pane active" role="tabpanel" aria-labelledby="tab-dash">
    <div class="grid-2col">
      <!-- Left Column: 3D Digital Twin -->
      <section class="card" aria-labelledby="headDashTwin">
        <div class="card-head">
          <h2 id="headDashTwin">🦾 Mô hình 3D Digital Twin (Thời gian thực)</h2>
          <span class="meta" id="dashPoseLabel">TCP: (0.0, 0.0, 0.0)</span>
        </div>
        <div class="viewport-box">
          <div class="viewport-toolbar">
            <div class="toolbar-group">
              <button class="tool-btn active" onclick="setDashView('3d', this)">3D Orbit</button>
              <button class="tool-btn" onclick="setDashView('side', this)">Side (X-Z)</button>
              <button class="tool-btn" onclick="setDashView('top', this)">Top (X-Y)</button>
            </div>
            <div class="toolbar-group">
              <button class="tool-btn" onclick="resetDashCam()">Reset View</button>
            </div>
          </div>
          <canvas id="dashCanvas" class="viewport-canvas" width="640" height="380" role="img" aria-label="Mô hình 3D cánh tay robot"></canvas>
        </div>
        <div class="hud-pill-bar">
          <div class="hud-item"><span class="lbl">TCP Target (X, Y, Z)</span><span class="val accent" id="hudDashTcp">-- mm</span></div>
          <div class="hud-item"><span class="lbl">Wrist Center</span><span class="val" id="hudDashWrist">-- mm</span></div>
          <div class="hud-item"><span class="lbl">Trạng thái động học</span><span class="val" id="hudDashStatus">Bình thường</span></div>
        </div>
      </section>

      <!-- Right Column: Status & Quick Actions -->
      <div>
        <section class="card" aria-labelledby="headSysState">
          <div class="card-head">
            <h2 id="headSysState">⚡ Trạng thái hệ thống</h2>
            <span class="meta">300 ms poll</span>
          </div>
          <div class="status-hero">
            <div>
              <div class="mode-title">Chế độ vận hành</div>
              <div class="mode-text" id="dashModeText">IDLE</div>
            </div>
            <span id="dashModeBadge" class="badge b-idle">idle</span>
          </div>

          <div class="stat-row"><span class="k">Mạng WiFi</span><span class="v" id="dashWifiInfo">-</span></div>
          <div class="stat-row"><span class="k">Khớp đã Home</span><span class="v" id="dashHomedCount">-/6</span></div>
          <div class="stat-row"><span class="k">Công tắc Endstop</span><span class="v" id="dashEsInfo">Tất cả mở</span></div>
          <div class="stat-row"><span class="k">Tiến độ Homing</span><span class="v" id="dashHomingProg">Sẵn sàng</span></div>

          <div class="home-track" aria-label="Trạng thái Homing từng trục">
            <span class="hchip" id="hc0">J1</span>
            <span class="hchip" id="hc1">J2</span>
            <span class="hchip" id="hc2">J3</span>
            <span class="hchip" id="hc3">J4</span>
            <span class="hchip" id="hc4">J5 (NVS)</span>
            <span class="hchip" id="hc5">J6 (NVS)</span>
          </div>

          <div class="btn-row">
            <button class="btn btn-primary need-idle" onclick="api('/api/home/all')">🚀 HOME ALL (J1-J4)</button>
            <button class="btn btn-warning" onclick="api('/api/stop')">⏹ STOP ALL</button>
            <button class="btn btn-ghost" onclick="clearFault()">CLEAR FAULT</button>
          </div>
        </section>

        <section class="card" aria-labelledby="headQuickJoints">
          <div class="card-head">
            <h2 id="headQuickJoints">📐 Góc khớp tức thời</h2>
            <span class="meta">Step vs Enc</span>
          </div>
          <div id="dashJointRows"></div>
        </section>
      </div>
    </div>
  </div>

  <!-- =========================================================================
       TAB 2: JOG & CALIBRATION (Unified Joints & Homing)
  ========================================================================== -->
  <div id="pane-jog" class="tab-pane" role="tabpanel" aria-labelledby="tab-jog">
    <div class="jog-header-bar">
      <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
        <span style="font-size:0.82rem;font-weight:600;color:var(--text-muted)">Bước Jog:</span>
        <div class="step-selector" id="stepSelector"></div>
      </div>
      <div style="display:flex;gap:6px;flex-wrap:wrap">
        <button class="btn btn-ghost need-idle" onclick="api('/api/home/axis?axis=0')">Home J1</button>
        <button class="btn btn-ghost need-idle" onclick="api('/api/home/axis?axis=1')">Home J2</button>
        <button class="btn btn-ghost need-idle" onclick="api('/api/home/axis?axis=2')">Home J3</button>
        <button class="btn btn-ghost need-idle" onclick="api('/api/home/axis?axis=3')">Home J4</button>
        <button class="btn btn-ghost need-idle" onclick="api('/api/sethome?axis=4')">Set Home J5</button>
        <button class="btn btn-ghost need-idle" onclick="api('/api/sethome?axis=5')">Set Home J6</button>
        <button class="btn btn-ghost need-idle" onclick="api('/api/sethome?axis=255')">Set Home J5+J6</button>
        <button class="btn btn-primary need-idle" onclick="api('/api/home/all')">🚀 HOME ALL (J1-J4)</button>
        <button class="btn btn-warning" onclick="clearFault()">🛡️ CLEAR FAULT</button>
        <button class="btn btn-danger" onclick="api('/api/stop')">⏹ STOP ALL</button>
      </div>
    </div>

    <div class="grid-cards-3" id="jointCardsGrid"></div>
  </div>

  <!-- =========================================================================
       TAB 3: CARTESIAN & DRAW STUDIO (Unified Motion & Trajectory Studio)
  ========================================================================== -->
  <div id="pane-motion" class="tab-pane" role="tabpanel" aria-labelledby="tab-motion">
    <div class="grid-2col">
      <!-- Left Column: Interactive Simulation Twin -->
      <section class="card" aria-labelledby="headSimTwin">
        <div class="card-head">
          <h2 id="headSimTwin">🎮 3D Trajectory &amp; IK Simulation Twin</h2>
          <span class="badge b-run" id="simIkStatusBadge">IK OK</span>
        </div>
        <div class="viewport-box">
          <div class="viewport-toolbar">
            <div class="toolbar-group">
              <button class="tool-btn active" onclick="setSimView('3d', this)">3D Orbit</button>
              <button class="tool-btn" onclick="setSimView('side', this)">Side (X-Z)</button>
              <button class="tool-btn" onclick="setSimView('top', this)">Top (X-Y)</button>
            </div>
            <div class="toolbar-group">
              <button class="tool-btn live-badge" id="btnSimLive" onclick="setSimSource('live')">🔴 Live Robot</button>
              <button class="tool-btn sim-badge active" id="btnSimSim" onclick="setSimSource('sim')">🟢 Interactive Sim</button>
              <button class="tool-btn" onclick="resetSimCam()">Reset Cam</button>
            </div>
          </div>
          <canvas id="simCanvas" class="viewport-canvas" width="680" height="380" role="img" aria-label="Mô phỏng quỹ đạo 3D"></canvas>
        </div>

        <div class="hud-pill-bar">
          <div class="hud-item"><span class="lbl">Sim TCP (X, Y, Z)</span><span class="val accent" id="hudSimTcp">-- mm</span></div>
          <div class="hud-item"><span class="lbl">Reach Status</span><span class="val" id="hudSimReach">Hợp lệ trong tầm với</span></div>
          <div class="hud-item"><span class="lbl">Playback</span><span class="val" id="hudSimPlay">0%</span></div>
        </div>

        <div style="margin-top:10px;display:flex;align-items:center;gap:8px">
          <button class="btn btn-ghost" id="btnSimPlay" onclick="toggleSimPlay()">▶ Chạy mô phỏng (Play)</button>
          <input type="range" id="simScrub" min="0" max="100" value="0" style="flex:1;accent-color:var(--primary);cursor:pointer" oninput="onSimScrubChange(this.value)">
          <span class="meta" id="simScrubVal" style="min-width:38px;text-align:right">0%</span>
        </div>
      </section>

      <!-- Right Column: Dual Motion Panels (Direct Move / Shape Draw) -->
      <div>
        <div class="mode-toggle-bar">
          <button class="mode-toggle-btn active" id="btnToggleMove" onclick="switchMotionMode('move')">📍 Di chuyển điểm (Move IK)</button>
          <button class="mode-toggle-btn" id="btnToggleDraw" onclick="switchMotionMode('draw')">✏️ Vẽ hình (Draw Shape)</button>
        </div>

        <!-- Panel A: Direct Cartesian Move -->
        <section class="card" id="panelCartMove" aria-labelledby="headCartMove">
          <div class="card-head">
            <h2 id="headCartMove">📍 Di chuyển Cartesian TCP</h2>
            <span class="meta">Bút vuông góc mặt bàn</span>
          </div>
          <div class="preset-pills">
            <span class="preset-pill" onclick="applySimPreset('home')">Home (0°)</span>
            <span class="preset-pill" onclick="applySimPreset('draw')">Ready Draw</span>
            <span class="preset-pill" onclick="applySimPreset('reach_fwd')">Reach +X</span>
            <span class="preset-pill" onclick="applySimPreset('reach_back')">Reach -X</span>
            <span class="preset-pill" onclick="applySimPreset('fold')">Folded</span>
          </div>

          <div class="slider-row">
            <label for="simX">Target X:</label>
            <input type="range" id="simX" min="-250" max="250" step="1" value="160" oninput="onSimCartChange()">
            <span class="val" id="simXVal">160 mm</span>
          </div>
          <div class="slider-row">
            <label for="simY">Target Y:</label>
            <input type="range" id="simY" min="-250" max="250" step="1" value="0" oninput="onSimCartChange()">
            <span class="val" id="simYVal">0 mm</span>
          </div>
          <div class="slider-row">
            <label for="simZ">Target Z:</label>
            <input type="range" id="simZ" min="-10" max="350" step="1" value="10" oninput="onSimCartChange()">
            <span class="val" id="simZVal">10 mm</span>
          </div>

          <div class="input-grid" style="margin-top:10px">
            <div class="input-field"><label for="mvFeed">Tốc độ Feed (mm/s)</label><input type="number" id="mvFeed" value="30"></div>
          </div>

          <div class="btn-row">
            <button class="btn btn-primary btn-block need-idle" onclick="sendSimPoseToRobot()">⚡ NẠP VỊ TRÍ XUỐNG ROBOT</button>
          </div>
        </section>

        <!-- Panel B: Shape Drawing Studio -->
        <section class="card" id="panelCartDraw" style="display:none" aria-labelledby="headCartDraw">
          <div class="card-head">
            <h2 id="headCartDraw">✏️ Vẽ hình trên mặt giấy</h2>
            <span class="meta">Line / Circle Planner</span>
          </div>
          <div class="input-field" style="margin-bottom:10px">
            <label for="dwShape">Chọn hình vẽ</label>
            <select id="dwShape" onchange="onDrawShapeChange()">
              <option value="line">Line (Đoạn thẳng)</option>
              <option value="circle">Circle (Đường tròn)</option>
            </select>
          </div>

          <div class="input-grid">
            <div class="input-field"><label id="dwA1Lbl" for="dwA1">X1 (mm)</label><input type="number" id="dwA1" value="100"></div>
            <div class="input-field"><label id="dwA2Lbl" for="dwA2">Y1 (mm)</label><input type="number" id="dwA2" value="-70"></div>
            <div class="input-field"><label id="dwA3Lbl" for="dwA3">X2 (mm)</label><input type="number" id="dwA3" value="180"></div>
            <div class="input-field" id="dwA4Group"><label id="dwA4Lbl" for="dwA4">Y2 (mm)</label><input type="number" id="dwA4" value="70"></div>
            <div class="input-field"><label for="dwZ">Z Giấy (mm)</label><input type="number" id="dwZ" value="10"></div>
            <div class="input-field"><label for="dwFeed">Feed (mm/s)</label><input type="number" id="dwFeed" value="20"></div>
          </div>

          <div class="btn-row">
            <button class="btn btn-success need-idle" onclick="startDraw()" style="flex:1">✏️ BẮT ĐẦU VẼ (START DRAW)</button>
            <button class="btn btn-ghost" onclick="syncDrawToSim()">Xem trước 3D</button>
            <button class="btn btn-danger" onclick="api('/api/stop')">DỪNG (ABORT)</button>
          </div>
        </section>

        <!-- Panel C: Fine Joint Sliders Fold -->
        <section class="card" aria-labelledby="headSimJointSliders">
          <div class="card-head">
            <h2 id="headSimJointSliders">🎛️ Tinh chỉnh từng khớp (Sim Angles)</h2>
          </div>
          <div id="simJointSliders"></div>
        </section>
      </div>
    </div>
  </div>

  <!-- =========================================================================
       TAB 4: SETTINGS & WIFI
  ========================================================================== -->
  <div id="pane-settings" class="tab-pane" role="tabpanel" aria-labelledby="tab-settings">
    <div class="grid-2col">
      <section class="card" aria-labelledby="headWifiCfg">
        <div class="card-head">
          <h2 id="headWifiCfg">📶 Cấu hình Mạng WiFi (NVS)</h2>
        </div>
        <div class="stat-row"><span class="k">Trạng thái kết nối</span><span class="v" id="wfModeText">STA OK</span></div>
        <div class="stat-row"><span class="k">Địa chỉ IP hiện tại</span><span class="v" id="wfIpText">192.168.1.2</span></div>
        <div class="stat-row"><span class="k">Mạng đang kết nối</span><span class="v" id="wfSsidNowText">-</span></div>
        <div class="stat-row"><span class="k">Cường độ sóng RSSI</span><span class="v" id="wfRssiText">- dBm</span></div>

        <div style="margin-top:14px">
          <div class="input-field" style="margin-bottom:8px">
            <label for="wfSsid">Tên mạng WiFi mới (SSID)</label>
            <input type="text" id="wfSsid" placeholder="Nhập tên WiFi..." autocomplete="off">
          </div>
          <div class="input-field" style="margin-bottom:12px">
            <label for="wfPass">Mật khẩu WiFi mới</label>
            <input type="password" id="wfPass" placeholder="Nhập mật khẩu..." autocomplete="current-password">
          </div>
          <button class="btn btn-success btn-block" onclick="saveWifi()">💾 LƯU VÀO NVS &amp; KHỞI ĐỘNG LẠI</button>
        </div>
      </section>

      <section class="card" aria-labelledby="headHardwareRef">
        <div class="card-head">
          <h2 id="headHardwareRef">⚙️ Thông số Động học &amp; Phần cứng</h2>
          <span class="meta">Craig MDH</span>
        </div>
        <div class="stat-row"><span class="k">D1 (Trụ đế)</span><span class="v">139.0 mm</span></div>
        <div class="stat-row"><span class="k">A2 (Cánh tay dưới)</span><span class="v">138.0 mm</span></div>
        <div class="stat-row"><span class="k">A3 + D4 (Cẳng tay)</span><span class="v">88.0 + 126.0 mm (L=153.7mm)</span></div>
        <div class="stat-row"><span class="k">D6 + Tool (Bút vẽ)</span><span class="v">31.0 + 20.0 mm (D_eff=51.0mm)</span></div>
        <div class="stat-row"><span class="k">Bộ truyền động</span><span class="v">J1-J4: TMC2209 | J5-J6: A4988</span></div>
        <div class="stat-row"><span class="k">Cảm biến vị trí</span><span class="v">AS5600 × 6 (PCA9548A I2C)</span></div>
      </section>
    </div>
  </div>
</main>

<button id="estop" class="btn btn-danger" onclick="api('/api/stop')" aria-label="Dừng khẩn cấp toàn bộ cánh tay robot">⚠️ E-STOP</button>
<div id="toast" role="status" aria-live="assertive"></div>

<script>
/* =============================================================================
   1. HARDWARE CONSTANTS & CRAIG MODIFIED DH PARAMETERS (src/config.h)
============================================================================= */
const D1 = 139.0, A2 = 138.0, A3 = 88.0, D4 = 126.0, D6 = 31.0, D_TOOL = 20.0, D_TOOL_EFF = 51.0;
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

const THEME_PALETTE = {
  grid: '#1f2937',
  axisPlane: '#0284c7',
  linkBase: '#475569',
  linkUpper: '#0ea5e9',
  linkFore1: '#10b981',
  linkFore2: '#059669',
  penTool: '#f43f5e',
  jointFill: '#f59e0b',
  jointBorder: '#0b0f19',
  reachCircle: '#38bdf8'
};

/* =============================================================================
   2. MATRIX MATH & FORWARD KINEMATICS
============================================================================= */
function deg2rad(d){ return d * Math.PI / 180.0; }
function rad2deg(r){ return r * 180.0 / Math.PI; }

function mat4Id(){ return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]; }
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
  const th = [ enc[0], enc[1] - 90.0, enc[2], enc[3], enc[4], enc[5] ];
  let T = mat4Id();
  T = mat4Mul(T, craigMDH(0.0, 0.0, D1, th[0]));
  const p_sh = [T[3], T[7], T[11]];
  T = mat4Mul(T, craigMDH(0.0, -90.0, 0.0, th[1]));
  T = mat4Mul(T, craigMDH(A2, 0.0, 0.0, th[2]));
  const p_el = [T[3], T[7], T[11]];
  const p_bend = [ T[0]*A3 + T[3], T[4]*A3 + T[7], T[8]*A3 + T[11] ];
  T = mat4Mul(T, craigMDH(A3, -90.0, D4, th[3]));
  const p_wr = [T[3], T[7], T[11]];
  T = mat4Mul(T, craigMDH(0.0, 90.0, 0.0, th[4]));
  T = mat4Mul(T, craigMDH(0.0, -90.0, D6, th[5]));
  const p_tcp = [ T[2]*D_TOOL + T[3], T[6]*D_TOOL + T[7], T[10]*D_TOOL + T[11] ];
  return { base: [0, 0, 0], shoulder: p_sh, elbow: p_el, fore_bend: p_bend, wrist: p_wr, tcp: p_tcp };
}

/* =============================================================================
   3. CLOSED-FORM PEN-DOWN IK SOLVER
============================================================================= */
function ikPenDown(tx, ty, tz){
  const cx = tx, cy = ty, cz = tz + D_TOOL_EFF;
  const t1 = Math.atan2(cy, cx);
  const r = Math.hypot(cx, cy);
  const h = cz - D1;
  const dist = Math.hypot(r, h);
  const maxR = A2 + L_FORE, minR = Math.abs(A2 - L_FORE);

  if(dist > maxR) return { ok: false, reason: "Vượt tầm với tối đa" };
  if(dist < minR) return { ok: false, reason: "Vào vùng chết cơ học" };

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

  if(!best) return { ok: false, reason: "Ngoài giới hạn góc quay khớp" };
  return { ok: true, angles: best };
}

/* =============================================================================
   4. 3D CANVAS RENDERER ENGINE
============================================================================= */
class Canvas3DRenderer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas ? this.canvas.getContext('2d') : null;
    this.viewMode = '3d';
    this.cam = { yaw: -45, pitch: 25, zoom: 1.15, panX: 0, panY: 0 };
    this.isDragging = false;
    this.lastX = 0;
    this.lastY = 0;
    if(this.canvas) this.initEvents();
  }

  initEvents() {
    const cv = this.canvas;
    cv.addEventListener('mousedown', e => {
      this.isDragging = true;
      this.lastX = e.clientX; this.lastY = e.clientY;
    });
    window.addEventListener('mousemove', e => {
      if(!this.isDragging) return;
      const dx = e.clientX - this.lastX, dy = e.clientY - this.lastY;
      this.lastX = e.clientX; this.lastY = e.clientY;
      if(this.viewMode === '3d') {
        this.cam.yaw += dx * 0.7;
        this.cam.pitch = Math.max(-85, Math.min(85, this.cam.pitch - dy * 0.7));
      } else {
        this.cam.panX += dx; this.cam.panY += dy;
      }
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
    if(!this.ctx) return;
    this.lastLm = landmarks; this.lastPath = pathPts;
    const ctx = this.ctx, W = this.canvas.width, H = this.canvas.height;
    ctx.clearRect(0, 0, W, H);

    // 1. Grid & Floor
    if(this.viewMode === '3d'){
      ctx.strokeStyle = THEME_PALETTE.grid; ctx.lineWidth = 1;
      for(let g = -250; g <= 250; g += 50){
        const p1 = this.project([g, -250, 0]), p2 = this.project([g, 250, 0]);
        ctx.beginPath(); ctx.moveTo(p1.u, p1.v); ctx.lineTo(p2.u, p2.v); ctx.stroke();
        const q1 = this.project([-250, g, 0]), q2 = this.project([250, g, 0]);
        ctx.beginPath(); ctx.moveTo(q1.u, q1.v); ctx.lineTo(q2.u, q2.v); ctx.stroke();
      }
    } else if(this.viewMode === 'side'){
      ctx.strokeStyle = THEME_PALETTE.grid; ctx.lineWidth = 1;
      for(let g = -100; g <= 300; g += 50){
        const p1 = this.project([g, 0, -20]), p2 = this.project([g, 0, 400]);
        ctx.beginPath(); ctx.moveTo(p1.u, p1.v); ctx.lineTo(p2.u, p2.v); ctx.stroke();
      }
      for(let z = 0; z <= 400; z += 50){
        const p1 = this.project([-120, 0, z]), p2 = this.project([300, 0, z]);
        ctx.beginPath(); ctx.moveTo(p1.u, p1.v); ctx.lineTo(p2.u, p2.v); ctx.stroke();
      }
      const t1 = this.project([-120, 0, 0]), t2 = this.project([300, 0, 0]);
      ctx.strokeStyle = THEME_PALETTE.axisPlane; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(t1.u, t1.v); ctx.lineTo(t2.u, t2.v); ctx.stroke();
    } else if(this.viewMode === 'top'){
      ctx.strokeStyle = THEME_PALETTE.grid; ctx.lineWidth = 1;
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
      ctx.strokeStyle = THEME_PALETTE.reachCircle; ctx.lineWidth = 1.2; ctx.setLineDash([4, 4]);
      ctx.beginPath(); cMax.forEach((p, i) => i === 0 ? ctx.moveTo(p.u, p.v) : ctx.lineTo(p.u, p.v)); ctx.stroke();
      ctx.setLineDash([]);
    }

    if(!landmarks) return;

    // 2. Trajectory Waypoints
    if(pathPts && pathPts.length > 0){
      ctx.strokeStyle = THEME_PALETTE.penTool; ctx.lineWidth = 2.5;
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
    ctx.strokeStyle = THEME_PALETTE.linkBase; ctx.lineWidth = 7; ctx.lineCap = 'round'; ctx.beginPath(); ctx.moveTo(pBase.u, pBase.v); ctx.lineTo(pSh.u, pSh.v); ctx.stroke();
    ctx.strokeStyle = THEME_PALETTE.linkUpper; ctx.lineWidth = 5.5; ctx.beginPath(); ctx.moveTo(pSh.u, pSh.v); ctx.lineTo(pEl.u, pEl.v); ctx.stroke();
    ctx.strokeStyle = THEME_PALETTE.linkFore1; ctx.lineWidth = 4.5; ctx.beginPath(); ctx.moveTo(pEl.u, pEl.v); ctx.lineTo(pBend.u, pBend.v); ctx.stroke();
    ctx.strokeStyle = THEME_PALETTE.linkFore2; ctx.lineWidth = 4.5; ctx.beginPath(); ctx.moveTo(pBend.u, pBend.v); ctx.lineTo(pWr.u, pWr.v); ctx.stroke();
    ctx.strokeStyle = THEME_PALETTE.penTool; ctx.lineWidth = 3.5; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(pWr.u, pWr.v); ctx.lineTo(pTcp.u, pTcp.v); ctx.stroke();
    ctx.setLineDash([]);

    // 4. Joint Markers
    const joints = [pSh, pEl, pBend, pWr];
    ctx.fillStyle = THEME_PALETTE.jointFill; ctx.strokeStyle = THEME_PALETTE.jointBorder; ctx.lineWidth = 1.5;
    for(const j of joints){
      ctx.beginPath(); ctx.arc(j.u, j.v, 4.5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }
    // TCP Pen Tip
    ctx.fillStyle = THEME_PALETTE.penTool; ctx.beginPath(); ctx.arc(pTcp.u, pTcp.v, 5.5, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = 'rgba(244,63,94,0.3)'; ctx.beginPath(); ctx.arc(pTcp.u, pTcp.v, 11.0, 0, Math.PI * 2); ctx.fill();
  }

  renderCurrent() { this.render(this.lastLm, this.lastPath); }
}

/* =============================================================================
   5. STATE & CONTROLLERS
============================================================================= */
let dashRenderer, simRenderer;
let simAngles = [0, 0, 0, 0, 0, 0];
let simSource = 'sim';
let simPathType = 'line';
let simWaypoints = [];
let simPlayAnimId = null;
let simLastAnimTime = 0;
let simScrubIdx = 0;
let stepSize = 1.0, pollTimer = null, failN = 0, toastTimer = null;

function initStudio() {
  dashRenderer = new Canvas3DRenderer('dashCanvas');
  simRenderer = new Canvas3DRenderer('simCanvas');

  const jContainer = document.getElementById('simJointSliders');
  if(jContainer){
    jContainer.innerHTML = '';
    for(let i=0; i<6; i++){
      const lim = JOINT_LIMITS[i];
      jContainer.insertAdjacentHTML('beforeend', `
        <div class="slider-row">
          <label for="simJ${i}">J${i+1}:</label>
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
    btn.parentElement.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
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
    btn.parentElement.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
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
  const bLive = document.getElementById('btnSimLive'), bSim = document.getElementById('btnSimSim');
  if(bLive) bLive.classList.toggle('active', src === 'live');
  if(bSim) bSim.classList.toggle('active', src === 'sim');
  if(src === 'live' && window.lastRobotAngles){
    updateSimFromAngles(window.lastRobotAngles);
  }
}

function updateSimFromAngles(angles){
  simAngles = [...angles];
  const lm = forwardKinematics(simAngles);

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

  const hudSimTcp = document.getElementById('hudSimTcp');
  if(hudSimTcp) hudSimTcp.textContent = `X=${lm.tcp[0].toFixed(1)} Y=${lm.tcp[1].toFixed(1)} Z=${lm.tcp[2].toFixed(1)}`;
  const hudSimReach = document.getElementById('hudSimReach');
  if(hudSimReach) hudSimReach.textContent = `Wrist: (${lm.wrist[0].toFixed(1)}, ${lm.wrist[1].toFixed(1)}, ${lm.wrist[2].toFixed(1)})`;
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
  const badge = document.getElementById('simIkStatusBadge');
  if(res.ok){
    if(badge){ badge.textContent = 'IK OK'; badge.className = 'badge b-run'; }
    updateSimFromAngles(res.angles);
  } else {
    if(badge){ badge.textContent = 'OUT OF REACH'; badge.className = 'badge b-fault'; }
  }
}

function applySimPreset(type){
  setSimSource('sim');
  if(type === 'home') updateSimFromAngles([0, 0, 0, 0, 0, 0]);
  else if(type === 'draw'){ const ik = ikPenDown(160, 0, 10); if(ik.ok) updateSimFromAngles(ik.angles); }
  else if(type === 'reach_fwd') updateSimFromAngles([0, 90, 0, 0, -DELTA_WRIST, 0]);
  else if(type === 'reach_back') updateSimFromAngles([0, -90, 0, 0, -DELTA_WRIST, 0]);
  else if(type === 'fold') updateSimFromAngles([0, 45, 90, 0, -45, 0]);
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

function onSimScrubChange(val){
  const idx = parseInt(val);
  simScrubIdx = idx;
  const scrubVal = document.getElementById('simScrubVal');
  const hudSimPlay = document.getElementById('hudSimPlay');
  const pct = `${Math.round(idx / Math.max(1, simWaypoints.length-1) * 100)}%`;
  if(scrubVal) scrubVal.textContent = pct;
  if(hudSimPlay) hudSimPlay.textContent = pct;
  if(simWaypoints[idx]){
    const wp = simWaypoints[idx];
    const ik = ikPenDown(wp.x, wp.y, wp.z);
    if(ik.ok) updateSimFromAngles(ik.angles);
  }
}

function simAnimStep(now){
  if(!simPlayAnimId) return;
  if(!document.hidden && now - simLastAnimTime >= 45){
    simLastAnimTime = now;
    simScrubIdx = (simScrubIdx + 1) % simWaypoints.length;
    const scrubEl = document.getElementById('simScrub');
    if(scrubEl) scrubEl.value = simScrubIdx;
    onSimScrubChange(simScrubIdx);
  }
  simPlayAnimId = requestAnimationFrame(simAnimStep);
}

function toggleSimPlay(){
  const btn = document.getElementById('btnSimPlay');
  if(simPlayAnimId){
    cancelAnimationFrame(simPlayAnimId);
    simPlayAnimId = null;
    if(btn) btn.textContent = '▶ Chạy mô phỏng (Play)';
  } else {
    if(btn) btn.textContent = '⏸ Tạm dừng (Pause)';
    simLastAnimTime = performance.now();
    simPlayAnimId = requestAnimationFrame(simAnimStep);
  }
}

function sendSimPoseToRobot(){
  const lm = forwardKinematics(simAngles);
  const feed = document.getElementById('mvFeed') ? document.getElementById('mvFeed').value : 30;
  post('/api/move', `x=${lm.tcp[0].toFixed(2)}&y=${lm.tcp[1].toFixed(2)}&z=${lm.tcp[2].toFixed(2)}&feed=${feed}`);
}

function switchMotionMode(mode){
  const bMove = document.getElementById('btnToggleMove'), bDraw = document.getElementById('btnToggleDraw');
  const pMove = document.getElementById('panelCartMove'), pDraw = document.getElementById('panelCartDraw');
  if(mode === 'move'){
    bMove.classList.add('active'); bDraw.classList.remove('active');
    pMove.style.display = 'block'; pDraw.style.display = 'none';
  } else {
    bDraw.classList.add('active'); bMove.classList.remove('active');
    pDraw.style.display = 'block'; pMove.style.display = 'none';
  }
}

function onDrawShapeChange(){
  const sh = document.getElementById('dwShape').value;
  const a1Lbl = document.getElementById('dwA1Lbl'), a2Lbl = document.getElementById('dwA2Lbl'), a3Lbl = document.getElementById('dwA3Lbl');
  const a4Group = document.getElementById('dwA4Group');
  if(sh === 'circle'){
    if(a1Lbl) a1Lbl.textContent = 'Tâm CX';
    if(a2Lbl) a2Lbl.textContent = 'Tâm CY';
    if(a3Lbl) a3Lbl.textContent = 'Bán kính R';
    if(a4Group) a4Group.style.display = 'none';
  } else {
    if(a1Lbl) a1Lbl.textContent = 'X1 (mm)';
    if(a2Lbl) a2Lbl.textContent = 'Y1 (mm)';
    if(a3Lbl) a3Lbl.textContent = 'X2 (mm)';
    if(a4Group) a4Group.style.display = 'flex';
  }
  syncDrawToSim();
}

function syncDrawToSim(){
  const sh = document.getElementById('dwShape').value;
  simPathType = sh;
  simWaypoints = [];
  const v = id => parseFloat(document.getElementById(id).value) || 0;
  const z = v('dwZ');
  if(sh === 'circle'){
    const cx = v('dwA1'), cy = v('dwA2'), r = v('dwA3');
    for(let a=0; a<=Math.PI*2; a+=0.15){
      simWaypoints.push({ x: cx + r*Math.cos(a), y: cy + r*Math.sin(a), z: z, drawing: true });
    }
  } else {
    const x1 = v('dwA1'), y1 = v('dwA2'), x2 = v('dwA3'), y2 = v('dwA4');
    for(let s=0; s<=1.0; s+=0.04){
      simWaypoints.push({ x: x1 + s*(x2-x1), y: y1 + s*(y2-y1), z: z, drawing: true });
    }
  }
  const scrubEl = document.getElementById('simScrub');
  if(scrubEl) scrubEl.max = Math.max(1, simWaypoints.length - 1);
  onSimScrubChange(0);
}

function startDraw(){
  const sh = document.getElementById('dwShape').value;
  const v = id => parseFloat(document.getElementById(id).value) || 0;
  let b = sh === 'line' ? `shape=line&x1=${v('dwA1')}&y1=${v('dwA2')}&x2=${v('dwA3')}&y2=${v('dwA4')}` : `shape=circle&cx=${v('dwA1')}&cy=${v('dwA2')}&r=${v('dwA3')}`;
  b += `&z=${v('dwZ')}&feed=${document.getElementById('dwFeed').value || 20}`;
  post('/api/draw', b);
}

/* =============================================================================
   6. REST API & UI BINDINGS
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
  }).catch(() => toast('Lỗi mạng / Mất kết nối robot', 'err'));
}
function post(url, body){
  return fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body })
    .then(r => r.text())
    .then(t => toast((t === 'OK' ? '✓ ' : '') + t, t === 'OK' ? 'ok' : 'warn'))
    .catch(() => toast('Lỗi mạng / Mất kết nối robot', 'err'));
}
function clearFault(){ post('/api/jog', 'fault_clear=1'); }
function jog(axis, dir){ post('/api/jog', `axis=${axis}&deg=${dir * stepSize}`); }
function confirmClearCalib(axisIdx){
  if(confirm(`Xóa cân chỉnh vị trí J${axisIdx + 1}? Giá trị zero đã lưu trong NVS sẽ bị xóa.`)){
    api(`/api/clearcalib?axis=${axisIdx}`);
  }
}
function saveWifi(){
  const s = document.getElementById('wfSsid').value.trim(), p = document.getElementById('wfPass').value;
  if(!s){ toast('Vui lòng nhập tên WiFi (SSID)', 'warn'); return; }
  post('/api/wifi', `ssid=${encodeURIComponent(s)}&pass=${encodeURIComponent(p)}`)
    .then(() => toast('Đã lưu WiFi! Robot đang khởi động lại...', 'ok'));
}

// Navigation Tabs
document.querySelectorAll('.nav-tab[data-t]').forEach(b => {
  b.onclick = () => {
    document.querySelectorAll('.nav-tab').forEach(x => { x.classList.remove('active'); x.setAttribute('aria-selected', 'false'); });
    document.querySelectorAll('.tab-pane').forEach(x => x.classList.remove('active'));
    b.classList.add('active'); b.setAttribute('aria-selected', 'true');
    const target = document.getElementById(b.dataset.t);
    if(target) target.classList.add('active');
    if(b.dataset.t === 'pane-dash' && dashRenderer) dashRenderer.renderCurrent();
    if(b.dataset.t === 'pane-motion' && simRenderer) simRenderer.renderCurrent();
  };
});

// Step selector
[0.5, 1, 5, 15, 30, 45].forEach(s => {
  const b = document.createElement('button');
  b.className = 'step-btn' + (s === stepSize ? ' active' : '');
  b.textContent = s + '°';
  b.onclick = () => {
    stepSize = s;
    document.querySelectorAll('.step-btn').forEach(x => x.classList.remove('active'));
    b.classList.add('active');
  };
  const container = document.getElementById('stepSelector');
  if(container) container.appendChild(b);
});

// Build Joint Cards & Quick Status Rows
function buildJointCards(){
  const grid = document.getElementById('jointCardsGrid');
  const dashRows = document.getElementById('dashJointRows');
  if(grid) grid.innerHTML = '';
  if(dashRows) dashRows.innerHTML = '';

  for(let i=0; i<6; i++){
    if(grid){
      grid.insertAdjacentHTML('beforeend', `
        <div class="jcard">
          <div class="jcard-top">
            <span class="jcard-name">${AXES[i]}</span>
            <span class="jcard-deg" id="jd${i}">0.0°</span>
          </div>
          <div class="jcard-enc" id="je${i}">Encoder: --</div>
          <div class="jcard-flags" id="jf${i}"></div>
          <div class="jog-controls">
            <button class="jog-btn" onclick="jog(${i},-1)" aria-label="Jog ${AXES[i]} âm">↺ &minus;</button>
            <button class="jog-btn" onclick="jog(${i},1)" aria-label="Jog ${AXES[i]} dương">+ ↻</button>
          </div>
          <div class="jog-calib-row">
            <button class="btn btn-ghost need-idle" onclick="api('/api/sethome?axis=${i}')">Set Home</button>
            <button class="btn btn-ghost" onclick="confirmClearCalib(${i})">Clear NVS</button>
          </div>
        </div>
      `);
    }
    if(dashRows){
      dashRows.insertAdjacentHTML('beforeend', `
        <div class="stat-row">
          <span class="k">${AXES[i].split(' ')[0]} (${AXES[i].split(' ')[1]})</span>
          <span class="v" id="dashJd${i}">0.0°</span>
        </div>
      `);
    }
  }
}
buildJointCards();

/* =============================================================================
   7. STATUS POLLER & TELEMETRY UPDATER
============================================================================= */
function updateUI(d){
  const m = d.mode || 'idle';
  const modeText = document.getElementById('dashModeText');
  const modeBadge = document.getElementById('dashModeBadge');
  const map = { idle: 'b-idle', homing: 'b-run', jog: 'b-run', cart: 'b-run', draw: 'b-run', fault: 'b-fault' };
  if(modeText) { modeText.textContent = m.toUpperCase(); modeText.className = 'mode-text ' + (m === 'fault' ? 'fault' : (m !== 'idle' ? 'run' : '')); }
  if(modeBadge) { modeBadge.textContent = m; modeBadge.className = 'badge ' + (map[m] || 'b-idle'); }

  const connLabel = document.getElementById('connLabel');
  if(connLabel) connLabel.textContent = `${d.wifi.ssid || '(AP)'} · ${d.wifi.ip}`;

  let hn = 0; d.joints.forEach(j => hn += j.homed ? 1 : 0);
  const homedCount = document.getElementById('dashHomedCount');
  if(homedCount) homedCount.textContent = `${hn}/6`;

  const wifiInfo = document.getElementById('dashWifiInfo');
  if(wifiInfo) wifiInfo.textContent = `${(d.wifi.mode||'').toUpperCase()} · RSSI ${d.wifi.rssi || 0} dBm`;

  const wfModeText = document.getElementById('wfModeText');
  if(wfModeText) wfModeText.textContent = (d.wifi.mode||'').toUpperCase();
  const wfIpText = document.getElementById('wfIpText');
  if(wfIpText) wfIpText.textContent = d.wifi.ip;
  const wfSsidNowText = document.getElementById('wfSsidNowText');
  if(wfSsidNowText) wfSsidNowText.textContent = d.wifi.ssid || '(AP Fallback)';
  const wfRssiText = document.getElementById('wfRssiText');
  if(wfRssiText) wfRssiText.textContent = `${d.wifi.rssi || 0} dBm`;

  document.querySelectorAll('.need-idle').forEach(b => { b.disabled = (d.busy || m === 'fault'); });

  const robotAngles = d.joints.map(j => j.deg);
  window.lastRobotAngles = robotAngles;

  // Render Dashboard Live 3D Twin
  const lmLive = forwardKinematics(robotAngles);
  if(dashRenderer) dashRenderer.render(lmLive, null);

  const dashPoseLabel = document.getElementById('dashPoseLabel');
  if(dashPoseLabel) dashPoseLabel.textContent = `TCP: (${lmLive.tcp[0].toFixed(1)}, ${lmLive.tcp[1].toFixed(1)}, ${lmLive.tcp[2].toFixed(1)})`;
  const hudDashTcp = document.getElementById('hudDashTcp');
  if(hudDashTcp) hudDashTcp.textContent = `X=${lmLive.tcp[0].toFixed(1)} Y=${lmLive.tcp[1].toFixed(1)} Z=${lmLive.tcp[2].toFixed(1)}`;
  const hudDashWrist = document.getElementById('hudDashWrist');
  if(hudDashWrist) hudDashWrist.textContent = `(${lmLive.wrist[0].toFixed(1)}, ${lmLive.wrist[1].toFixed(1)}, ${lmLive.wrist[2].toFixed(1)})`;

  if(simSource === 'live'){
    updateSimFromAngles(robotAngles);
  }

  // Update Joint Cards & Quick Status
  d.joints.forEach((j, i) => {
    const jd = document.getElementById('jd' + i); if(jd) jd.textContent = j.deg.toFixed(1) + '°';
    const dashJd = document.getElementById('dashJd' + i); if(dashJd) dashJd.textContent = j.deg.toFixed(1) + '°';
    const je = document.getElementById('je' + i); if(je) je.textContent = 'Encoder: ' + (j.encOK ? j.encDeg.toFixed(1) + '°' : 'MẤT KẾT NỐI');
    const jf = document.getElementById('jf' + i);
    if(jf){
      const flags = [];
      flags.push(`<span class="${j.homed ? 'flag-homed' : 'flag-unhomed'}">${j.homed ? (j.restored ? 'HOMED (NVS)' : 'HOMED') : 'CHƯA HOME'}</span>`);
      if(j.drift) flags.push('<span class="flag-drift">DRIFT</span>');
      if(!j.encOK) flags.push('<span class="flag-encerr">ENC ERR</span>');
      jf.innerHTML = flags.join(' · ');
    }
  });

  // Homing Chips
  for(let i=0; i<4; i++){
    const hc = document.getElementById('hc' + i);
    if(hc){
      hc.classList.toggle('done', d.joints[i].homed);
    }
  }
}

function setOnline(on){
  const pill = document.getElementById('connPill');
  if(pill) pill.classList.toggle('offline', !on);
  const label = document.getElementById('connLabel');
  if(!on && label) label.textContent = 'MẤT KẾT NỐI — Đang thử lại...';
}

function pollOnce(){
  fetch('/api/status').then(r => r.json())
    .then(d => { failN = 0; setOnline(true); updateUI(d); })
    .catch(() => { if(++failN >= 3) setOnline(false); });
}

initStudio();
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
WorkPlane* workPlanePtr = nullptr;

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
    if (!srv->hasArg("axis") || !srv->hasArg("deg")) {
        srv->send(400, "text/plain", "missing axis or deg");
        return;
    }
    const int axis = srv->arg("axis").toInt();
    const float deg = srv->arg("deg").toFloat();
    if (axis < 0 || axis >= NUM_MOTORS) { srv->send(400, "text/plain", "bad axis"); return; }
    if (!std::isfinite(deg) || fabsf(deg) < 0.01f || fabsf(deg) > 180.0f) {
        srv->send(400, "text/plain", "bad deg");
        return;
    }
    ArmCommand c;
    c.type = ArmCommand::JOG_REL;
    c.axis = static_cast<uint8_t>(axis);
    c.value = deg;
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleStop() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    ArmCommand c;
    c.type = ArmCommand::STOP_ALL;
    const bool ok = armPtr->submit(c, 50);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "queue full");
}

void handleMove() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (!srv->hasArg("x") || !srv->hasArg("y") || !srv->hasArg("z")) {
        srv->send(400, "text/plain", "missing x, y, or z");
        return;
    }
    const float x = srv->arg("x").toFloat();
    const float y = srv->arg("y").toFloat();
    const float z = srv->arg("z").toFloat();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        srv->send(400, "text/plain", "non-finite coordinates");
        return;
    }
    if (z < -15.0f || z > 435.0f || (x * x + y * y > 350.0f * 350.0f)) {
        srv->send(400, "text/plain", "z or xy out of range");
        return;
    }
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
    if (!srv->hasArg("axis")) { srv->send(400, "text/plain", "missing axis"); return; }
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
    if (!srv->hasArg("axis")) { srv->send(400, "text/plain", "missing axis"); return; }
    const int axis = srv->arg("axis").toInt();
    if (axis < 0 || (axis >= NUM_MOTORS && axis != 255)) { srv->send(400, "text/plain", "bad axis"); return; }
    if (armPtr != nullptr && armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::SET_HOME;
    c.axis = static_cast<uint8_t>(axis);
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleClearCalib() {
    if (jointsPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    // Flash Write Isolation Guard: Cấm xóa NVS khi robot đang chuyển động
    if (armPtr != nullptr && armPtr->busy()) {
        srv->send(409, "text/plain", "CANNOT_WRITE_FLASH_WHILE_MOVING");
        return;
    }
    if (!srv->hasArg("axis")) { srv->send(400, "text/plain", "missing axis"); return; }
    const int axis = srv->arg("axis").toInt();
    if (axis < 0 || axis >= NUM_MOTORS) { srv->send(400, "text/plain", "bad axis"); return; }
    jointsPtr->forgetHome(static_cast<uint8_t>(axis));
    srv->send(200, "text/plain", "OK");
}

void handleWifiSave() {
    if (wifiPtr == nullptr || nvsPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    // Flash Write Isolation Guard: Cấm ghi NVS khi robot đang chuyển động
    if (armPtr != nullptr && armPtr->busy()) {
        srv->send(409, "text/plain", "CANNOT_WRITE_FLASH_WHILE_MOVING");
        return;
    }
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

void handleWorkPlaneCalib() {
    if (workPlanePtr == nullptr) { srv->send(500, "text/plain", "WorkPlane not initialized"); return; }
    if (armPtr != nullptr && armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    const Point3D p1{srv->arg("p1x").toFloat(), srv->arg("p1y").toFloat(), srv->arg("p1z").toFloat()};
    const Point3D p2{srv->arg("p2x").toFloat(), srv->arg("p2y").toFloat(), srv->arg("p2z").toFloat()};
    const Point3D p3{srv->arg("p3x").toFloat(), srv->arg("p3y").toFloat(), srv->arg("p3z").toFloat()};
    if (!std::isfinite(p1.x) || !std::isfinite(p1.y) || !std::isfinite(p1.z) ||
        !std::isfinite(p2.x) || !std::isfinite(p2.y) || !std::isfinite(p2.z) ||
        !std::isfinite(p3.x) || !std::isfinite(p3.y) || !std::isfinite(p3.z)) {
        srv->send(400, "application/json", "{\"calibrated\":false,\"error\":\"Non-finite coordinates\"}");
        return;
    }
    if (workPlanePtr->setThreePointCalibration(p1, p2, p3)) {
        srv->send(200, "application/json", "{\"calibrated\":true,\"error\":\"\"}");
    } else {
        String json = "{\"calibrated\":false,\"error\":\"" + workPlanePtr->getLastError() + "\"}";
        srv->send(400, "application/json", json);
    }
}

void handleWorkPlaneToggle() {
    if (workPlanePtr == nullptr) { srv->send(500, "text/plain", "WorkPlane not initialized"); return; }
    if (armPtr != nullptr && armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    const bool en = (srv->arg("en").toInt() != 0);
    workPlanePtr->setEnabled(en);
    srv->send(200, "application/json", workPlanePtr->isEnabled() ? "{\"enabled\":true}" : "{\"enabled\":false}");
}

void handleWorkPlaneStatus() {
    if (workPlanePtr == nullptr) { srv->send(500, "text/plain", "WorkPlane not initialized"); return; }
    const Point3D o = workPlanePtr->getOrigin();
    const Point3D n = workPlanePtr->getNormal();
    char buf[512];
    snprintf(buf, sizeof(buf),
             "{\"isCalibrated\":%s,\"isEnabled\":%s,\"origin\":[%.2f,%.2f,%.2f],\"normal\":[%.3f,%.3f,%.3f],\"error\":\"%s\"}",
             workPlanePtr->isCalibrated() ? "true" : "false",
             workPlanePtr->isEnabled() ? "true" : "false",
             o.x, o.y, o.z, n.x, n.y, n.z, workPlanePtr->getLastError().c_str());
    sendJson(200, buf);
}

} // namespace

void webBegin(WebServer& server, ArmController* arm, WifiManager* wifi,
              JointModel* joints, NvsStore* nvs, WorkPlane* workPlane) {
    srv = &server;
    armPtr = arm;
    wifiPtr = wifi;
    jointsPtr = joints;
    nvsPtr = nvs;
    workPlanePtr = workPlane;

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
    server.on("/api/workplane/calib", HTTP_POST, handleWorkPlaneCalib);
    server.on("/api/workplane/toggle", HTTP_POST, handleWorkPlaneToggle);
    server.on("/api/workplane/status", HTTP_GET, handleWorkPlaneStatus);

    server.begin();
    Serial.printf("[WEB] HTTP server on port %u\n", WEB_SERVER_PORT);
}