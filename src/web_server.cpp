#include "web_server.h"
#include "web_validation.h"
#include "arm.h"
#include "config.h"
#include "joint_model.h"
#include "kinematics.h"
#include "nvs_store.h"
#include "wifi_manager.h"

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
  --canvas-bg: #070a10;
  --overlay: #111827;
  --border: #1f2a3d;
  --border-strong: #334155;
  --text-main: #f8fafc;
  --text-muted: #94a3b8;
  --text-dim: #a7b7cc;
  --primary: #38bdf8;
  --primary-hover: #0ea5e9;
  --success: #10b981;
  --success-ink: #032b20;
  --success-bg: rgba(16,185,129,0.12);
  --success-text: #6ee7b7;
  --danger: #dc2626;
  --danger-ink: #ffffff;
  --danger-bg: rgba(239,68,68,0.14);
  --danger-text: #fca5a5;
  --warning: #f59e0b;
  --warning-ink: #261601;
  --warning-bg: rgba(245,158,11,0.14);
  --warning-text: #fcd34d;
  --toast-ok-bg: #0c1a17;
  --toast-warn-bg: #1c1917;
  --toast-err-bg: #7f1d1d;
  --radius-xs: 4px;
  --radius-sm: 6px;
  --radius-md: 10px;
  --radius-lg: 14px;
  --radius-pill: 9999px;
  --z-toast: 20;
  --z-estop: 30;
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

/* ---- Operator Console Navigation ---- */
.mode-tabs {
  display: grid; grid-template-columns: 1fr 1fr; gap: 4px;
  max-width: 360px; padding: 4px; margin-bottom: 8px;
  background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius-md);
}
.mode-tab {
  min-height: 42px; border: 0; border-radius: var(--radius-sm); cursor: pointer;
  background: transparent; color: var(--text-muted); font: 600 0.9rem var(--font-sans);
}
.mode-tab:hover { color: var(--text-main); background: var(--surface-elevated); }
.mode-tab.active { color: var(--bg); background: var(--primary); }
.nav-tabs {
  display: flex; gap: 20px; margin: 0 0 18px; padding: 0 4px;
  border-bottom: 1px solid var(--border); flex-wrap: wrap;
}
.nav-tab {
  background: transparent; border: none; color: var(--text-muted);
  padding: 9px 0; border-radius: 0; font-size: 0.82rem;
  font-weight: 600; cursor: pointer; min-height: 38px;
  display: inline-flex; align-items: center; gap: 6px;
  transition: background 150ms ease, color 150ms ease;
}
.nav-tab:hover { color: var(--text-main); }
.nav-tab.active { color: var(--primary); box-shadow: inset 0 -2px var(--primary); font-weight: 700; }
.tab-pane { display: none; }
.tab-pane.active { display: block; }
.setup-nav { display: none; }
body.setup-mode .operate-nav { display: none; }
body.setup-mode .setup-nav { display: inline-flex; }
body:not(.setup-mode) .setup-only { display: none !important; }
body.setup-mode .jog-controls { display: none; }

/* ---- Layout Grids ---- */
.grid-2col { display: grid; grid-template-columns: 1.25fr 0.75fr; gap: 14px; }
@media (max-width: 900px) { .grid-2col { grid-template-columns: 1fr; } }
.grid-cards-3 { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 12px; }

/* ---- Cards ---- */
.card {
  background: var(--surface); border-radius: var(--radius-md);
  padding: 16px; border: 1px solid var(--border); margin-bottom: 14px;
}
.card-head {
  display: flex; align-items: center; justify-content: space-between; gap: 10px;
  margin-bottom: 12px; padding-bottom: 8px; border-bottom: 1px solid var(--border);
}
.card-head h2 { font-size: 0.95rem; font-weight: 650; color: var(--text-main); display: flex; align-items: center; gap: 6px; }
.card-head .meta { font-size: 0.76rem; color: var(--text-muted); font-family: var(--font-mono); }

/* ---- 3D Viewport Box ---- */
.viewport-box {
  position: relative; background: var(--canvas-bg); border: 1px solid var(--border);
  border-radius: var(--radius-md); overflow: hidden; margin-bottom: 10px;
}
.viewport-canvas { display: block; width: 100%; height: clamp(240px, 48vw, 380px); cursor: grab; touch-action: none; }
.viewport-canvas:active { cursor: grabbing; }
.viewport-toolbar {
  position: absolute; top: 8px; left: 8px; right: 8px;
  display: flex; justify-content: space-between; align-items: center;
  pointer-events: none; gap: 6px; flex-wrap: wrap; z-index: 2;
}
.toolbar-group {
  display: flex; gap: 4px; background: var(--overlay);
  padding: 3px; border-radius: var(--radius-sm); border: 1px solid var(--border-strong);
  pointer-events: auto;
}
.tool-btn {
  background: transparent; border: none; color: var(--text-muted);
  padding: 4px 10px; border-radius: var(--radius-xs); font-size: 0.74rem;
  font-weight: 600; cursor: pointer; min-height: 28px;
  display: inline-flex; align-items: center; justify-content: center;
  transition: background 120ms ease, color 120ms ease, transform 80ms ease;
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
.b-showoff { background: linear-gradient(135deg, rgba(99,102,241,0.25), rgba(168,85,247,0.3)); color: #c084fc; border: 1px solid rgba(168,85,247,0.6); animation: pulse-glow 1.5s infinite alternate; }
.mode-text.showoff { background: linear-gradient(135deg, #818cf8, #c084fc, #f472b6); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
.btn-showoff { background: linear-gradient(135deg, #6366f1 0%, #a855f7 50%, #ec4899 100%); color: #ffffff !important; font-weight: 700; border: none; box-shadow: 0 2px 10px rgba(168,85,247,0.35); transition: transform 120ms ease, box-shadow 120ms ease, opacity 120ms ease; }
.btn-showoff:hover:not(:disabled) { transform: translateY(-1px); box-shadow: 0 4px 14px rgba(168,85,247,0.5); }
@keyframes pulse-glow { 0% { box-shadow: 0 0 3px rgba(168,85,247,0.2); } 100% { box-shadow: 0 0 10px rgba(168,85,247,0.6); } }

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
.btn-success { background: var(--success); color: var(--success-ink); font-weight: 700; }
.btn-danger { background: var(--danger); color: var(--danger-ink); }
.btn-warning { background: var(--warning); color: var(--warning-ink); font-weight: 700; }
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
.flag-drift { color: var(--danger-text); background: var(--danger-bg); padding: 1px 4px; border-radius: var(--radius-xs); }
.flag-encerr { color: var(--warning-text); background: var(--warning-bg); padding: 1px 4px; border-radius: var(--radius-xs); }

.jog-controls { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-top: 4px; }
.jog-btn {
  min-height: 42px; font-size: 1.1rem; font-weight: 700; border-radius: var(--radius-sm);
  background: var(--surface-elevated); border: 1px solid var(--border-strong); color: var(--text-main);
  cursor: pointer; display: flex; align-items: center; justify-content: center;
  transition: border-color 100ms ease, color 100ms ease, transform 80ms ease;
}
.jog-btn:hover:not(:disabled) { border-color: var(--primary); color: var(--primary); }
.jog-btn:active:not(:disabled) { transform: scale(0.96); }
.jog-calib-row { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-top: 2px; }
.jog-calib-row .btn { min-height: 30px; font-size: 0.72rem; padding: 4px 8px; }
.jcard-calib { min-height: 1.1em; color: var(--text-muted); font-family: var(--font-mono); font-size: 0.68rem; }
.btn-toggle {
  background: var(--surface-sub); color: var(--text-muted);
  border: 1px solid var(--border-strong); padding: 5px 12px;
  font-size: 0.78rem; font-weight: 600; border-radius: var(--radius-sm);
  cursor: pointer; display: inline-flex; align-items: center; gap: 6px;
  transition: all 150ms ease; min-height: 32px;
}
.btn-toggle:hover { color: var(--text-main); border-color: var(--primary); }
.btn-toggle.active {
  background: rgba(56,189,248,0.18); color: var(--primary);
  border-color: var(--primary); font-weight: 700;
}
.btn-toggle.locked {
  background: rgba(245,158,11,0.18); color: var(--warning);
  border-color: var(--warning); font-weight: 700;
}
.tool-len-box {
  display: inline-flex; align-items: center; gap: 6px;
  background: var(--surface-sub); padding: 3px 8px;
  border-radius: var(--radius-sm); border: 1px solid var(--border);
}
.moveto-row { display: flex; gap: 6px; align-items: center; margin-top: 4px; }
.moveto-input {
  flex: 1; padding: 5px 8px; background: var(--surface-elevated);
  border: 1px solid var(--border); border-radius: var(--radius-xs);
  color: var(--text-main); font-size: 0.8rem; font-family: var(--font-mono); min-width: 0;
}
.moveto-input:focus { border-color: var(--primary); outline: none; }
.target-tag {
  font-size: 0.68rem; color: var(--text-dim); font-family: var(--font-mono);
  background: var(--surface-elevated); padding: 2px 6px; border-radius: var(--radius-xs);
}

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
.preset-pill:hover, .preset-pill[aria-pressed="true"] { color: var(--text-main); border-color: var(--primary); }
.preset-pill[aria-pressed="true"] { background: var(--primary); color: var(--bg); font-weight: 700; }

/* ---- Guided quick drawing ---- */
.draw-guided { background: var(--surface-sub); border: 1px solid var(--border); border-radius: var(--radius-md); padding: 12px; margin-bottom: 12px; }
.draw-guided p { color: var(--text-muted); font-size: 0.8rem; line-height: 1.5; margin-bottom: 10px; }
.draw-profile-list { display: flex; flex-wrap: wrap; gap: 6px; margin: 7px 0 10px; }
.draw-profile { min-width: 118px; text-align: left; font-family: var(--font-mono); }
.draw-profile small { display: block; margin-top: 2px; font-family: var(--font-sans); font-size: 0.68rem; opacity: 0.85; }
.draw-guided .input-field { max-width: 260px; }

/* ---- Numeric Typography & Labels ---- */
.jcard-deg, .stat-row .v, .hud-item .val { font-variant-numeric: tabular-nums; }
.hud-item .lbl, .input-field label, .jcard-flags { letter-spacing: 0.04em; }

/* ---- Loading Spinner State ---- */
.btn.btn-loading { color: transparent !important; position: relative; pointer-events: none; }
.btn.btn-loading::after {
  content: ""; position: absolute; width: 16px; height: 16px; top: 50%; left: 50%;
  margin: -8px 0 0 -8px; border: 2px solid currentColor; border-right-color: transparent;
  border-radius: 50%; animation: spin 0.6s linear infinite;
}
@keyframes spin { to { transform: rotate(360deg); } }

/* ---- Teach & Playback Component Styles ---- */
.teach-card { margin-top: 16px; }
.teach-jog-box {
  background: var(--surface-sub); border: 1px solid var(--border);
  border-radius: var(--radius-md); padding: 12px; margin-bottom: 12px;
}
.teach-jog-header {
  display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; flex-wrap: wrap; gap: 6px;
}
.teach-jog-title { font-size: 0.80rem; color: var(--text-dim); font-weight: 600; text-transform: uppercase; letter-spacing: 0.04em; }
.teach-jog-grid {
  display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 8px;
}
.teach-jog-pair { display: flex; gap: 4px; }
.teach-jog-pair .btn { flex: 1; min-height: 38px; font-weight: 600; }
.teach-actions { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px; }
.teach-progress { margin-top: 8px; font-size: 0.85rem; color: var(--primary); font-family: var(--font-mono); font-weight: 600; }

/* ---- Floating E-STOP & Toast ---- */
#estop {
  position: fixed; bottom: 18px; right: 18px; z-index: var(--z-estop);
  padding: 14px 24px; font-size: 1.05rem; font-weight: 800; border-radius: var(--radius-pill);
  box-shadow: 0 6px 24px rgba(220,38,38,0.5); letter-spacing: 0.02em;
  animation: estopPulse 2.2s ease-in-out infinite;
}
@keyframes estopPulse {
  0%, 100% { box-shadow: 0 6px 24px rgba(220,38,38,0.5); }
  50% { box-shadow: 0 6px 36px rgba(220,38,38,0.9); }
}
#toast {
  position: fixed; bottom: 18px; left: 18px; z-index: var(--z-toast);
  background: var(--surface-elevated); border: 1px solid var(--border-strong);
  border-left-width: 4px; border-radius: var(--radius-md); padding: 10px 18px; font-size: 0.84rem;
  color: var(--text-main); display: none; max-width: 80vw;
}
#toast.show {
  display: block;
  animation: toastIn 0.15s cubic-bezier(0.16, 1, 0.3, 1);
}
@keyframes toastIn {
  from { opacity: 0; transform: translateY(8px); }
  to { opacity: 1; transform: translateY(0); }
}
#toast.t-ok { border-color: var(--success); background: var(--toast-ok-bg); color: var(--success-text); }
#toast.t-warn { border-color: var(--warning); background: var(--toast-warn-bg); color: var(--warning-text); }
#toast.t-err { border-color: var(--danger); background: var(--toast-err-bg); color: var(--danger-text); }
.sr-only { position: absolute; width: 1px; height: 1px; padding: 0; margin: -1px; overflow: hidden; clip: rect(0, 0, 0, 0); white-space: nowrap; border: 0; }

/* ---- HELLO STUDIO STYLES ---- */
.hello-hero {
  display: flex; align-items: center; justify-content: space-between;
  background: linear-gradient(135deg, rgba(56,189,248,0.12), rgba(99,102,241,0.10));
  padding: 12px 16px; border-radius: var(--radius-md); border: 1px solid rgba(56,189,248,0.3);
  margin-bottom: 12px; flex-wrap: wrap; gap: 8px;
}
.hello-homing-bar {
  display: flex; align-items: center; justify-content: space-between;
  background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius-md);
  padding: 8px 12px; margin-bottom: 12px; flex-wrap: wrap; gap: 8px;
}
.hello-letters-row {
  display: grid; grid-template-columns: repeat(5, 1fr); gap: 8px; margin-bottom: 14px;
}
.hello-letter-card {
  background: var(--surface); border: 2px solid var(--border); border-radius: var(--radius-md);
  padding: 10px 4px; text-align: center; cursor: pointer; transition: all 120ms ease;
  display: flex; flex-direction: column; align-items: center; gap: 4px; user-select: none;
}
.hello-letter-card:hover { border-color: var(--primary); transform: translateY(-2px); }
.hello-letter-card.active {
  border-color: var(--primary); background: rgba(56,189,248,0.18);
  box-shadow: 0 0 14px rgba(56,189,248,0.35); transform: translateY(-2px);
}
.hello-glyph { font-size: 2.2rem; font-weight: 900; font-family: var(--font-mono); color: var(--text-main); line-height: 1; }
.hello-letter-card.active .hello-glyph { color: var(--primary); }
.hello-badge { font-size: 0.68rem; font-weight: 700; padding: 2px 6px; border-radius: var(--radius-pill); background: var(--surface-elevated); color: var(--text-dim); }
.hello-letter-card.active .hello-badge { background: var(--primary); color: var(--bg); }
.hello-visual-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
@media (max-width: 820px) { .hello-visual-grid { grid-template-columns: 1fr; } }
.hello-box {
  background: var(--surface-sub); border: 1px solid var(--border); border-radius: var(--radius-md); padding: 12px;
}
.hello-diagram-wrap {
  display: flex; flex-direction: column; align-items: center; justify-content: center; min-height: 220px;
}
.hello-press-bar {
  display: flex; align-items: center; justify-content: space-between; background: var(--surface-sub);
  border: 1px solid var(--border); border-radius: var(--radius-md); padding: 8px 12px; margin: 8px 0; flex-wrap: wrap; gap: 6px;
}
.hello-table {
  width: 100%; border-collapse: collapse; font-family: var(--font-mono); font-size: 0.78rem; margin-top: 8px;
}
.hello-table th, .hello-table td {
  padding: 5px 8px; text-align: left; border-bottom: 1px solid var(--border);
}
.hello-table th { color: var(--text-dim); font-size: 0.68rem; text-transform: uppercase; }

@media (pointer: coarse) {
  .mode-tab, .nav-tab { min-height: 44px; font-size: 0.9rem; }
  .btn, .tool-btn, .step-btn, .preset-pill, .mode-toggle-btn { min-height: 44px; }
  .jog-btn { min-height: 48px; }
  .jog-calib-row .btn { min-height: 44px; font-size: 0.78rem; padding: 8px; }
  .teach-jog-pair .btn { min-height: 44px; }
  .viewport-canvas { height: clamp(240px, 70vw, 340px); }
}
@media (prefers-reduced-motion: reduce) {
  * { transition-duration: 0.01ms !important; animation-duration: 0.01ms !important; }
  #estop { animation: none !important; }
}
</style>
</head>
<body class="operate-mode">
<main class="app" id="main-content">
  <!-- Header -->
  <header class="header">
    <div class="brand">
      <h1>Arm <b>Console</b></h1>
    </div>
    <div class="conn-pill" id="connPill">
      <span class="conn-dot"></span>
      <span id="connLabel">Đang kết nối...</span>
    </div>
  </header>

  <nav class="mode-tabs" aria-label="Chế độ giao diện">
    <button id="mode-operate" class="mode-tab active" data-mode="operate" aria-pressed="true">Operate</button>
    <button id="mode-setup" class="mode-tab" data-mode="setup" aria-pressed="false">Setup</button>
  </nav>
  <nav class="nav-tabs" id="operateNav" role="tablist" aria-label="Công cụ vận hành">
    <button id="tab-hello" class="nav-tab operate-nav active" data-t="pane-hello" role="tab" aria-selected="true" aria-controls="pane-hello" tabindex="0">✍️ VIẾT CHỮ HELLO</button>
    <button id="tab-dash" class="nav-tab operate-nav" data-t="pane-dash" role="tab" aria-selected="false" aria-controls="pane-dash" tabindex="-1">Overview</button>
    <button id="tab-jog" class="nav-tab operate-nav" data-t="pane-jog" role="tab" aria-selected="false" aria-controls="pane-jog" tabindex="-1">Khớp &amp; Homing</button>
    <button id="tab-motion" class="nav-tab setup-nav" data-t="pane-motion" role="tab" aria-selected="false" aria-controls="pane-motion" tabindex="-1">🎨 Studio Nâng Cao</button>
    <button id="tab-calib" class="nav-tab setup-nav" data-t="pane-jog" role="tab" aria-selected="false" aria-controls="pane-jog" tabindex="-1">Calibration</button>
    <button id="tab-settings" class="nav-tab setup-nav" data-t="pane-settings" role="tab" aria-selected="false" aria-controls="pane-settings" tabindex="-1">WiFi &amp; Hardware</button>
  </nav>

  <!-- =========================================================================
       TAB 0: DEDICATED HELLO STUDIO (VIẾT CHỮ HELLO TỪNG CHỮ CÁI)
  ========================================================================== -->
  <div id="pane-hello" class="tab-pane active" role="tabpanel" aria-labelledby="tab-hello" tabindex="0">
    <div class="hello-hero">
      <div>
        <div style="font-size:0.72rem;color:var(--text-dim);text-transform:uppercase;font-weight:700">Studio Viết Chữ</div>
        <div style="font-size:1.35rem;font-weight:800;color:var(--text-main)">Xưởng Viết "HELLO" (Từng Chữ Tự Chỉnh Giấy)</div>
      </div>
      <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
        <span class="badge b-idle" id="helloPlayStatusBadge">Sẵn sàng</span>
        <button class="btn btn-warning" id="btnHelloFree" onclick="toggleHelloFree()">🔓 Thả lỏng tay để kéo</button>
      </div>
    </div>

    <!-- Homing Status & 1-Click Home for Drawing -->
    <div class="hello-homing-bar" id="helloHomingBar">
      <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
        <span class="badge" id="helloHomedBadge">Đang kiểm tra Home...</span>
        <span style="font-size:0.75rem;color:var(--text-muted)" id="helloHomedNotice">
          Sau khi bật nguồn, bắt buộc Home để ngòi bút chuẩn xác từng milimet!
        </span>
      </div>
      <div style="display:flex;gap:6px;align-items:center;flex-wrap:wrap">
        <button class="btn btn-primary" id="btnHelloHomeAll" style="font-weight:700;padding:6px 14px" onclick="homeHelloDrawAxes()">
          🎯 HOME J3..J1 (Chuẩn Bị Vẽ)
        </button>
        <button class="btn btn-ghost" style="padding:4px 8px;font-size:0.74rem" onclick="api('/api/home/axis?axis=2')">Home J3</button>
        <button class="btn btn-ghost" style="padding:4px 8px;font-size:0.74rem" onclick="api('/api/home/axis?axis=1')">Home J2</button>
        <button class="btn btn-ghost" style="padding:4px 8px;font-size:0.74rem" onclick="api('/api/home/axis?axis=0')">Home J1</button>
      </div>
    </div>

    <!-- Letter Selector Row: H, E, L1, L2, O -->
    <div class="hello-letters-row" id="helloLettersRow">
      <div class="hello-letter-card active" onclick="selectHelloLetter('H')" id="cardLetterH">
        <span class="hello-glyph">H</span>
        <span class="hello-badge" id="badgeLetterH">0 điểm</span>
      </div>
      <div class="hello-letter-card" onclick="selectHelloLetter('E')" id="cardLetterE">
        <span class="hello-glyph">E</span>
        <span class="hello-badge" id="badgeLetterE">0 điểm</span>
      </div>
      <div class="hello-letter-card" onclick="selectHelloLetter('L1')" id="cardLetterL1">
        <span class="hello-glyph">L₁</span>
        <span class="hello-badge" id="badgeLetterL1">0 điểm</span>
      </div>
      <div class="hello-letter-card" onclick="selectHelloLetter('L2')" id="cardLetterL2">
        <span class="hello-glyph">L₂</span>
        <span class="hello-badge" id="badgeLetterL2">0 điểm</span>
      </div>
      <div class="hello-letter-card" onclick="selectHelloLetter('O')" id="cardLetterO">
        <span class="hello-glyph">O</span>
        <span class="hello-badge" id="badgeLetterO">0 điểm</span>
      </div>
    </div>

    <!-- Main Two-Column Work Area -->
    <div class="hello-visual-grid">
      <!-- Left Column: Visual Diagram & Stroke Analysis -->
      <div class="hello-box">
        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">
          <h3 style="font-size:0.95rem;font-weight:700;margin:0" id="helloLetterTitle">Sơ đồ nét: Chữ H</h3>
          <span style="font-size:0.75rem;color:var(--text-muted)">Quy ước: Xanh = Vẽ · Vàng đứt = Bay</span>
        </div>
        <div class="hello-diagram-wrap" id="helloDiagramWrap">
          <!-- Dynamic SVG Diagram inserted by JS -->
        </div>
        <div style="margin-top:10px;background:rgba(245,158,11,0.08);border:1px solid rgba(245,158,11,0.25);border-radius:var(--radius-xs);padding:8px 10px;font-size:0.76rem;color:var(--text-main)">
          🛡️ <b>Mẹo chống bay giấy ở đoạn xa:</b> Với nét vẽ vươn từ trong ra ngoài bàn, hãy ghi thêm 1 điểm trung gian ở giữa đoạn (hoặc chọn <i>Độ đè bút &ge; 1.0&deg;</i>) để triệt tiêu độ cong hình học!
        </div>
      </div>

      <!-- Right Column: Step-by-Step Recording & Jog Controls -->
      <div class="hello-box">
        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">
          <h3 style="font-size:0.95rem;font-weight:700;margin:0">Thao tác &amp; Ghi điểm</h3>
          <span class="badge b-run" id="helloActiveStepBadge">Điểm tiếp theo: #1</span>
        </div>

        <!-- Primary Recording Buttons -->
        <div class="btn-row" style="margin-top:4px">
          <button class="btn btn-primary" style="flex:1;font-weight:700;min-height:42px" onclick="helloRecordPoint(true)">
            🖊️ GHI ĐIỂM HẠ BÚT (Vẽ)
          </button>
          <button class="btn btn-warning" style="flex:1;font-weight:700;min-height:42px" onclick="helloRecordPoint(false)">
            ✈️ GHI ĐIỂM NHẤC (Bay)
          </button>
        </div>
        <div class="btn-row" style="margin-top:6px">
          <button class="btn btn-ghost" style="flex:1" onclick="helloUndoPoint()">↩ Hoàn tác điểm</button>
          <button class="btn btn-danger" style="flex:1" onclick="helloClearPoints()">🗑 Xóa nét chữ này</button>
        </div>

        <!-- Inline Jog for Precision Positioning -->
        <div class="teach-jog-box" style="margin-top:10px">
          <div class="teach-jog-header">
            <span class="teach-jog-title">Jog chỉnh vị trí ngòi bút:</span>
            <div class="step-selector">
              <button class="step-btn" id="btnHStep1" onclick="setHelloJogStep(1)">1&deg;</button>
              <button class="step-btn active" id="btnHStep5" onclick="setHelloJogStep(5)">5&deg;</button>
              <button class="step-btn" id="btnHStep10" onclick="setHelloJogStep(10)">10&deg;</button>
            </div>
          </div>
          <div class="teach-jog-grid">
            <div class="teach-jog-pair">
              <button class="btn btn-ghost" onclick="jogHello(1,-1)">J1 &minus;</button>
              <button class="btn btn-ghost" onclick="jogHello(1,1)">J1 +</button>
            </div>
            <div class="teach-jog-pair">
              <button class="btn btn-ghost" onclick="jogHello(2,-1)">J2 &minus;</button>
              <button class="btn btn-ghost" onclick="jogHello(2,1)">J2 +</button>
            </div>
            <div class="teach-jog-pair">
              <button class="btn btn-ghost" onclick="jogHello(3,-1)">J3 &minus;</button>
              <button class="btn btn-ghost" onclick="jogHello(3,1)">J3 +</button>
            </div>
          </div>
        </div>

        <!-- Pen Press Depth (Bù đè bút J2) -->
        <div class="hello-press-bar">
          <span style="font-size:0.78rem;font-weight:700;color:var(--text-main)">⚓ Độ đè bút J2:</span>
          <div style="display:flex;gap:4px;align-items:center">
            <button class="step-btn" onclick="setHelloPenPress(0.0)">0&deg;</button>
            <button class="step-btn active" id="btnPress03" onclick="setHelloPenPress(0.35)">0.35&deg;</button>
            <button class="step-btn" id="btnPress05" onclick="setHelloPenPress(0.5)">0.5&deg;</button>
            <button class="step-btn" id="btnPress10" onclick="setHelloPenPress(1.0)">1.0&deg;</button>
            <input type="number" id="inputHelloPressDeg" min="0" max="4.0" step="0.05" value="0.35" style="width:52px;padding:3px;text-align:center;background:var(--surface);border:1px solid var(--border);border-radius:var(--radius-xs);color:var(--text-main);font-size:0.78rem" onchange="setHelloPenPress(this.value)">
          </div>
        </div>

        <!-- Action / Execution Section -->
        <div style="margin-top:12px;display:flex;gap:8px;flex-wrap:wrap">
          <button class="btn btn-success" id="btnHelloPlay" style="flex:2;min-height:44px;font-size:0.95rem;font-weight:800" onclick="playHelloLetter()">
            ▶ BẮT ĐẦU VIẾT CHỮ <span id="btnHelloPlayLetter">H</span>
          </button>
          <button class="btn btn-danger" style="flex:1" onclick="api('/api/teach/stop')">⏹ DỪNG</button>
        </div>

        <!-- Move Paper & Next Letter Button -->
        <div style="margin-top:10px">
          <button class="btn btn-ghost btn-block" style="border-color:var(--primary);color:var(--primary);font-weight:700;min-height:40px" onclick="nextHelloLetter()">
            ➡️ ĐÃ VIẾT XONG! DỜI GIẤY SANG CHỮ TIẾP THEO ⏩
          </button>
        </div>

        <!-- Quick Template Preset Button -->
        <div style="margin-top:8px;display:flex;justify-content:space-between;align-items:center">
          <button class="btn btn-ghost" style="font-size:0.75rem;padding:4px 8px;min-height:30px" onclick="autoPresetCurrentHelloLetter()">
            ✨ Nạp nét mẫu chống bay giấy
          </button>
          <button class="btn btn-ghost" style="font-size:0.75rem;padding:4px 8px;min-height:30px" onclick="saveHelloAllToNvs()">
            💾 Lưu Flash NVS
          </button>
        </div>

        <!-- Points Table for current letter -->
        <div style="margin-top:10px;max-height:160px;overflow-y:auto">
          <table class="hello-table">
            <thead>
              <tr>
                <th style="width:30px">#</th>
                <th>Trạng thái</th>
                <th>J1 (Base)</th>
                <th>J2 (Vai)</th>
                <th>J3 (Khuỷu)</th>
              </tr>
            </thead>
            <tbody id="helloPointsTableBody">
              <tr><td colspan="5" style="text-align:center;color:var(--text-dim)">Chưa có điểm nào. Hãy ghi điểm hoặc nạp nét mẫu!</td></tr>
            </tbody>
          </table>
        </div>
      </div>
    </div>
  </div>

  <!-- =========================================================================
       TAB 1: DASHBOARD (Live Twin & System Health)
  ========================================================================== -->
  <div id="pane-dash" class="tab-pane" role="tabpanel" aria-labelledby="tab-dash" tabindex="0">
    <div class="grid-2col">
      <!-- Left Column: 3D Digital Twin -->
      <section class="card" aria-labelledby="headDashTwin">
        <div class="card-head">
          <h2 id="headDashTwin">Live robot</h2>
          <span class="meta" id="dashPoseLabel">TCP: (0.0, 0.0, 0.0)</span>
        </div>
        <div class="viewport-box">
          <div class="viewport-toolbar">
            <div class="toolbar-group">
              <button class="tool-btn active" onclick="setDashView('3d', this)" aria-pressed="true">3D Orbit</button>
              <button class="tool-btn" onclick="setDashView('side', this)" aria-pressed="false">Side (X-Z)</button>
              <button class="tool-btn" onclick="setDashView('top', this)" aria-pressed="false">Top (X-Y)</button>
            </div>
            <div class="toolbar-group">
              <button class="tool-btn" onclick="resetDashCam()">Reset View</button>
            </div>
          </div>
          <canvas id="dashCanvas" class="viewport-canvas" width="640" height="380" role="application" tabindex="0" aria-label="Mô hình 3D cánh tay robot" aria-describedby="dashCanvasHelp" aria-keyshortcuts="ArrowUp ArrowDown ArrowLeft ArrowRight Plus Minus 0"></canvas>
          <p id="dashCanvasHelp" class="sr-only">Kéo để xoay hoặc pan camera. Dùng các phím mũi tên để đổi góc nhìn, cộng trừ để zoom và phím 0 để reset.</p>
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
            <h2 id="headSysState">System</h2>
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
          <div class="stat-row"><span class="k">Khóa cổ tay (J4..J6)</span><span class="v" id="dashWristLock">Đang khóa 🔒</span></div>
          <div class="stat-row"><span class="k">Active Hold</span><span class="v" id="dashActiveHold">BẬT ⚓</span></div>
          <div class="stat-row"><span class="k">Dài bút (tool)</span><span class="v" id="dashToolLen">30.0 mm</span></div>
          <div class="stat-row"><span class="k">Công tắc Endstop</span><span class="v" id="dashEsInfo">Tất cả mở</span></div>
          <div class="stat-row"><span class="k">Tiến độ Homing</span><span class="v" id="dashHomingProg">Sẵn sàng</span></div>
          <div class="stat-row"><span class="k">Command latency</span><span class="v" id="dashCommandLatency">-- ms</span></div>

          <div class="home-track" aria-label="Trạng thái Homing từng trục">
            <span class="hchip" id="hc0">J1</span>
            <span class="hchip" id="hc1">J2</span>
            <span class="hchip" id="hc2">J3</span>
            <span class="hchip" id="hc3">J4</span>
            <span class="hchip" id="hc4">J5 (NVS)</span>
            <span class="hchip" id="hc5">J6 (NVS)</span>
          </div>

          <div class="btn-row">
            <button class="btn btn-showoff need-idle" onclick="startShowOff()">✨ SHOW OFF</button>
            <button class="btn btn-ghost" onclick="clearFault()">CLEAR FAULT</button>
            <button class="btn btn-danger" onclick="stopRobot()">⏹ STOP ALL</button>
          </div>
        </section>

        <section class="card" aria-labelledby="headQuickJoints">
          <div class="card-head">
            <h2 id="headQuickJoints">Joint positions</h2>
          </div>
          <div id="dashJointRows"></div>
        </section>
      </div>
    </div>
  </div>

  <!-- =========================================================================
       TAB 2: JOG & CALIBRATION (Unified Joints & Homing)
  ========================================================================== -->
  <div id="pane-jog" class="tab-pane" role="tabpanel" aria-labelledby="tab-jog" tabindex="0">
    <div class="jog-header-bar">
      <div class="operate-only" style="display:flex;align-items:center;gap:10px;flex-wrap:wrap">
        <span style="font-size:0.82rem;font-weight:600;color:var(--text-muted)">Bước Jog:</span>
        <div class="step-selector" id="stepSelector"></div>
        <button class="btn-toggle" id="btnWristLock" onclick="toggleWristLock()" title="Bật/Tắt khóa cụm cổ tay J4-J6">🔒 Khóa cổ tay</button>
        <button class="btn-toggle" id="btnActiveHold" onclick="toggleActiveHold()" title="Bật/Tắt giữ vị trí closed-loop">⚓ Active Hold: TẮT</button>
        <div class="tool-len-box">
          <span style="font-size:0.75rem;font-weight:600;color:var(--text-dim)">🖊️ Bút:</span>
          <input type="number" id="inputToolLength" min="0" max="300" step="1" style="width:50px;padding:3px 4px;background:var(--surface-elevated);border:1px solid var(--border);border-radius:var(--radius-xs);color:var(--text-main);font-size:0.8rem;text-align:center" value="30">
          <span style="font-size:0.75rem;color:var(--text-muted)">mm</span>
          <button class="btn btn-ghost" style="min-height:26px;padding:2px 8px;font-size:0.72rem" onclick="applyToolLength()">Lưu</button>
        </div>
      </div>
      <div class="setup-only" style="display:flex;gap:6px;flex-wrap:wrap">
        <button class="btn btn-ghost need-idle" onclick="api('/api/home/axis?axis=0')">Home J1</button>
        <button class="btn btn-ghost need-idle" onclick="api('/api/home/axis?axis=1')">Home J2</button>
        <button class="btn btn-ghost need-idle" onclick="api('/api/home/axis?axis=2')">Home J3</button>
        <button class="btn btn-ghost need-idle" id="btnHomeJ4" onclick="api('/api/home/axis?axis=3')">Home J4</button>
        <button class="btn btn-ghost need-idle" id="btnSetHomeJ5" onclick="confirmSetHome(4)">Set Home J5</button>
        <button class="btn btn-ghost need-idle" id="btnSetHomeJ6" onclick="confirmSetHome(5)">Set Home J6</button>
        <button class="btn btn-showoff need-idle" onclick="startShowOff()">✨ Show Off</button>
        <button class="btn btn-warning" onclick="clearFault()">🛡️ CLEAR FAULT</button>
        <button class="btn btn-danger" onclick="stopRobot()">⏹ STOP ALL</button>
      </div>
    </div>

    <div class="grid-cards-3" id="jointCardsGrid"></div>
    <section class="card" aria-labelledby="headGripper" style="margin-top:16px">
      <div class="card-head">
        <h2 id="headGripper">Gripper · MG90</h2>
        <span class="meta" id="gripperStatus" role="status">Waiting for connection</span>
      </div>
      <form class="btn-row" onsubmit="setGripper(event)">
        <label for="gripperAngle">Target angle (°)</label>
        <input id="gripperAngle" class="need-gripper" type="number" min="0" max="180" step="1" value="90" required disabled>
        <button class="btn btn-primary need-gripper" type="submit" disabled>Set angle</button>
        <button class="btn btn-danger" type="button" onclick="stopRobot()">STOP ALL</button>
      </form>
      <p class="meta">Target only, no position feedback. STOP ALL cuts servo pulses; the gripper may lose its hold.</p>
    </section>
  </div>

  <!-- =========================================================================
       TAB 3: TEACH & DRAW STUDIO (Unified Trajectory & Simulation Studio)
  ========================================================================== -->
  <div id="pane-motion" class="tab-pane" role="tabpanel" aria-labelledby="tab-motion" tabindex="0">
    <div class="grid-2col">
      <!-- Left Column: Interactive Simulation Twin & 3D Path Preview -->
      <section class="card" aria-labelledby="headSimTwin">
        <div class="card-head">
          <h2 id="headSimTwin">Quỹ đạo 3D &amp; Mô hình Arm</h2>
          <span class="badge b-run" id="simIkStatusBadge">3D Twin</span>
        </div>
        <div class="viewport-box">
          <div class="viewport-toolbar">
            <div class="toolbar-group">
              <button class="tool-btn active" onclick="setSimView('3d', this)" aria-pressed="true">3D Orbit</button>
              <button class="tool-btn" onclick="setSimView('side', this)" aria-pressed="false">Side (X-Z)</button>
              <button class="tool-btn" onclick="setSimView('top', this)" aria-pressed="false">Top (X-Y)</button>
            </div>
            <div class="toolbar-group">
              <button class="tool-btn live-badge" id="btnSimLive" onclick="setSimSource('live')" aria-pressed="false">🔴 Live Robot</button>
              <button class="tool-btn sim-badge active" id="btnSimSim" onclick="setSimSource('sim')" aria-pressed="true">🟢 Interactive Sim</button>
              <button class="tool-btn" onclick="resetSimCam()">Reset Cam</button>
            </div>
          </div>
          <canvas id="simCanvas" class="viewport-canvas" width="680" height="380" role="application" tabindex="0" aria-label="Mô phỏng quỹ đạo 3D" aria-describedby="simCanvasHelp"></canvas>
          <p id="simCanvasHelp" class="sr-only">Kéo để xoay hoặc pan camera. Dùng các phím mũi tên để đổi góc nhìn, cộng trừ để zoom và phím 0 để reset.</p>
        </div>

        <div class="hud-pill-bar">
          <div class="hud-item"><span class="lbl">Sim TCP (X, Y, Z)</span><span class="val accent" id="hudSimTcp">-- mm</span></div>
          <div class="hud-item"><span class="lbl">Trạng thái bút</span><span class="val" id="hudSimReach">Sẵn sàng</span></div>
          <div class="hud-item"><span class="lbl">Tiến độ mô phỏng</span><span class="val" id="hudSimPlay">0%</span></div>
        </div>

        <div style="margin-top:10px;display:flex;align-items:center;gap:8px">
          <button class="btn btn-ghost" id="btnSimPlay" onclick="toggleSimPlay()">▶ Chạy mô phỏng (Play)</button>
          <input type="range" id="simScrub" min="0" max="100" value="0" style="flex:1;accent-color:var(--primary);cursor:pointer" oninput="onSimScrubChange(this.value)">
          <span class="meta" id="simScrubVal" style="min-width:65px;text-align:right">0/0 (0%)</span>
        </div>
      </section>

      <!-- Right Column: Teach & Playback Studio -->
      <div>
        <section class="card teach-card" aria-labelledby="headTeach">
          <div class="card-head">
            <h2 id="headTeach">🎨 Dạy &amp; Chạy lại nét vẽ (Teach &amp; Playback)</h2>
            <span class="badge b-idle" id="teachTrajBadge">0 điểm</span>
          </div>
          <p class="meta">Bỏ IK hoàn toàn. Thả lỏng J1..J3 để kéo tay hoặc dùng Jog hỗ trợ lưu điểm. J4..J6 tự động khóa cứng 0°.</p>

          <div class="btn-row" style="margin-bottom:12px">
            <button id="btnTeachFree" class="btn btn-warning" onclick="toggleTeachFree()">🔓 Thả lỏng J1..J3 (Kéo tay)</button>
            <button class="btn btn-primary" onclick="teachRecordPoint(true)" style="font-weight:700">💾 GHI ĐIỂM VẼ (Hạ bút)</button>
            <button class="btn btn-ghost" onclick="teachRecordPoint(false)">✈️ GHI ĐIỂM NHẤC (Travel)</button>
            <button class="btn btn-ghost" onclick="teachUndoPoint()">↩ Hoàn tác</button>
            <button class="btn btn-danger" onclick="teachClearPoints()">🗑 Xóa hết</button>
          </div>

          <!-- Jog nhanh hỗ trợ khi hộp số kéo tay nặng -->
          <div class="teach-jog-box">
            <div class="teach-jog-header">
              <span class="teach-jog-title">Jog hỗ trợ (hộp số 20:1):</span>
              <div class="step-selector">
                <button class="step-btn" id="btnJStep1" onclick="setTeachJogStep(1)">1°</button>
                <button class="step-btn active" id="btnJStep5" onclick="setTeachJogStep(5)">5°</button>
                <button class="step-btn" id="btnJStep10" onclick="setTeachJogStep(10)">10°</button>
              </div>
            </div>
            <div class="teach-jog-grid">
              <div class="teach-jog-pair">
                <button class="btn btn-ghost" onclick="jogTeach(1,-1)" aria-label="Jog J1 âm">J1 &minus;</button>
                <button class="btn btn-ghost" onclick="jogTeach(1,1)" aria-label="Jog J1 dương">J1 +</button>
              </div>
              <div class="teach-jog-pair">
                <button class="btn btn-ghost" onclick="jogTeach(2,-1)" aria-label="Jog J2 âm">J2 &minus;</button>
                <button class="btn btn-ghost" onclick="jogTeach(2,1)" aria-label="Jog J2 dương">J2 +</button>
              </div>
              <div class="teach-jog-pair">
                <button class="btn btn-ghost" onclick="jogTeach(3,-1)" aria-label="Jog J3 âm">J3 &minus;</button>
                <button class="btn btn-ghost" onclick="jogTeach(3,1)" aria-label="Jog J3 dương">J3 +</button>
              </div>
            </div>
          </div>

          <div class="teach-actions">
            <div class="btn-row" style="margin-top:0">
              <button class="btn btn-success" id="btnTeachPlay" onclick="teachPlayTrajectory()" style="font-weight:700">▶ BẮT ĐẦU CHẠY TRÊN ROBOT</button>
              <button class="btn btn-danger" onclick="api('/api/teach/stop')">⏹ DỪNG</button>
            </div>
            <div class="btn-row" style="margin-top:0">
              <button class="btn btn-ghost" onclick="downloadTeachJson()">📥 Tải JSON</button>
              <button class="btn btn-ghost" onclick="document.getElementById('fileTeachUpload').click()">📤 Nạp JSON</button>
              <button class="btn btn-ghost" onclick="saveTeachToNvs()">💾 Lưu Flash (NVS)</button>
              <button class="btn btn-ghost" onclick="loadTeachFromNvs()">📂 Đọc Flash (NVS)</button>
              <input type="file" id="fileTeachUpload" accept=".json" style="display:none" aria-hidden="true" tabindex="-1" onchange="uploadTeachJson(event)">
            </div>
          </div>
          <div id="teachPlayProgress" class="teach-progress" role="status" aria-live="polite"></div>
        </section>

        <!-- Simulation Fine Joint Sliders Fold -->
        <section class="card" aria-labelledby="headSimJointSliders">
          <div class="card-head">
            <h2 id="headSimJointSliders">Simulation joints (Fine-tune)</h2>
          </div>
          <div id="simJointSliders"></div>
        </section>
      </div>
    </div>
  </div>

  <!-- =========================================================================
       TAB 4: SETTINGS & WIFI
  ========================================================================== -->
  <div id="pane-settings" class="tab-pane" role="tabpanel" aria-labelledby="tab-settings" tabindex="0">
    <div class="grid-2col">
      <section class="card" aria-labelledby="headWifiCfg">
        <div class="card-head">
          <h2 id="headWifiCfg">WiFi</h2>
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
          <button class="btn btn-success btn-block need-idle" onclick="saveWifi()">💾 LƯU VÀO NVS &amp; KHỞI ĐỘNG LẠI</button>
        </div>
      </section>

      <section class="card" aria-labelledby="headHardwareRef">
        <div class="card-head">
          <h2 id="headHardwareRef">Hardware reference</h2>
          <span class="meta">Craig MDH</span>
        </div>
        <div class="stat-row"><span class="k">D1 (Trụ đế)</span><span class="v">139.0 mm</span></div>
        <div class="stat-row"><span class="k">A2 (Cánh tay dưới)</span><span class="v">138.0 mm</span></div>
        <div class="stat-row"><span class="k">A3 + D4 (Cẳng tay)</span><span class="v">88.0 + 125.0 mm (L=152.87mm)</span></div>
        <div class="stat-row"><span class="k">D6 + Tool (Bút vẽ)</span><span class="v">55.0 + 30.0 mm (D_eff=85.0mm)</span></div>
        <div class="stat-row"><span class="k">Bộ truyền động</span><span class="v">J1-J4: TMC2209 | J5-J6: A4988</span></div>
        <div class="stat-row"><span class="k">Cảm biến vị trí</span><span class="v">AS5600 × 6 (PCA9548A I2C)</span></div>
      </section>
    </div>
  </div>
</main>

<button id="estop" class="btn btn-danger" onclick="stopRobot()" aria-label="Dừng khẩn cấp toàn bộ cánh tay robot">⚠️ E-STOP</button>
<div id="toast" role="status" aria-live="assertive"></div>

<script>
/* =============================================================================
   1. HARDWARE CONSTANTS & CRAIG MODIFIED DH PARAMETERS (src/config.h)
============================================================================= */
const CURRENT_TWIN = {
  geometry: [139.0, 138.0, 88.0, 125.0, 55.0, 30.0],
  offsets: [-90.0, 0.0],
  limits: [[-90,90],[-90,90],[-20,90],[-75,75],[-90,90],[-360,360]]
};
let D1, A2, A3, D4, D6, D_TOOL, D_TOOL_EFF, L_FORE, DELTA_WRIST, DELTA_RAD;
let THETA2_OFFSET, THETA5_OFFSET;
const PEN_LIFT = 5.0, LINE_SEGMENT = 2.0;
const DRAW_PLANE_Z = 20.0;
let JOINT_LIMITS;
const AXES = ["J1 Base Yaw", "J2 Shoulder", "J3 Elbow", "J4 Wrist Pan", "J5 Revolute", "J6 Revolute"];

function setTwinConfig(geometry, offsets, limits){
  [D1, A2, A3, D4, D6, D_TOOL] = geometry;
  [THETA2_OFFSET, THETA5_OFFSET] = offsets;
  D_TOOL_EFF = D6 + D_TOOL;
  L_FORE = Math.hypot(A3, D4);
  DELTA_RAD = Math.atan2(D4, A3);
  DELTA_WRIST = rad2deg(DELTA_RAD);
  JOINT_LIMITS = limits.map(pair => [...pair]);
}
setTwinConfig(CURRENT_TWIN.geometry, CURRENT_TWIN.offsets, CURRENT_TWIN.limits);

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
  const th = [ enc[0], enc[1] + THETA2_OFFSET, enc[2], enc[3], enc[4] + THETA5_OFFSET, enc[5] ];
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

function liveVisualizationKinematics(enc){
  const visualEnc = [...enc];
  return forwardKinematics(visualEnc);
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
    const e2 = rad2deg(t2) - THETA2_OFFSET;
    const e3 = rad2deg(t3);

    if(e1 < JOINT_LIMITS[0][0] || e1 > JOINT_LIMITS[0][1]) continue;
    if(e2 < JOINT_LIMITS[1][0] || e2 > JOINT_LIMITS[1][1]) continue;
    if(e3 < JOINT_LIMITS[2][0] || e3 > JOINT_LIMITS[2][1]) continue;

    if(!best || Math.abs(e3) < bestJ3){
      best = [e1, e2, e3, 0.0, 0.0, 0.0]; // Khóa cứng J4, J5, J6 = 0.0°
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
    cv.addEventListener('pointerdown', e => {
      this.isDragging = true;
      this.lastX = e.clientX; this.lastY = e.clientY;
      cv.setPointerCapture(e.pointerId);
    });
    cv.addEventListener('pointermove', e => {
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
    const endDrag = e => {
      this.isDragging = false;
      if(cv.hasPointerCapture(e.pointerId)) cv.releasePointerCapture(e.pointerId);
    };
    cv.addEventListener('pointerup', endDrag);
    cv.addEventListener('pointercancel', endDrag);
    cv.addEventListener('wheel', e => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.08 : 0.92;
      this.cam.zoom = Math.max(0.4, Math.min(3.5, this.cam.zoom * factor));
      this.renderCurrent();
    }, { passive: false });
    cv.addEventListener('keydown', e => {
      const step = e.shiftKey ? 24 : 8;
      let handled = true;
      if(e.key === 'ArrowLeft') this.viewMode === '3d' ? this.cam.yaw -= step : this.cam.panX -= step;
      else if(e.key === 'ArrowRight') this.viewMode === '3d' ? this.cam.yaw += step : this.cam.panX += step;
      else if(e.key === 'ArrowUp') this.viewMode === '3d' ? this.cam.pitch = Math.min(85, this.cam.pitch + step) : this.cam.panY -= step;
      else if(e.key === 'ArrowDown') this.viewMode === '3d' ? this.cam.pitch = Math.max(-85, this.cam.pitch - step) : this.cam.panY += step;
      else if(e.key === '+' || e.key === '=') this.cam.zoom = Math.min(3.5, this.cam.zoom * 1.08);
      else if(e.key === '-' || e.key === '_') this.cam.zoom = Math.max(0.4, this.cam.zoom * 0.92);
      else if(e.key === '0') this.cam = { yaw: -45, pitch: 25, zoom: 1.15, panX: 0, panY: 0 };
      else handled = false;
      if(handled) { e.preventDefault(); this.renderCurrent(); }
    });
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
    if(pathPts && pathPts.length > 1){
      for(let i = 1; i < pathPts.length; i++){
        const pPrev = this.project([pathPts[i-1].x, pathPts[i-1].y, pathPts[i-1].z]);
        const pCurr = this.project([pathPts[i].x, pathPts[i].y, pathPts[i].z]);
        const isDrawing = (pathPts[i].drawing === true || (pathPts[i].drawing === undefined && pathPts[i].pen !== false));
        ctx.beginPath();
        if(isDrawing){
          ctx.strokeStyle = THEME_PALETTE.penTool;
          ctx.lineWidth = 2.5;
          ctx.setLineDash([]);
        } else {
          ctx.strokeStyle = '#f59e0b';
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 4]);
        }
        ctx.moveTo(pPrev.u, pPrev.v);
        ctx.lineTo(pCurr.u, pCurr.v);
        ctx.stroke();
      }
      ctx.setLineDash([]);
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
let stepSize = 1.0, statusPollController = null, failN = 0, toastTimer = null;
let latestStatus = null, pendingCommands = 0;
let drawProfiles = [], selectedDrawProfile = 0;
let helloRunId = 0;

function isPaneActive(id){
  const pane = document.getElementById(id);
  return !!pane && pane.classList.contains('active');
}

function syncCommandState(){
  document.querySelectorAll('.need-gripper').forEach(b => {
    b.disabled = !latestStatus?.gripper?.available || failN > 0 || pendingCommands > 0;
  });
  const moving = !!latestStatus && latestStatus.busy;
  const fault = !!latestStatus && latestStatus.mode === 'fault';
  const released = !!latestStatus && latestStatus.mode === 'release';
  document.querySelectorAll('.need-idle').forEach(b => { b.disabled = moving || fault || pendingCommands > 0; });
  const releaseButton = document.getElementById('btnReleaseJ1J4');
  if(releaseButton) releaseButton.disabled = moving || released || pendingCommands > 0;
  document.querySelectorAll('.need-release').forEach(b => { b.disabled = !released || pendingCommands > 0; });
}

function pendingTrigger(){
  const el = document.activeElement;
  return el instanceof HTMLButtonElement ? el : null;
}

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

  updateSimFromAngles(simAngles);
  updateTeachPathPreview();
}

function setDashView(v, btn){
  dashRenderer.viewMode = v;
  if(btn){
    btn.parentElement.querySelectorAll('.tool-btn').forEach(b => { b.classList.remove('active'); b.setAttribute('aria-pressed', 'false'); });
    btn.classList.add('active');
    btn.setAttribute('aria-pressed', 'true');
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
    btn.parentElement.querySelectorAll('.tool-btn').forEach(b => { b.classList.remove('active'); b.setAttribute('aria-pressed', 'false'); });
    btn.classList.add('active');
    btn.setAttribute('aria-pressed', 'true');
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
  if(bLive) bLive.setAttribute('aria-pressed', String(src === 'live'));
  if(bSim) bSim.setAttribute('aria-pressed', String(src === 'sim'));
  if(src === 'live' && window.lastRobotAngles){
    updateSimFromAngles(window.lastRobotAngles);
  }
}

function updateSimFromAngles(angles){
  simAngles = [...angles];
  const lm = simSource === 'live' ? liveVisualizationKinematics(simAngles) : forwardKinematics(simAngles);

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

  simRenderer.lastLm = lm;
  simRenderer.lastPath = simWaypoints;
  if(isPaneActive('pane-motion')) simRenderer.renderCurrent();

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

function startShowOff(){
  post('/api/showoff', '');
  toast('Đang khởi động điệu múa Show Off 6 trục đồng bộ!', 'info');
}

function onSimScrubChange(val){
  const idx = parseInt(val);
  simScrubIdx = idx;
  const scrubVal = document.getElementById('simScrubVal');
  const hudSimPlay = document.getElementById('hudSimPlay');
  const total = (s_teachPoints && s_teachPoints.length) ? s_teachPoints.length : 1;
  const pct = `${idx + 1}/${total} (${Math.round(idx / Math.max(1, total - 1) * 100)}%)`;
  if(scrubVal) scrubVal.textContent = pct;
  if(hudSimPlay) hudSimPlay.textContent = pct;

  if(s_teachPoints && s_teachPoints[idx]){
    const pt = s_teachPoints[idx];
    const angles = [pt[0], pt[1], pt[2], 0, 0, 0];
    updateSimFromAngles(angles);
    const hudReach = document.getElementById('hudSimReach');
    if(hudReach) hudReach.textContent = pt[3] ? '🖊 ĐANG VẼ (Hạ bút)' : '✈️ DI CHUYỂN (Nhấc bút)';
  }
}

function simAnimStep(now){
  if(!simPlayAnimId) return;
  if(!document.hidden && now - simLastAnimTime >= 40){
    simLastAnimTime = now;
    const total = (s_teachPoints && s_teachPoints.length) ? s_teachPoints.length : 1;
    simScrubIdx = (simScrubIdx + 1) % total;
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
    if(!s_teachPoints || s_teachPoints.length < 2){
      toast('Cần ít nhất 2 điểm để mô phỏng', 'err');
      return;
    }
    setSimSource('sim');
    if(btn) btn.textContent = '⏸ Tạm dừng (Pause)';
    simLastAnimTime = performance.now();
    simPlayAnimId = requestAnimationFrame(simAnimStep);
  }
}

/* =============================================================================
   6. REST API & UI BINDINGS
============================================================================= */
function toast(msg, cls){
  const t = document.getElementById('toast');
  if(!t) return;
  t.style.display = '';
  t.textContent = msg;
  t.className = (cls ? ('t-' + cls) : '') + ' show';
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.className = ''; }, 2600);
}
async function requestCommand(url, options, trigger){
  if(statusPollController) statusPollController.abort();
  const control = trigger || pendingTrigger();
  if(control) {
    control.disabled = true;
    control.setAttribute('aria-busy', 'true');
    control.classList.add('btn-loading');
  }
  pendingCommands++;
  syncCommandState();
  try {
    const response = await fetch(url, options);
    const text = await response.text();
    if(!response.ok){
      toast(text || `Lệnh thất bại (${response.status})`, 'err');
      return null;
    }
    if(!text.startsWith('{')){
      toast((text === 'OK' ? '✓ ' : '') + (text || 'Đã nhận lệnh'), 'ok');
    }
    return text;
  } catch (_) {
    toast('Lỗi mạng / Mất kết nối robot', 'err');
    return null;
  } finally {
    pendingCommands = Math.max(0, pendingCommands - 1);
    if(control) {
      control.removeAttribute('aria-busy');
      control.classList.remove('btn-loading');
    }
    if(control && !control.classList.contains('need-idle')) control.disabled = false;
    syncCommandState();
    pollOnce();
  }
}
function api(url, trigger){
  return requestCommand(url, {method:'POST'}, trigger);
}
function stopRobot(){
  helloRunId++;
  return api('/api/stop');
}
function post(url, body, trigger){
  return requestCommand(url, { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body }, trigger);
}
function clearFault(){ post('/api/jog', 'fault_clear=1'); }
function jog(axis, dir){
  if(axis >= 3 && wristLockedState){ toast('Trục cổ tay J4..J6 đang khóa cứng!', 'warn'); return; }
  post('/api/jog', `axis=${axis}&deg=${dir * stepSize}`);
}
function setGripper(event){
  event.preventDefault();
  const input = document.getElementById('gripperAngle');
  if(input.reportValidity()) post('/api/gripper', `deg=${encodeURIComponent(input.value)}`, event.submitter);
}
let s_teachJogStep = 5;
let s_teachIsFree = false;

function setTeachJogStep(deg) {
  s_teachJogStep = deg;
  ['btnJStep1', 'btnJStep5', 'btnJStep10'].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.classList.toggle('active', id === 'btnJStep' + deg);
  });
  toast('Bước Jog: ' + deg + '°');
}

function jogTeach(axis, sign) {
  post('/api/jog', `axis=${axis - 1}&deg=${sign * s_teachJogStep}`);
}

function toggleTeachFree() {
  const nextFree = !s_teachIsFree;
  api(`/api/teach/free?free=${nextFree ? 1 : 0}`).then(ok => {
    if (ok === null) return;
    s_teachIsFree = nextFree;
    const btn = document.getElementById('btnTeachFree');
    if (btn) {
      btn.textContent = s_teachIsFree ? '🔒 Khóa J1..J3' : '🔓 Thả lỏng J1..J3 (Kéo tay)';
      btn.className = s_teachIsFree ? 'btn btn-primary' : 'btn btn-warning';
    }
  });
}

let s_teachPoints = [];

function updateTeachPathPreview() {
  fetch('/api/teach/points')
    .then(r => r.json())
    .then(pts => {
      s_teachPoints = pts || [];
      const scrubEl = document.getElementById('simScrub');
      if(scrubEl) {
        scrubEl.max = Math.max(1, s_teachPoints.length - 1);
        if(simScrubIdx >= s_teachPoints.length) simScrubIdx = 0;
        scrubEl.value = simScrubIdx;
      }
      if(!pts || !pts.length) {
        simWaypoints = [];
        if(dashRenderer) dashRenderer.lastPath = null;
        if(simRenderer) simRenderer.lastPath = null;
        const badge = document.getElementById('teachTrajBadge');
        if(badge) { badge.textContent = '0 điểm'; badge.className = 'badge b-idle'; }
        onSimScrubChange(0);
        return;
      }
      const downCount = pts.filter(p => p[3] !== 0).length;
      const badge = document.getElementById('teachTrajBadge');
      if(badge) {
        badge.textContent = `${pts.length} điểm (${downCount} nét vẽ)`;
        badge.className = 'badge b-run';
      }
      const wps = pts.map(p => {
        const angles = [p[0], p[1], p[2], 0, 0, 0];
        const lm = forwardKinematics(angles);
        return { x: lm.tcp[0], y: lm.tcp[1], z: lm.tcp[2], pen: (p[3] !== 0) };
      });
      simWaypoints = wps;
      if(dashRenderer) dashRenderer.lastPath = wps;
      if(simRenderer) simRenderer.lastPath = wps;
      onSimScrubChange(simScrubIdx);
      if(isPaneActive('pane-motion')) simRenderer.renderCurrent();
      if(isPaneActive('pane-dash')) dashRenderer.renderCurrent();
    })
    .catch(() => {});
}

function teachRecordPoint(penDown) {
  const isDown = (penDown !== false);
  post('/api/teach/record', `pen=${isDown ? 1 : 0}`).then(ok => {
    if (ok !== null) setTimeout(updateTeachPathPreview, 50);
  });
}

function teachUndoPoint() {
  api('/api/teach/undo').then(ok => {
    if (ok !== null) setTimeout(updateTeachPathPreview, 50);
  });
}

function teachClearPoints() {
  if (confirm('Xóa toàn bộ điểm dạy vẽ trong RAM?')) {
    api('/api/teach/clear').then(ok => {
      if (ok !== null) setTimeout(updateTeachPathPreview, 50);
    });
  }
}

function teachPlayTrajectory() {
  api('/api/teach/play?step_deg=0.5&speed=15');
}

function downloadTeachJson() {
  fetch('/api/teach/points')
    .then(r => r.json())
    .then(data => {
      const blob = new Blob([JSON.stringify(data, null, 2)], {type: 'application/json'});
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = 'draw_trajectory.json';
      a.click();
      toast('Đã tải file draw_trajectory.json');
    })
    .catch(e => toast('Lỗi tải file', 'err'));
}

function uploadTeachJson(evt) {
  const file = evt.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = function(e) {
    fetch('/api/teach/load', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: e.target.result
    }).then(r => {
      if (r.ok) {
        toast('Nạp file JSON thành công!');
        setTimeout(updateTeachPathPreview, 150);
      } else toast('Lỗi nội dung file JSON', 'err');
    });
  };
  reader.readAsText(file);
}

function saveTeachToNvs() {
  post('/api/teach/nvs/save')
    .then(txt => {
      if(!txt) return;
      try {
        const d = JSON.parse(txt);
        if(d.ok) toast('💾 Đã lưu quỹ đạo vào Flash (NVS)!', 'ok');
        else toast('❌ Lưu Flash thất bại: ' + (d.msg || ''), 'err');
      } catch(e) {
        toast('❌ Lỗi lưu Flash', 'err');
      }
    })
    .catch(() => toast('❌ Lỗi kết nối Flash', 'err'));
}

function loadTeachFromNvs() {
  post('/api/teach/nvs/load')
    .then(txt => {
      if(!txt) return;
      try {
        const d = JSON.parse(txt);
        if(d.ok) {
          toast(`📂 Đã nạp ${d.count} điểm từ Flash (NVS)!`, 'ok');
          updateTeachPathPreview();
        } else {
          toast('❌ ' + (d.msg || 'Không có quỹ đạo trong Flash'), 'err');
        }
      } catch(e) {
        toast('❌ Lỗi nạp Flash', 'err');
      }
    })
    .catch(() => toast('❌ Lỗi nạp Flash', 'err'));
}
/* =========================================================================
   DEDICATED HELLO STUDIO JAVASCRIPT ENGINE
   ========================================================================= */
let s_lastStatus = null;
let s_helloJogStep = 5;
let s_activeHelloLetter = localStorage.getItem('hello_active_letter') || 'H';
if (!['H', 'E', 'L1', 'L2', 'O'].includes(s_activeHelloLetter)) s_activeHelloLetter = 'H';
const HELLO_LETTERS = ['H', 'E', 'L1', 'L2', 'O'];

const HELLO_CONFIG = {
  H: {
    title: 'Chữ H (3 nét: Cột trái -> Cột phải -> Gạch ngang)',
    svg: `<svg viewBox="0 0 220 220" width="180" height="180">
      <rect x="30" y="20" width="160" height="180" fill="none" stroke="rgba(255,255,255,0.08)" stroke-dasharray="4"/>
      <line x1="60" y1="35" x2="60" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="60" y1="185" x2="160" y2="35" stroke="#f59e0b" stroke-width="2" stroke-dasharray="6"/>
      <line x1="160" y1="35" x2="160" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="160" y1="185" x2="60" y2="110" stroke="#f59e0b" stroke-width="2" stroke-dasharray="6"/>
      <line x1="60" y1="110" x2="160" y2="110" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <circle cx="60" cy="35" r="9" fill="#38bdf8"/><text x="60" y="38" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">1</text>
      <circle cx="60" cy="185" r="9" fill="#38bdf8"/><text x="60" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">2</text>
      <circle cx="160" cy="35" r="9" fill="#f59e0b"/><text x="160" y="38" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">3</text>
      <circle cx="160" cy="185" r="9" fill="#38bdf8"/><text x="160" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">4</text>
      <circle cx="60" cy="110" r="9" fill="#f59e0b"/><text x="60" y="113" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">5</text>
      <circle cx="160" cy="110" r="9" fill="#38bdf8"/><text x="160" y="113" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">6</text>
    </svg>`,
    steps: [
      { id: 1, text: 'Đỉnh cột trái (Hạ bút)', pen: true },
      { id: 2, text: 'Chân cột trái (Hạ bút)', pen: true },
      { id: 3, text: 'Đỉnh cột phải (Nhấc bút bay sang)', pen: false },
      { id: 4, text: 'Chân cột phải (Hạ bút)', pen: true },
      { id: 5, text: 'Giữa cột trái (Nhấc bút bay sang)', pen: false },
      { id: 6, text: 'Giữa cột phải (Hạ bút gạch ngang)', pen: true },
      { id: 7, text: 'Nhấc bút an toàn kết thúc', pen: false }
    ]
  },
  E: {
    title: 'Chữ E (4 nét: Cột đứng -> Gạch đáy -> Gạch giữa -> Gạch trên)',
    svg: `<svg viewBox="0 0 220 220" width="180" height="180">
      <rect x="30" y="20" width="160" height="180" fill="none" stroke="rgba(255,255,255,0.08)" stroke-dasharray="4"/>
      <line x1="60" y1="35" x2="60" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="60" y1="185" x2="160" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="160" y1="185" x2="60" y2="110" stroke="#f59e0b" stroke-width="2" stroke-dasharray="6"/>
      <line x1="60" y1="110" x2="145" y2="110" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="145" y1="110" x2="60" y2="35" stroke="#f59e0b" stroke-width="2" stroke-dasharray="6"/>
      <line x1="60" y1="35" x2="160" y2="35" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <circle cx="60" cy="35" r="9" fill="#38bdf8"/><text x="60" y="38" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">1</text>
      <circle cx="60" cy="185" r="9" fill="#38bdf8"/><text x="60" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">2</text>
      <circle cx="160" cy="185" r="9" fill="#38bdf8"/><text x="160" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">3</text>
      <circle cx="60" cy="110" r="9" fill="#f59e0b"/><text x="60" y="113" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">4</text>
      <circle cx="145" cy="110" r="9" fill="#38bdf8"/><text x="145" y="113" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">5</text>
      <circle cx="60" cy="35" r="7" fill="#f59e0b"/><text x="60" y="38" fill="#000" font-size="8" font-weight="bold" text-anchor="middle">6</text>
      <circle cx="160" cy="35" r="9" fill="#38bdf8"/><text x="160" y="38" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">7</text>
    </svg>`,
    steps: [
      { id: 1, text: 'Đỉnh cột đứng (Hạ bút)', pen: true },
      { id: 2, text: 'Góc dưới trái (Hạ bút)', pen: true },
      { id: 3, text: 'Đầu gạch đáy phải (Hạ bút)', pen: true },
      { id: 4, text: 'Giữa cột đứng (Nhấc bút bay sang)', pen: false },
      { id: 5, text: 'Đầu gạch giữa (Hạ bút)', pen: true },
      { id: 6, text: 'Đỉnh cột đứng (Nhấc bút bay lên)', pen: false },
      { id: 7, text: 'Đầu gạch trên phải (Hạ bút)', pen: true },
      { id: 8, text: 'Nhấc bút an toàn kết thúc', pen: false }
    ]
  },
  L1: {
    title: 'Chữ L thứ nhất (2 nét: Cột đứng -> Gạch chân)',
    svg: `<svg viewBox="0 0 220 220" width="180" height="180">
      <rect x="30" y="20" width="160" height="180" fill="none" stroke="rgba(255,255,255,0.08)" stroke-dasharray="4"/>
      <line x1="70" y1="35" x2="70" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="70" y1="185" x2="165" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <circle cx="70" cy="35" r="9" fill="#38bdf8"/><text x="70" y="38" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">1</text>
      <circle cx="70" cy="185" r="9" fill="#38bdf8"/><text x="70" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">2</text>
      <circle cx="165" cy="185" r="9" fill="#38bdf8"/><text x="165" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">3</text>
    </svg>`,
    steps: [
      { id: 1, text: 'Đỉnh cột đứng (Hạ bút)', pen: true },
      { id: 2, text: 'Góc đáy (Hạ bút)', pen: true },
      { id: 3, text: 'Đầu gạch chân phải (Hạ bút)', pen: true },
      { id: 4, text: 'Nhấc bút an toàn kết thúc', pen: false }
    ]
  },
  L2: {
    title: 'Chữ L thứ hai (2 nét: Cột đứng -> Gạch chân)',
    svg: `<svg viewBox="0 0 220 220" width="180" height="180">
      <rect x="30" y="20" width="160" height="180" fill="none" stroke="rgba(255,255,255,0.08)" stroke-dasharray="4"/>
      <line x1="70" y1="35" x2="70" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="70" y1="185" x2="165" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <circle cx="70" cy="35" r="9" fill="#38bdf8"/><text x="70" y="38" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">1</text>
      <circle cx="70" cy="185" r="9" fill="#38bdf8"/><text x="70" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">2</text>
      <circle cx="165" cy="185" r="9" fill="#38bdf8"/><text x="165" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">3</text>
    </svg>`,
    steps: [
      { id: 1, text: 'Đỉnh cột đứng (Hạ bút)', pen: true },
      { id: 2, text: 'Góc đáy (Hạ bút)', pen: true },
      { id: 3, text: 'Đầu gạch chân phải (Hạ bút)', pen: true },
      { id: 4, text: 'Nhấc bút an toàn kết thúc', pen: false }
    ]
  },
  O: {
    title: 'Chữ O (Hình chữ nhật: Đỉnh ngang -> Cạnh phải -> Đáy ngang -> Cạnh trái khép góc)',
    svg: `<svg viewBox="0 0 220 220" width="180" height="180">
      <rect x="30" y="20" width="160" height="180" fill="none" stroke="rgba(255,255,255,0.08)" stroke-dasharray="4"/>
      <line x1="60" y1="35" x2="160" y2="35" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="160" y1="35" x2="160" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="160" y1="185" x2="60" y2="185" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <line x1="60" y1="185" x2="60" y2="35" stroke="#10b981" stroke-width="6" stroke-linecap="round"/>
      <circle cx="60" cy="35" r="9" fill="#38bdf8"/><text x="60" y="38" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">1</text>
      <circle cx="160" cy="35" r="9" fill="#38bdf8"/><text x="160" y="38" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">2</text>
      <circle cx="160" cy="185" r="9" fill="#38bdf8"/><text x="160" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">3</text>
      <circle cx="60" cy="185" r="9" fill="#38bdf8"/><text x="60" y="188" fill="#000" font-size="10" font-weight="bold" text-anchor="middle">4</text>
      <circle cx="60" cy="35" r="5" fill="#f59e0b"/>
    </svg>`,
    steps: [
      { id: 1, text: 'Góc trên trái (Hạ bút)', pen: true },
      { id: 2, text: 'Góc trên phải (Hạ bút nét ngang đỉnh)', pen: true },
      { id: 3, text: 'Góc dưới phải (Hạ bút nét đứng phải)', pen: true },
      { id: 4, text: 'Góc dưới trái (Hạ bút nét ngang đáy)', pen: true },
      { id: 5, text: 'Góc trên trái (Hạ bút nét đứng trái khép kín)', pen: true },
      { id: 6, text: 'Nhấc bút an toàn kết thúc', pen: false }
    ]
  }
};

function setHelloJogStep(deg) {
  s_helloJogStep = deg;
  ['btnHStep1', 'btnHStep5', 'btnHStep10'].forEach(id => {
    const el = document.getElementById(id);
    if(el) el.classList.toggle('active', id === 'btnHStep' + deg);
  });
  toast('Bước Jog: ' + deg + '°');
}

function jogHello(axis, sign) {
  post('/api/jog', `axis=${axis - 1}&deg=${sign * s_helloJogStep}`);
}

function toggleHelloFree() {
  const nextFree = !s_teachIsFree;
  api(`/api/teach/free?free=${nextFree ? 1 : 0}`).then(ok => {
    if (ok === null) return;
    s_teachIsFree = nextFree;
    const btn = document.getElementById('btnHelloFree');
    if (btn) {
      btn.textContent = s_teachIsFree ? '🔒 Khóa giữ tay' : '🔓 Thả lỏng tay để kéo';
      btn.className = s_teachIsFree ? 'btn btn-primary' : 'btn btn-warning';
    }
    const btnTeach = document.getElementById('btnTeachFree');
    if(btnTeach) {
      btnTeach.textContent = s_teachIsFree ? '🔒 Khóa J1..J3' : '🔓 Thả lỏng J1..J3 (Kéo tay)';
      btnTeach.className = s_teachIsFree ? 'btn btn-primary' : 'btn btn-warning';
    }
  });
}

function selectHelloLetter(letter) {
  if(!HELLO_LETTERS.includes(letter)) return;
  s_activeHelloLetter = letter;
  localStorage.setItem('hello_active_letter', letter);
  HELLO_LETTERS.forEach(l => {
    const card = document.getElementById('cardLetter' + l);
    if(card) card.classList.toggle('active', l === letter);
  });
  const playLet = document.getElementById('btnHelloPlayLetter');
  if(playLet) playLet.textContent = letter;
  const cfg = HELLO_CONFIG[letter];
  if(cfg) {
    const tit = document.getElementById('helloLetterTitle');
    if(tit) tit.textContent = 'Sơ đồ nét: ' + cfg.title;
    const diag = document.getElementById('helloDiagramWrap');
    if(diag) diag.innerHTML = cfg.svg;
  }

  const saved = localStorage.getItem('hello_letter_' + letter);
  let loadedPts = [];
  if(saved) {
    try {
      const parsed = JSON.parse(saved);
      if(Array.isArray(parsed) && parsed.length > 0) loadedPts = parsed;
    } catch(_) {}
  }

  if(loadedPts.length > 0) {
    // Chữ cái này đã có bộ nhớ riêng -> Nạp vào RAM robot
    s_teachPoints = loadedPts;
    updateHelloPointsTable();
    updateAllLetterBadges();
    updateTeachPathPreview();
    fetch('/api/teach/load', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(loadedPts)
    });
  } else {
    // Chữ cái này chưa có điểm -> Xóa sạch buffer trong RAM robot, bắt đầu từ điểm 1!
    s_teachPoints = [];
    localStorage.removeItem('hello_letter_' + letter);
    updateHelloPointsTable();
    updateAllLetterBadges();
    updateTeachPathPreview();
    api('/api/teach/clear');
  }
}

function syncHelloPointsFromRobot() {
  fetch('/api/teach/points')
    .then(r => r.json())
    .then(pts => {
      s_teachPoints = pts || [];
      if(s_teachPoints.length > 0) {
        localStorage.setItem('hello_letter_' + s_activeHelloLetter, JSON.stringify(s_teachPoints));
      } else {
        localStorage.removeItem('hello_letter_' + s_activeHelloLetter);
      }
      updateHelloPointsTable();
      updateAllLetterBadges();
      updateTeachPathPreview();
    })
    .catch(() => {});
}

function updateHelloPointsTable() {
  const tbody = document.getElementById('helloPointsTableBody');
  const badgeStep = document.getElementById('helloActiveStepBadge');
  if(!tbody) return;
  if(!s_teachPoints || s_teachPoints.length === 0) {
    tbody.innerHTML = '<tr><td colspan="5" style="text-align:center;color:var(--text-dim)">Chưa có điểm nào. Hãy bấm "GHI ĐIỂM" hoặc "Nạp nét mẫu"!</td></tr>';
    if(badgeStep) badgeStep.textContent = 'Điểm tiếp theo: #1';
    return;
  }
  let html = '';
  s_teachPoints.forEach((p, idx) => {
    const isDown = (p[3] !== 0);
    const badge = isDown
      ? '<span style="color:var(--success);font-weight:700">🖊️ Hạ bút (Vẽ)</span>'
      : '<span style="color:var(--warning);font-weight:700">✈️ Nhấc bút (Bay)</span>';
    html += `<tr>
      <td>${idx + 1}</td>
      <td>${badge}</td>
      <td>${p[0].toFixed(1)}°</td>
      <td>${p[1].toFixed(1)}°</td>
      <td>${p[2].toFixed(1)}°</td>
    </tr>`;
  });
  tbody.innerHTML = html;
  if(badgeStep) {
    const cfg = HELLO_CONFIG[s_activeHelloLetter];
    const n = s_teachPoints.length + 1;
    const desc = (cfg && cfg.steps && cfg.steps[n - 1]) ? cfg.steps[n - 1].text : '';
    badgeStep.textContent = `Điểm tiếp theo: #${n} ${desc ? '(' + desc + ')' : ''}`;
  }
}

function updateAllLetterBadges() {
  HELLO_LETTERS.forEach(l => {
    const badge = document.getElementById('badgeLetter' + l);
    if(!badge) return;
    if(l === s_activeHelloLetter) {
      badge.textContent = s_teachPoints.length ? `${s_teachPoints.length} điểm` : 'Chưa dạy';
    } else {
      const saved = localStorage.getItem('hello_letter_' + l);
      if(saved) {
        try {
          const pts = JSON.parse(saved);
          badge.textContent = (pts && pts.length) ? `${pts.length} điểm` : 'Chưa dạy';
        } catch(_) { badge.textContent = 'Chưa dạy'; }
      } else {
        badge.textContent = 'Chưa dạy';
      }
    }
  });
}

function helloRecordPoint(penDown) {
  const isDown = (penDown !== false);
  post('/api/teach/record', `pen=${isDown ? 1 : 0}`).then(ok => {
    if(ok !== null) setTimeout(syncHelloPointsFromRobot, 60);
  });
}

function helloUndoPoint() {
  api('/api/teach/undo').then(ok => {
    if(ok !== null) setTimeout(syncHelloPointsFromRobot, 60);
  });
}

function helloClearPoints() {
  if(confirm(`Xóa toàn bộ điểm của chữ ${s_activeHelloLetter}?`)) {
    s_teachPoints = [];
    localStorage.removeItem('hello_letter_' + s_activeHelloLetter);
    updateHelloPointsTable();
    updateAllLetterBadges();
    updateTeachPathPreview();
    api('/api/teach/clear');
    toast(`🗑 Đã xóa toàn bộ điểm của chữ [${s_activeHelloLetter}]!`, 'ok');
  }
}

function setHelloPenPress(deg) {
  deg = parseFloat(deg);
  if(isNaN(deg) || deg < 0) deg = 0.35;
  ['btnPress03', 'btnPress05', 'btnPress10'].forEach(id => {
    const el = document.getElementById(id);
    if(el) el.classList.toggle('active', (id === 'btnPress03' && Math.abs(deg - 0.35) < 0.05) ||
                                          (id === 'btnPress05' && Math.abs(deg - 0.5) < 0.05) ||
                                          (id === 'btnPress10' && Math.abs(deg - 1.0) < 0.05));
  });
  post('/api/teach/press', `deg=${deg}`).then(ok => {
    if(ok !== null) {
      toast('Độ đè bút J2: ' + deg.toFixed(2) + '° (khử rơ & chống bay ngòi)');
      const inp = document.getElementById('inputHelloPressDeg');
      if(inp) inp.value = deg.toFixed(2);
    }
  });
}

function homeHelloDrawAxes() {
  api('/api/home/all').then(ok => {
    if(ok !== null) toast('🎯 Bắt đầu chuỗi Home J3 ➔ J2 ➔ J1 an toàn...', 'ok');
  });
}

function updateHelloHomingStatus(d) {
  const badge = document.getElementById('helloHomedBadge');
  const notice = document.getElementById('helloHomedNotice');
  const btnHome = document.getElementById('btnHelloHomeAll');
  if(!badge || !d || !d.joints) return;

  const j1 = d.joints[0], j2 = d.joints[1], j3 = d.joints[2];
  const allPhysHomed = j1.homed && !j1.restored && j2.homed && !j2.restored && j3.homed && !j3.restored;
  const anyRestored = j1.restored || j2.restored || j3.restored;
  const anyUnhomed = !j1.homed || !j2.homed || !j3.homed;

  if(d.homing && d.homing.active) {
    badge.className = 'badge b-run';
    badge.textContent = `Đang Home J${d.homing.axis} (${d.homing.phase})`;
    if(notice) notice.textContent = 'Robot đang dò cữ cơ học... Xin đợi!';
    if(btnHome) btnHome.disabled = true;
  } else if(allPhysHomed) {
    badge.className = 'badge b-ok';
    badge.textContent = '✅ ĐÃ HOME VẬT LÝ (SẴN SÀNG VẼ)';
    if(notice) notice.textContent = 'Tọa độ chuẩn xác từng 0.1mm. Sẵn sàng viết chữ!';
    if(btnHome) {
      btnHome.disabled = false;
      btnHome.className = 'btn btn-ghost';
      btnHome.textContent = '🎯 Home lại J3..J1';
    }
  } else {
    badge.className = 'badge b-warn';
    badge.textContent = anyRestored ? '⚠️ VỊ TRÍ NVS (CHƯA HOME VẬT LÝ)' : '❌ CHƯA HOME';
    if(notice) notice.textContent = anyRestored
      ? 'Vừa bật nguồn: Tọa độ chỉ ước lượng từ NVS. Bấm [HOME J3..J1] để ngòi chạm chuẩn mặt giấy!'
      : 'Cần Home J3..J1 trước khi viết để tránh lệch mặt giấy!';
    if(btnHome) {
      btnHome.disabled = false;
      btnHome.className = 'btn btn-primary';
      btnHome.textContent = '🎯 HOME J3..J1 (Chuẩn Bị Vẽ)';
    }
  }
}

function playHelloLetter() {
  if(!s_teachPoints || s_teachPoints.length < 2) {
    toast(`Chữ ${s_activeHelloLetter} cần ít nhất 2 điểm để vẽ! Hãy ghi điểm hoặc bấm "Nạp nét mẫu".`, 'warn');
    return;
  }
  if(s_lastStatus && s_lastStatus.joints) {
    const j1 = s_lastStatus.joints[0], j2 = s_lastStatus.joints[1], j3 = s_lastStatus.joints[2];
    const anyRestored = j1.restored || j2.restored || j3.restored;
    const anyUnhomed = !j1.homed || !j2.homed || !j3.homed;
    if(anyUnhomed) {
      toast('❌ Không thể viết: Khớp J1..J3 chưa Home! Hãy bấm nút [HOME J3..J1].', 'err');
      return;
    }
    if(anyRestored) {
      if(confirm('⚠️ Robot vừa bật nguồn và tọa độ chỉ ước lượng từ NVS (chưa Home vật lý bằng cữ endstop, ngòi bút có thể bị lệch mặt giấy).\n\nBạn có muốn Home chuẩn trước khi viết không?\n- Bấm OK để chạy HOME tự động (J3 ➔ J2 ➔ J1).\n- Bấm Cancel để viết ngay với tọa độ NVS.')) {
        homeHelloDrawAxes();
        return;
      }
    }
  }
  toast(`Bắt đầu viết chữ [${s_activeHelloLetter}] trên robot...`, 'ok');
  api('/api/teach/play?step_deg=0.5&speed=15');
}

function nextHelloLetter() {
  const curIdx = HELLO_LETTERS.indexOf(s_activeHelloLetter);
  const nextIdx = (curIdx + 1) % HELLO_LETTERS.length;
  const nextL = HELLO_LETTERS[nextIdx];
  selectHelloLetter(nextL);
  toast(`👉 Đã chuyển sang chữ [${nextL}]! Hãy kéo dịch giấy sang trái ~2.5cm rồi bấm [Viết]!`, 'ok');
}

function saveHelloAllToNvs() {
  saveTeachToNvs();
}

function addCartesianStroke(pts, x1, y1, x2, y2, penDown, zDraw, zLift) {
  const z = penDown ? zDraw : zLift;
  const dx = x2 - x1;
  const dy = y2 - y1;
  const dist = Math.hypot(dx, dy);
  // Nét ngang (dx == 0): chỉ cần 2 hoặc 3 phân đoạn mượt mà (tránh chia vụn 4mm gây giật nảy)
  const maxSegMm = (Math.abs(dx) < 0.5) ? 10.0 : 4.0;
  const steps = Math.max(1, Math.ceil(dist / maxSegMm));
  for(let i = 0; i <= steps; i++) {
    const t = i / steps;
    const x = x1 + t * dx;
    const y = y1 + t * dy;
    const ik = ikPenDown(x, y, z);
    if(ik && ik.ok) {
      const p = [
        parseFloat(ik.angles[0].toFixed(2)),
        parseFloat(ik.angles[1].toFixed(2)),
        parseFloat(ik.angles[2].toFixed(2)),
        penDown ? 1 : 0
      ];
      // Bỏ qua điểm đầu trùng lặp với điểm cuối của nét trước đó
      if(pts.length > 0 && i === 0) {
        const last = pts[pts.length - 1];
        if(Math.abs(last[0] - p[0]) < 0.05 && Math.abs(last[1] - p[1]) < 0.05 && Math.abs(last[2] - p[2]) < 0.05) {
          last[3] = p[3];
          continue;
        }
      }
      pts.push(p);
    }
  }
}

function autoPresetCurrentHelloLetter() {
  const curAngles = window.lastRobotAngles || [0, 32, 78, 0, 0, 0];
  const lm = forwardKinematics(curAngles);
  const tcp = (lm && lm.tcp) ? lm.tcp : [160, 0, 45];
  const X0 = Math.min(190, Math.max(135, tcp[0] || 160));
  const Z_draw = tcp[2] || 45.0;
  const Z_lift = Z_draw + 8.0;

  const W = 20.0;
  const H = 26.0;
  const xTop = X0 + H / 2;
  const xBot = X0 - H / 2;
  const xMid = X0;
  const yL = -W / 2;
  const yR = +W / 2;

  let pts = [];

  if(s_activeHelloLetter === 'H') {
    addCartesianStroke(pts, xTop, yL, xBot, yL, true, Z_draw, Z_lift);
    addCartesianStroke(pts, xBot, yL, xTop, yR, false, Z_draw, Z_lift);
    addCartesianStroke(pts, xTop, yR, xBot, yR, true, Z_draw, Z_lift);
    addCartesianStroke(pts, xBot, yR, xMid, yL, false, Z_draw, Z_lift);
    addCartesianStroke(pts, xMid, yL, xMid, yR, true, Z_draw, Z_lift);
    addCartesianStroke(pts, xMid, yR, xMid, yR, false, Z_draw, Z_lift);
  } else if(s_activeHelloLetter === 'E') {
    addCartesianStroke(pts, xTop, yL, xBot, yL, true, Z_draw, Z_lift);
    addCartesianStroke(pts, xBot, yL, xBot, yR, true, Z_draw, Z_lift);
    addCartesianStroke(pts, xBot, yR, xMid, yL, false, Z_draw, Z_lift);
    addCartesianStroke(pts, xMid, yL, xMid, yR - 4, true, Z_draw, Z_lift);
    addCartesianStroke(pts, xMid, yR - 4, xTop, yL, false, Z_draw, Z_lift);
    addCartesianStroke(pts, xTop, yL, xTop, yR, true, Z_draw, Z_lift);
    addCartesianStroke(pts, xTop, yR, xTop, yR, false, Z_draw, Z_lift);
  } else if(s_activeHelloLetter === 'L1' || s_activeHelloLetter === 'L2') {
    addCartesianStroke(pts, xTop, yL, xBot, yL, true, Z_draw, Z_lift);
    addCartesianStroke(pts, xBot, yL, xBot, yR, true, Z_draw, Z_lift);
    addCartesianStroke(pts, xBot, yR, xBot, yR, false, Z_draw, Z_lift);
  } else if(s_activeHelloLetter === 'O') {
    // Hình chữ nhật 4 cạnh: Đỉnh ngang -> Cạnh phải -> Đáy ngang -> Cạnh trái khép kín -> Nhấc bút
    addCartesianStroke(pts, xTop, yL, xTop, yR, true, Z_draw, Z_lift);  // Đỉnh ngang
    addCartesianStroke(pts, xTop, yR, xBot, yR, true, Z_draw, Z_lift);  // Cạnh đứng phải
    addCartesianStroke(pts, xBot, yR, xBot, yL, true, Z_draw, Z_lift);  // Đáy ngang
    addCartesianStroke(pts, xBot, yL, xTop, yL, true, Z_draw, Z_lift);  // Cạnh đứng trái khép kín
    addCartesianStroke(pts, xTop, yL, xTop, yL, false, Z_draw, Z_lift); // Nhấc bút kết thúc
  }

  fetch('/api/teach/load', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(pts)
  }).then(r => {
    if(r.ok) {
      s_teachPoints = pts;
      localStorage.setItem('hello_letter_' + s_activeHelloLetter, JSON.stringify(pts));
      updateHelloPointsTable();
      updateAllLetterBadges();
      updateTeachPathPreview();
      toast(`✨ Đã nạp nét mẫu chữ [${s_activeHelloLetter}] chuẩn phẳng không cong!`, 'ok');
    } else {
      toast('Lỗi nạp nét mẫu', 'err');
    }
  });
}

function confirmSetHome(axis){
  const label = (axis === 255) ? 'J5 + J6' : `J${axis + 1}`;
  if(confirm(`Đặt vị trí hiện tại làm mốc Home (0°) cho ${label} và lưu vào NVS?`)){
    api(`/api/sethome?axis=${axis}`);
  }
}
function confirmClearCalib(axisIdx){
  if(confirm(`Xóa cân chỉnh vị trí J${axisIdx + 1}? Giá trị zero đã lưu trong NVS sẽ bị xóa.`)){
    api(`/api/clearcalib?axis=${axisIdx}`);
  }
}
function saveWifi(){
  const s = document.getElementById('wfSsid').value.trim(), p = document.getElementById('wfPass').value;
  if(!s){ toast('Vui lòng nhập tên WiFi (SSID)', 'warn'); return; }
  post('/api/wifi', `ssid=${encodeURIComponent(s)}&pass=${encodeURIComponent(p)}`)
    .then(t => { if(t) toast('Đã lưu WiFi! Robot đang khởi động lại...', 'ok'); });
}

// Operator Console modes + contextual tabs
const navTabs = Array.from(document.querySelectorAll('.nav-tab[data-t]'));
const modeTabs = Array.from(document.querySelectorAll('.mode-tab[data-mode]'));
let lastOperateTab = document.getElementById('tab-hello') || document.getElementById('tab-dash');
function activateTab(tab, moveFocus = false){
  navTabs.forEach(x => {
    const active = x === tab;
    x.classList.toggle('active', active);
    x.setAttribute('aria-selected', String(active));
    x.tabIndex = active ? 0 : -1;
  });
  document.querySelectorAll('.tab-pane').forEach(x => {
    const active = x.id === tab.dataset.t;
    x.classList.toggle('active', active);
    if(active) x.setAttribute('aria-labelledby', tab.id);
  });
  if(moveFocus) tab.focus();
  if(tab.dataset.t === 'pane-dash' && dashRenderer) dashRenderer.renderCurrent();
  if(tab.dataset.t === 'pane-motion' && simRenderer) {
    if(simSource === 'live' && window.lastRobotAngles) updateSimFromAngles(window.lastRobotAngles);
    else simRenderer.renderCurrent();
  }
  if(tab.classList.contains('operate-nav')) lastOperateTab = tab;
}
navTabs.forEach((tab, index) => {
  tab.addEventListener('click', () => activateTab(tab));
  tab.addEventListener('keydown', e => {
    const visibleTabs = navTabs.filter(x => getComputedStyle(x).display !== 'none');
    const current = visibleTabs.indexOf(tab);
    let next = current;
    if(e.key === 'ArrowRight') next = (current + 1) % visibleTabs.length;
    else if(e.key === 'ArrowLeft') next = (current - 1 + visibleTabs.length) % visibleTabs.length;
    else if(e.key === 'Home') next = 0;
    else if(e.key === 'End') next = visibleTabs.length - 1;
    else return;
    e.preventDefault();
    activateTab(visibleTabs[next], true);
  });
});
modeTabs.forEach(tab => tab.addEventListener('click', () => {
  const setup = tab.dataset.mode === 'setup';
  document.body.classList.toggle('setup-mode', setup);
  document.body.classList.toggle('operate-mode', !setup);
  modeTabs.forEach(x => {
    const active = x === tab;
    x.classList.toggle('active', active);
    x.setAttribute('aria-pressed', String(active));
  });
  activateTab(setup ? document.getElementById('tab-calib') : lastOperateTab);
}));

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

let wristLockedState = true;
let activeHoldState = false;
let currentToolLength = 30.0;

function updateWristLockUI(isLocked){
  wristLockedState = isLocked;
  const btn = document.getElementById('btnWristLock');
  if(btn){
    btn.className = 'btn-toggle' + (isLocked ? ' locked' : ' active');
    btn.innerHTML = isLocked ? '🔒 Cổ tay: ĐÃ KHÓA' : '🔓 Cổ tay: MỞ KHÓA';
  }
  const dashTxt = document.getElementById('dashWristLock');
  if(dashTxt){
    dashTxt.textContent = isLocked ? 'Đang khóa 🔒' : 'Mở khóa 🔓';
    dashTxt.style.color = isLocked ? 'var(--warning)' : 'var(--success)';
  }

  for(let i=3; i<6; i++){
    const card = document.getElementById(`jcard${i}`);
    if(card){
      card.classList.toggle('jcard-locked', isLocked);
      card.style.opacity = isLocked ? '0.65' : '1.0';
      card.style.filter = isLocked ? 'grayscale(0.4)' : 'none';
      card.style.borderColor = isLocked ? 'var(--border-dim)' : 'var(--border)';
    }
    const tag = document.getElementById(`jlocktag${i}`);
    if(tag) tag.style.display = isLocked ? 'inline' : 'none';

    ['jbtnm', 'jbtnp', 'mbtn', 'hbtn', 'cbtn'].forEach(prefix => {
      const el = document.getElementById(`${prefix}${i}`);
      if(el){
        el.disabled = isLocked;
        el.style.opacity = isLocked ? '0.38' : '1.0';
        el.style.cursor = isLocked ? 'not-allowed' : 'pointer';
        if(isLocked) el.title = 'Cụm cổ tay J4..J6 đang khóa cứng';
        else el.removeAttribute('title');
      }
    });
  }

  const h4 = document.getElementById('btnHomeJ4');
  if(h4){
    h4.disabled = isLocked;
    h4.style.opacity = isLocked ? '0.38' : '1.0';
    h4.style.cursor = isLocked ? 'not-allowed' : 'pointer';
    h4.textContent = isLocked ? 'Home J4 🔒' : 'Home J4';
  }
  const s5 = document.getElementById('btnSetHomeJ5');
  if(s5){
    s5.disabled = isLocked;
    s5.style.opacity = isLocked ? '0.38' : '1.0';
    s5.style.cursor = isLocked ? 'not-allowed' : 'pointer';
    s5.textContent = isLocked ? 'Set Home J5 🔒' : 'Set Home J5';
  }
  const s6 = document.getElementById('btnSetHomeJ6');
  if(s6){
    s6.disabled = isLocked;
    s6.style.opacity = isLocked ? '0.38' : '1.0';
    s6.style.cursor = isLocked ? 'not-allowed' : 'pointer';
    s6.textContent = isLocked ? 'Set Home J6 🔒' : 'Set Home J6';
  }
}

function updateActiveHoldUI(enabled){
  activeHoldState = enabled;
  const btn = document.getElementById('btnActiveHold');
  if(btn){
    btn.className = 'btn-toggle' + (enabled ? ' active' : '');
    btn.innerHTML = enabled ? '⚓ Active Hold: BẬT' : '⚓ Active Hold: TẮT';
  }
  const dashTxt = document.getElementById('dashActiveHold');
  if(dashTxt){
    dashTxt.textContent = enabled ? 'BẬT ⚓' : 'TẮT';
    dashTxt.style.color = enabled ? 'var(--success)' : 'var(--text-muted)';
  }
}

function updateToolLengthUI(len){
  currentToolLength = len;
  const inp = document.getElementById('inputToolLength');
  if(inp && document.activeElement !== inp) inp.value = len;
  const dashTxt = document.getElementById('dashToolLen');
  if(dashTxt) dashTxt.textContent = `${len.toFixed(1)} mm`;
  if(typeof D_TOOL !== 'undefined') D_TOOL = len;
  if(typeof D_TOOL_EFF !== 'undefined' && typeof D6 !== 'undefined') D_TOOL_EFF = D6 + len;
}

async function toggleWristLock(){
  const next = !wristLockedState;
  await api(`/api/wristlock?lock=${next ? 1 : 0}`);
  updateWristLockUI(next);
  toast(next ? 'Đã khóa cụm cổ tay J4..J6' : 'Đã mở khóa cụm cổ tay J4..J6');
}

async function toggleActiveHold(){
  const next = !activeHoldState;
  await api(`/api/activehold?enable=${next ? 1 : 0}`);
  updateActiveHoldUI(next);
  toast(next ? 'Đã bật Active Hold (giữ vị trí closed-loop)' : 'Đã tắt Active Hold');
}

async function applyToolLength(){
  const inp = document.getElementById('inputToolLength');
  if(!inp) return;
  const val = parseFloat(inp.value);
  if(isNaN(val) || val < 0 || val > 300){
    toast('Chiều dài bút không hợp lệ (0..300mm)', 'err');
    return;
  }
  await api(`/api/toollength?len=${val}`);
  updateToolLengthUI(val);
  toast(`Đã cập nhật chiều dài bút: ${val.toFixed(1)} mm`);
}

async function moveToAngle(axis){
  const inp = document.getElementById(`mto${axis}`);
  if(!inp) return;
  const val = parseFloat(inp.value);
  if(isNaN(val)){
    toast('Vui lòng nhập góc mục tiêu (°)', 'warn');
    return;
  }
  await api(`/api/moveto?axis=${axis}&deg=${val}`);
  toast(`J${axis+1} điều khiển tới ${val.toFixed(1)}°`);
}

// Build Joint Cards & Quick Status Rows
function buildJointCards(){
  const grid = document.getElementById('jointCardsGrid');
  const dashRows = document.getElementById('dashJointRows');
  if(grid) grid.innerHTML = '';
  if(dashRows) dashRows.innerHTML = '';

  for(let i=0; i<6; i++){
    const isLocked = (i >= 3) && wristLockedState;
    if(grid){
      grid.insertAdjacentHTML('beforeend', `
        <div class="jcard${isLocked ? ' jcard-locked' : ''}" id="jcard${i}" style="${isLocked ? 'opacity:0.65;filter:grayscale(0.4);border-color:var(--border-dim);' : ''}">
          <div class="jcard-top">
            <span class="jcard-name">${AXES[i]} <span id="jlocktag${i}" style="display:${isLocked ? 'inline' : 'none'};font-size:0.72rem;color:var(--warning);font-weight:700;">[ĐÃ KHÓA 🔒]</span></span>
            <div style="display:flex;align-items:baseline;gap:8px">
              <span class="target-tag" id="jtar${i}">Mục tiêu: 0.0°</span>
              <span class="jcard-deg" id="jd${i}">0.0°</span>
            </div>
          </div>
          <div class="jcard-enc" id="je${i}">Encoder: --</div>
          <div class="jcard-flags" id="jf${i}"></div>
          <div class="jcard-calib" id="jc${i}">NVS: --</div>
          <div class="jog-controls">
            <button class="jog-btn" id="jbtnm${i}" ${isLocked ? 'disabled style="cursor:not-allowed;opacity:0.38"' : `onclick="jog(${i},-1)"`} aria-label="Jog ${AXES[i]} âm">↺ &minus;</button>
            <button class="jog-btn" id="jbtnp${i}" ${isLocked ? 'disabled style="cursor:not-allowed;opacity:0.38"' : `onclick="jog(${i},1)"`} aria-label="Jog ${AXES[i]} dương">+ ↻</button>
          </div>
          <div class="moveto-row">
            <input type="number" step="0.5" class="moveto-input" id="mto${i}" placeholder="Góc mục tiêu °" onkeydown="if(event.key==='Enter')moveToAngle(${i})">
            <button class="btn btn-ghost need-idle" id="mbtn${i}" style="min-height:28px;padding:3px 10px;font-size:0.75rem" ${isLocked ? 'disabled style="cursor:not-allowed;opacity:0.38"' : `onclick="moveToAngle(${i})"`}>Tới góc</button>
          </div>
          <div class="jog-calib-row">
            <button class="btn btn-ghost need-idle" id="hbtn${i}" ${isLocked ? 'disabled style="cursor:not-allowed;opacity:0.38"' : `onclick="confirmSetHome(${i})"`}>Set Home</button>
            <button class="btn btn-ghost need-idle" id="cbtn${i}" ${isLocked ? 'disabled style="cursor:not-allowed;opacity:0.38"' : `onclick="confirmClearCalib(${i})"`}>Clear NVS</button>
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
  latestStatus = d;
  const m = d.mode || 'idle';
  const modeText = document.getElementById('dashModeText');
  const modeBadge = document.getElementById('dashModeBadge');
  const map = { idle: 'b-idle', release: 'b-run', homing: 'b-run', jog: 'b-run', cart: 'b-run', draw: 'b-run', show_off: 'b-showoff', fault: 'b-fault' };
  if(modeText) { modeText.textContent = m.toUpperCase(); modeText.className = 'mode-text ' + (m === 'fault' ? 'fault' : (m === 'show_off' ? 'showoff' : (m !== 'idle' ? 'run' : ''))); }
  if(modeBadge) { modeBadge.textContent = m; modeBadge.className = 'badge ' + (map[m] || 'b-idle'); }

  const connLabel = document.getElementById('connLabel');
  if(connLabel) connLabel.textContent = `${d.wifi.ssid || '(AP)'} · ${d.wifi.ip}`;

  let hn = 0; d.joints.forEach(j => hn += j.homed ? 1 : 0);
  const homedCount = document.getElementById('dashHomedCount');
  if(homedCount) homedCount.textContent = `${hn}/6`;

  const wifiInfo = document.getElementById('dashWifiInfo');
  if(wifiInfo) wifiInfo.textContent = `${(d.wifi.mode||'').toUpperCase()} · RSSI ${d.wifi.rssi || 0} dBm`;
  const commandLatency = document.getElementById('dashCommandLatency');
  if(commandLatency) commandLatency.textContent = `${((d.commandLatencyUs || 0) / 1000).toFixed(1)} ms`;
  const esInfo = document.getElementById('dashEsInfo');
  if(esInfo && d.endstops){
    const pressedList = [];
    d.endstops.forEach((es, i) => {
      if(es.min && (es.min.pressed || es.min.latched)) pressedList.push(`J${i+1} MIN`);
      if(es.max && (es.max.pressed || es.max.latched)) pressedList.push(`J${i+1} MAX`);
    });
    if(pressedList.length === 0){
      const hasAnyPin = d.endstops.some(es => es.min !== null || es.max !== null);
      esInfo.textContent = hasAnyPin ? 'Tất cả mở' : 'Chưa bật (PIN=-1)';
      esInfo.className = 'v';
    } else {
      esInfo.textContent = 'CHẠM: ' + pressedList.join(', ');
      esInfo.className = 'v text-danger';
    }
  }
  const teachStatus = document.getElementById('teachStatus');
  if(d.gripper){
    const g = d.gripper;
    document.getElementById('gripperAngle').min = g.minDeg;
    document.getElementById('gripperAngle').max = g.maxDeg;
    document.getElementById('gripperStatus').textContent = !g.ready ? 'PWM unavailable' :
      g.active ? `Target ${g.targetDeg.toFixed(1)}°` : 'Pulses off';
  }
  if(teachStatus && d.teachPoints){
    const errors = ['', 'Save/clear failed — check remaining points and retry', 'Home all six joints and check encoders', 'Home changed — teach points again', 'No valid point or playback rejected'];
    teachStatus.textContent = d.teachPoints.map((v, i) => `${String.fromCharCode(65 + i)} ${v ? '✓' : '—'}`).join('  ') + (d.teachError ? ' · ' + errors[d.teachError] : '');
  }
  const teachBadge = document.getElementById('teachTrajBadge');
  const teachProg = document.getElementById('teachPlayProgress');
  if(d.teachTrajectory){
    if(d.teachTrajectory.free !== undefined && d.teachTrajectory.free !== s_teachIsFree){
      s_teachIsFree = d.teachTrajectory.free;
      const btn = document.getElementById('btnTeachFree');
      if (btn) {
        btn.textContent = s_teachIsFree ? '🔒 Khóa J1..J3' : '🔓 Thả lỏng J1..J3 (Kéo tay)';
        btn.className = s_teachIsFree ? 'btn btn-primary' : 'btn btn-warning';
      }
      const btnH = document.getElementById('btnHelloFree');
      if (btnH) {
        btnH.textContent = s_teachIsFree ? '🔒 Khóa giữ tay' : '🔓 Thả lỏng tay để kéo';
        btnH.className = s_teachIsFree ? 'btn btn-primary' : 'btn btn-warning';
      }
    }
    const hBadge = document.getElementById('helloPlayStatusBadge');
    if(hBadge) {
      if(d.teachTrajectory.playing){
        hBadge.textContent = `ĐANG VIẾT (${d.teachTrajectory.segment + 1}/${d.teachTrajectory.count})`;
        hBadge.className = 'badge b-run';
      } else {
        hBadge.textContent = 'Sẵn sàng';
        hBadge.className = 'badge b-idle';
      }
    }
    if(teachBadge) {
      teachBadge.textContent = d.teachTrajectory.count + ' điểm';
      teachBadge.className = 'badge ' + (d.teachTrajectory.count > 0 ? 'b-run' : 'b-idle');
    }
    if(teachProg){
      if(d.teachTrajectory.playing){
        teachProg.textContent = `▶ Đang chạy lại: đoạn ${d.teachTrajectory.segment + 1}/${d.teachTrajectory.count}`;
      } else {
        teachProg.textContent = '';
      }
    }
  }

  const wfModeText = document.getElementById('wfModeText');
  if(wfModeText) wfModeText.textContent = (d.wifi.mode||'').toUpperCase();
  const wfIpText = document.getElementById('wfIpText');
  if(wfIpText) wfIpText.textContent = d.wifi.ip;
  const wfSsidNowText = document.getElementById('wfSsidNowText');
  if(wfSsidNowText) wfSsidNowText.textContent = d.wifi.ssid || '(AP Fallback)';
  const wfRssiText = document.getElementById('wfRssiText');
  if(wfRssiText) wfRssiText.textContent = `${d.wifi.rssi || 0} dBm`;

  syncCommandState();

  const robotAngles = d.joints.map(j => j.deg);
  window.lastRobotAngles = robotAngles;

  // Lưu scene mới, chỉ vẽ canvas tab đang xem để polling không tốn CPU vô ích.
  const lmLive = liveVisualizationKinematics(robotAngles);
  if(dashRenderer) {
    dashRenderer.lastLm = lmLive;
    dashRenderer.lastPath = simWaypoints;
    if(isPaneActive('pane-dash')) dashRenderer.renderCurrent();
  }

  const dashPoseLabel = document.getElementById('dashPoseLabel');
  if(dashPoseLabel) dashPoseLabel.textContent = `TCP: (${lmLive.tcp[0].toFixed(1)}, ${lmLive.tcp[1].toFixed(1)}, ${lmLive.tcp[2].toFixed(1)})`;
  const hudDashTcp = document.getElementById('hudDashTcp');
  if(hudDashTcp) hudDashTcp.textContent = `X=${lmLive.tcp[0].toFixed(1)} Y=${lmLive.tcp[1].toFixed(1)} Z=${lmLive.tcp[2].toFixed(1)}`;
  const hudDashWrist = document.getElementById('hudDashWrist');
  if(hudDashWrist) hudDashWrist.textContent = `(${lmLive.wrist[0].toFixed(1)}, ${lmLive.wrist[1].toFixed(1)}, ${lmLive.wrist[2].toFixed(1)})`;

  if(simSource === 'live' && isPaneActive('pane-motion')){
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
    const jc = document.getElementById('jc' + i);
    if(jc){
      if(!j.homed) jc.textContent = 'NVS: chưa có mốc home';
      else if(j.restored) jc.textContent = 'NVS: đã khôi phục sau khởi động';
      else if(j.encOK) jc.textContent = 'NVS: mốc home đang hoạt động';
      else jc.textContent = 'NVS: không thể lưu mốc mới — encoder lỗi';
    }
  });

  // Homing Chips
  const homingProg = document.getElementById('dashHomingProg');
  if(homingProg && d.homing){
    if(d.homing.active){
      homingProg.textContent = `Đang Home J${d.homing.axis} (${d.homing.phase})`;
    } else {
      homingProg.textContent = d.homing.lastOk ? 'Sẵn sàng' : 'Lỗi Homing';
    }
  }
  for(let i=0; i<6; i++){
    const hc = document.getElementById('hc' + i);
    if(hc && d.joints[i]){
      const isHomingThis = !!(d.homing && d.homing.active && d.homing.axis === (i + 1));
      hc.classList.toggle('active', isHomingThis);
      hc.classList.toggle('done', !isHomingThis && d.joints[i].homed);
    }
  }

  if(d.wristLocked !== undefined && d.wristLocked !== wristLockedState){
    updateWristLockUI(d.wristLocked);
  }
  if(d.activeHold !== undefined && d.activeHold !== activeHoldState){
    updateActiveHoldUI(d.activeHold);
  }
  if(d.toolLength !== undefined && Math.abs(d.toolLength - currentToolLength) > 0.05){
    updateToolLengthUI(d.toolLength);
  }
  if(d.targetAngles && Array.isArray(d.targetAngles)){
    for(let i=0; i<6; i++){
      const el = document.getElementById('jtar' + i);
      if(el && d.targetAngles[i] !== undefined){
        el.textContent = 'Mục tiêu: ' + d.targetAngles[i].toFixed(1) + '°';
      }
    }
  }

  s_lastStatus = d;
  updateHelloHomingStatus(d);
}

function setOnline(on){
  const pill = document.getElementById('connPill');
  if(pill) pill.classList.toggle('offline', !on);
  const label = document.getElementById('connLabel');
  if(!on && label) label.textContent = 'MẤT KẾT NỐI — Đang thử lại...';
}

function pollOnce(){
  if(statusPollController || pendingCommands > 0) return;
  const controller = new AbortController();
  statusPollController = controller;
  fetch('/api/status', { signal: controller.signal }).then(r => {
    if(!r.ok) throw new Error(`status ${r.status}`);
    return r.json();
  })
    .then(d => { failN = 0; setOnline(true); updateUI(d); })
    .catch(err => {
      if(err.name !== 'AbortError') {
        if(++failN >= 3) setOnline(false);
        syncCommandState();
      }
    })
    .finally(() => { if(statusPollController === controller) statusPollController = null; });
}

initStudio();
try { selectHelloLetter(s_activeHelloLetter); } catch(_) {}
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
        if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
        ArmCommand c;
        c.type = ArmCommand::CLEAR_FAULT;
        const bool ok = armPtr->submit(c);
        srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
        return;
    }
    if (!srv->hasArg("axis") || !srv->hasArg("deg")) {
        srv->send(400, "text/plain", "missing axis or deg");
        return;
    }
    int axis = 0;
    float deg = 0.0f;
    if (!webval::parseInt(srv->arg("axis").c_str(), axis) || axis < 0 || axis >= NUM_MOTORS) {
        srv->send(400, "text/plain", "bad axis"); return;
    }
    if (armPtr->isWristLocked() && axis >= 3) {
        srv->send(400, "text/plain", "J4..J6 is locked");
        return;
    }
    if (!webval::parseFiniteFloat(srv->arg("deg").c_str(), deg) || fabsf(deg) < 0.01f || fabsf(deg) > 180.0f) {
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

void handleGripper() {
    if (armPtr == nullptr) { srv->send(503, "text/plain", "not ready"); return; }
    float deg = 0.0f;
    if (!webval::parseFiniteFloat(srv->arg("deg").c_str(), deg) ||
        deg < GRIPPER_MIN_DEG || deg > GRIPPER_MAX_DEG) {
        srv->send(400, "text/plain", "invalid gripper angle"); return;
    }
    if (!armPtr->gripperAvailable()) {
        srv->send(409, "text/plain", "gripper unavailable: check idle, Release, fault and PWM"); return;
    }
    ArmCommand command;
    command.type = ArmCommand::SET_GRIPPER;
    command.value = deg;
    const bool ok = armPtr->submit(command);
    srv->send(ok ? 202 : 409, "text/plain", ok ? "Queued" : "gripper unavailable");
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
    int axis = 0;
    if (!webval::parseInt(srv->arg("axis").c_str(), axis) || axis < 0 || axis >= 4) {
        srv->send(400, "text/plain", "axis 0..3 only"); return;
    }
    if (armPtr->isWristLocked() && axis >= 3) {
        srv->send(400, "text/plain", "J4..J6 is locked");
        return;
    }
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
    int axis = 0;
    if (!webval::parseInt(srv->arg("axis").c_str(), axis) ||
        axis < 0 || (axis >= NUM_MOTORS && axis != 255)) { srv->send(400, "text/plain", "bad axis"); return; }
    if (armPtr != nullptr && armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::SET_HOME;
    c.axis = static_cast<uint8_t>(axis);
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleShowOff() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::SHOW_OFF;
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleTeachClear() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    ArmCommand c;
    c.type = ArmCommand::TEACH_CLEAR;
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleTeachFree() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    int freeVal = 1;
    if (srv->hasArg("free")) {
        (void)webval::parseInt(srv->arg("free").c_str(), freeVal);
    }
    ArmCommand c;
    c.type = ArmCommand::TEACH_FREE;
    c.value = freeVal ? 1.0f : 0.0f;
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleTeachRecord() {
    if (armPtr == nullptr || jointsPtr == nullptr || armPtr->teachManager() == nullptr) {
        srv->send(500, "text/plain", "not ready"); return;
    }
    for (uint8_t a = 0; a < 3; ++a) {
        if (!jointsPtr->isHomed(a) || !jointsPtr->encOK(a)) {
            srv->send(409, "text/plain", "J1..J3 chua Home hoac Encoder loi!");
            return;
        }
    }
    if (armPtr->teachManager()->pointCount() >= TeachManager::MAX_POINTS) {
        srv->send(409, "text/plain", "Da dat gioi han 250 diem!");
        return;
    }
    ArmCommand c;
    c.type = ArmCommand::TEACH_RECORD;
    c.value = 1.0f; // mac dinh penDown = true
    if (srv->hasArg("pen")) {
        int penVal = 1;
        if (webval::parseInt(srv->arg("pen").c_str(), penVal)) {
            c.value = (penVal != 0) ? 1.0f : 0.0f;
        }
    }
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? (c.value >= 0.5f ? "Da ghi diem VE (Ha but)" : "Da ghi diem NHAC BUT") : "busy");
}

void handleTeachUndo() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    ArmCommand c;
    c.type = ArmCommand::TEACH_UNDO;
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleTeachGetPoints() {
    if (armPtr == nullptr || armPtr->teachManager() == nullptr) {
        srv->send(500, "text/plain", "not ready"); return;
    }
    String json = armPtr->teachManager()->getPointsJson();
    srv->send(200, "application/json", json);
}

void handleTeachLoadPoints() {
    if (armPtr == nullptr || armPtr->teachManager() == nullptr) {
        srv->send(500, "text/plain", "not ready"); return;
    }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    if (!srv->hasArg("plain")) {
        srv->send(400, "text/plain", "missing body"); return;
    }
    String body = srv->arg("plain");
    const bool ok = armPtr->teachManager()->loadPointsJson(body);
    srv->send(ok ? 200 : 400, "text/plain", ok ? "OK" : "bad json");
}

void handleTeachTrajPlay() {
    if (armPtr == nullptr || jointsPtr == nullptr || armPtr->teachManager() == nullptr) {
        srv->send(500, "text/plain", "not ready"); return;
    }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    if (armPtr->teachManager()->pointCount() < 2) {
        srv->send(409, "text/plain", "Can it nhat 2 diem de chay lai!");
        return;
    }
    for (uint8_t a = 0; a < 3; ++a) {
        if (!jointsPtr->isHomed(a) || !jointsPtr->encOK(a)) {
            srv->send(409, "text/plain", "J1..J3 chua Home hoac Encoder loi!");
            return;
        }
    }
    float stepDeg = 0.5f;
    float speedDelay = 15.0f;
    if (srv->hasArg("step_deg")) (void)webval::parseFiniteFloat(srv->arg("step_deg").c_str(), stepDeg);
    if (srv->hasArg("speed")) (void)webval::parseFiniteFloat(srv->arg("speed").c_str(), speedDelay);
    ArmCommand c;
    c.type = ArmCommand::TEACH_PLAY;
    c.p[0] = stepDeg;
    c.p[1] = speedDelay;
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "Bat dau chay quy dao" : "busy");
}

void handleTeachStop() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    ArmCommand c;
    c.type = ArmCommand::TEACH_STOP;
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
}

void handleClearCalib() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    // Flash Write Isolation Guard: Cấm xóa NVS khi robot đang chuyển động
    if (armPtr->busy()) {
        srv->send(409, "text/plain", "CANNOT_WRITE_FLASH_WHILE_MOVING");
        return;
    }
    ArmCommand c;
    c.type = ArmCommand::CLEAR_CALIB;
    if (srv->hasArg("axis")) {
        int axis = 0;
        if (!webval::parseInt(srv->arg("axis").c_str(), axis) || axis < 0 || axis >= NUM_MOTORS) {
            srv->send(400, "text/plain", "bad axis"); return;
        }
        c.axis = static_cast<uint8_t>(axis);
    } else {
        c.axis = NUM_MOTORS; // clear all
    }
    const bool ok = armPtr->submit(c, 20);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
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
    Serial.println("[WEB] WiFi creds moi da luu — restart sau 1s (async)...");
    static esp_timer_handle_t s_rebootTimer = nullptr;
    if (s_rebootTimer == nullptr) {
        esp_timer_create_args_t timerArgs{};
        timerArgs.callback = [](void*) {
            ESP.restart();
        };
        timerArgs.name = "async_reboot";
        esp_timer_create(&timerArgs, &s_rebootTimer);
    }
    esp_timer_start_once(s_rebootTimer, 1000000ULL);
}

void handleTeachNvsSave() {
    if (armPtr == nullptr || nvsPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    const bool ok = armPtr->teachManager()->saveToNvs(nvsPtr);
    srv->send(ok ? 200 : 500, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

void handleTeachNvsLoad() {
    if (armPtr == nullptr || nvsPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }
    const bool ok = armPtr->teachManager()->loadFromNvs(nvsPtr);
    if (ok) {
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"ok\":true,\"count\":%u}",
                 static_cast<unsigned>(armPtr->teachManager()->pointCount()));
        srv->send(200, "application/json", buf);
    } else {
        srv->send(400, "application/json", "{\"ok\":false,\"msg\":\"Khong co quy dao hop le\"}");
    }
}

void handleTeachPress() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (srv->hasArg("deg")) {
        float deg = 0.0f;
        if (webval::parseFiniteFloat(srv->arg("deg").c_str(), deg) && deg >= 0.0f && deg <= 4.0f) {
            armPtr->teachManager()->setPenPressDeg(deg);
            srv->send(200, "application/json", String("{\"ok\":true,\"pressDeg\":") + String(deg, 2) + "}");
            return;
        }
        srv->send(400, "text/plain", "invalid deg 0..4.0");
        return;
    }
    srv->send(200, "application/json", String("{\"pressDeg\":") + String(armPtr->teachManager()->penPressDeg(), 2) + "}");
}

void handleWristLock() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (srv->hasArg("lock")) {
        const bool lock = (srv->arg("lock") == "1" || srv->arg("lock") == "true");
        armPtr->setWristLocked(lock);
        srv->send(200, "application/json", String("{\"ok\":true,\"wristLocked\":") + (lock ? "true" : "false") + "}");
    } else {
        srv->send(200, "application/json", String("{\"wristLocked\":") + (armPtr->isWristLocked() ? "true" : "false") + "}");
    }
}

void handleActiveHold() {
    if (armPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (srv->hasArg("enable")) {
        const bool en = (srv->arg("enable") == "1" || srv->arg("enable") == "true");
        armPtr->setActiveHold(en);
        srv->send(200, "application/json", String("{\"ok\":true,\"activeHold\":") + (en ? "true" : "false") + "}");
    } else {
        srv->send(200, "application/json", String("{\"activeHold\":") + (armPtr->isActiveHoldEnabled() ? "true" : "false") + "}");
    }
}

void handleToolLength() {
    if (srv->hasArg("len")) {
        float len = 0.0f;
        if (webval::parseFiniteFloat(srv->arg("len").c_str(), len) && len >= 0.0f && len <= 300.0f) {
            kin::setToolLength(len);
            srv->send(200, "application/json", String("{\"ok\":true,\"toolLength\":") + String(len, 1) + "}");
            return;
        }
        srv->send(400, "text/plain", "invalid len 0..300");
        return;
    }
    srv->send(200, "application/json", String("{\"toolLength\":") + String(kin::getToolLength(), 1) + "}");
}

void handleMoveTo() {
    if (armPtr == nullptr || jointsPtr == nullptr) { srv->send(500, "text/plain", "not ready"); return; }
    if (!srv->hasArg("axis") || !srv->hasArg("deg")) {
        srv->send(400, "text/plain", "missing axis or deg"); return;
    }
    int axis = 0;
    float targetDeg = 0.0f;
    if (!webval::parseInt(srv->arg("axis").c_str(), axis) || axis < 0 || axis >= NUM_MOTORS) {
        srv->send(400, "text/plain", "bad axis"); return;
    }
    if (armPtr->isWristLocked() && axis >= 3) {
        srv->send(400, "text/plain", "J4..J6 is locked"); return;
    }
    if (!webval::parseFiniteFloat(srv->arg("deg").c_str(), targetDeg)) {
        srv->send(400, "text/plain", "bad deg"); return;
    }
    if (armPtr->busy()) { srv->send(409, "text/plain", "busy"); return; }

    const float cur = (jointsPtr->isHomed(axis) && jointsPtr->encOK(axis))
        ? jointsPtr->angleFromEncoder(axis)
        : jointsPtr->angleFromSteps(axis);
    const float delta = targetDeg - cur;
    if (fabsf(delta) < 0.05f) {
        armPtr->setTargetHoldAngle(axis, targetDeg);
        srv->send(200, "text/plain", "already at angle");
        return;
    }

    ArmCommand c;
    c.type = ArmCommand::JOG_REL;
    c.axis = static_cast<uint8_t>(axis);
    c.value = delta;
    const bool ok = armPtr->submit(c, 20);
    if (ok) armPtr->setTargetHoldAngle(axis, targetDeg);
    srv->send(ok ? 200 : 503, "text/plain", ok ? "OK" : "busy");
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
    server.on("/api/stop", HTTP_POST, handleStop);
    server.on("/api/gripper", HTTP_POST, handleGripper);
    server.on("/api/home/all", HTTP_POST, handleHomeAll);
    server.on("/api/home/axis", HTTP_POST, handleHomeAxis);
    server.on("/api/sethome", HTTP_POST, handleSetHome);
    server.on("/api/showoff", HTTP_POST, handleShowOff);
    server.on("/api/teach/play", HTTP_POST, handleTeachTrajPlay);
    server.on("/api/teach/clear", HTTP_POST, handleTeachClear);
    server.on("/api/teach/free", HTTP_POST, handleTeachFree);
    server.on("/api/teach/record", HTTP_POST, handleTeachRecord);
    server.on("/api/teach/undo", HTTP_POST, handleTeachUndo);
    server.on("/api/teach/points", HTTP_GET, handleTeachGetPoints);
    server.on("/api/teach/load", HTTP_POST, handleTeachLoadPoints);
    server.on("/api/teach/stop", HTTP_POST, handleTeachStop);
    server.on("/api/teach/nvs/save", HTTP_POST, handleTeachNvsSave);
    server.on("/api/teach/nvs/load", HTTP_POST, handleTeachNvsLoad);
    server.on("/api/teach/press", HTTP_ANY, handleTeachPress);
    server.on("/api/clearcalib", HTTP_POST, handleClearCalib);
    server.on("/api/wifi", HTTP_POST, handleWifiSave);
    server.on("/api/wristlock", HTTP_ANY, handleWristLock);
    server.on("/api/activehold", HTTP_ANY, handleActiveHold);
    server.on("/api/toollength", HTTP_ANY, handleToolLength);
    server.on("/api/moveto", HTTP_POST, handleMoveTo);

    server.begin();
    Serial.printf("[WEB] HTTP server on port %u\n", WEB_SERVER_PORT);
}
