#include <TMCStepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>

#define SERIAL_PORT   Serial2
#define R_SENSE       0.11f

// --- Motor 1 Pins & UART Address ---
#define STEP_PIN_1    18
#define EN_PIN_1      5
#define DRIVER_ADDR_1 0b00

// --- Motor 2 Pins & UART Address ---
#define STEP_PIN_2    19
#define EN_PIN_2      4
#define DRIVER_ADDR_2 0b01

TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRIVER_ADDR_1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDR_2);
WebServer server(80);

struct Motor {
  TMC2209Stepper* driver;
  uint8_t stepPin;
  uint8_t enPin;
  uint8_t address;
  const char* name;
  volatile bool running;
  volatile bool dirCW;
  volatile uint32_t stepIntervalUs;
  volatile uint32_t targetSteps;
  volatile uint32_t stepsRemaining;
  volatile uint16_t currentMa;
  volatile bool spreadCycleMode;
  volatile uint16_t microstepsVal;
  volatile uint8_t holdScale;       // Hold current scale (0 to 31 CS scale)
  uint32_t lastStepUs;
  bool stepPinState;
};

Motor motor1 = { &driver1, STEP_PIN_1, EN_PIN_1, DRIVER_ADDR_1, "Motor 1", false, true, 1000, 1600, 0, 700, true, 16, 8, 0, false };
Motor motor2 = { &driver2, STEP_PIN_2, EN_PIN_2, DRIVER_ADDR_2, "Motor 2", false, true, 1000, 1600, 0, 700, true, 16, 8, 0, false };

void applyDirection(Motor& m, bool cw) {
  m.dirCW = cw;
  m.driver->shaft(!cw);
}

void applyChopMode(Motor& m, bool enableSpreadCycle) {
  m.spreadCycleMode = enableSpreadCycle;
  m.driver->en_spreadCycle(m.spreadCycleMode);
  if (m.spreadCycleMode) {
    m.driver->pwm_autoscale(false);
    m.driver->pwm_autograd(false);
  } else {
    m.driver->pwm_autoscale(true);
    m.driver->pwm_autograd(true);
  }
}

void handleMotorStepping(Motor& m) {
  if (!m.running) return;
  uint32_t now = micros();
  if (now - m.lastStepUs >= m.stepIntervalUs) {
    m.lastStepUs = now;
    m.stepPinState = !m.stepPinState;
    digitalWrite(m.stepPin, m.stepPinState);

    if (m.stepPinState) { // Rising edge: 1 step pulse executed
      if (m.stepsRemaining > 0) {
        m.stepsRemaining--;
      }
    } else { // Falling edge: pulse cycle completed
      if (m.stepsRemaining == 0) {
        m.running = false;
      }
    }
  }
}

void handleStepping() {
  handleMotorStepping(motor1);
  handleMotorStepping(motor2);
}

// --- Web page ---
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Dual TMC2209 Controller</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    background: #0f172a;
    color: #f8fafc;
    min-height: 100vh;
    padding: 20px;
    display: flex;
    flex-direction: column;
    align-items: center;
  }
  .header {
    text-align: center;
    margin-bottom: 20px;
  }
  .header h1 {
    font-size: 1.6rem;
    color: #38bdf8;
    margin-bottom: 6px;
  }
  .header p {
    font-size: 0.875rem;
    color: #94a3b8;
  }
  .master-bar {
    background: #1e293b;
    border-radius: 12px;
    padding: 16px;
    width: 100%;
    max-width: 980px;
    margin-bottom: 24px;
    box-shadow: 0 4px 12px rgba(0,0,0,0.3);
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
  }
  .master-title {
    font-weight: 700;
    font-size: 0.95rem;
    color: #cbd5e1;
    text-transform: uppercase;
    letter-spacing: 0.05em;
  }
  .master-btns {
    display: flex;
    gap: 10px;
    flex-wrap: wrap;
  }
  .m-btn {
    padding: 10px 16px;
    border-radius: 8px;
    border: none;
    font-weight: 600;
    font-size: 0.9rem;
    cursor: pointer;
    transition: transform 0.1s ease;
  }
  .m-btn:active { transform: scale(0.98); }
  .m-cw { background: #2563eb; color: white; }
  .m-ccw { background: #d97706; color: white; }
  .m-stop { background: #dc2626; color: white; }
  
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(340px, 1fr));
    gap: 24px;
    width: 100%;
    max-width: 980px;
  }
  .card {
    background: #1e293b;
    border-radius: 16px;
    padding: 24px;
    box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.4);
    display: flex;
    flex-direction: column;
    gap: 18px;
  }
  .card-title {
    font-size: 1.25rem;
    font-weight: 700;
    color: #38bdf8;
    display: flex;
    justify-content: space-between;
    align-items: center;
    border-bottom: 1px solid #334155;
    padding-bottom: 10px;
  }
  .addr-tag {
    font-size: 0.75rem;
    background: #0f172a;
    color: #94a3b8;
    padding: 4px 8px;
    border-radius: 6px;
    border: 1px solid #334155;
  }
  label {
    display: block;
    font-size: 0.8rem;
    font-weight: 600;
    color: #94a3b8;
    margin-bottom: 6px;
    text-transform: uppercase;
    letter-spacing: 0.05em;
  }
  .input-group {
    display: flex;
    gap: 8px;
    margin-bottom: 8px;
  }
  input[type=number] {
    flex: 1;
    background: #0f172a;
    border: 1px solid #334155;
    color: #f8fafc;
    font-size: 1rem;
    padding: 8px 12px;
    border-radius: 8px;
    outline: none;
  }
  .presets {
    display: flex;
    flex-wrap: wrap;
    gap: 6px;
  }
  .preset-btn {
    background: #334155;
    color: #cbd5e1;
    border: none;
    padding: 6px 10px;
    border-radius: 6px;
    font-size: 0.8rem;
    font-weight: 600;
    cursor: pointer;
  }
  .preset-btn:hover { background: #475569; color: #fff; }
  .preset-btn.active { background: #38bdf8; color: #0f172a; }
  .btn-group {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
  }
  button.action-btn {
    font-size: 0.95rem;
    font-weight: 600;
    padding: 12px;
    border-radius: 8px;
    border: none;
    cursor: pointer;
  }
  .cw { background: #2563eb; color: white; }
  .ccw { background: #d97706; color: white; }
  .stop {
    width: 100%;
    background: #dc2626;
    color: white;
    font-size: 0.95rem;
    font-weight: 600;
    padding: 10px;
    border-radius: 8px;
    border: none;
    cursor: pointer;
  }
  .mode-toggle {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 6px;
    background: #0f172a;
    padding: 4px;
    border-radius: 8px;
    border: 1px solid #334155;
  }
  .mode-btn {
    background: transparent;
    border: none;
    color: #94a3b8;
    padding: 8px;
    border-radius: 6px;
    font-size: 0.8rem;
    font-weight: 600;
    cursor: pointer;
  }
  .mode-btn.active { background: #38bdf8; color: #0f172a; }
  .range-container {
    background: #0f172a;
    padding: 12px;
    border-radius: 8px;
    border: 1px solid #334155;
  }
  input[type=range] {
    width: 100%;
    accent-color: #38bdf8;
    cursor: pointer;
  }
  .info-row {
    display: flex;
    justify-content: space-between;
    font-size: 0.8rem;
    color: #94a3b8;
    margin-top: 4px;
  }
  .status-box {
    background: #0f172a;
    border-radius: 8px;
    padding: 12px;
    border: 1px solid #334155;
  }
  .status-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 0.85rem;
    font-weight: 600;
    margin-bottom: 6px;
  }
  .badge {
    padding: 3px 8px;
    border-radius: 12px;
    font-size: 0.7rem;
    text-transform: uppercase;
  }
  .badge-idle { background: #1e3a8a; color: #93c5fd; }
  .badge-running { background: #166534; color: #4ade80; }
  .progress-bg {
    background: #334155;
    height: 6px;
    border-radius: 3px;
    overflow: hidden;
  }
  .progress-fill {
    background: #38bdf8;
    height: 100%;
    width: 0%;
    transition: width 0.2s ease;
  }
</style>
</head>
<body>
  <div class="header">
    <h1>Dual TMC2209 Controller</h1>
    <p>Independent & Synchronized Dual Stepper Motor Control (Continuous Enable & Position Hold)</p>
  </div>

  <div class="master-bar">
    <div class="master-title">Synchronized Dual Control</div>
    <div class="master-btns">
      <button class="m-btn m-ccw" onclick="run('all', 'ccw')">&#x27f2; Move Both CCW</button>
      <button class="m-btn m-cw" onclick="run('all', 'cw')">&#x27f3; Move Both CW</button>
      <button class="m-btn m-stop" onclick="stopMotor('all')">&#x23f9; STOP ALL</button>
    </div>
  </div>

  <div class="grid">
    <!-- MOTOR 1 CARD -->
    <div class="card" id="cardM1">
      <div class="card-title">
        <span>Motor 1</span>
        <span class="addr-tag">ADDR 0b00 | STEP Pin 18</span>
      </div>

      <div>
        <label>Target Step Amount</label>
        <div class="input-group">
          <input type="number" id="stepsM1" value="1600" min="1" step="1">
        </div>
        <div class="presets">
          <button class="preset-btn" onclick="setPreset('M1', 200)">200</button>
          <button class="preset-btn" onclick="setPreset('M1', 400)">400</button>
          <button class="preset-btn" onclick="setPreset('M1', 800)">800</button>
          <button class="preset-btn" onclick="setPreset('M1', 1600)">1600</button>
          <button class="preset-btn" onclick="setPreset('M1', 3200)">3200</button>
        </div>
      </div>

      <div>
        <label>Movement Control</label>
        <div class="btn-group">
          <button class="action-btn ccw" onclick="run(1, 'ccw')">&#x27f2; Move CCW</button>
          <button class="action-btn cw" onclick="run(1, 'cw')">&#x27f3; Move CW</button>
        </div>
        <button class="stop" style="margin-top:8px;" onclick="stopMotor(1)">&#x23f9; STOP M1</button>
      </div>

      <div>
        <label>Holding Current / Position Lock</label>
        <div class="range-container">
          <input type="range" min="0" max="31" step="1" value="8" id="holdM1" oninput="updateHoldVal(1, this.value)" onchange="sendHold(1, this.value)">
          <div class="info-row">
            <span id="holdValM1">8 / 31 scale</span>
            <span id="holdLblM1">Active Hold Torque</span>
          </div>
        </div>
      </div>

      <div>
        <label>Microstepping</label>
        <div class="presets">
          <button class="preset-btn ms-m1" id="ms1_1" onclick="setMicrosteps(1, 1)">1/1</button>
          <button class="preset-btn ms-m1" id="ms1_2" onclick="setMicrosteps(1, 2)">1/2</button>
          <button class="preset-btn ms-m1" id="ms1_4" onclick="setMicrosteps(1, 4)">1/4</button>
          <button class="preset-btn ms-m1" id="ms1_8" onclick="setMicrosteps(1, 8)">1/8</button>
          <button class="preset-btn ms-m1" id="ms1_16" onclick="setMicrosteps(1, 16)">1/16</button>
          <button class="preset-btn ms-m1" id="ms1_32" onclick="setMicrosteps(1, 32)">1/32</button>
        </div>
      </div>

      <div>
        <label>Driver Mode</label>
        <div class="mode-toggle">
          <button id="btnSpreadM1" class="mode-btn active" onclick="setMode(1, 1)">SpreadCycle</button>
          <button id="btnStealthM1" class="mode-btn" onclick="setMode(1, 0)">StealthChop</button>
        </div>
      </div>

      <div>
        <label>Run Current (RMS)</label>
        <div class="range-container">
          <input type="range" min="200" max="1500" step="50" value="700" id="currentM1" oninput="updateCurrentVal(1, this.value)" onchange="sendCurrent(1, this.value)">
          <div class="info-row">
            <span id="curValM1">700 mA</span>
            <span id="curLblM1">Cool & Safe</span>
          </div>
        </div>
      </div>

      <div>
        <label>Speed (Step Interval)</label>
        <div class="range-container">
          <input type="range" min="100" max="5000" value="1000" id="speedM1" oninput="updateSpeedVal(1, this.value)" onchange="sendSpeed(1, this.value)">
          <div class="info-row">
            <span id="spdUsM1">1000 &#181;s</span>
            <span id="spdRateM1">1000 steps/s</span>
          </div>
        </div>
      </div>

      <div class="status-box">
        <div class="status-header">
          <span id="statusTxtM1">Status: Holding Position</span>
          <span id="badgeM1" class="badge badge-idle">Holding Position</span>
        </div>
        <div class="progress-bg">
          <div id="progFillM1" class="progress-fill"></div>
        </div>
      </div>
    </div>

    <!-- MOTOR 2 CARD -->
    <div class="card" id="cardM2">
      <div class="card-title">
        <span>Motor 2</span>
        <span class="addr-tag">ADDR 0b01 | STEP Pin 19</span>
      </div>

      <div>
        <label>Target Step Amount</label>
        <div class="input-group">
          <input type="number" id="stepsM2" value="1600" min="1" step="1">
        </div>
        <div class="presets">
          <button class="preset-btn" onclick="setPreset('M2', 200)">200</button>
          <button class="preset-btn" onclick="setPreset('M2', 400)">400</button>
          <button class="preset-btn" onclick="setPreset('M2', 800)">800</button>
          <button class="preset-btn" onclick="setPreset('M2', 1600)">1600</button>
          <button class="preset-btn" onclick="setPreset('M2', 3200)">3200</button>
        </div>
      </div>

      <div>
        <label>Movement Control</label>
        <div class="btn-group">
          <button class="action-btn ccw" onclick="run(2, 'ccw')">&#x27f2; Move CCW</button>
          <button class="action-btn cw" onclick="run(2, 'cw')">&#x27f3; Move CW</button>
        </div>
        <button class="stop" style="margin-top:8px;" onclick="stopMotor(2)">&#x23f9; STOP M2</button>
      </div>

      <div>
        <label>Holding Current / Position Lock</label>
        <div class="range-container">
          <input type="range" min="0" max="31" step="1" value="8" id="holdM2" oninput="updateHoldVal(2, this.value)" onchange="sendHold(2, this.value)">
          <div class="info-row">
            <span id="holdValM2">8 / 31 scale</span>
            <span id="holdLblM2">Active Hold Torque</span>
          </div>
        </div>
      </div>

      <div>
        <label>Microstepping</label>
        <div class="presets">
          <button class="preset-btn ms-m2" id="ms2_1" onclick="setMicrosteps(2, 1)">1/1</button>
          <button class="preset-btn ms-m2" id="ms2_2" onclick="setMicrosteps(2, 2)">1/2</button>
          <button class="preset-btn ms-m2" id="ms2_4" onclick="setMicrosteps(2, 4)">1/4</button>
          <button class="preset-btn ms-m2" id="ms2_8" onclick="setMicrosteps(2, 8)">1/8</button>
          <button class="preset-btn ms-m2" id="ms2_16" onclick="setMicrosteps(2, 16)">1/16</button>
          <button class="preset-btn ms-m2" id="ms2_32" onclick="setMicrosteps(2, 32)">1/32</button>
        </div>
      </div>

      <div>
        <label>Driver Mode</label>
        <div class="mode-toggle">
          <button id="btnSpreadM2" class="mode-btn active" onclick="setMode(2, 1)">SpreadCycle</button>
          <button id="btnStealthM2" class="mode-btn" onclick="setMode(2, 0)">StealthChop</button>
        </div>
      </div>

      <div>
        <label>Run Current (RMS)</label>
        <div class="range-container">
          <input type="range" min="200" max="1500" step="50" value="700" id="currentM2" oninput="updateCurrentVal(2, this.value)" onchange="sendCurrent(2, this.value)">
          <div class="info-row">
            <span id="curValM2">700 mA</span>
            <span id="curLblM2">Cool & Safe</span>
          </div>
        </div>
      </div>

      <div>
        <label>Speed (Step Interval)</label>
        <div class="range-container">
          <input type="range" min="100" max="5000" value="1000" id="speedM2" oninput="updateSpeedVal(2, this.value)" onchange="sendSpeed(2, this.value)">
          <div class="info-row">
            <span id="spdUsM2">1000 &#181;s</span>
            <span id="spdRateM2">1000 steps/s</span>
          </div>
        </div>
      </div>

      <div class="status-box">
        <div class="status-header">
          <span id="statusTxtM2">Status: Holding Position</span>
          <span id="badgeM2" class="badge badge-idle">Holding Position</span>
        </div>
        <div class="progress-bg">
          <div id="progFillM2" class="progress-fill"></div>
        </div>
      </div>
    </div>
  </div>

<script>
let pollInterval = null;

function setPreset(motorTag, val) {
  document.getElementById('steps' + motorTag).value = val;
}

function run(mId, dir) {
  let steps = 1600;
  if (mId === 1) steps = parseInt(document.getElementById('stepsM1').value) || 1600;
  else if (mId === 2) steps = parseInt(document.getElementById('stepsM2').value) || 1600;
  else steps = parseInt(document.getElementById('stepsM1').value) || 1600;

  fetch('/run?motor=' + mId + '&dir=' + dir + '&steps=' + steps)
    .then(() => startPolling());
}

function stopMotor(mId) {
  fetch('/stop?motor=' + mId).then(() => fetchStatus());
}

function setMode(mId, spread) {
  fetch('/mode?motor=' + mId + '&spread=' + spread).then(() => fetchStatus());
}

function setMicrosteps(mId, val) {
  fetch('/microsteps?motor=' + mId + '&val=' + val).then(() => fetchStatus());
}

function updateSpeedVal(mId, val) {
  const tag = mId === 1 ? 'M1' : 'M2';
  document.getElementById('spdUs' + tag).innerText = val + ' \u00b5s';
  const stepsPerSec = Math.round(1000000 / val);
  document.getElementById('spdRate' + tag).innerText = stepsPerSec + ' steps/s';
}

function sendSpeed(mId, val) {
  fetch('/speed?motor=' + mId + '&val=' + val);
}

function updateCurrentVal(mId, val) {
  const tag = mId === 1 ? 'M1' : 'M2';
  document.getElementById('curVal' + tag).innerText = val + ' mA';
  const label = document.getElementById('curLbl' + tag);
  if (val <= 600) label.innerText = 'Very Cool';
  else if (val <= 900) label.innerText = 'Cool & Safe';
  else if (val <= 1200) label.innerText = 'Warm / Medium';
  else label.innerText = 'Hot / High Torque';
}

function sendCurrent(mId, val) {
  fetch('/current?motor=' + mId + '&val=' + val);
}

function updateHoldVal(mId, val) {
  const tag = mId === 1 ? 'M1' : 'M2';
  document.getElementById('holdVal' + tag).innerText = val + ' / 31 scale';
  const label = document.getElementById('holdLbl' + tag);
  if (val == 0) label.innerText = 'Free Spin (No Hold)';
  else if (val <= 8) label.innerText = 'Cool Hold Torque';
  else if (val <= 16) label.innerText = 'Medium Hold Torque';
  else label.innerText = 'Strong Hold Torque';
}

function sendHold(mId, val) {
  fetch('/hold?motor=' + mId + '&val=' + val);
}

function startPolling() {
  if (!pollInterval) {
    pollInterval = setInterval(fetchStatus, 250);
  }
}

function stopPolling() {
  if (pollInterval) {
    clearInterval(pollInterval);
    pollInterval = null;
  }
}

function updateMotorUI(tag, data) {
  const badge = document.getElementById('badge' + tag);
  const statusTxt = document.getElementById('statusTxt' + tag);
  const progFill = document.getElementById('progFill' + tag);

  // Sync current value if changed externally
  if (data.currentMa && document.getElementById('current' + tag).value != data.currentMa) {
    document.getElementById('current' + tag).value = data.currentMa;
    updateCurrentVal(tag === 'M1' ? 1 : 2, data.currentMa);
  }

  // Sync hold scale value
  if (data.holdScale !== undefined && document.getElementById('hold' + tag).value != data.holdScale) {
    document.getElementById('hold' + tag).value = data.holdScale;
    updateHoldVal(tag === 'M1' ? 1 : 2, data.holdScale);
  }

  // Sync mode buttons
  const btnSpread = document.getElementById('btnSpread' + tag);
  const btnStealth = document.getElementById('btnStealth' + tag);
  if (data.spreadCycle) {
    btnSpread.className = 'mode-btn active';
    btnStealth.className = 'mode-btn';
  } else {
    btnSpread.className = 'mode-btn';
    btnStealth.className = 'mode-btn active';
  }

  // Sync microstep buttons
  if (data.microsteps) {
    document.querySelectorAll('.ms-' + tag.toLowerCase()).forEach(b => b.classList.remove('active'));
    const msId = (tag === 'M1' ? 'ms1_' : 'ms2_') + data.microsteps;
    const msBtn = document.getElementById(msId);
    if (msBtn) msBtn.classList.add('active');
  }

  if (data.running) {
    badge.className = 'badge badge-running';
    badge.innerText = 'Moving';
    const completed = data.targetSteps - data.stepsRemaining;
    const pct = data.targetSteps > 0 ? Math.min(100, Math.round((completed / data.targetSteps) * 100)) : 0;
    statusTxt.innerText = data.dir.toUpperCase() + ': ' + completed + ' / ' + data.targetSteps + ' steps (' + pct + '%)';
    progFill.style.width = pct + '%';
  } else {
    badge.className = 'badge badge-idle';
    badge.innerText = 'Holding Position';
    statusTxt.innerText = 'Status: Holding Position';
    progFill.style.width = '0%';
  }
}

function fetchStatus() {
  fetch('/status')
    .then(r => r.json())
    .then(data => {
      updateMotorUI('M1', data.m1);
      updateMotorUI('M2', data.m2);

      if (data.m1.running || data.m2.running) {
        startPolling();
      } else {
        stopPolling();
      }
    })
    .catch(() => {});
}

// Initial status check
fetchStatus();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleRun() {
  String motorArg = server.hasArg("motor") ? server.arg("motor") : "1";
  String dir = server.arg("dir");
  uint32_t steps = 1600;
  if (server.hasArg("steps")) {
    long parsed = server.arg("steps").toInt();
    if (parsed > 0) steps = parsed;
  }

  auto startMotor = [](Motor& m, bool cw, uint32_t st) {
    applyDirection(m, cw);
    m.stepsRemaining = st;
    m.targetSteps = st;
    m.stepPinState = false;
    digitalWrite(m.stepPin, LOW);
    m.lastStepUs = micros();
    m.running = true;
  };

  if (motorArg == "2") {
    startMotor(motor2, dir == "cw", steps);
  } else if (motorArg == "all" || motorArg == "both") {
    startMotor(motor1, dir == "cw", steps);
    startMotor(motor2, dir == "cw", steps);
  } else {
    startMotor(motor1, dir == "cw", steps);
  }

  server.send(200, "text/plain", "OK");
}

void handleStop() {
  String motorArg = server.hasArg("motor") ? server.arg("motor") : "all";
  
  auto stopM = [](Motor& m) {
    m.running = false;
    m.stepsRemaining = 0;
    m.targetSteps = 0;
    digitalWrite(m.stepPin, LOW);
  };

  if (motorArg == "1") {
    stopM(motor1);
  } else if (motorArg == "2") {
    stopM(motor2);
  } else {
    stopM(motor1);
    stopM(motor2);
  }

  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("val")) {
    uint32_t val = server.arg("val").toInt();
    String motorArg = server.hasArg("motor") ? server.arg("motor") : "1";
    if (motorArg == "2") {
      motor2.stepIntervalUs = val;
    } else if (motorArg == "all") {
      motor1.stepIntervalUs = val;
      motor2.stepIntervalUs = val;
    } else {
      motor1.stepIntervalUs = val;
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleCurrent() {
  if (server.hasArg("val")) {
    uint16_t val = server.arg("val").toInt();
    if (val < 100) val = 100;
    if (val > 1800) val = 1800;

    auto setC = [](Motor& m, uint16_t c) {
      m.currentMa = c;
      m.driver->rms_current(c);
    };

    String motorArg = server.hasArg("motor") ? server.arg("motor") : "1";
    if (motorArg == "2") {
      setC(motor2, val);
    } else if (motorArg == "all") {
      setC(motor1, val);
      setC(motor2, val);
    } else {
      setC(motor1, val);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleHold() {
  if (server.hasArg("val")) {
    uint8_t val = server.arg("val").toInt();
    if (val > 31) val = 31;

    auto setH = [](Motor& m, uint8_t h) {
      m.holdScale = h;
      m.driver->ihold(h);
    };

    String motorArg = server.hasArg("motor") ? server.arg("motor") : "1";
    if (motorArg == "2") {
      setH(motor2, val);
    } else if (motorArg == "all") {
      setH(motor1, val);
      setH(motor2, val);
    } else {
      setH(motor1, val);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleMode() {
  if (server.hasArg("spread")) {
    bool spread = (server.arg("spread") == "1");
    String motorArg = server.hasArg("motor") ? server.arg("motor") : "1";
    if (motorArg == "2") {
      applyChopMode(motor2, spread);
    } else if (motorArg == "all") {
      applyChopMode(motor1, spread);
      applyChopMode(motor2, spread);
    } else {
      applyChopMode(motor1, spread);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleMicrosteps() {
  if (server.hasArg("val")) {
    uint16_t val = server.arg("val").toInt();
    if (val == 1 || val == 2 || val == 4 || val == 8 || 
        val == 16 || val == 32 || val == 64 || val == 128 || val == 256) {
      auto setMs = [](Motor& m, uint16_t ms) {
        m.microstepsVal = ms;
        m.driver->microsteps(ms);
      };
      String motorArg = server.hasArg("motor") ? server.arg("motor") : "1";
      if (motorArg == "2") {
        setMs(motor2, val);
      } else if (motorArg == "all") {
        setMs(motor1, val);
        setMs(motor2, val);
      } else {
        setMs(motor1, val);
      }
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  auto motorJson = [](const Motor& m) {
    String j = "{";
    j += "\"running\":" + String(m.running ? "true" : "false") + ",";
    j += "\"dir\":\"" + String(m.dirCW ? "cw" : "ccw") + "\",";
    j += "\"stepsRemaining\":" + String(m.stepsRemaining) + ",";
    j += "\"targetSteps\":" + String(m.targetSteps) + ",";
    j += "\"stepIntervalUs\":" + String(m.stepIntervalUs) + ",";
    j += "\"currentMa\":" + String(m.currentMa) + ",";
    j += "\"holdScale\":" + String(m.holdScale) + ",";
    j += "\"spreadCycle\":" + String(m.spreadCycleMode ? "true" : "false") + ",";
    j += "\"microsteps\":" + String(m.microstepsVal);
    j += "}";
    return j;
  };

  String json = "{";
  json += "\"m1\":" + motorJson(motor1) + ",";
  json += "\"m2\":" + motorJson(motor2);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);

  // Keep both drivers continuously enabled (EN pins LOW)
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(EN_PIN_1, OUTPUT);
  digitalWrite(EN_PIN_1, LOW);

  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);
  digitalWrite(EN_PIN_2, LOW);

  SERIAL_PORT.begin(115200, SERIAL_8N1, 16, 17);

  // --- Initialize Driver 1 ---
  driver1.begin();
  driver1.toff(4);
  driver1.pdn_disable(true);
  driver1.I_scale_analog(false);
  driver1.mstep_reg_select(true);
  driver1.rms_current(700);
  driver1.microsteps(16);
  applyChopMode(motor1, true);
  driver1.ihold(8); // Set active position holding current scale
  driver1.iholddelay(10);

  // --- Initialize Driver 2 ---
  driver2.begin();
  driver2.toff(4);
  driver2.pdn_disable(true);
  driver2.I_scale_analog(false);
  driver2.mstep_reg_select(true);
  driver2.rms_current(700);
  driver2.microsteps(16);
  applyChopMode(motor2, true);
  driver2.ihold(8); // Set active position holding current scale
  driver2.iholddelay(10);

  Serial.println("Dual TMC2209 Controller (Continuous Driver Enable & Position Hold Active)");
  Serial.print("Driver 1 Version: 0x");
  Serial.println(driver1.version(), HEX);
  Serial.print("Driver 2 Version: 0x");
  Serial.println(driver2.version(), HEX);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/run", handleRun);
  server.on("/stop", handleStop);
  server.on("/speed", handleSpeed);
  server.on("/current", handleCurrent);
  server.on("/hold", handleHold);
  server.on("/mode", handleMode);
  server.on("/microsteps", handleMicrosteps);
  server.on("/status", handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();
  handleStepping();
}