#include <Arduino.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "arm.h"
#include "config.h"
#include "endstop.h"
#include "homing.h"
#include "joint_model.h"
#include "motor.h"
#include "nvs_store.h"
#include "planner.h"
#include "sensor.h"
#include "web_server.h"
#include "wifi_manager.h"
#include "work_plane.h"

// ---- Đối tượng toàn cục (static allocation, không heap cho module chính) ----
static SemaphoreHandle_t g_uartMutex = nullptr;

static Motor g_motors[NUM_MOTORS] = {
    {&SERIAL_PORT_1, R_SENSE, UART_ADDR_J1, STEP_PIN_0, PIN_UNSET, "J1-Base"},
    {&SERIAL_PORT_1, R_SENSE, UART_ADDR_J2, STEP_PIN_1, PIN_UNSET, "J2-Shoulder"},
    {&SERIAL_PORT_1, R_SENSE, UART_ADDR_J3, STEP_PIN_2, PIN_UNSET, "J3-Elbow"},
    {&SERIAL_PORT_1, R_SENSE, UART_ADDR_J4, STEP_PIN_3, PIN_UNSET, "J4-WristPan"},
    {nullptr,        R_SENSE, 0,            STEP_PIN_4, DIR_PIN_4,  "J5-Tilt"},
    {nullptr,        R_SENSE, 0,            STEP_PIN_5, DIR_PIN_5,  "J6-Roll"},
};

static Sensor g_sensor;
static Endstops g_endstops;
static NvsStore g_nvs;
static JointModel g_joints;
static HomingController g_homing;
static WorkPlane g_workPlane;
static Planner g_planner;
static WifiManager g_wifi;
static WebServer g_server(WEB_SERVER_PORT);
static ArmController g_arm;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.printf("\n\n=== %s v%s (%s) ===\n", FW_NAME, FW_VERSION, FW_BUILD_DATE);

    // 1) NVS
    g_nvs.begin();

    // 2) UART chung cho 4x TMC2209 + mutex
    SERIAL_PORT_1.begin(TMC_UART_BAUD, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
    g_uartMutex = xSemaphoreCreateMutex();

    // 3) Motors (set mutex TRƯỚC begin)
    Serial.println("[MAIN] Khoi tao motors...");
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        g_motors[i].setUartMutex(&g_uartMutex);
    }
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        g_motors[i].begin(DEFAULT_NORMAL_CURRENT, DEFAULT_MICROSTEPS,
                          true, DEFAULT_HOLD_SCALE);
    }

    // 4) Endstops (ISR abort an toàn)
    Motor* motorPtrs[NUM_MOTORS];
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motorPtrs[i] = &g_motors[i];
    g_endstops.begin(motorPtrs);

    // 5) Sensors + joint model + restore vị trí từ NVS
    g_sensor.begin();
    g_joints.begin(motorPtrs, &g_sensor);
    g_joints.attachNvs(&g_nvs);
    delay(300); // cho sensor task quét ít nhất vài vòng trước khi restore
    const uint8_t restoredCount = g_joints.restoreFromNVS();
    Serial.printf("[MAIN] Restore tu NVS: %u khop (khong can home lai neu du 6)\n",
                  restoredCount);

    // 6) Homing FSM + Planner + Arm arbiter (motion task core 1)
    g_homing.begin(motorPtrs, &g_endstops, &g_joints);
    g_planner.begin(motorPtrs, &g_joints);
    g_planner.setWorkPlane(&g_workPlane);
    armSetWifiProvider(&g_wifi);
    g_arm.begin(motorPtrs, &g_sensor, &g_endstops, &g_joints, &g_homing, &g_planner);

    // 7) WiFi: STA(NVS creds) -> AP fallback
    g_wifi.begin(&g_nvs);

    // 8) mDNS cả hai chế độ
    if (MDNS.begin(DEFAULT_MDNS_HOST)) {
        MDNS.addService("http", "tcp", WEB_SERVER_PORT);
        Serial.printf("[MAIN] mDNS: http://%s.local\n", DEFAULT_MDNS_HOST);
    }

    // 9) Web app
    webBegin(g_server, &g_arm, &g_wifi, &g_joints, &g_nvs, &g_workPlane);

    Serial.printf("[MAIN] San sang! IP: %s\n", g_wifi.ipString().c_str());
}

void loop() {
    g_server.handleClient();
}
