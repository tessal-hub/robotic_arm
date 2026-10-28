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
#include "sensor.h"
#include "web_server.h"
#include "wifi_manager.h"

// ---- Đối tượng toàn cục (static allocation, không heap cho module chính) ----
static SemaphoreHandle_t g_uartMutex = nullptr;

static Motor g_motors[NUM_MOTORS] = {
    {&SERIAL_PORT_1, R_SENSE, UART_ADDR_J1, STEP_PIN_0, PIN_UNSET, "J1-Base"},
    {&SERIAL_PORT_1, R_SENSE, UART_ADDR_J2, STEP_PIN_1, PIN_UNSET, "J2-Shoulder"},
    {&SERIAL_PORT_1, R_SENSE, UART_ADDR_J3, STEP_PIN_2, PIN_UNSET, "J3-Elbow"},
    {&SERIAL_PORT_1, R_SENSE, UART_ADDR_J4, STEP_PIN_3, PIN_UNSET, "J4-WristPan"},
    {nullptr,        R_SENSE, 0,            STEP_PIN_4, DIR_PIN_4,  "J5-Revolute"},
    {nullptr,        R_SENSE, 0,            STEP_PIN_5, DIR_PIN_5,  "J6-Roll"},
};

static Sensor g_sensor;
static Endstops g_endstops;
static NvsStore g_nvs;
static JointModel g_joints;
static HomingController g_homing;
static WifiManager g_wifi;
static WebServer g_server(WEB_SERVER_PORT);
static ArmController g_arm;

static String s_serialBuf = "";

static void processSerialLine(String line) {
    line.trim();
    if (line.isEmpty()) return;
    String upper = line;
    upper.toUpperCase();

    if (upper == "FREE") {
        ArmCommand c; c.type = ArmCommand::TEACH_FREE; c.value = 1.0f;
        (void)g_arm.submit(c);
        Serial.println("[SERIAL] TEACH_FREE: J1..J3 tha long de keo tay, J4..J6 khoa cung Home.");
    } else if (upper == "LOCK") {
        ArmCommand c; c.type = ArmCommand::TEACH_FREE; c.value = 0.0f;
        (void)g_arm.submit(c);
        Serial.println("[SERIAL] TEACH_LOCK: J1..J3 da khoa lai va dong bo encoder.");
    } else if (upper == "REC" || upper == "SAVE" || upper == "DOWN") {
        ArmCommand c; c.type = ArmCommand::TEACH_RECORD; c.value = 1.0f;
        (void)g_arm.submit(c);
        Serial.println("[SERIAL] TEACH_RECORD: Ghi diem VE (Pen DOWN)");
    } else if (upper == "UP" || upper == "TRAVEL") {
        ArmCommand c; c.type = ArmCommand::TEACH_RECORD; c.value = 0.0f;
        (void)g_arm.submit(c);
        Serial.println("[SERIAL] TEACH_RECORD: Ghi diem NHAC BUT (Pen UP/Travel)");
    } else if (upper == "UNDO") {
        ArmCommand c; c.type = ArmCommand::TEACH_UNDO;
        (void)g_arm.submit(c);
    } else if (upper == "CLEAR") {
        ArmCommand c; c.type = ArmCommand::TEACH_CLEAR;
        (void)g_arm.submit(c);
        Serial.println("[SERIAL] TEACH_CLEAR: Da xoa toan bo diem.");
    } else if (upper == "PLAY") {
        ArmCommand c; c.type = ArmCommand::TEACH_PLAY; c.p[0] = 0.5f; c.p[1] = 15.0f;
        (void)g_arm.submit(c);
        Serial.println("[SERIAL] TEACH_PLAY: Bat dau chay lai quy dao...");
    } else if (upper == "STOP" || upper == "S" || upper == "X") {
        (void)g_arm.submit({ArmCommand::STOP_ALL});
        Serial.println("[SERIAL] STOP ALL");
    } else if (upper == "LIST") {
        if (g_arm.teachManager() != nullptr) {
            Serial.printf("[SERIAL] Danh sach diem (%u diem):\n%s\n",
                          static_cast<unsigned>(g_arm.teachManager()->pointCount()),
                          g_arm.teachManager()->getPointsJson().c_str());
        }
    } else if (upper == "POS") {
        Serial.printf("[SERIAL] POS: J1=%.2f, J2=%.2f, J3=%.2f\n",
                      g_joints.angleFromEncoder(0),
                      g_joints.angleFromEncoder(1),
                      g_joints.angleFromEncoder(2));
    } else if (upper == "SAVE FLASH" || upper == "FLASH SAVE") {
        if (g_arm.busy()) {
            Serial.println("[SERIAL] Tu choi: Robot dang ban, khong the ghi Flash!");
        } else if (g_arm.teachManager() != nullptr && g_arm.teachManager()->saveToNvs(&g_nvs)) {
            Serial.println("[SERIAL] Da luu quy dao vao Flash (NVS) thanh cong!");
        } else {
            Serial.println("[SERIAL] Luu vao Flash (NVS) that bai!");
        }
    } else if (upper == "LOAD FLASH" || upper == "FLASH LOAD") {
        if (g_arm.busy()) {
            Serial.println("[SERIAL] Tu choi: Robot dang ban, khong the doc Flash!");
        } else if (g_arm.teachManager() != nullptr && g_arm.teachManager()->loadFromNvs(&g_nvs)) {
            Serial.println("[SERIAL] Da doc quy dao tu Flash (NVS) thanh cong!");
        } else {
            Serial.println("[SERIAL] Doc tu Flash (NVS) that bai!");
        }
    } else if (upper.startsWith("JOG ")) {
        int axis = 0;
        float deg = 0;
        // SỬA LỖI: Dùng upper.c_str() thay vì line.c_str() để nhận cả chữ thường "jog 1 10"
        if (sscanf(upper.c_str(), "JOG %d %f", &axis, &deg) == 2 && axis >= 1 && axis <= 6) {
            if (g_arm.isWristLocked() && axis >= 4) {
                Serial.println("[SERIAL] Tu choi: J4..J6 dang khoa cung! (Dung lenh WRIST UNLOCK de mo)");
            } else {
                ArmCommand c; c.type = ArmCommand::JOG_REL; c.axis = static_cast<uint8_t>(axis - 1); c.value = deg;
                (void)g_arm.submit(c);
                Serial.printf("[SERIAL] JOG J%d %+f deg\n", axis, deg);
            }
        }
    } else if (upper == "WRIST UNLOCK") {
        g_arm.setWristLocked(false);
    } else if (upper == "WRIST LOCK") {
        g_arm.setWristLocked(true);
    } else {
        Serial.println("[SERIAL] Lenh ho tro: FREE, LOCK, REC, UP, UNDO, CLEAR, LIST, PLAY, STOP, POS, FLASH SAVE, FLASH LOAD, JOG <1..3> <deg>");
    }
}

void setup() {
    // 0) Tách JTAG/ROM matrix (gpio_reset_pin), khóa LOW và cấu hình push-pull 40mA cho tất cả các chân STEP/DIR của 6 motor ngay từ đầu
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        const uint8_t sp = g_motors[i].getStepPin();
        const uint8_t dp = g_motors[i].getDirPin();
        gpio_reset_pin(static_cast<gpio_num_t>(sp));
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = (1ULL << sp);
        cfg.mode = GPIO_MODE_OUTPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
        gpio_set_level(static_cast<gpio_num_t>(sp), 0);
        gpio_set_drive_capability(static_cast<gpio_num_t>(sp), GPIO_DRIVE_CAP_3);

        if (dp != PIN_UNSET && dp != 255) {
            gpio_reset_pin(static_cast<gpio_num_t>(dp));
            gpio_config_t dirCfg = {};
            dirCfg.pin_bit_mask = (1ULL << dp);
            dirCfg.mode = GPIO_MODE_OUTPUT;
            dirCfg.pull_up_en = GPIO_PULLUP_DISABLE;
            dirCfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            dirCfg.intr_type = GPIO_INTR_DISABLE;
            gpio_config(&dirCfg);
            gpio_set_level(static_cast<gpio_num_t>(dp), 0);
            gpio_set_drive_capability(static_cast<gpio_num_t>(dp), GPIO_DRIVE_CAP_3);
        }
    }

    Serial.begin(115200);
    delay(1000); // Startup wait before UART init; verify on cold power-up.
    Serial.printf("\n\n=== %s v%s (%s) ===\n", FW_NAME, FW_VERSION, FW_BUILD_DATE);

    // 1) NVS
    g_nvs.begin();

    // 2) UART chung cho 4x TMC2209 + mutex
    SERIAL_PORT_1.begin(TMC_UART_BAUD, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
    delay(50); // Cho bus UART và driver ổn định (như trong tools/j3_test)
    g_uartMutex = xSemaphoreCreateMutex();

    // 3) Motors (set mutex TRƯỚC begin)
    Serial.println("[MAIN] Khoi tao motors...");
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        g_motors[i].setUartMutex(&g_uartMutex);
    }
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        delay(10);
        const uint8_t holdScale = (i == 3) ? WRIST_J4_HOLD_SCALE : DEFAULT_HOLD_SCALE;
        const bool useSpreadCycle = (i == 1); // J2 dùng SpreadCycle đồng nhất với Homing
        g_motors[i].begin(DEFAULT_AXIS_RUN_CURRENTS[i], DEFAULT_MICROSTEPS,
                          useSpreadCycle, holdScale);
    }

    // 4) Endstops (ISR abort an toàn)
    Motor* motorPtrs[NUM_MOTORS];
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motorPtrs[i] = &g_motors[i];
    g_endstops.begin(motorPtrs);

    // 5) Sensors + joint model + restore vị trí từ NVS
    g_sensor.begin();
    g_joints.begin(motorPtrs, &g_sensor);
    g_joints.attachNvs(&g_nvs);
    delay(400); // cho sensor task quét ít nhất vài vòng trước khi restore
    const uint8_t restoredCount = g_joints.restoreFromNVS();
    Serial.printf("[MAIN] Restore tu NVS: %u khop (khong can home lai neu du 6)\n",
                  restoredCount);
    if (LOCK_WRIST_J4_J6) {
        for (uint8_t a = 3; a < NUM_MOTORS; ++a) {
            if (!g_joints.isHomed(a)) {
                g_joints.setHomeHere(a);
                Serial.printf("[MAIN] LOCK_WRIST_J4_J6: Tu dong dat moc Home 0 deg cho J%u\n", a + 1);
            }
        }
    }

    // 6) Homing FSM + Arm arbiter (motion task core 1)
    g_homing.begin(motorPtrs, &g_endstops, &g_joints);
    armSetWifiProvider(&g_wifi);
    g_arm.begin(motorPtrs, &g_sensor, &g_endstops, &g_joints, &g_homing, &g_nvs);

    // Tự động khôi phục Teach Trajectory từ Flash (NVS) nếu có
    if (g_arm.teachManager() != nullptr) {
        g_arm.teachManager()->loadFromNvs(&g_nvs);
    }

    // 7) WiFi: STA(NVS creds) -> AP fallback
    g_wifi.begin(&g_nvs);

    // 8) mDNS cả hai chế độ
    if (MDNS.begin(DEFAULT_MDNS_HOST)) {
        MDNS.addService("http", "tcp", WEB_SERVER_PORT);
        Serial.printf("[MAIN] mDNS: http://%s.local\n", DEFAULT_MDNS_HOST);
    }

    // 9) Web app
    webBegin(g_server, &g_arm, &g_wifi, &g_joints, &g_nvs);

    Serial.printf("[MAIN] San sang! IP: %s\n", g_wifi.ipString().c_str());
}

void loop() {
    g_server.handleClient();
    while (Serial.available()) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (s_serialBuf.length() > 0) {
                processSerialLine(s_serialBuf);
                s_serialBuf = "";
            }
        } else {
            s_serialBuf += c;
        }
    }
}
