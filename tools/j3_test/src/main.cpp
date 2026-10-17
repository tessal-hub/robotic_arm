#include <Arduino.h>
#include <TMCStepper.h>
#include <memory>

// ==============================================================================
// CẤU HÌNH PHẦN CỨNG J3 (TRÍCH NGUYÊN TỪ src/config.h)
// ==============================================================================
constexpr uint8_t J3_STEP_PIN         = 41;      // STEP Pin của Motor 2 (J3 - Elbow Pitch)
constexpr int8_t  TMC_RX_PIN          = 16;      // ESP32 RX1
constexpr int8_t  TMC_TX_PIN          = 15;      // ESP32 TX1
constexpr uint32_t TMC_UART_BAUD      = 115200;  // Baudrate TMC2209
constexpr float   R_SENSE             = 0.11f;   // Điện trở cảm biến dòng
constexpr uint8_t DEFAULT_UART_ADDR   = 0b10;    // Địa chỉ UART chuẩn J3 (MS1=GND, MS2=VIO)
constexpr uint16_t DEFAULT_CURRENT_MA = 1700;    // Dòng định mức chuẩn J3 (mA)
constexpr uint8_t  DEFAULT_HOLD_SCALE = 30;      // Khóa cứng trục motor ở standstill (30/31 ~ 97%)
constexpr uint16_t DEFAULT_MICROSTEPS = 16;      // Vi bước 1/16

// Giữ các chân STEP của các trục khác ở mức LOW để chống rung / trôi
constexpr uint8_t OTHER_STEP_PINS[]   = { 1, 2, 42, 38, 40 };

// Biến điều khiển UART & Driver
static uint8_t currentUartAddr = DEFAULT_UART_ADDR;
static uint16_t currentMa = DEFAULT_CURRENT_MA;
static uint8_t currentHold = DEFAULT_HOLD_SCALE;
static bool useSpreadCycle = false;              // false = StealthChop, true = SpreadCycle
static std::unique_ptr<TMC2209Stepper> driver;

// Biến trạng thái chuyển động & gia tốc mềm (soft-ramp)
static bool moving = false;
static bool continuous = false;
static bool currentShaftDir = false;
static int32_t stepsRemaining = 0;
static uint32_t stepCounter = 0;
static uint32_t accelSteps = 150;
static uint32_t decelSteps = 150;
static uint32_t targetIntervalUs = 1000;         // Tốc độ đích (~1000 steps/s)
static uint32_t currentIntervalUs = 2500;        // Tốc độ khởi động mềm (~400 steps/s)
static uint32_t defaultStepBatch = 1600;         // Số bước mặc định khi bấm +/- đơn lẻ
static uint32_t lastStepMicros = 0;
static uint32_t totalStepsExecuted = 0;

void printHelp();
void printStatus();
void scanUartAddresses();
void initDriver(uint8_t addr, bool spread);
void clearShortLatch();

void printHelp() {
    Serial.println("\n=======================================================");
    Serial.println("   CHƯƠNG TRÌNH TEST J3 CHẨN ĐOÁN ÂM THANH & CHIỀU QUAY   ");
    Serial.println("=======================================================");
    Serial.println("LỆNH DI CHUYỂN:");
    Serial.println("  + / f         : Quay TIẾN (mặc định 1600 bước có gia tốc êm)");
    Serial.println("  - / r         : Quay LÙI (mặc định 1600 bước có gia tốc êm)");
    Serial.println("  + <bước>      : Quay TIẾN số bước tuỳ chọn (VD: + 3200 hoặc f 5000)");
    Serial.println("  - <bước>      : Quay LÙI số bước tuỳ chọn (VD: - 3200 hoặc r 5000)");
    Serial.println("  c+ / cont+    : Quay TIẾN LIÊN TỤC (cho tới khi bấm dừng)");
    Serial.println("  c- / cont-    : Quay LÙI LIÊN TỤC (cho tới khi bấm dừng)");
    Serial.println("  s / x / space : DỪNG NGAY LẬP TỨC (Stop)");
    Serial.println("-------------------------------------------------------");
    Serial.println("LỆNH CHẨN ĐOÁN LỖI 'ĐỘNG CƠ KHÔNG CỨNG / KHÔNG CHẠY':");
    Serial.println("  clear         : Reset chốt bảo vệ cầu H (toff: 4->0->4)");
    Serial.println("  chop <0|1>    : 0 = StealthChop (Êm), 1 = SpreadCycle (Khỏe/Ồn)");
    Serial.println("  curr <mA>     : Đổi dòng điện (VD: curr 1700, curr 1500)");
    Serial.println("  hold <0..31>  : Đổi lực giữ lúc đứng yên (VD: hold 30 cứng nhất)");
    Serial.println("  addr <0..3>   : Đổi địa chỉ UART đang test (VD: addr 0, addr 2)");
    Serial.println("  scan          : QUÉT TÌM địa chỉ thực của driver trên bus");
    Serial.println("  speed <us>    : Đổi chu kỳ xung (VD: speed 800, speed 1500)");
    Serial.println("  shaft <0|1>   : Ghi thử trực tiếp bit đảo chiều shaft qua UART");
    Serial.println("  info / status : Đọc toàn bộ thanh ghi TMC2209 & DRV_STATUS");
    Serial.println("  pulse <n>     : Phát cứng n xung STEP không qua UART");
    Serial.println("  h / help / ?  : Hiện lại menu hướng dẫn này");
    Serial.println("=======================================================\n");
}

void scanUartAddresses() {
    Serial.println("\n--- [QUÉT TOÀN BỘ 4 ĐỊA CHỈ UART TMC2209] ---");
    Serial.println("Đang gửi lệnh kiểm tra Version (kỳ vọng 0x21) tới từng địa chỉ...");
    int foundCount = 0;
    for (uint8_t a = 0; a <= 3; ++a) {
        TMC2209Stepper testDrv(&Serial1, R_SENSE, a);
        while (Serial1.available()) Serial1.read();
        const uint8_t v = testDrv.version();
        const bool err = testDrv.CRCerror;
        while (Serial1.available()) Serial1.read();
        const uint8_t ifcnt = testDrv.IFCNT();

        Serial.printf("  * Địa chỉ 0b%c%c (addr %u): ", (a & 2) ? '1' : '0', (a & 1) ? '1' : '0', a);
        if (!err && v == 0x21) {
            Serial.printf(">>> PHÁT HIỆN DRIVER! <<< (ver=0x%02X, IFCNT=%u)\n", v, ifcnt);
            foundCount++;
        } else {
            Serial.printf("Không phản hồi (ver=0x%02X, CRCerror=%d)\n", v, err);
        }
    }
    if (foundCount == 0) {
        Serial.println("\n[!] CẢNH BÁO: KHÔNG TÌM THẤY BẤT KỲ DRIVER NÀO TRÊN BUS UART!");
    } else {
        Serial.printf("\n[OK] Tìm thấy %d driver. Địa chỉ đang cấu hình: 0b%c%c (addr %u)\n",
                      foundCount, (currentUartAddr & 2) ? '1' : '0', (currentUartAddr & 1) ? '1' : '0', currentUartAddr);
    }
    Serial.println("----------------------------------------------\n");
}

void printStatus() {
    Serial.printf("\n--- TRẠNG THÁI DRIVER J3 (TMC2209 @ UART 0b%c%c / Addr %u) ---\n",
                  (currentUartAddr & 2) ? '1' : '0', (currentUartAddr & 1) ? '1' : '0', currentUartAddr);
    Serial.printf("Chân STEP: GPIO %u | UART RX: GPIO %u | TX: GPIO %u\n",
                  J3_STEP_PIN, TMC_RX_PIN, TMC_TX_PIN);

    while (Serial1.available()) Serial1.read();
    const uint8_t ver = driver->version();
    const bool verErr = driver->CRCerror;
    Serial.printf("Version: 0x%02X (Kỳ vọng: 0x21) -> %s (CRCerror=%d)\n",
                  ver, (!verErr && ver == 0x21) ? "OK (KẾT NỐI TỐT)" : "LỖI / KHÔNG PHẢN HỒI", verErr);

    while (Serial1.available()) Serial1.read();
    const uint8_t ifcnt = driver->IFCNT();
    Serial.printf("Giao tiếp IFCNT: %u (CRCerror=%d)\n", ifcnt, driver->CRCerror);

    while (Serial1.available()) Serial1.read();
    const bool shaftVal = driver->shaft();
    Serial.printf("Chiều quay SHAFT hiện tại: %d (CRCerror=%d)\n", shaftVal, driver->CRCerror);

    while (Serial1.available()) Serial1.read();
    const uint8_t toffVal = driver->toff();
    Serial.printf("Cầu H TOFF (0=Tắt, >0=Bật): %u\n", toffVal);

    while (Serial1.available()) Serial1.read();
    const uint32_t ioin = driver->IOIN();
    if (!driver->CRCerror) {
        Serial.printf("Chân phần cứng IOIN (0x%08X): ENN=%d (%s), DIR=%d, STEP=%d, MS1=%d, MS2=%d\n",
                      ioin,
                      ioin & 0x01, (ioin & 0x01) ? "BỊ TẮT BỞI CHÂN EN" : "ENABLED",
                      (ioin >> 9) & 0x01,
                      (ioin >> 7) & 0x01,
                      (ioin >> 2) & 0x01,
                      (ioin >> 3) & 0x01);
    }

    while (Serial1.available()) Serial1.read();
    const uint32_t drvStatus = driver->DRV_STATUS();
    const bool drvErr = driver->CRCerror;
    Serial.printf("DRV_STATUS: 0x%08X (CRCerror=%d)\n", drvStatus, drvErr);
    if (!drvErr) {
        const bool stst    = (drvStatus >> 31) & 0x01;
        const bool stealth = (drvStatus >> 30) & 0x01;
        const uint8_t cs   = (drvStatus >> 16) & 0x1F;
        const bool olb     = (drvStatus >> 7) & 0x01;
        const bool ola     = (drvStatus >> 6) & 0x01;
        const bool s2vsb   = (drvStatus >> 5) & 0x01;
        const bool s2vsa   = (drvStatus >> 4) & 0x01;
        const bool s2gb    = (drvStatus >> 3) & 0x01;
        const bool s2ga    = (drvStatus >> 2) & 0x01;
        const bool ot      = (drvStatus >> 1) & 0x01;
        const bool otpw    = drvStatus & 0x01;

        Serial.printf("  - Standstill (stst)       : %d\n", stst);
        Serial.printf("  - StealthChop active      : %d\n", stealth);
        Serial.printf("  - Current Scale (cs)      : %u / 31\n", cs);
        Serial.printf("  - Open Load A (ola)       : %d %s\n", ola, ola ? ">>> CẢNH BÁO: HỞ CUỘN A HOẶC MẤT NGUỒN 12V/24V! <<<" : "OK");
        Serial.printf("  - Open Load B (olb)       : %d %s\n", olb, olb ? ">>> CẢNH BÁO: HỞ CUỘN B! <<<" : "OK");
        Serial.printf("  - Short to VS Pha B(s2vsb): %d %s\n", s2vsb, s2vsb ? ">>> CẢNH BÁO: BỊ KHÓA BẢO VỆ NGẮN MẠCH / VM=0V! <<<" : "OK");
        Serial.printf("  - Short to VS Pha A(s2vsa): %d\n", s2vsa);
        Serial.printf("  - Short to GND (s2ga/b)   : %d / %d\n", s2ga, s2gb);
        Serial.printf("  - Overtemp (ot / otpw)    : %d / %d\n", ot, otpw);

        if (ola || s2vsb) {
            Serial.println("\n------------------------------------------------------------");
            Serial.println("[!] NGUYÊN NHÂN 'ĐỘNG CƠ KHÔNG CỨNG / KHÔNG CHẠY':");
            Serial.println("    1. NGUỒN 12V/24V (VM): Cần chắc chắn nguồn ngoài 12V/24V đã bật công tắc.");
            Serial.println("       (Nếu chỉ cắm cáp USB thì chip UART vẫn chạy nhưng motor không có điện).");
            Serial.println("    2. DÂY MOTOR J3: Kiểm tra giắc 4 chân cắm vào motor J3 có bị lỏng/tuột không.");
            Serial.println("    3. Gõ lệnh 'clear' để xóa chốt bảo vệ ngắn mạch (reset toff 0 -> 4).");
            Serial.println("    4. Gõ lệnh 'chop 1' để chuyển sang SpreadCycle thử lại.");
            Serial.println("------------------------------------------------------------\n");
        }
    }
    Serial.printf("Chế độ âm thanh: %s\n", useSpreadCycle ? "SpreadCycle (Khỏe/Ồn)" : "StealthChop (ÊM RU)");
    Serial.printf("Dòng cài đặt: %u mA | Lực giữ ihold: %u/31 | Vi bước: %u\n",
                  currentMa, currentHold, DEFAULT_MICROSTEPS);
    Serial.printf("Tốc độ đích: %u us/bước (~%u steps/s)\n",
                  targetIntervalUs, (targetIntervalUs > 0) ? (1000000 / targetIntervalUs) : 0);
    Serial.printf("Trạng thái chuyển động: %s (Đã phát tổng: %u bước)\n",
                  moving ? (continuous ? "ĐANG QUAY LIÊN TỤC" : "ĐANG CHẠY THEO BƯỚC") : "ĐỨNG YÊN",
                  totalStepsExecuted);
    Serial.println("--------------------------------------------------\n");
}

void initDriver(uint8_t addr, bool spread) {
    currentUartAddr = addr;
    useSpreadCycle = spread;

    Serial.printf("[TMC2209] Đang cấu hình driver tại địa chỉ 0b%c%c (addr %u), chế độ: %s, dòng: %u mA...\n",
                  (addr & 2) ? '1' : '0', (addr & 1) ? '1' : '0', addr,
                  spread ? "SpreadCycle" : "StealthChop (ÊM RU)", currentMa);

    driver = std::make_unique<TMC2209Stepper>(&Serial1, R_SENSE, addr);
    delay(20);

    driver->begin();
    driver->toff(4); // Bật cầu H driver
    driver->pdn_disable(true);
    driver->I_scale_analog(false);
    driver->mstep_reg_select(true);
    driver->rms_current(currentMa);
    driver->microsteps(DEFAULT_MICROSTEPS);

    if (spread) {
        driver->en_spreadCycle(true);
        driver->pwm_autoscale(false);
        driver->pwm_autograd(false);
    } else {
        // StealthChop
        driver->en_spreadCycle(false);
        driver->pwm_autoscale(true);
        driver->pwm_autograd(true);
    }

    driver->ihold(currentHold);
    driver->iholddelay(10);
    driver->shaft(false);

    while (Serial1.available()) Serial1.read();
    const uint8_t ver = driver->version();
    const bool err = driver->CRCerror;

    if (!err && ver == 0x21) {
        Serial.printf(">>> [TMC2209 OK] KẾT NỐI UART THÀNH CÔNG! Version=0x%02X, Dòng=%u mA, Vi bước=%u, Chế độ=%s <<<\n",
                      ver, currentMa, DEFAULT_MICROSTEPS, spread ? "SpreadCycle" : "StealthChop (ÊM RU)");
    } else {
        Serial.printf("\n[CẢNH BÁO LỖI UART] Địa chỉ 0b%c%c (addr %u) KHÔNG PHẢN HỒI (ver=0x%02X, CRCerror=%d)!\n",
                      (addr & 2) ? '1' : '0', (addr & 1) ? '1' : '0', addr, ver, err);
    }
}

void clearShortLatch() {
    Serial.println("\n[THỰC HIỆN] Xóa chốt bảo vệ ngắn mạch / reset cầu H (toff: 4 -> 0 -> 4)...");
    while (Serial1.available()) Serial1.read();
    driver->toff(0);
    delay(100);
    while (Serial1.available()) Serial1.read();
    driver->toff(4);
    delay(100);
    Serial.println("[OK] Đã gửi lệnh mở lại cầu H!");
    printStatus();
}

bool setDirectionViaUart(bool forward) {
    while (Serial1.available()) Serial1.read();
    driver->shaft(forward);
    currentShaftDir = forward;

    delay(2);
    while (Serial1.available()) Serial1.read();
    const bool verified = driver->shaft();
    const bool err = driver->CRCerror;

    if (err) {
        Serial.printf("\n[LỖI UART] Ghi shaft(%d) thất bại: Lỗi CRC/Timeout! Driver không nhận lệnh đổi chiều!\n", forward ? 1 : 0);
        return false;
    }
    if (verified != forward) {
        Serial.printf("\n[LỖI UART] Ghi shaft(%d) nhưng chip đọc lại là %d!\n", forward ? 1 : 0, verified ? 1 : 0);
        return false;
    }
    return true;
}

void startMotion(bool forward, int32_t steps = -1) {
    bool uartOk = setDirectionViaUart(forward);
    if (!uartOk) {
        Serial.println(">>> [CHÚ Ý] Lệnh đổi chiều qua UART KHÔNG THÀNH CÔNG!");
        Serial.println("    Motor vẫn sẽ phát xung bước nhưng CHỈ QUAY THEO 1 CHIỀU của chân DIR!");
    }

    if (steps > 0) {
        continuous = false;
        stepsRemaining = steps;
        stepCounter = 0;
        accelSteps = (steps < 600) ? (steps / 4) : 150;
        decelSteps = accelSteps;
        currentIntervalUs = 2500; // Khởi động êm ở 400 steps/s rồi tăng dần
        Serial.printf("[J3] Bắt đầu quay %s: %ld bước (Đích: %u us/bước, Gia tốc: %u bước)...\n",
                      forward ? "TIẾN (+)" : "LÙI (-)", static_cast<long>(steps), targetIntervalUs, accelSteps);
    } else {
        continuous = true;
        stepsRemaining = 0;
        stepCounter = 0;
        accelSteps = 150;
        decelSteps = 0;
        currentIntervalUs = 2500;
        Serial.printf("[J3] Bắt đầu quay %s LIÊN TỤC (Đích: %u us/bước). Bấm 's' hoặc phím Cách để dừng.\n",
                      forward ? "TIẾN (+)" : "LÙI (-)", targetIntervalUs);
    }
    lastStepMicros = micros();
    moving = true;
}

void stopMotion() {
    moving = false;
    continuous = false;
    stepsRemaining = 0;
    digitalWrite(J3_STEP_PIN, LOW);
    Serial.println("[J3] ĐÃ DỪNG!");
}

void handleCommand(String line) {
    line.trim();
    if (line.length() == 0) return;

    if (line.equalsIgnoreCase("s") || line.equalsIgnoreCase("x") || line.equalsIgnoreCase("stop") || line.equalsIgnoreCase("dung")) {
        stopMotion();
        return;
    }
    if (line.equalsIgnoreCase("h") || line.equalsIgnoreCase("help") || line == "?") {
        printHelp();
        return;
    }
    if (line.equalsIgnoreCase("status") || line.equalsIgnoreCase("info") || line.equalsIgnoreCase("i")) {
        printStatus();
        return;
    }
    if (line.equalsIgnoreCase("scan")) {
        scanUartAddresses();
        return;
    }
    if (line.equalsIgnoreCase("clear") || line.equalsIgnoreCase("reset")) {
        clearShortLatch();
        return;
    }
    if (line.startsWith("addr ") || line.startsWith("a ")) {
        int idx = line.indexOf(' ');
        int a = line.substring(idx + 1).toInt();
        if (a >= 0 && a <= 3) {
            initDriver(static_cast<uint8_t>(a), useSpreadCycle);
        } else {
            Serial.println("[LỖI] Địa chỉ UART hợp lệ: 0, 1, 2, 3 (tương ứng 0b00, 0b01, 0b10, 0b11)");
        }
        return;
    }
    if (line.startsWith("chop ")) {
        int idx = line.indexOf(' ');
        int m = line.substring(idx + 1).toInt();
        initDriver(currentUartAddr, m != 0);
        return;
    }
    if (line.startsWith("hold ")) {
        int idx = line.indexOf(' ');
        int h = line.substring(idx + 1).toInt();
        if (h >= 0 && h <= 31) {
            currentHold = static_cast<uint8_t>(h);
            driver->ihold(currentHold);
            Serial.printf("[J3] Đã cài đặt ihold: %u/31\n", currentHold);
        } else {
            Serial.println("[LỖI] hold hợp lệ: 0..31 (30 là cứng nhất)");
        }
        return;
    }
    if (line.startsWith("speed ") || line.startsWith("v ")) {
        int idx = line.indexOf(' ');
        long us = line.substring(idx + 1).toInt();
        if (us >= 100 && us <= 10000) {
            targetIntervalUs = static_cast<uint32_t>(us);
            Serial.printf("[J3] Đã đổi chu kỳ đích thành %u us (~%u steps/s)\n",
                          targetIntervalUs, 1000000 / targetIntervalUs);
        } else {
            Serial.println("[LỖI] Chu kỳ speed hợp lệ: 100..10000 us");
        }
        return;
    }
    if (line.startsWith("curr ") || line.startsWith("c ")) {
        int idx = line.indexOf(' ');
        long ma = line.substring(idx + 1).toInt();
        if (ma >= 200 && ma <= 2200) {
            currentMa = static_cast<uint16_t>(ma);
            driver->rms_current(currentMa);
            Serial.printf("[J3] Đã cài đặt dòng điện: %u mA\n", currentMa);
        } else {
            Serial.println("[LỖI] Dòng điện hợp lệ: 200..2200 mA");
        }
        return;
    }
    if (line.startsWith("shaft ")) {
        int idx = line.indexOf(' ');
        int s = line.substring(idx + 1).toInt();
        setDirectionViaUart(s != 0);
        return;
    }
    if (line.startsWith("pulse ")) {
        int idx = line.indexOf(' ');
        long n = line.substring(idx + 1).toInt();
        if (n <= 0) n = 100;
        Serial.printf("[J3] Phát %ld xung cứng trực tiếp trên GPIO %u...\n", n, J3_STEP_PIN);
        for (long i = 0; i < n; ++i) {
            digitalWrite(J3_STEP_PIN, HIGH);
            delayMicroseconds(4);
            digitalWrite(J3_STEP_PIN, LOW);
            delayMicroseconds(targetIntervalUs - 4);
        }
        Serial.println("[J3] Hoàn thành phát xung cứng.");
        return;
    }
    if (line.equalsIgnoreCase("c+") || line.equalsIgnoreCase("cont+") || line.equalsIgnoreCase("tien+")) {
        startMotion(false, -1);
        return;
    }
    if (line.equalsIgnoreCase("c-") || line.equalsIgnoreCase("cont-") || line.equalsIgnoreCase("lui-")) {
        startMotion(true, -1);
        return;
    }
    if (line.startsWith("+") || line.startsWith("f ") || line.equalsIgnoreCase("f") || line.startsWith("tien")) {
        int32_t steps = defaultStepBatch;
        int idx = line.indexOf(' ');
        if (idx > 0) {
            long s = line.substring(idx + 1).toInt();
            if (s > 0) steps = s;
        }
        startMotion(false, steps);
        return;
    }
    if (line.startsWith("-") || line.startsWith("r ") || line.equalsIgnoreCase("r") || line.startsWith("lui")) {
        int32_t steps = defaultStepBatch;
        int idx = line.indexOf(' ');
        if (idx > 0) {
            long s = line.substring(idx + 1).toInt();
            if (s > 0) steps = s;
        }
        startMotion(true, steps);
        return;
    }

    Serial.printf("[?] Không hiểu lệnh '%s'. Gõ 'help' hoặc '?' để xem hướng dẫn.\n", line.c_str());
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Cấu hình chân STEP của J3 làm OUTPUT và kéo xuống LOW
    pinMode(J3_STEP_PIN, OUTPUT);
    digitalWrite(J3_STEP_PIN, LOW);

    // Giữ các chân STEP của các trục khác ở LOW để chống nhiễu
    for (uint8_t pin : OTHER_STEP_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    // Khởi động cổng UART1 cho TMC2209
    Serial1.begin(TMC_UART_BAUD, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
    delay(50);

    printHelp();
    initDriver(currentUartAddr, useSpreadCycle);
    printStatus();
}

static String rxBuffer = "";

void loop() {
    // 1. Đọc và xử lý lệnh từ Serial
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        // Cho phép phím nóng dừng khẩn cấp lập tức bằng phím Space hoặc 's'/'x'
        if ((c == ' ' || c == 's' || c == 'x' || c == 'S' || c == 'X') && moving) {
            stopMotion();
            rxBuffer = "";
            continue;
        }
        if (c == '\n' || c == '\r') {
            if (rxBuffer.length() > 0) {
                handleCommand(rxBuffer);
                rxBuffer = "";
            }
        } else {
            rxBuffer += c;
        }
    }

    // 2. Phát xung bước cho J3 có gia tốc mềm (soft-ramp)
    if (moving) {
        const uint32_t now = micros();
        if (now - lastStepMicros >= currentIntervalUs) {
            lastStepMicros = now;

            // Xung bước sạch 4us
            digitalWrite(J3_STEP_PIN, HIGH);
            delayMicroseconds(4);
            digitalWrite(J3_STEP_PIN, LOW);

            totalStepsExecuted++;
            stepCounter++;

            if (!continuous) {
                stepsRemaining--;

                // Tăng tốc mềm (ramp up)
                if (stepCounter < accelSteps) {
                    float prog = static_cast<float>(stepCounter) / static_cast<float>(accelSteps);
                    currentIntervalUs = 2500 - static_cast<uint32_t>(prog * (2500 - targetIntervalUs));
                }
                // Giảm tốc mềm (ramp down)
                else if (static_cast<uint32_t>(stepsRemaining) < decelSteps) {
                    float prog = static_cast<float>(stepsRemaining) / static_cast<float>(decelSteps);
                    currentIntervalUs = 2500 - static_cast<uint32_t>(prog * (2500 - targetIntervalUs));
                } else {
                    currentIntervalUs = targetIntervalUs;
                }

                if (stepsRemaining <= 0) {
                    moving = false;
                    digitalWrite(J3_STEP_PIN, LOW);
                    Serial.println("[J3] Đã hoàn thành số bước chỉ định!");
                }
            } else {
                // Quay liên tục: tăng tốc mềm đến targetIntervalUs
                if (stepCounter < accelSteps) {
                    float prog = static_cast<float>(stepCounter) / static_cast<float>(accelSteps);
                    currentIntervalUs = 2500 - static_cast<uint32_t>(prog * (2500 - targetIntervalUs));
                } else {
                    currentIntervalUs = targetIntervalUs;
                }
            }
        }
    }
}
