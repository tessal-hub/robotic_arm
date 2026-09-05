#include <Arduino.h>

// Pin map from src/config.h. This firmware deliberately does not use arm code.
constexpr uint8_t J5_STEP_PIN = 38;
constexpr uint8_t J5_DIR_PIN = 39;
constexpr uint8_t J6_STEP_PIN = 40;
constexpr uint8_t J6_DIR_PIN = 47;

constexpr uint16_t TEST_STEPS = 200;
constexpr uint16_t STEP_PERIOD_US = 4000;  // 250 steps/s: slow, easy to observe.
constexpr uint16_t STEP_HIGH_US = 4;       // A4988 requires >= 1 us.

void printHelp() {
  Serial.println("\nA4988 individual test");
  Serial.println("  5+ / 5- : J5, 200 steps");
  Serial.println("  6+ / 6- : J6, 200 steps");
  Serial.println("  h       : help");
  Serial.println("Each command moves one module only.");
}

void runMotor(const char *name, uint8_t stepPin, uint8_t dirPin, bool forward) {
  Serial.printf("[TEST] %s %c: %u pulses at %u steps/s\n",
                name, forward ? '+' : '-', static_cast<unsigned>(TEST_STEPS),
                static_cast<unsigned>(1000000UL / STEP_PERIOD_US));
  digitalWrite(dirPin, forward ? HIGH : LOW);
  delayMicroseconds(10);

  for (uint16_t step = 0; step < TEST_STEPS; ++step) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_PERIOD_US - STEP_HIGH_US);
  }

  Serial.printf("[TEST] %s done\n", name);
}

void setup() {
  Serial.begin(115200);
  pinMode(J5_STEP_PIN, OUTPUT);
  pinMode(J5_DIR_PIN, OUTPUT);
  pinMode(J6_STEP_PIN, OUTPUT);
  pinMode(J6_DIR_PIN, OUTPUT);
  digitalWrite(J5_STEP_PIN, LOW);
  digitalWrite(J5_DIR_PIN, LOW);
  digitalWrite(J6_STEP_PIN, LOW);
  digitalWrite(J6_DIR_PIN, LOW);

  delay(300);
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  const char joint = static_cast<char>(Serial.read());
  if (joint == 'h' || joint == 'H') {
    printHelp();
    return;
  }
  if (joint != '5' && joint != '6') return;

  while (!Serial.available()) delay(1);
  const char direction = static_cast<char>(Serial.read());
  if (direction != '+' && direction != '-') {
    Serial.println("Use 5+, 5-, 6+, or 6-.");
    return;
  }

  const bool forward = direction == '+';
  if (joint == '5') runMotor("J5", J5_STEP_PIN, J5_DIR_PIN, forward);
  else runMotor("J6", J6_STEP_PIN, J6_DIR_PIN, forward);
}
