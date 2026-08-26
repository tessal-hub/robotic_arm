# embedded-agent-skills

Claude Code Agent Skills for embedded systems development.

## gpio-config

An agent skill that handles GPIO pin assignment and configuration for embedded platforms. Given a project description, it:

1. Detects the target platform and variant
2. Looks up device requirements from a 28-device profile database
3. Assigns GPIO pins while avoiding conflicts (flash pins, strapping pins, peripheral groups)
4. Validates the full assignment against electrical and protocol constraints
5. Generates platform-specific config files (`config.txt` for RPi, `sdkconfig` comments for ESP32)
6. Generates initialization code in the appropriate framework
7. Provides wiring notes with safety warnings

### Supported Platforms

| Platform | Variants | Frameworks |
|---|---|---|
| Raspberry Pi | Pi 3, Pi 4, Pi 5, Zero 2W | gpiozero, RPi.GPIO |
| ESP32 | ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6 | Arduino, ESP-IDF |

### Features

- **28 device profiles** covering sensors, displays, communication modules, and motor controllers
- **20 RPi device tree overlay definitions** with conflict matrix
- **Safety warnings** for strapping pins, flash voltage pins (GPIO12/GPIO45), and boot-mode implications
- **I2C address collision detection** across 13 common address ranges
- **Cross-platform support** with platform-appropriate pin numbers, function names, and code patterns
- **Variant-aware validation** — ESP32-S3 allows GPIO6-11 (blocked on base ESP32), different strapping pin sets per variant

### Protocols

I2C, SPI, UART, PWM, 1-Wire, ADC, CAN

### Installation

```bash
git clone https://github.com/your-org/embedded-agent-skills.git
cd embedded-agent-skills
```

### Usage

Example prompts for Claude Code with this skill installed:

```
"Set up a BME280 temperature sensor and SSD1306 OLED on a Raspberry Pi 4"
```

```
"Configure an ESP32-S3 with SPI LoRa, I2C OLED, and UART GPS"
```

```
"I want to use GPIO12 as a button on an ESP32 — is that safe?"
```
> The skill will warn that GPIO12 is a strapping pin (MTDI) that sets flash voltage.
> If pulled HIGH at boot, it selects 1.8V which can damage 3.3V flash chips.

```
"Add a CAN bus with MCP2515 on SPI0 to my Pi 4 project"
```

### Testing

```bash
cd embedded-agent-skills/gpio-config
python3 -m venv .venv
source .venv/bin/activate
pip install pytest
pytest tests/ -v
```

Expected: 94 tests passed across 5 modules.

### Repository Structure

```
embedded-agent-skills/
├── .gitattributes
├── .gitignore
├── LICENSE
├── README.md
└── embedded-agent-skills/
    └── gpio-config/
        ├── SKILL.md
        ├── assets/
        ├── references/
        │   ├── common-devices.md
        │   ├── electrical-constraints.md
        │   ├── protocol-quick-ref.md
        │   └── platforms/
        │       ├── esp32-pins.md
        │       ├── esp32-specifics.md
        │       ├── rpi-overlays.md
        │       └── rpi-pins.md
        ├── scripts/
        │   ├── generate_config.py
        │   ├── validate_pinmap.py
        │   └── platforms/
        │       ├── __init__.py
        │       ├── base.py
        │       ├── esp32.py
        │       └── rpi.py
        └── tests/
            ├── __init__.py
            ├── conftest.py
            ├── test_rpi_scenarios.py
            ├── test_esp32_scenarios.py
            ├── test_cross_platform.py
            ├── test_regression.py
            ├── test_syntax_validation.py
            └── fixtures/
                ├── s1_rpi_i2c_spi.json
                ├── s2_rpi_complex.json
                ├── s3_rpi_can.json
                ├── s4_rpi_uart_onewire.json
                ├── s5a_rpi_duplicate_conflict.json
                ├── s5b_rpi_shared_i2c.json
                ├── s6_esp32_i2c_spi.json
                ├── s7a_esp32_adc1_wifi.json
                ├── s7b_esp32_adc2_wifi.json
                ├── s8a_esp32_rtc_gpio.json
                ├── s8b_esp32_non_rtc.json
                ├── s9_esp32s3_multi.json
                ├── s10_esp32_strapping.json
                ├── s11a_rpi_bmesd.json
                ├── s11b_esp32_bmesd.json
                ├── regression_rpi_basic.json
                ├── regression_esp32_basic.json
                ├── regression_esp32_flash_pin.json
                ├── regression_esp32s3_flash_pin.json
                └── regression_esp32s3_strapping.json
```

### Contributing

Areas for contribution:

- Additional device profiles (sensors, actuators, communication modules)
- New platform support (STM32, RP2040, nRF52)
- ESP-IDF code generation improvements
- CAN bus code generation

### License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
