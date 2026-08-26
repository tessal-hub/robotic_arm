# Electronics Agent Kit

AI-powered agents for electronics engineering: schematics, PCB layout, firmware, and manufacturing.

> Bring Antigravity-style agentic development to electronics engineering with KiCad and PlatformIO.

**Status:** v0.1.0 - Early development. See [ROADMAP](docs/ROADMAP.md) for development plans.

## Overview

The Electronics Agent Kit provides specialized AI agents, skills, and workflows for:

- **Schematic Design** - Circuit capture with KiCad
- **PCB Layout** - Board design, routing, and DRC
- **Firmware Development** - Embedded code with PlatformIO
- **Manufacturing** - Gerber generation, BOM, assembly files

## Quick Start

### 1. Clone the Kit

```bash
git clone https://github.com/o2scale/electronics-agent-kit.git
cd electronics-agent-kit
```

### 2. Copy to Your Project

Copy the `.agent` folder to your electronics project:

```bash
cp -r .agent /path/to/your/project/
```

### 3. Use with AI Coding Assistant

The agents will be available in AI coding assistants that support agent kits (OpenCode, Antigravity, etc.):

```
User: Create a new ESP32 temperature sensor project

Agent: I'll use schematic-agent to design the circuit and 
       firmware-agent to set up the PlatformIO project...
```

## What's Included

### Agents

| Agent | Purpose |
|-------|---------|
| `schematic-agent` | KiCad schematic design, component selection, ERC |
| `pcb-agent` | PCB layout, routing, DRC, Gerber export |
| `firmware-agent` | PlatformIO, Arduino, ESP-IDF, STM32 |
| `verification-agent` | DRC/ERC validation, quality assurance |
| `bom-agent` | Bill of Materials, JLCPCB/LCSC optimization |

### Skills

| Skill | Purpose |
|-------|---------|
| `kicad-cli` | Complete kicad-cli command reference |
| `kicad-file-format` | S-expression file format for .kicad_sch/.kicad_pcb |
| `platformio` | PlatformIO project setup, build, and debugging |

### Workflows

| Workflow | Purpose |
|----------|---------|
| `/new-project` | Create new electronics project with KiCad + PlatformIO |
| `/verify` | Run DRC/ERC validation on designs |
| `/manufacture` | Generate Gerbers, BOM, and assembly files |

## Architecture

```
electronics-agent-kit/
└── .agent/
    ├── ARCHITECTURE.md           # Design philosophy and overview
    ├── agents/
    │   ├── schematic-agent.md
    │   ├── pcb-agent.md
    │   ├── firmware-agent.md
    │   ├── verification-agent.md
    │   └── bom-agent.md
    ├── skills/
    │   ├── kicad-cli/SKILL.md
    │   ├── kicad-file-format/SKILL.md
    │   └── platformio/SKILL.md
    └── workflows/
        ├── new-project.md
        ├── verify.md
        └── manufacture.md
```

## Current Status & Limitations

### What Works Now
- **Verification** - DRC/ERC validation via kicad-cli
- **Manufacturing** - Gerber, BOM, CPL generation
- **Firmware** - PlatformIO project setup and development
- **File Reading** - Parse and analyze existing schematics/PCBs

### Known Limitations
- **Schematic Generation** - LLMs struggle with spatial/visual reasoning required for component placement and wire routing. We're developing a template-based approach.
- **Complex PCB Modifications** - Direct file manipulation works for simple changes but risks corruption on complex boards.

### Future Plans
See [ROADMAP.md](docs/ROADMAP.md) for our development roadmap, including:
- Protobuf API integration for safer PCB operations
- Template-based schematic assembly
- Potential KiCad upstream contributions

## Design Philosophy

### Three-Layer Architecture

| Layer | Status | Use Case |
|-------|--------|----------|
| **kicad-cli** | Production | DRC/ERC, Gerber export, BOM, netlist, PDF |
| **Protobuf API** | Experimental | PCB operations (80% coverage), schematic queries (20%) |
| **File manipulation** | Careful | Reading files, simple modifications |

We prioritize safe, reliable operations over experimental capabilities.

### JLCPCB Optimized

The BOM agent understands JLCPCB's assembly service:
- Basic parts (free assembly) vs Extended parts
- LCSC part number integration
- Proper BOM and CPL formatting

## Example Usage

### Create a New Project

```
User: Create a new ESP32-S3 project with USB-C and an I2C sensor

Agent: I'll create the project structure and initialize both hardware and firmware...
       
       [Creates KiCad project with schematic template]
       [Sets up PlatformIO with ESP32-S3 configuration]
       [Adds USB-C connector and I2C interface to schematic]
```

### Run Verification

```
User: Check my PCB for errors

Agent: Running DRC on your PCB...

       === DRC Results ===
       Errors:   2
       Warnings: 3
       
       [ERROR] Clearance violation at (45.2, 67.8): 0.08mm < 0.15mm
       [ERROR] Unconnected net: VCC on U1.pin3
       
       Would you like me to fix these issues?
```

### Generate Manufacturing Files

```
User: Generate files for JLCPCB

Agent: Generating manufacturing files...

       Created:
       - output/gerbers.zip (ready for upload)
       - output/bom/jlcpcb_bom.csv
       - output/assembly/jlcpcb_cpl.csv
       
       Board: 50x40mm, 2-layer
       Components: 23 (18 basic, 5 extended)
       Estimated cost: ~$15 for 5 boards + assembly
```

## Requirements

### For Hardware Design
- [KiCad 8.x](https://www.kicad.org/) - Schematic and PCB editor
- `kicad-cli` - Command-line interface (included with KiCad)

### For Firmware Development
- [PlatformIO](https://platformio.org/) - Embedded development platform
- Python 3.x (for PlatformIO)

### Install PlatformIO

```bash
pip install platformio
```

## Supported Platforms

### Hardware (KiCad)
- Any KiCad 8.x project
- 2-layer and 4-layer boards
- JLCPCB/PCBWay manufacturing

### Firmware (PlatformIO)
- **ESP32** - ESP32, ESP32-S3, ESP32-C3, ESP32-C6
- **STM32** - Nucleo, Blue Pill, custom boards
- **RP2040** - Raspberry Pi Pico, Pico W
- **AVR** - Arduino Uno, Nano, Mega

## Contributing

Contributions welcome! Areas to expand:

- [ ] Simulation agent (SPICE/ngspice)
- [ ] Symbol/footprint library management
- [ ] More workflow templates
- [ ] Additional MCU platform support

## License

MIT License - see [LICENSE](LICENSE) for details.

## Related Projects

- [KiCad](https://www.kicad.org/) - Open source EDA
- [PlatformIO](https://platformio.org/) - Embedded development platform
- [OpenCode](https://opencode.ai/) - AI coding assistant
- [Antigravity Kit](https://github.com/anthropics/antigravity-kit) - Agent kit template

---

Built with AI for electronics engineers.
