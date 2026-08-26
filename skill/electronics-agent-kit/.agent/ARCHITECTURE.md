# Electronics Agent Kit Architecture

> AI-powered agents for electronics engineering: schematics, PCB layout, firmware, and manufacturing.

## Overview

The Electronics Agent Kit brings Antigravity-style agentic development to electronics engineering. It provides specialized agents, skills, and workflows for KiCad-based hardware design and PlatformIO-based firmware development.

---

## Current Capabilities (Honest Assessment)

### What Works NOW

| Capability | Status | How |
|------------|--------|-----|
| DRC/ERC validation | READY | kicad-cli |
| Gerber/BOM/netlist export | READY | kicad-cli |
| Read existing schematics | READY | File parsing |
| Read existing PCBs | READY | File parsing |
| Firmware development | READY | PlatformIO |
| Component selection advice | READY | Agent knowledge |

### What Needs Development

| Capability | Status | Blocker |
|------------|--------|---------|
| PCB modification via API | IN PROGRESS | Protobuf API (80% ready) |
| Schematic modification via API | BLOCKED | Protobuf API (20% ready) |
| Schematic generation from scratch | RESEARCH | LLM spatial reasoning limits |
| Live KiCad sync | IN PROGRESS | Protobuf client needed |

### The Hard Problem: Schematic Generation

```
┌─────────────────────────────────────────────────────────────────┐
│                 WHY SCHEMATIC GENERATION IS HARD                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  LLMs struggle with spatial/visual reasoning:                   │
│                                                                  │
│  1. Component placement needs visual conventions                 │
│     - Inputs on left, outputs on right                          │
│     - Power rails at top/bottom                                  │
│     - Logical grouping of related components                    │
│                                                                  │
│  2. Wires need orthogonal routing                               │
│     - 90-degree angles only                                      │
│     - Avoid crossings where possible                            │
│     - Junctions must be explicit                                │
│                                                                  │
│  3. Thousands of component symbols                              │
│     - Each with unique pin arrangements                          │
│     - Multiple footprint options                                 │
│     - Orientation matters                                        │
│                                                                  │
│  Current approach options:                                       │
│  A) Fine-tune LLM on 10,000+ schematic examples                 │
│  B) Contribute to KiCad schematic API (upstream)                │
│  C) Template-based assembly (hybrid approach)                   │
│                                                                  │
│  See: docs/ROADMAP.md for development plan                      │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Integration Architecture

### Three-Layer Approach

```
┌─────────────────────────────────────────────────────────────────┐
│                    Electronics Agent Kit                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │           Layer 1: kicad-cli (Validation/Export)           │ │
│  │                                                            │ │
│  │  Status: PRODUCTION READY                                  │ │
│  │  • DRC/ERC validation                                      │ │
│  │  • Export: Gerber, BOM, Netlist, PDF, 3D                  │ │
│  │  • Format upgrades                                         │ │
│  │  • Automation/CI workflows                                 │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │           Layer 2: Protobuf IPC API (Live Control)         │ │
│  │                                                            │ │
│  │  Status: PCB 80% ready, Schematic 20% ready                │ │
│  │  • PCB: Item CRUD, zones, vias, nets ✓                    │ │
│  │  • Schematic: Queries only, no modifications ✗            │ │
│  │  • Requires: KiCad 8.x running                            │ │
│  │  • Socket: /tmp/kicad/api.sock                            │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │           Layer 3: File Manipulation (Offline)             │ │
│  │                                                            │ │
│  │  Status: READING works, WRITING is experimental           │ │
│  │  • Parse .kicad_sch / .kicad_pcb (S-expression)           │ │
│  │  • Works offline, no KiCad needed                         │ │
│  │  • Best for: Analysis, extraction, simple patches         │ │
│  │  • Risky for: Complex generation from scratch             │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Design Philosophy

### Agent Specialization

Each agent focuses on one domain with deep expertise:

```
schematic-agent  →  Circuit design, component selection, ERC
pcb-agent        →  Board layout, routing, DRC, manufacturing
firmware-agent   →  Embedded code, PlatformIO, RTOS
verification-agent → DRC/ERC validation, quality assurance
bom-agent        →  Component sourcing, cost optimization
```

---

## Directory Structure

```
electronics-agent-kit/
├── .agent/
│   ├── agents/
│   │   ├── schematic-agent.md      # KiCad schematic operations
│   │   ├── pcb-agent.md            # PCB layout and routing
│   │   ├── firmware-agent.md       # PlatformIO/Arduino/ESP-IDF
│   │   ├── verification-agent.md   # DRC/ERC validation
│   │   └── bom-agent.md            # Bill of Materials management
│   │
│   ├── skills/
│   │   ├── kicad-cli/
│   │   │   └── SKILL.md            # kicad-cli command reference
│   │   ├── kicad-file-format/
│   │   │   └── SKILL.md            # S-expression file format
│   │   └── platformio/
│   │       └── SKILL.md            # PlatformIO reference
│   │
│   ├── workflows/
│   │   ├── new-project.md          # Create new electronics project
│   │   ├── verify.md               # Run DRC/ERC validation
│   │   └── manufacture.md          # Generate manufacturing files
│   │
│   └── ARCHITECTURE.md             # This file
```

---

## Agents

### schematic-agent

**Purpose:** Circuit design and schematic capture in KiCad.

**Capabilities:**
- Create and modify schematics
- Component selection and specification
- Power distribution design
- Net naming and organization
- ERC validation

**Skills:** kicad-cli, kicad-file-format, electronics-fundamentals

### pcb-agent

**Purpose:** PCB layout and routing in KiCad.

**Capabilities:**
- Component placement
- Trace routing and optimization
- Design rule configuration
- Power plane management
- Gerber generation

**Skills:** kicad-cli, kicad-file-format, pcb-design-rules

### firmware-agent

**Purpose:** Embedded firmware development with PlatformIO.

**Capabilities:**
- Project setup and configuration
- Peripheral driver development
- RTOS task management
- Communication protocols
- Build and upload

**Skills:** platformio, embedded-c, rtos-patterns

### verification-agent

**Purpose:** Design validation and quality assurance.

**Capabilities:**
- ERC (Electrical Rules Check)
- DRC (Design Rules Check)
- Manufacturing validation
- Systematic design review

**Skills:** kicad-cli, design-rules

### bom-agent

**Purpose:** Bill of Materials and component management.

**Capabilities:**
- BOM generation and formatting
- Component selection
- JLCPCB/LCSC optimization
- Cost analysis
- Supply chain management

**Skills:** kicad-cli, component-sourcing

---

## Skills

### kicad-cli

Complete reference for the KiCad command-line interface:
- Schematic export (netlist, BOM, PDF, SVG)
- PCB export (Gerber, drill, position, 3D)
- DRC/ERC validation
- Automation scripts

### kicad-file-format

S-expression file format reference:
- .kicad_sch (schematic) format
- .kicad_pcb (PCB) format
- Reading and writing files
- UUID generation
- Coordinate systems

### platformio

PlatformIO embedded development reference:
- Project configuration
- Board/platform selection
- Library management
- Build and upload
- Testing and debugging

---

## Workflows

### /new-project

Creates a new electronics project with:
- KiCad project files (schematic, PCB)
- Optional PlatformIO firmware project
- Proper directory structure
- Git initialization

### /verify

Runs comprehensive validation:
- ERC on schematic
- DRC on PCB
- Error categorization and fixes
- Pre-manufacturing checklist

### /manufacture

Generates manufacturing files:
- Gerber files (zipped)
- Drill files
- BOM (JLCPCB format)
- Component placement (CPL)
- 3D model export

---

## Integration Points

### KiCad Integration

```
┌─────────────────┐
│  Agent/Workflow │
└────────┬────────┘
         │
         ├─── kicad-cli (validation, export)
         │    └── DRC, ERC, Gerber, BOM, netlist
         │
         └─── File I/O (modification)
              └── .kicad_sch, .kicad_pcb (S-expression)
```

### PlatformIO Integration

```
┌─────────────────┐
│  firmware-agent │
└────────┬────────┘
         │
         └─── pio CLI
              ├── pio run (build)
              ├── pio run -t upload (flash)
              ├── pio device monitor (serial)
              ├── pio test (unit tests)
              └── pio pkg (libraries)
```

---

## Usage

### With OpenCode/Antigravity

This kit is designed to be loaded as an agent kit in AI coding assistants:

```
project/
├── .agent/                    # Electronics Agent Kit
│   ├── agents/
│   ├── skills/
│   └── workflows/
├── hardware/
│   ├── project.kicad_pro
│   ├── project.kicad_sch
│   └── project.kicad_pcb
└── firmware/
    ├── platformio.ini
    └── src/main.cpp
```

### Example Interactions

```
User: Create a new ESP32 sensor project with temperature and humidity monitoring

Agent: I'll use schematic-agent to design the circuit and firmware-agent 
       to set up the PlatformIO project...

User: Run DRC on my PCB

Agent: I'll use verification-agent to run the design rules check...
       [Runs kicad-cli pcb drc]
       Found 2 errors and 3 warnings. Here are the issues...

User: Generate manufacturing files for JLCPCB

Agent: I'll use the manufacture workflow to generate all required files...
       [Generates Gerbers, BOM, CPL]
       Files ready in output/ directory.
```

---

## Future Roadmap

### Phase 2: Enhanced Integration
- [ ] KiCad protobuf API support (when stable)
- [ ] Simulation agent (SPICE/ngspice)
- [ ] Symbol/footprint library management
- [ ] Design template library

### Phase 3: Advanced Features
- [ ] Auto-routing assistance
- [ ] EMC/signal integrity analysis
- [ ] 3D mechanical integration
- [ ] Version control for hardware

---

## References

- KiCad Documentation: https://docs.kicad.org/
- KiCad CLI Reference: https://docs.kicad.org/master/en/cli/
- PlatformIO Documentation: https://docs.platformio.org/
- KiCad File Formats: https://dev-docs.kicad.org/en/file-formats/

---

## Version

- Kit Version: 0.1.0
- KiCad Target: 8.x
- PlatformIO Target: 6.x
