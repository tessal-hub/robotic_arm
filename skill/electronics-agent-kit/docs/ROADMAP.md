# Electronics Agent Kit - Development Roadmap

> Realistic path from current state to production-ready AI electronics assistant

---

## Current State Assessment

### What We Have (v0.1.0)

```
electronics-agent-kit/
└── .agent/
    ├── agents/          # 5 agent definitions (documentation)
    ├── skills/          # 3 skill references (kicad-cli, file-format, platformio)
    └── workflows/       # 3 workflow definitions
```

**Reality check:** This is a **documentation kit**, not a **working integration**. The agents describe what SHOULD happen, but we lack:

1. **Protobuf API client** - No code to control running KiCad
2. **File manipulation code** - No actual S-expression parser/generator
3. **Schematic generation capability** - The hardest problem
4. **Model fine-tuning** - No training on electronics domain

### The Hard Problem: Schematic Generation

```
┌─────────────────────────────────────────────────────────────────┐
│                    THE SCHEMATIC CHALLENGE                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Why LLMs struggle with schematic generation:                   │
│                                                                  │
│  1. SPATIAL REASONING                                           │
│     - Components need visual placement (not just logical)       │
│     - Wires must be orthogonal, avoid crossings                 │
│     - Inputs on left, outputs on right (convention)             │
│     - Power rails at top/bottom                                  │
│                                                                  │
│  2. VISUAL CONVENTIONS                                          │
│     - Symbol orientation matters                                 │
│     - Text placement for readability                            │
│     - Hierarchical sheet organization                           │
│     - Net label positioning                                      │
│                                                                  │
│  3. DOMAIN COMPLEXITY                                           │
│     - Thousands of component symbols                            │
│     - Multiple footprint options per symbol                     │
│     - Pin numbering schemes vary by manufacturer                │
│     - Power pin connections are special                         │
│                                                                  │
│  Current LLM capability: ~20% success rate on complex schematics│
│  Required for production: >90% success rate                     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Strategic Architecture

### Three-Layer Approach

```
┌─────────────────────────────────────────────────────────────────┐
│                    TARGET ARCHITECTURE                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │              Layer 1: kicad-cli (Validation/Export)        │ │
│  │                                                            │ │
│  │  Status: READY NOW                                         │ │
│  │  • DRC/ERC validation                                      │ │
│  │  • Gerber, BOM, netlist export                            │ │
│  │  • PDF/SVG/3D export                                       │ │
│  │  • Format upgrades                                         │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │              Layer 2: Protobuf IPC API (Live Control)      │ │
│  │                                                            │ │
│  │  Status: PCB 80% ready, Schematic 20% ready                │ │
│  │  • PCB: Component placement, routing, zones, vias          │ │
│  │  • Schematic: Basic queries only (needs development)       │ │
│  │  • Requires KiCad 8.x running                              │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │              Layer 3: Schematic Generation (Research)       │ │
│  │                                                            │ │
│  │  Status: NOT READY - Requires R&D                          │ │
│  │  Option A: Fine-tune LLM on schematic dataset              │ │
│  │  Option B: Contribute to KiCad schematic API               │ │
│  │  Option C: Template-based assembly (hybrid)                │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Development Phases

### Phase 1: Protobuf API Integration (Weeks 1-4)

**Goal:** Working Protobuf client for PCB operations

| Task | Priority | Complexity |
|------|----------|------------|
| Implement Protobuf client (TypeScript/Python) | HIGH | Medium |
| PCB item CRUD operations | HIGH | Medium |
| Net and layer queries | HIGH | Low |
| Zone management | MEDIUM | Medium |
| Via placement | MEDIUM | Low |
| Live DRC feedback | MEDIUM | Medium |

**Deliverables:**
- `lib/kicad-api/` - Protobuf client library
- Working PCB manipulation in running KiCad
- Updated pcb-agent with real tool integration

**Technical details:**
```
Communication: Unix socket at /tmp/kicad/api.sock
Protocol: Protobuf (envelope.proto, editor_commands.proto, board_types.proto)
Requirements: KiCad 8.x running
```

### Phase 2: Schematic Reading & Analysis (Weeks 5-8)

**Goal:** Parse and understand existing schematics

| Task | Priority | Complexity |
|------|----------|------------|
| S-expression parser for .kicad_sch | HIGH | Medium |
| Component extraction | HIGH | Low |
| Net connectivity analysis | HIGH | Medium |
| Hierarchical sheet handling | MEDIUM | High |
| Pin mapping to firmware | HIGH | Medium |

**Deliverables:**
- `lib/kicad-parser/` - S-expression parser
- Schematic-to-firmware pin mapping
- Component dependency graph
- Updated schematic-agent for reading (not writing)

**Key insight:** We can READ schematics reliably, even if we can't GENERATE them well yet.

### Phase 3: Template-Based Schematic Assembly (Weeks 9-12)

**Goal:** Assemble schematics from pre-made blocks

| Task | Priority | Complexity |
|------|----------|------------|
| Create subcircuit template library | HIGH | Medium |
| Template parameter system | HIGH | Medium |
| Placement constraint engine | MEDIUM | High |
| Inter-template connection logic | HIGH | High |
| Visual layout optimizer | MEDIUM | High |

**Template Library (Initial):**
```
templates/
├── mcu/
│   ├── esp32-s3-minimal.kicad_sch
│   ├── esp32-c3-minimal.kicad_sch
│   ├── stm32g4-minimal.kicad_sch
│   └── rp2040-minimal.kicad_sch
├── power/
│   ├── usb-c-pd.kicad_sch
│   ├── ldo-3v3.kicad_sch
│   ├── buck-5v.kicad_sch
│   └── battery-charger.kicad_sch
├── interface/
│   ├── i2c-pullups.kicad_sch
│   ├── spi-connector.kicad_sch
│   ├── uart-level-shift.kicad_sch
│   └── can-transceiver.kicad_sch
├── sensors/
│   ├── temperature-ds18b20.kicad_sch
│   ├── imu-mpu6050.kicad_sch
│   └── pressure-bmp280.kicad_sch
└── display/
    ├── oled-ssd1306.kicad_sch
    └── lcd-st7789.kicad_sch
```

**Key insight:** AI selects and connects templates, rather than generating components from scratch.

### Phase 4: Schematic API Contribution (Months 4-6)

**Goal:** Improve KiCad's schematic Protobuf API

| Task | Priority | Complexity |
|------|----------|------------|
| Study KiCad eeschema codebase | HIGH | High |
| Add symbol CRUD to API | HIGH | High |
| Add wire/connection operations | HIGH | High |
| Add label/net operations | MEDIUM | Medium |
| Submit PRs to KiCad | HIGH | Political |

**Required skills:**
- C++ (KiCad is C++)
- wxWidgets (UI framework)
- Protobuf (API definition)
- KiCad architecture understanding

**Files to modify in KiCad:**
```
eeschema/api/api_handler_sch.cpp    # Add command handlers
api/proto/schematic/schematic_commands.proto  # Define new commands
api/proto/schematic/schematic_types.proto     # Define data types
```

**Timeline reality:** Getting code merged into KiCad takes time:
- Initial PR: 2-4 weeks of development
- Review cycle: 2-8 weeks
- Revisions: 1-3 iterations
- Total: 2-4 months per significant feature

### Phase 5: Model Fine-Tuning Research (Months 6-12)

**Goal:** Explore AI-native schematic generation

| Approach | Pros | Cons |
|----------|------|------|
| Fine-tune on schematic files | Direct generation | Needs massive dataset |
| Train visual layout model | Better spatial reasoning | Complex architecture |
| Reinforcement learning | Learns from ERC feedback | Slow training |
| Multimodal (vision + text) | Understands schematic images | Experimental |

**Data requirements:**
- 10,000+ complete KiCad projects (open source)
- Labeled component relationships
- Quality ratings for layouts
- ERC pass/fail status

**Potential data sources:**
- GitHub KiCad projects
- KiCad community libraries
- Open hardware repositories (OSHWA)
- Academic EDA datasets

---

## Risk Assessment

### Technical Risks

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Schematic API contribution rejected | Medium | High | Build relationship with maintainers first |
| Fine-tuning doesn't converge | High | High | Template approach as fallback |
| Protobuf API changes in KiCad 9 | Low | Medium | Abstract API layer |
| Template library insufficient | Medium | Medium | Community contributions |

### Strategic Risks

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Competitor launches first | Medium | Medium | Focus on depth, not breadth |
| KiCad moves away from Protobuf | Low | High | File manipulation fallback |
| No market demand | Low | High | Validate with EE community early |

---

## Success Metrics

### Phase 1-2 (Foundation)

| Metric | Target |
|--------|--------|
| PCB operations via API | 90% success rate |
| Schematic parsing accuracy | 99% (reading) |
| Pin mapping accuracy | 95% |

### Phase 3 (Templates)

| Metric | Target |
|--------|--------|
| Template coverage | 50+ subcircuits |
| Assembly success rate | 80% |
| ERC pass rate | 90% (after generation) |

### Phase 4-5 (Advanced)

| Metric | Target |
|--------|--------|
| KiCad PRs merged | 3+ |
| Schematic generation success | 70%+ |
| User satisfaction | 4/5 stars |

---

## Resource Requirements

### Development Team

| Role | Time | Skills |
|------|------|--------|
| Core developer | Full-time | TypeScript, Python, Protobuf |
| KiCad contributor | Part-time | C++, wxWidgets |
| ML engineer | Part-time | Fine-tuning, RLHF |
| Electronics engineer | Advisory | Circuit design, validation |

### Infrastructure

| Resource | Purpose | Cost |
|----------|---------|------|
| GPU compute | Model fine-tuning | $500-2000/month |
| KiCad build environment | API development | Minimal |
| Test PCB fabrication | Validation | $100-500/month |

---

## Open Questions

1. **KiCad vs Altium/Eagle:** Should we also support commercial EDA tools?
2. **Cloud vs Local:** Should schematic generation happen in cloud (better models)?
3. **Monetization:** Open core, SaaS, or fully open source?
4. **Community:** Fork KiCad for faster iteration, or work within upstream?

---

## Next Immediate Actions

1. [ ] Implement Protobuf client for PCB operations
2. [ ] Create S-expression parser for schematic reading
3. [ ] Build initial template library (10 subcircuits)
4. [ ] Join KiCad developer mailing list
5. [ ] Collect open source KiCad projects for training data
6. [ ] Update ARCHITECTURE.md with this roadmap

---

*Document created: 2025-02-03*
*Status: Strategic Planning*
*Authors: AI + Human collaboration*
