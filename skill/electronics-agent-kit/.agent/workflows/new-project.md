---
description: Create a new electronics project with KiCad and optional firmware. Sets up directory structure, initializes KiCad project, and optionally creates PlatformIO firmware project.
---

# /new-project - Create Electronics Project

$ARGUMENTS

---

## Task

This command creates a new electronics project with proper structure.

### Steps:

1. **Gather Requirements**
   - Project name and description
   - Include firmware? (PlatformIO)
   - MCU platform (if firmware)
   - Project type (prototype, production)

2. **Create Directory Structure**

   ```
   project-name/
   ├── hardware/
   │   ├── project-name.kicad_pro
   │   ├── project-name.kicad_sch
   │   ├── project-name.kicad_pcb
   │   ├── symbols/           # Project-specific symbols
   │   ├── footprints/        # Project-specific footprints
   │   └── 3dmodels/          # 3D models
   ├── firmware/              # If firmware requested
   │   ├── platformio.ini
   │   ├── src/
   │   │   └── main.cpp
   │   ├── include/
   │   ├── lib/
   │   └── test/
   ├── docs/
   │   └── README.md
   ├── output/                # Manufacturing outputs
   │   ├── gerbers/
   │   ├── bom/
   │   └── assembly/
   └── .gitignore
   ```

3. **Initialize KiCad Project**
   - Create minimal .kicad_pro file
   - Create empty schematic with title block
   - Create empty PCB with design rules

4. **Initialize Firmware (Optional)**
   - Create platformio.ini for selected platform
   - Create main.cpp template
   - Add common libraries

5. **Create Documentation**
   - Project README with structure overview
   - Add .gitignore for build artifacts

---

## Usage Examples

```
/new-project blink-led
/new-project sensor-hub --firmware esp32
/new-project power-supply --no-firmware
/new-project usb-device --firmware stm32
```

---

## Templates

### KiCad Project File (.kicad_pro)

```json
{
  "meta": {
    "filename": "project.kicad_pro",
    "version": 1
  },
  "project": {
    "name": "project-name"
  }
}
```

### Minimal Schematic (.kicad_sch)

```lisp
(kicad_sch
  (version 20231120)
  (generator "eeschema")
  (generator_version "8.0")
  (uuid "...")
  (paper "A4")
  (title_block
    (title "Project Name")
    (date "YYYY-MM-DD")
    (rev "0.1")
  )
  (lib_symbols)
)
```

### Minimal PCB (.kicad_pcb)

```lisp
(kicad_pcb
  (version 20231014)
  (generator "pcbnew")
  (generator_version "8.0")
  (general
    (thickness 1.6)
  )
  (paper "A4")
  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (32 "B.Adhes" user "B.Adhesive")
    ...
  )
  (setup
    (pad_to_mask_clearance 0.1)
  )
)
```

---

## Agents Used

- `schematic-agent` - For schematic template setup
- `pcb-agent` - For PCB template setup  
- `firmware-agent` - For PlatformIO initialization (if requested)
