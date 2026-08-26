---
description: Generate manufacturing files for PCB fabrication and assembly. Creates Gerbers, drill files, BOM, and position files for JLCPCB/PCBWay.
---

# /manufacture - Generate Manufacturing Files

$ARGUMENTS

---

## Task

This command generates all files needed for PCB manufacturing and assembly.

### Steps:

1. **Pre-flight Check**
   - Run ERC verification
   - Run DRC verification
   - Abort if errors found (unless --force)

2. **Generate Gerber Files**
   ```bash
   kicad-cli pcb export gerbers \
     --output output/gerbers/ \
     --layers "F.Cu,B.Cu,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts" \
     project.kicad_pcb
   ```

3. **Generate Drill Files**
   ```bash
   kicad-cli pcb export drill \
     --output output/gerbers/ \
     --format excellon \
     --excellon-units mm \
     project.kicad_pcb
   ```

4. **Generate BOM**
   ```bash
   kicad-cli sch export bom \
     --output output/bom/bom.csv \
     --fields "Reference,Value,Footprint,LCSC,MPN" \
     project.kicad_sch
   ```

5. **Generate Position File (CPL)**
   ```bash
   kicad-cli pcb export pos \
     --output output/assembly/cpl.csv \
     --format csv \
     --units mm \
     --side front \
     project.kicad_pcb
   ```

6. **Create Zip Package**
   ```bash
   cd output/gerbers && zip -r ../gerbers.zip *.g* *.drl
   ```

7. **Generate Summary**
   - List all generated files
   - Board dimensions
   - Layer count
   - Component count
   - Estimated cost factors

---

## Usage Examples

```
/manufacture                    # Generate all files
/manufacture --gerbers-only     # Only Gerbers
/manufacture --jlcpcb           # JLCPCB-formatted outputs
/manufacture --pcbway           # PCBWay-formatted outputs
/manufacture --force            # Skip verification
```

---

## Output Structure

```
output/
├── gerbers/
│   ├── project-F_Cu.gtl
│   ├── project-B_Cu.gbl
│   ├── project-F_SilkS.gto
│   ├── project-B_SilkS.gbo
│   ├── project-F_Mask.gts
│   ├── project-B_Mask.gbs
│   ├── project-Edge_Cuts.gm1
│   ├── project-PTH.drl
│   └── project-NPTH.drl
├── gerbers.zip                 # Ready for upload
├── bom/
│   ├── bom.csv                # Full BOM
│   └── jlcpcb_bom.csv         # JLCPCB format
├── assembly/
│   ├── cpl.csv                # Component placement
│   └── jlcpcb_cpl.csv         # JLCPCB format
└── summary.txt                # Manufacturing summary
```

---

## JLCPCB-Specific Formatting

### BOM Format
```csv
Comment,Designator,Footprint,LCSC
10k,R1,0402,C25744
100nF,C1 C2 C3,0402,C1525
ESP32-S3,U1,QFN-48,C12345
```

### CPL Format  
```csv
Designator,Mid X,Mid Y,Layer,Rotation
R1,10.5,20.3,top,0
C1,15.2,25.1,top,90
U1,50.0,50.0,top,0
```

---

## Agents Used

- `verification-agent` - Pre-flight DRC/ERC
- `pcb-agent` - Gerber generation
- `bom-agent` - BOM formatting
