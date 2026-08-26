---
description: Run DRC and ERC validation on KiCad project. Checks schematic for electrical errors and PCB for design rule violations.
---

# /verify - Run DRC/ERC Validation

$ARGUMENTS

---

## Task

This command runs comprehensive validation on a KiCad project.

### Steps:

1. **Locate Project**
   - Find .kicad_sch and .kicad_pcb files
   - Verify files exist and are readable

2. **Run ERC (Electrical Rules Check)**
   ```bash
   kicad-cli sch erc \
     --output reports/erc-report.rpt \
     --severity-all \
     --exit-code-violations \
     project.kicad_sch
   ```

3. **Run DRC (Design Rules Check)**
   ```bash
   kicad-cli pcb drc \
     --output reports/drc-report.rpt \
     --severity-all \
     --exit-code-violations \
     project.kicad_pcb
   ```

4. **Parse Results**
   - Count errors and warnings
   - Categorize issues by type
   - Identify critical vs minor issues

5. **Report Summary**
   - Display error/warning counts
   - List critical issues
   - Suggest fixes for common problems

6. **Fix Assistance (Optional)**
   - If `--fix` flag provided
   - Attempt automatic fixes for simple issues
   - Provide step-by-step guidance for complex issues

---

## Usage Examples

```
/verify                       # Verify current project
/verify hardware/project      # Verify specific project
/verify --erc-only            # Only run ERC
/verify --drc-only            # Only run DRC
/verify --fix                 # Attempt to fix issues
```

---

## Common ERC Errors and Fixes

| Error | Automatic Fix |
|-------|---------------|
| Unconnected input pin | Add no-connect flag |
| Power pin not driven | Verify power symbol placement |
| Different net connected | Manual review required |

## Common DRC Errors and Fixes

| Error | Automatic Fix |
|-------|---------------|
| Clearance violation | Adjust trace routing |
| Track width violation | Increase trace width |
| Unconnected items | Complete routing |
| Silkscreen on pad | Move silkscreen |

---

## Output Format

```
=== ERC Results ===
Errors:   2
Warnings: 5

[ERROR] Pin U1.3 (VCC) not connected
[ERROR] Net conflict: GND and GPIO0 shorted
[WARN]  Input pin R1.1 not driven
...

=== DRC Results ===
Errors:   1
Warnings: 3

[ERROR] Clearance violation at (45.2, 67.8): 0.08mm < 0.15mm minimum
[WARN]  Silkscreen overlaps pad at U1
...

=== Summary ===
Total: 3 errors, 8 warnings
Status: FAILED - Fix errors before manufacturing
```

---

## Agents Used

- `verification-agent` - Runs checks and interprets results
- `schematic-agent` - For ERC fix suggestions
- `pcb-agent` - For DRC fix suggestions
