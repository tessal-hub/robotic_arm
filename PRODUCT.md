# Product

## Register

product

## Users

Engineers and operators standing at the physical robotic arm. They use the web interface on a laptop, tablet, or phone connected to the same WiFi network (STA mode) or directly to the arm's AP fallback. Context: workshop / lab environment, variable ambient light, often wearing safety glasses, need glanceable status and confident control.

## Product Purpose

A 6-axis robotic arm controller (NEMA-6AXIS-ARM-CONTROLLER) running on ESP32-S3. The embedded web app is the primary human interface for:
- **Commissioning**: homing (J1–J4 auto, J5/J6 manual), setting joint zeros, tuning StallGuard thresholds
- **Running draw jobs**: loading line/circle shapes, previewing toolpaths on a top-view canvas, starting/aborting draws on paper
- **Manual jog & teaching**: moving individual joints in relative steps (0.5°–15°), verifying encoder health
- **Monitoring & diagnostics**: real-time joint angles (step-count + encoder), endstop state, motor current, drift detection, WiFi status

Success = operator can reliably home, jog, and execute draw jobs without touching firmware or serial monitor.

## Brand Personality

**Bold, Capable, Modern**

- **Bold**: decisive visual hierarchy, high-contrast dark UI that reads at a glance in workshop lighting
- **Capable**: every control feels precise and intentional; status is information-dense but not cluttered
- **Modern**: clean, no skeuomorphism or decorative flourishes; motion is functional (reduced-motion respected)

## Anti-references

- **Over-designed "futuristic" UIs**: excessive glassmorphism/blur, neon glow, decorative grid backgrounds, motion for motion's sake, gradient text, side-stripe borders on cards
- **Generic SaaS dashboards**: cream/sand/beige near-white backgrounds, identical card grids, tiny uppercase eyebrow labels on every section, hero-metric templates
- **Cluttered industrial HMIs**: dense tables with tiny text, too many simultaneous panels, dated Windows 95 aesthetic

## Design Principles

1. **Glanceable at arm's length** — primary status (mode, homed joints, faults) readable from 1–2m away in variable lighting
2. **Action confidence over density** — primary actions (HOME, STOP, DRAW) are unmistakable; destructive actions (Clear Calib) require confirmation
3. **Honest feedback** — toast for every command, connection loss indicator after 3 failed polls, real-time pose from step-count (not encoder)
4. **Respect the physical context** — dark theme for workshop glare reduction; E-STOP always visible; reduced-motion honored
5. **No mystery state** — if the arm is busy, disabled, or faulted, the UI shows why and what to do next

## Accessibility & Inclusion

- **Reduced motion**: all animations respect `prefers-reduced-motion: reduce` (E-STOP pulse, transitions)
- **WCAG AA contrast target**: body text ≥4.5:1 against background; status badges and critical actions verified
- **Color-blind safe status palette**: homed (green), running (blue), fault (red), warning (amber) distinguishable by deuteranopes/protanopes — not relying on hue alone
- **Keyboard operable**: tab order logical, focus-visible on all interactive elements
- **Touch targets**: ≥44×44px for jog buttons and primary actions on tablet/phone