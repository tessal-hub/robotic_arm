---
name: "6-Axis Arm Controller"
description: "Embedded web UI for NEMA-6AXIS-ARM-CONTROLLER — dark, technical, high-contrast control surface"
colors:
  # Primary Accent (Sky Blue - used for active states, highlights, primary actions)
  primary: "#38bdf8"
  primary-deep: "#0ea5e9"
  primary-muted: "#7dd3fc"
  
  # Semantic Status Colors
  success: "#059669"
  success-bg: "#0c1a17"
  success-text: "#6ee7b7"
  
  warning: "#d97706"
  warning-bg: "#1c1917"
  warning-text: "#fcd34d"
  
  danger: "#dc2626"
  danger-bg: "#7f1d1d"
  danger-text: "#fca5a5"
  
  info: "#2563eb"
  info-bg: "#1e3a8a"
  info-text: "#93c5fd"
  
  # Neutral / Surface Colors (Dark theme)
  bg: "#0f172a"
  surface: "#1e293b"
  surface-elevated: "#0f172a"
  border: "#334155"
  border-strong: "#475569"
  
  # Text Colors
  text-primary: "#f8fafc"
  text-secondary: "#cbd5e1"
  text-muted: "#94a3b8"
  text-dim: "#64748b"
  
  # Special
  focus-ring: "#38bdf8"
  estop-glow: "rgba(220,38,38,0.5)"
  estop-glow-strong: "rgba(220,38,38,0.9)"
  shadow-card: "rgba(0,0,0,0.35)"
  shadow-toast: "rgba(0,0,0,0.4)"
  overlay-backdrop: "rgba(0,0,0,0.4)"

typography:
  display:
    fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif'
    fontSize: "clamp(1.25rem, 4vw, 1.5rem)"
    fontWeight: 700
    lineHeight: 1.2
    letterSpacing: "normal"
  headline:
    fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif'
    fontSize: "1.05rem"
    fontWeight: 600
    lineHeight: 1.3
    letterSpacing: "normal"
  title:
    fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif'
    fontSize: "0.92rem"
    fontWeight: 600
    lineHeight: 1.4
    letterSpacing: "normal"
  body:
    fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif'
    fontSize: "0.85rem"
    fontWeight: 400
    lineHeight: 1.55
    letterSpacing: "normal"
  label:
    fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif'
    fontSize: "0.78rem"
    fontWeight: 600
    lineHeight: 1.4
    letterSpacing: "0.04em"
    textTransform: "uppercase"
  mono:
    fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Consolas, monospace'
    fontSize: "0.78rem"
    fontWeight: 400
    lineHeight: 1.5
    letterSpacing: "normal"
  tabular:
    fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Consolas, monospace'
    fontSize: "1.5rem"
    fontWeight: 400
    lineHeight: 1.2
    fontVariantNumeric: "tabular-nums"

rounded:
  xs: "6px"
  sm: "8px"
  md: "10px"
  lg: "14px"
  pill: "11px"
  badge: "12px"

spacing:
  xs: "4px"
  sm: "6px"
  md: "8px"
  lg: "10px"
  xl: "12px"
  2xl: "14px"
  3xl: "16px"
  4xl: "18px"
  gutter: "12px"
  card-padding: "18px"
  card-gap: "14px"
  tab-gap: "6px"
  page-padding: "16px"
  max-width: "900px"

components:
  # Buttons
  btn-primary:
    backgroundColor: "{colors.info}"
    textColor: "{colors.text-primary}"
    rounded: "{rounded.sm}"
    padding: "10px 14px"
    typography: "{typography.body}"
    fontWeight: 600
    border: "none"
    transition: "transform 0.08s"
  btn-primary-hover:
    backgroundColor: "{colors.primary-deep}"
    transform: "scale(0.97)"
  btn-primary-active:
    transform: "scale(0.97)"
  btn-primary-disabled:
    opacity: 0.4
    cursor: "not-allowed"
    transform: "none"
  
  btn-warn:
    backgroundColor: "{colors.warning}"
    textColor: "{colors.text-primary}"
    rounded: "{rounded.sm}"
    padding: "10px 14px"
    typography: "{typography.body}"
    fontWeight: 600
  
  btn-danger:
    backgroundColor: "{colors.danger}"
    textColor: "{colors.text-primary}"
    rounded: "{rounded.sm}"
    padding: "10px 14px"
    typography: "{typography.body}"
    fontWeight: 600
  
  btn-ghost:
    backgroundColor: "{colors.border}"
    textColor: "{colors.text-secondary}"
    rounded: "{rounded.sm}"
    padding: "10px 14px"
    typography: "{typography.body}"
    fontWeight: 600
  
  btn-ok:
    backgroundColor: "{colors.success}"
    textColor: "{colors.text-primary}"
    rounded: "{rounded.sm}"
    padding: "10px 14px"
    typography: "{typography.body}"
    fontWeight: 600
  
  btn-step:
    backgroundColor: "{colors.border}"
    textColor: "{colors.text-secondary}"
    rounded: "{rounded.xs}"
    padding: "4px 8px"
    typography: "{typography.label}"
    fontSize: "0.75rem"
    fontWeight: 600
  btn-step-active:
    backgroundColor: "{colors.primary}"
    textColor: "{colors.bg}"
  
  btn-tab:
    backgroundColor: "transparent"
    textColor: "{colors.text-muted}"
    rounded: "{rounded.sm}"
    padding: "8px 14px"
    typography: "{typography.body}"
    fontSize: "0.85rem"
    fontWeight: 600
    border: "none"
  btn-tab-active:
    backgroundColor: "{colors.primary}"
    textColor: "{colors.bg}"
  btn-tab-disabled:
    opacity: 0.35
    cursor: "not-allowed"
  
  btn-estop:
    backgroundColor: "{colors.danger}"
    textColor: "{colors.text-primary}"
    rounded: "{rounded.sm}"
    padding: "18px 30px"
    typography: "{typography.headline}"
    fontSize: "1.15rem"
    fontWeight: 600
    boxShadow: "0 6px 24px {colors.estop-glow}"
    animation: "estopPulse 2.2s ease-in-out infinite"
    position: "fixed"
    bottom: "16px"
    right: "16px"
    zIndex: 99
  
  # Cards
  card-main:
    backgroundColor: "{colors.surface}"
    rounded: "{rounded.lg}"
    padding: "{spacing.card-padding}"
    boxShadow: "0 8px 20px {colors.shadow-card}"
    marginBottom: "{spacing.card-gap}"
    maxWidth: "{spacing.max-width}"
    width: "100%"
  card-header:
    typography: "{typography.headline}"
    color: "{colors.primary}"
    borderBottom: "1px solid {colors.border}"
    paddingBottom: "8px"
    marginBottom: "12px"
  
  card-joint:
    backgroundColor: "{colors.bg}"
    border: "1px solid {colors.border}"
    rounded: "{rounded.md}"
    padding: "12px"
  
  # Inputs
  input-text:
    backgroundColor: "{colors.bg}"
    border: "1px solid {colors.border}"
    color: "{colors.text-primary}"
    rounded: "{rounded.sm}"
    padding: "9px 12px"
    typography: "{typography.body}"
    fontSize: "1rem"
    outline: "none"
    width: "100%"
    marginBottom: "8px"
  input-text-focus:
    borderColor: "{colors.focus-ring}"
  
  # Chips / Badges
  badge:
    padding: "3px 10px"
    rounded: "{rounded.badge}"
    typography: "{typography.label}"
    fontSize: "0.7rem"
    fontWeight: 700
    textTransform: "uppercase"
  badge-idle:
    backgroundColor: "{colors.info-bg}"
    color: "{colors.info-text}"
  badge-run:
    backgroundColor: "{colors.success-bg}"
    color: "{colors.success-text}"
  badge-fault:
    backgroundColor: "{colors.danger-bg}"
    color: "{colors.danger-text}"
  badge-warn:
    backgroundColor: "{colors.warning-bg}"
    color: "{colors.warning-text}"
  
  chip-home:
    backgroundColor: "{colors.border}"
    color: "{colors.text-muted}"
    rounded: "{rounded.pill}"
    padding: "3px 12px"
    typography: "{typography.label}"
    fontSize: "0.72rem"
    fontWeight: 700
  chip-home-active:
    backgroundColor: "{colors.warning}"
    color: "{colors.text-primary}"
  chip-home-done:
    backgroundColor: "{colors.success-bg}"
    color: "{colors.success-text}"
  
  # Tabs
  tabs-bar:
    backgroundColor: "{colors.surface}"
    rounded: "{rounded.md}"
    padding: "6px"
    gap: "{spacing.tab-gap}"
    display: "flex"
    flexWrap: "wrap"
    justifyContent: "center"
    maxWidth: "{spacing.max-width}"
    width: "100%"
    marginBottom: "16px"
  
  # Toast
  toast:
    backgroundColor: "{colors.bg}"
    border: "1px solid {colors.border}"
    borderLeftWidth: "4px"
    rounded: "{rounded.sm}"
    padding: "10px 14px"
    typography: "{typography.body}"
    fontSize: "0.84rem"
    color: "{colors.text-secondary}"
    boxShadow: "0 6px 20px {colors.shadow-toast}"
    position: "fixed"
    bottom: "16px"
    left: "16px"
    zIndex: 98
    maxWidth: "70vw"
  toast-ok:
    borderColor: "{colors.success}"
  toast-warn:
    borderColor: "{colors.warning}"
  toast-err:
    borderColor: "{colors.danger}"
  
  # Stat line
  stat-line:
    display: "flex"
    justifyContent: "space-between"
    typography: "{typography.body}"
    fontSize: "0.85rem"
    color: "{colors.text-muted}"
    margin: "4px 0"
  
  # Grid
  grid-joints:
    display: "grid"
    gridTemplateColumns: "repeat(auto-fit, minmax(260px, 1fr))"
    gap: "{spacing.gutter}"
  
  # Row (flex container)
  row:
    display: "flex"
    gap: "10px"
    flexWrap: "wrap"
    alignItems: "center"

---

# Design System: 6-Axis Arm Controller

## 1. Overview

**Creative North Star: "The Mission Control Console"**

A dark, high-contrast technical interface designed for engineers standing at a physical robotic arm in a workshop. Every pixel serves operational clarity: status is glanceable from arm's length, primary actions are unmistakable, and the system never hides what the machine is doing. This is not a dashboard — it's a control surface.

The aesthetic rejects decorative futurism (glassmorphism, neon glow, gradient text, decorative grids) and generic SaaS patterns (cream backgrounds, card grids, tiny uppercase eyebrows). Instead it embraces **bold, capable, modern** — the visual language of professional tools that earn trust through precision and honesty.

**Key Characteristics:**
- Dark theme (slate-900 bg) reduces glare in workshop lighting; sky-blue primary pops without eye strain
- Single accent color (sky blue) carries ≤10% of any screen — its rarity signals "this is the active thing"
- Information-dense but not cluttered: tabular numbers, compact chips, real-time pose updates
- Motion is functional only: E-STOP pulse for urgency, button press scale for tactility, toast entrance, loading spinners, reduced-motion respected
- Honest feedback: toast for every command, connection loss after 3 failed polls, pose from step-count (not encoder)
- **Design tokens as CSS custom properties** — single source of truth at `:root`, zero hard-coded values in component rules
- **Full interaction states** — every interactive element has hover, focus-visible, active, disabled, loading
- **Accessibility first** — ARIA roles, live regions, semantic HTML, keyboard operable

## 2. Colors

A single primary accent (sky blue) with semantic status colors and a deep neutral ramp. The palette is **restrained**: one accent ≤10%, neutrals carry 90%+.

All colors are defined as **CSS custom properties** at `:root` — the single source of truth. Component rules reference `var(--color-*)` exclusively; no hard-coded hex values appear in component styles.

### Primary
- **Mission Sky** (`--color-primary` #38bdf8 / oklch(72% 0.15 220)): Active tab, joint angle readout, focus rings, step selector active, primary data highlights. Used sparingly — only for "live now" or "this is selected."
- **Mission Sky Deep** (`--color-primary-deep` #0ea5e9): Button hover state.
- **Mission Sky Muted** (`--color-primary-muted` #7dd3fc): Subtle accents, canvas stroke.

### Semantic Status (each with bg/text pair for badges)
- **Operational Green** (`--color-success` #059669): Success toasts, "homed" flags, homing complete chips, SET HOME buttons. Background `--color-success-bg` #0c1a17, text `--color-success-text` #6ee7b7.
- **Caution Amber** (`--color-warning` #d97706): Warning toasts, "in progress" homing chips, jog step selector hover. Background `--color-warning-bg` #1c1917, text `--color-warning-text` #fcd34d.
- **Fault Red** (`--color-danger` #dc2626): Error toasts, "not homed" flags, drift fault, encoder error, E-STOP button, endstop latch badges. Background `--color-danger-bg` #7f1d1d, text `--color-danger-text` #fca5a5.
- **Info Blue** (`--color-info` #2563eb): Primary action buttons (HOME ALL, MOVE), idle mode badge. Background `--color-info-bg` #1e3a8a, text `--color-info-text` #93c5fd.

### Neutral
- **Void** (`--color-bg` #0f172a): Page background, joint card background, toast background.
- **Slate-800** (`--color-surface` #1e293b): Main card surface, tabs bar, homing progress chips (inactive).
- **Slate-700** (`--color-border` #334155): Borders, dividers, input backgrounds, step buttons (inactive).
- **Slate-600** (`--color-border-strong` #475569): Stronger borders, hover targets.
- **Slate-400** (`--color-text-muted` #94a3b8): Secondary text, labels, inactive tab text, encoder readouts.
- **Slate-500** (`--color-text-dim` #64748b): Muted text, timestamps, helper copy, placeholders.
- **Slate-200** (`--color-text-secondary` #cbd5e1): Primary body text, joint names, flag labels.
- **Slate-50** (`--color-text-primary` #f8fafc): Headlines, header title — highest contrast on dark.

### Special
- **Focus Ring** (`--color-focus-ring` #38bdf8): Universal focus indicator for all interactive elements.
- **E-STOP Glow** (`--color-estop-glow` rgba(220,38,38,0.5) → `--color-estop-glow-strong` rgba(220,38,38,0.9)): Pulsing shadow on emergency stop.
- **Shadows** (`--color-shadow-card`, `--color-shadow-toast`): Card and toast elevation.

### Named Rules

**The One Voice Rule.** The primary accent (Mission Sky) appears on ≤10% of any given screen. Its rarity is the signal. If everything is highlighted, nothing is.

**The Semantic Pair Rule.** Every status color provides both a background and a text token. Never use a raw status hue on the page background — always pair with its designated surface.

**The Dark-First Rule.** This is a dark-theme-only system. No light mode tokens exist. If light mode is ever needed, it will be a separate palette, not an inversion.

**The Token-First Rule.** No hard-coded color values in component CSS. All colors reference `var(--color-*)` custom properties. This ensures global changes propagate instantly and DESIGN.md stays in sync with code.

## 3. Typography

**Display / Body / Label Font:** System UI stack (`--font-sans`: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif) — no webfonts, zero latency, matches OS chrome.

**Mono / Tabular Font:** `--font-mono`: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace — used for joint angles (tabular-nums), encoder values, coordinates, code snippets.

All typography tokens are defined as **CSS custom properties** at `:root` — font families, sizes, weights, line heights, letter spacing.

### Hierarchy
- **Display** (`--fw-bold`, `--fs-display` clamp(1.25rem, 4vw, 1.5rem), `--lh-tight` 1.2): Page title only ("6-Axis Robotic Arm"). Cap at ~24px — above that shouts, not designs.
- **Headline** (`--fw-semibold`, `--fs-headline` 1.05rem, `--lh-normal` 1.3): Card titles ("System Status", "Manual Joint Control"). Mission Sky.
- **Title** (`--fw-semibold`, `--fs-title` 0.92rem, `--lh-relaxed` 1.4): Sub-section headers within cards.
- **Body** (`--fw-regular`, `--fs-body` 0.85rem, `--lh-loose` 1.55): All prose, stat values, button labels, toast messages. Max line length ~75ch via max-width 900px container.
- **Label** (`--fw-semibold`, `--fs-label` 0.78rem, `--lh-relaxed` 1.4, `--ls-label` 0.04em, UPPERCASE): Input labels, badge text, chip text, step buttons. Tracking prevents cramped caps.
- **Mono** (`--fw-regular`, `--fs-mono` 0.78rem, `--lh-loose` 1.5): Encoder values, coordinates, code snippets.
- **Tabular** (`--fw-regular`, `--fs-tabular` 1.5rem, `--lh-tight` 1.2, tabular-nums): Joint degree readouts — monospace, fixed width, aligns vertically across 6 joints.

### Named Rules

**The Tabular Numbers Rule.** All numeric readouts that align vertically (joint angles, coordinates, feed rates) use `font-variant-numeric: tabular-nums` with the mono stack. No exceptions.

**The Label Tracking Floor.** Uppercase labels never go below 0.04em letter-spacing. Tighter reads as cramped, not "designed."

**The Display Ceiling.** Clamp max ≤ 1.5rem (~24px). Hero headings larger than this dominate the control surface; this is a tool, not a landing page.

**The Token-First Rule (Typography).** No hard-coded font sizes/weights in component CSS. All typography references `var(--fs-*)`, `var(--fw-*)`, `var(--lh-*)`, `var(--ls-*)` custom properties.

## 4. Elevation

**Flat by default, shadows only on persistent surfaces.** Cards and the E-STOP button carry elevation; buttons, chips, inputs, tabs are flat. Depth is conveyed through tonal layering (Void → Slate-800 → Slate-700) not shadows.

All shadows are defined as **CSS custom properties** at `:root` — `--shadow-card`, `--shadow-toast`, `--shadow-estop`, `--shadow-estop-strong`.

### Shadow Vocabulary
- **Card Ambient** (`--shadow-card` `0 8px 20px var(--color-shadow-card)`): Main content cards only. Subtle, diffuse, implies "this panel floats above the void."
- **E-STOP Glow** (`--shadow-estop` `0 6px 24px var(--color-estop-glow)` pulsing to `--shadow-estop-strong` `0 6px 36px var(--color-estop-glow-strong)`): Emergency stop button only. The pulse is the only continuous animation in the system. Respects `prefers-reduced-motion`.
- **Toast Lift** (`--shadow-toast` `0 6px 20px var(--color-shadow-toast)`): Transient notifications. Higher than cards to ensure visibility over content.

### Named Rules

**The Flat-By-Default Rule.** Surfaces are flat at rest. Shadows appear only on persistent containers (cards) and the one critical action (E-STOP). Buttons, tabs, chips, inputs — no shadows ever.

**The No-Ghost-Card Rule.** Never pair a 1px border with a wide soft shadow (blur ≥16px) on the same element. Pick one: a clean border at the brand color, OR a defined shadow ≤8px blur. The card uses shadow; joint cards use border. Never both.

**The Token-First Rule (Elevation).** No hard-coded shadow values in component CSS. All shadows reference `var(--shadow-*)` custom properties.

## 5. Components

### Buttons
- **Shape:** 8px radius (`--radius-sm`), 10px 14px padding, 600 weight (`--fw-semibold`), 0.9rem (`--fs-body`). Active state: `transform: scale(0.97)` (80ms, `--transition-fast` + `--ease-spring`). Disabled: 40% opacity, no transform.
- **Primary** (`--color-info`): Main actions — HOME ALL, MOVE, START DRAW. Highest visual weight. Hover: `--color-primary-deep`.
- **Warn** (`--color-warning`): Secondary destructive — jog step selectors (when not active).
- **Danger** (`--color-danger`): E-STOP (oversized: 18px 30px, 1.15rem), ABORT, jog negative direction.
- **OK** (`--color-success`): Constructive — SET HOME, SAVE WIFI.
- **Ghost** (`--color-border`): Secondary actions — individual joint home, clear fault. Hover: `--color-border-strong`.
- **Step Selector** (`--radius-xs` 6px, 4px padding, 0.75rem): Toggle group for jog step size (0.5°/1°/5°/15°). Active = Mission Sky on Void. `role="radio" aria-checked`.
- **Tab** (`--radius-sm` 8px, 8px 14px, 0.85rem, 600): Transparent inactive, Mission Sky active on Void. Hover: `--color-border`/`--color-text-secondary`. Focus-visible: 2px Mission Sky outline. `role="tab" aria-selected`.

**All buttons have:** hover, focus-visible, active, disabled, loading (`.btn-loading` with spinner) states.

### Chips / Badges
- **Status Badge** (`--radius-badge` 12px, 3px 10px, 0.7rem `--fs-label`, 700, UPPERCASE): Four semantic variants (idle/run/fault/warn) with paired bg/text tokens. Used for mode, homing progress, WiFi mode.
- **Homing Progress Chip** (`--radius-pill` 11px, 3px 12px, 0.72rem, 700): J1–J4 progress track. Inactive = `--color-border`/`--color-text-muted`; Active = `--color-warning`/white; Done = `--color-success-bg`/`--color-success-text`.

### Cards / Containers
- **Main Card** (`--radius-lg` 14px, `--space-card-padding` 18px, `--shadow-card`, `--color-surface`): Primary content containers. Header: Mission Sky headline (`--fs-headline`), 1px `--color-border` divider.
- **Joint Card** (`--radius-md` 10px, 12px padding, 1px `--color-border` border, `--color-bg`): Dense data cards in auto-fit grid (`--space-gutter` 12px gap, minmax 260px). No shadow — tonal separation from main cards.

### Inputs / Fields
- **Text/Password/Select Input** (`--radius-sm` 8px, 9px 12px, 1rem, `--color-border` border, `--color-bg`, `--color-text-primary`): Full width, no outline. Focus: `--color-focus-ring` border (2px equivalent). Placeholder: `--color-text-dim`. Select: native `appearance:none` + custom dropdown arrow SVG.
- **Label** (`--fs-label` 0.78rem, 600, UPPERCASE, `--ls-label` 0.04em tracking, `--color-text-muted`): Always above input, 4px gap.

### Navigation
- **Tab Bar** (`--color-surface` bg, `--radius-md` 10px, 6px padding, `--space-tab-gap` 6px gap, flex-wrap, centered, `--space-max-width` 900px): Horizontal scroll on mobile. Tab buttons as defined above. `role="tablist"`.

### Toast
- **Toast** (`--radius-sm` 8px, 10px 14px, 0.84rem, `--color-border` border, 4px left accent border, `--shadow-toast`): Fixed bottom-left, max 70vw. Three variants by left border color: OK (`--color-success`), Warn (`--color-warning`), Err (`--color-danger`). Entrance animation `toastIn` 150ms `--ease-out`. Auto-dismiss 2.6s. ARIA: `role="status" aria-live="polite" aria-atomic="true"`.

### Stat Line
- **Stat Line** (flex space-between, `--fs-body` 0.85rem, `--color-text-muted`): Key-value rows in Dashboard card. Label left, value right.

### Grid
- **Joint Grid** (auto-fit, minmax 260px, 1fr, `--space-gutter` 12px gap): Responsive joint card layout. No breakpoints — fluid by design.

### Canvas Preview
- **Draw Canvas** (420×300, `--radius-sm` 8px, 1px `--color-border` border, `--color-bg`): Top-view preview with 50mm grid (`--color-border`), shape stroke (`--color-primary` 2px), current TCP dot (`--color-text-primary` 4px), home marker (green 3px + label `--color-text-dim` 10px).

## 6. Do's and Don'ts

### Do:
- **Do** use the system font stack — zero latency, native feel, matches OS chrome.
- **Do** use `font-variant-numeric: tabular-nums` on every aligning numeric readout (joint angles, coordinates, feed rates).
- **Do** pair every status color with its designated background and text tokens (Semantic Pair Rule).
- **Do** keep Mission Sky to ≤10% of screen area — its rarity signals "active now."
- **Do** make primary actions (HOME, STOP, DRAW) visually dominant; destructive actions (Clear Calib) require confirmation.
- **Do** show honest feedback: toast for every command, connection loss indicator after 3 failed polls, real-time pose from step-count.
- **Do** respect `prefers-reduced-motion` — E-STOP pulse disabled, transitions instant.
- **Do** keep touch targets ≥44×44px on primary actions (E-STOP is 18×30 padding ≈ 60×90px).
- **Do** use auto-fit grids (`repeat(auto-fit, minmax(260px, 1fr))`) — no hardcoded breakpoints.
- **Do** use focus-visible (2px Mission Sky) on all interactive elements — keyboard operable.
- **Do** use CSS custom properties for all design tokens — colors, spacing, radius, typography, shadows, transitions. No hard-coded values in component CSS.
- **Do** implement all interaction states: hover, focus-visible, active, disabled, loading for every button/input.
- **Do** use semantic HTML and ARIA: roles (tablist, tab, tabpanel, list, listitem, radio, status), aria-live, aria-label, aria-selected, aria-checked.
- **Do** use Unicode characters (✓, °, ⚠, ⬆, ⬇) instead of HTML entities for readability.
- **Do** add loading spinners on async actions (SAVE & REBOOT, MOVE, START DRAW) with `.btn-loading` class.

### Don't:
- **Don't** use glassmorphism, backdrop-filter, or decorative blur — rejected by PRODUCT.md anti-references ("over-designed futuristic UIs").
- **Don't** use gradient text (`background-clip: text`) — decorative, never meaningful.
- **Don't** use side-stripe borders (`border-left` > 1px as colored accent) — rewrite with full borders, background tints, or leading icons.
- **Don't** use cream/sand/beige near-white backgrounds — the AI default of 2026, rejected by PRODUCT.md.
- **Don't** use identical card grids with icon + heading + text repeated endlessly — PRODUCT.md anti-reference.
- **Don't** put tiny uppercase tracked eyebrows above every section — the 2023-era kicker is AI grammar.
- **Don't** use numbered section markers (01/02/03) as default scaffolding — numbers earn their place only for real sequences.
- **Don't** pair 1px border + wide soft shadow (blur ≥16px) on the same element — the "ghost-card" pattern.
- **Don't** use border-radius >16px on cards/inputs — cards top out at 14px; full-pill only for tags/buttons.
- **Don't** use hand-drawn/sketchy SVG illustrations — reads as amateurish, not whimsical.
- **Don't** use repeating-linear-gradient stripe backgrounds — pure decoration.
- **Don't** use decorative grid backgrounds (CSS gradient overlays) — unless the surface is an actual canvas/blueprint.
- **Don't** use meta-criticism copy (ironic modifiers, staged strawmen) — make the specific claim instead.
- **Don't** let text overflow containers — test heading copy at every breakpoint; reduce clamp max or rewrite copy.
- **Don't** animate CSS layout properties — only transform/opacity/color/box-shadow.
- **Don't** use bounce/elastic easing — ease-out-quart/quint/expo only.
- **Don't** gate content visibility on class-triggered transitions — reveal animations must enhance an already-visible default.
- **Don't** hard-code color/spacing/typography values in component CSS — always use `var(--token-name)`.
- **Don't** omit focus-visible styles — keyboard users must see where they are.
- **Don't** omit loading states on async actions — users need feedback that something is happening.
- **Don't** use `alert()` or bare `confirm()` without accessible alternatives — use toast + inline confirm.
- **Don't** forget `prefers-reduced-motion` — all animations/transitions must respect it.