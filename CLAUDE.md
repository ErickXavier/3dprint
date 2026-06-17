# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Maintenance Rule

**When you modify `app.js` or `index.html`, update this file to reflect the changes** — especially line numbers in the function map, new/removed functions, changed constants, or shifted coordinates. Line numbers drift with every edit; stale references are worse than none.

## Project Overview

Browser-based 6-axis robotic 3D printer simulator. A single-page app (two files: `index.html` + `app.js`) that renders and animates a full robotic arm with automatic tool changer (ATC), multi-color filament system, and real-time print simulation using Three.js r128.

## Running

Open `index.html` directly in a browser — no build step, no server required. For local dev: `python3 -m http.server 8080` then open `localhost:8080`.

## Architecture

### Two-file structure
- **`index.html`** (848 lines) — Full UI: CSS variables/theming, grid dashboard layout, footer console log. All styles inline in `<style>`. CDN script tags at bottom.
- **`app.js`** (2157 lines) — All simulation logic in vanilla JS, organized in commented section blocks.

### External dependencies (CDN, no local install)
- Three.js r128: `cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js`
- OrbitControls: `cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js`
- STLLoader: `cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js`
- Font: Share Tech Mono (Google Fonts)

---

## app.js — Complete Map

### Section Blocks (top to bottom)
| Line | Section |
|------|---------|
| L1 | Constants & State |
| L120 | Inverse Kinematics Solver |
| L188 | Initialization & Geometry Generators |
| L273 | Camera & Window Resize |
| L292 | Build Three.js Graphics |
| L738 | Toolhead Model Class |
| L810 | Dynamic Filament Bowden Tubes |
| L860 | Filament Print Extruder (InstancedMesh) |
| L962 | Slicer Engine |
| L1326 | Model Configuration & Hologram Transformations |
| L1454 | Built-in Procedural Models Generator |
| L1518 | Simulation State Machine (Tick & Physics) |
| L1627 | Tool Change Automated Sequence |
| L1817 | Robot Joint Smoothing Physics |
| L1870 | UI Handlers & Listeners |
| L2152 | Window Onload |

### Constants (L1–L18, L59)
| Const | Value | Purpose |
|-------|-------|---------|
| `L1` | 3.5 | Base column height |
| `L2` | 7.0 | Upper arm length |
| `L3` | 6.0 | Forearm length |
| `L4` | 1.5 | Toolhead length |
| `BASE_OFFSET_Z` | -6.0 | Robot base Z offset behind bed |
| `DOCK_X/Y/Z` | 0.0 / 3.0 / -11.0 | Docking insertion point |
| `APPROACH_Z` | -9.0 | Docking approach Z |
| `SLOT_LOCAL_XS` | [-4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5] | 10 toolhead slot X positions (1.0 unit spacing) |
| `COLOR_HEXS` | [0xff2a2a, 0x4af626, 0x00e5ff, 0xffb300, 0x1a1a1a, 0xffffff, 0xaa44ff, 0xff69b4, 0xff6600, 0x2255ff] | Red, Green, Cyan, Amber, Black, White, Purple, Pink, Orange, Blue |
| `COLOR_NAMES` | ["T0_RED"..."T9_BLUE"] | 10 toolhead labels |
| `MAX_FILAMENT_SEGMENTS` | 120000 | InstancedMesh buffer cap |

### Critical State Variables (L20–L80)
| Variable | Line | Purpose |
|----------|------|---------|
| `scene, camera, renderer, controls` | L21 | Three.js core objects |
| `robotBase, joint1–6, flange, flangeLed` | L22 | Joint hierarchy chain |
| `clawL, clawR` | L23 | Gripper claws on flange |
| `cassetteCarriage, cassetteRail` | L24 | ATC sliding carriage |
| `spools[]` | L24 | Filament spool groups (for rotation animation) |
| `toolheads[]` | L25 | Toolhead instances array |
| `activeToolhead` | L26 | Currently mounted toolhead ref |
| `currentCarriageX / targetCarriageX` | L27–28 | Carriage slide position (actual/target) |
| `simSpeed` | L31 | Simulation speed multiplier |
| `isPrinting / isPaused` | L33–34 | Print state flags |
| `currentModelType` | L36 | 'vase' / 'gear' / 'knot' / 'custom' |
| `modelConfig` | L43–53 | Position, rotation, scale, nozzle, color mode, snap |
| `hologramMesh` | L55 | Transparent preview mesh |
| `filamentMesh` | L58 | InstancedMesh for printed segments |
| `printQueue[] / queueIndex` | L61–62 | Sliced command queue and current index |
| `tcpTargetPos` | L68 | Target Tool Center Point (Vector3) |
| `actualTheta[0–5] / targetTheta[0–5]` | L71–72 | Current vs desired joint angles |
| `atcState` | L75 | Tool change state machine step |
| `currentToolIdx / targetToolIdx` | L77–78 | Active and pending toolhead index (-1 = none) |

### Function Map

#### Logging (L93)
| Function | Lines | Purpose |
|----------|-------|---------|
| `logConsole(message, type)` | L93–118 | Append timestamped message to console DOM. Types: "success"/"error"/"action"/default. Caps at 150 entries. |

#### Inverse Kinematics (L124)
| Function | Lines | Purpose |
|----------|-------|---------|
| `solveIK(tx, ty, tz)` | L124–186 | Analytical 6-DOF Puma-style IK. Returns `{theta1–6, outOfRange}`. Wrist center offset accounts for `BASE_OFFSET_Z` and base Y=0.5. Uses law of cosines for J2/J3. |

#### Scene Setup (L191)
| Function | Lines | Purpose |
|----------|-------|---------|
| `init()` | L191–271 | Creates scene, camera, renderer, lights (ambient + directional + cyan fill + red rim), calls all build functions, starts `animate()` loop. |
| `resetCamera()` | L276–281 | Resets camera to (11, 8, 12) looking at origin. |
| `onWindowResize()` | L283–290 | Updates renderer and camera on window resize. |

#### 3D Construction (L295)
| Function | Lines | Purpose |
|----------|-------|---------|
| `createGroundGrid()` | L295–354 | Procedural grid plane + build bed (8×8) with corner clamps and bed light. |
| `buildRobot()` | L356–584 | Builds entire robot arm as nested joint hierarchy. Creates `robotBase → joint1 → joint2 → joint3 → joint4 → joint5 → joint6 → flange` with claws. ~229 lines. |
| `buildCassette()` | L586–780+ | Builds ATC system: pillars, rail, Creality-style U-frame spool holders (side plates, base, feet, roller, rivets), spools, sliding carriage with 10 slots, toolhead instances, Bowden tubes. |
| `Toolhead(colorIdx, colorHex)` | L741–808 | Constructor function (not a class). Builds toolhead geometry: base block, collar, pin, fan, heatsink fins, nozzle, LED bulb. Reparented between cassette slots and flange during tool changes. |

#### Bowden Tubes (L813)
| Function | Lines | Purpose |
|----------|-------|---------|
| `createBowdenTubes()` | L855–870 | Creates quadratic bezier tube curves (spool → toolhead) for all slots. |
| `updateBowdenTubes()` | L830–858 | Recomputes tube geometry each frame based on current toolhead world positions and spool positions. |

#### Filament Rendering (L863)
| Function | Lines | Purpose |
|----------|-------|---------|
| `createFilamentExtruder()` | L863–891 | Creates InstancedMesh with MAX_FILAMENT_SEGMENTS capacity + active extruder line. |
| `addPrintedSegment(p1, p2, colorHex)` | L893–928 | Adds one filament cylinder between two points to the InstancedMesh. |
| `updateActiveLine(p1, p2, colorHex)` | L930–936 | Updates the live extruder line position/color. |
| `hideActiveLine()` | L938–943 | Hides active extruder line (during tool changes). |
| `clearFilament()` | L945–960 | Resets all filament segments to hidden. |

#### Slicer Engine (L965)
| Function | Lines | Purpose |
|----------|-------|---------|
| `getTrianglesFromGeometry(geo)` | L965–992 | Extracts triangle arrays from Three.js BufferGeometry (indexed or non-indexed). |
| `findFlattestNormal(tris)` | L994–1119 | Physics-based flat-side detection for auto-orientation of STL models. Groups face normals, finds largest coplanar cluster. |
| `processAndNormalizeTriangles(tris)` | L1121–1165 | Rotates model so flattest face is down, centers and scales to fit 8×8 bed. |
| `intersectTrianglePlane(tri, sliceY)` | L1167–1190 | Intersects one triangle with a horizontal plane at Y=sliceY. |
| `connectSegments(segments)` | L1192–1224 | Connects unordered line segments into ordered contour paths. |
| `startProgressiveSlice(numLayers, onComplete)` | L1227–1276 | Progressive async slicer with UI progress bar. Slices in batches to avoid blocking. |
| `generatePrintQueue(slicedLayers)` | L1278–1324 | Converts sliced layers into `printQueue` array of `{type, x, y, z, colorIdx}` commands (TRAVEL / PRINT / TOOLCHANGE). |

#### Model Configuration (L1326)
| Function | Lines | Purpose |
|----------|-------|---------|
| `getTransformedTriangles()` | L1329–1353 | Applies modelConfig transforms (position, rotation, scale) to base triangles. |
| `updateHologramGeometry(geom)` | L1355–1411 | Creates/updates transparent hologram preview mesh from geometry. |
| `applyConfigTransformsToHologram()` | L1413–1418 | Syncs hologram mesh transforms with modelConfig. |
| `reSliceModel()` | L1420–1430 | Re-triggers slicing after config change. |
| `updateUIStates()` | L1432–1452 | Syncs all UI elements with current state (button labels, disabled states). |

#### Built-in Models (L1457)
| Function | Lines | Purpose |
|----------|-------|---------|
| `loadBuiltInModel(type)` | L1457–1516 | Generates parametric vase (lathe), gear (extrude), or knot (tube) geometry. |

#### Animation & Simulation (L1527)
| Function | Lines | Purpose |
|----------|-------|---------|
| `animate()` | L1527–1555 | Main `requestAnimationFrame` loop. Calls `processPrintExecution`, `processToolChangeSequence`, joint smoothing, Bowden update, spool rotation, time display. |
| `processPrintExecution()` | L1557–1625 | Steps through `printQueue`: handles TRAVEL (lerp TCP), PRINT (lerp + deposit segment), TOOLCHANGE (triggers tool change sequence). |

#### Tool Change State Machine (L1627)
| Function | Lines | Purpose |
|----------|-------|---------|
| `triggerToolChange(colorIdx)` | L1629–1645 | Initiates tool change to specified color index. Sets `atcState = 'DOCK_APPROACH'`. |
| `processToolChangeSequence()` | L1647–1815 | Executes the 10-state ATC sequence. Each state lerps positions and transitions on proximity threshold. |

**ATC State Machine Flow:**
```
IDLE → DOCK_APPROACH → DOCK_INSERT → DOCK_RELEASE → FLANGE_RETRACT → SLIDE_CASSETTE → FLANGE_DESCEND → LOCK_GRIP → RETRACT_DOCK → COMPLETE
```
- DOCK_APPROACH: Move TCP to approach position (0, 3, -9)
- DOCK_INSERT: Move TCP to dock (0, 3, -11)
- DOCK_RELEASE: Open claws, release toolhead into slot
- FLANGE_RETRACT: Pull back to approach Z
- SLIDE_CASSETTE: Slide carriage to align target slot
- FLANGE_DESCEND: Lower into new slot
- LOCK_GRIP: Close claws, grab new toolhead
- RETRACT_DOCK: Pull out with new toolhead
- COMPLETE: Return to previous print position, resume

#### Joint Physics (L1817)
Inline in `animate()` — lerps `actualTheta` toward `targetTheta` with smoothing factor `0.08`. Never set `actualTheta` directly or the arm teleports.

#### UI Event Listeners (L1870)
| Function | Lines | Purpose |
|----------|-------|---------|
| `setupUIEventListeners()` | L1872–2080 | Wires all buttons, sliders, file input. Model buttons, config sliders, play/pause/reset, speed, resolution, color mode, tool buttons, camera reset, STL upload. |
| `pausePrint()` | L2082–2087 | Pauses print, updates button text. |
| `resetPrint()` | L2089–2112 | Full simulation reset: clears filament, resets TCP, docks active toolhead, resets carriage. |

#### Entry Point (L2152)
`window.onload = () => { init(); };`

### Robot Joint Hierarchy
The arm is a nested Three.js Group chain — each joint's transform is **relative to its parent**:
```
robotBase (Y=0.5, Z=-6.0)
  → joint1 (Y-rotation, base swivel)
    → joint2 (X-rotation, shoulder)
      → joint3 (X-rotation, elbow)
        → joint4 (Y-rotation, wrist roll)
          → joint5 (X-rotation, wrist pitch)
            → joint6 (Z-rotation, wrist yaw)
              → flange
                → clawL / clawR
                → [active toolhead when mounted]
```

### Key World Coordinates
| Object | Position | Notes |
|--------|----------|-------|
| Robot base | (0, 0.5, -6.0) | Behind the 8×8 build bed |
| Build bed | (0, 0.1, 0) | 8×8 units, centered at origin |
| Cassette rail | (0, 4.5, -12.5) | Along X-axis |
| Cassette carriage | (0, 4.5, -12.5) | Slides along X |
| U-frame holders | (sx, 5.0, -14.9) | Creality-style, roller at Y=7.5 |
| Pillars | (±5.5, 2.25, -12.5) | Support rail |
| Spool positions | (sx, 7.5, -14.9) | sx from SLOT_LOCAL_XS (10 spools) |
| Bowden tube start | (sx, 8.35, -14.05) | Top-front of each spool |
| Dock position | (0, 3.0, -11.0) | Tool change insertion |
| Approach position | (0, 3.0, -9.0) | Pre-dock waypoint |
| Camera default | (11, 8, 12) | Looking at origin |
| Main light | (5, 12, 6) | Directional, shadow-casting |
| Fill light | (-8, 5, -5) | Cyan tint |
| Rim light | (0, 2, -10) | Red tint |

---

## index.html — Structure (848 lines)

### CSS Theme Variables (L16–L23)
| Variable | Value | Usage |
|----------|-------|-------|
| `--bg-color` | #08080a | Page background |
| `--panel-bg` | #0c0c0f | Panel backgrounds |
| `--border-color` | #ff2a2a | Borders, accent red |
| `--text-color` | #eaeaea | Primary text |
| `--accent-green` | #4af626 | Success, progress |
| `--accent-cyan` | #00e5ff | Info accent |
| `--accent-amber` | #ffb300 | Warning accent |
| `--grid-border` | #22222a | Grid lines |

### Dashboard Grid Layout (L60–L185)
```
3-column × 3-row grid:
  Row 1 (50px):   Header spanning all 3 columns
  Row 2 (1fr):    [Left Panel 340px] [Viewport 1fr] [Right Panel 340px]
  Row 3 (150px):  Footer/Console spanning all 3 columns
```

### UI Panels
**Left Panel** (`#left-panel`, L526) — Controls:
- Section 01: Model Selection — STL upload (`#btn-browse`, `#stl-file`), built-in models (`#btn-model-vase/gear/knot`)
- Section 02: Model Configuration — Position X/Z sliders, Rotation X/Y/Z sliders, hologram toggle, snap rotation toggle, scale slider, nozzle size slider, color mode (single/multi)
- Section 03: Print Controls — Play/pause/reset, speed slider, resolution slider, manual tool buttons (T0–T3), camera reset

**Viewport** (`#viewport-container` > `#canvas3d`, L666–L667) — Three.js canvas. Contains:
- Warning badge overlay (`#warning-badge`, L680)
- Slicing progress modal (`#slicing-modal`, L686)

**Right Panel** (`#right-panel`, L697) — Telemetry:
- TCP Coordinates: X/Y/Z readouts (`#coord-x/y/z`)
- Joint Angles: J1–J6 values + progress bars (`#j1-val` through `#j6-bar`)
- Stats: Active toolhead, ATC slot pos, filament count, print progress

**Footer** (L828) — Console log (`#console-log`)

### All DOM Element IDs
**Buttons:** `btn-browse`, `btn-cam-reset`, `btn-color-multi`, `btn-color-single`, `btn-model-gear`, `btn-model-knot`, `btn-model-vase`, `btn-play-pause`, `btn-reset`, `btn-toggle-hologram`, `btn-toggle-snap`, `btn-tool-0` through `btn-tool-9`

**Sliders:** `slider-nozzle`, `slider-pos-x`, `slider-pos-z`, `slider-resolution`, `slider-rot-x/y/z`, `slider-scale`, `slider-speed`

**Displays:** `coord-x/y/z`, `j1-val` through `j6-val`, `j1-bar` through `j6-bar`, `nozzle-val`, `pos-val`, `resolution-val`, `rot-val`, `scale-val`, `speed-val`, `stat-atc-pos`, `stat-filaments`, `stat-progress`, `stat-toolhead`, `systime`

**Containers:** `canvas3d`, `console-log`, `dashboard`, `left-panel`, `right-panel`, `slicing-bar-fill`, `slicing-modal`, `slicing-percentage`, `stl-file`, `viewport-container`, `warning-badge`, `warning-text`

---

## Key Constraints
- Three.js r128 loaded from CDN — no ES modules, all globals (`THREE.Scene`, `THREE.Mesh`, etc.)
- No bundler, no transpiler, no package.json — raw browser JS
- `InstancedMesh` filament rendering has a hard cap at `MAX_FILAMENT_SEGMENTS = 120000`
- IK solver assumes fixed arm segment lengths; changing `L1`–`L4` requires re-tuning reach checks
- ATC tool change is a sequential state machine — states must not be skipped or reordered
- Animation loop lerps `actualTheta` toward `targetTheta` with smoothing factor — never set `actualTheta` directly or the arm will teleport
- `Toolhead` is a constructor function (not a class); instances are reparented between cassette slots and the flange during tool changes via `group.add()` / `group.remove()`
- Spool positions, bracket positions, and Bowden tube start points all derive from `SLOT_LOCAL_XS` — changing slot spacing requires updating all three
