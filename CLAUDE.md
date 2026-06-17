# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Browser-based 6-axis robotic 3D printer simulator. A single-page app (two files: `index.html` + `app.js`) that renders and animates a full robotic arm with automatic tool changer (ATC), multi-color filament system, and real-time print simulation using Three.js r128.

## Running

Open `index.html` directly in a browser — no build step, no server required. All dependencies load from CDN (Three.js, OrbitControls, STLLoader).

## Architecture

### Two-file structure
- **`index.html`** — Full UI: CSS variables/theming, grid dashboard layout (left sidebar controls, center 3D viewport, right sidebar telemetry), footer console log. All styles are inline in `<style>`. CDN script tags at bottom.
- **`app.js`** — All simulation logic in vanilla JS. ~2150 lines, organized in large commented section blocks.

### Key systems in app.js (top to bottom)
1. **Constants & State** — Robot arm dimensions (L1–L4), dock coordinates, color palette (4 toolheads: red/green/cyan/amber), simulation flags
2. **Inverse Kinematics** (`solveIK`) — Analytical 6-DOF Puma-style IK solver. Converts TCP target position to 6 joint angles. Uses `BASE_OFFSET_Z = -6.0` to account for robot mounting behind the bed
3. **Scene Setup** (`init`) — Three.js scene, camera, renderer, shadow-mapped lighting (ambient + directional main + cyan fill + red rim), ground grid, build bed with clamps
4. **Robot Arm** (`buildRobot`) — Hierarchical joint chain: `robotBase → joint1 → joint2 → joint3 → joint4 → joint5 → joint6 → flange` with gripper claws
5. **Cassette ATC** (`buildCassette`) — Linear rail at Z=-12.5, sliding carriage with 4 toolhead slots, filament spools on rear rack with Bowden tubes
6. **Model Generation** — Built-in parametric models (vase/gear/knot) + STL file upload via `THREE.STLLoader`. Physics-based flat-side detection for auto-orientation
7. **Slicer** (`sliceModel`) — Planar slicing with configurable layer count and nozzle size. Generates `printQueue` array of TRAVEL/PRINT/TOOLCHANGE commands
8. **Print Execution** (`processPrintExecution`) — Steps through `printQueue`, moves TCP via lerp, deposits filament segments as `InstancedMesh` (up to 120k segments)
9. **Tool Change Sequence** (`processToolChangeSequence`) — Multi-state machine: `IDLE → DOCK_APPROACH → DOCK_INSERT → DOCK_RELEASE → FLANGE_RETRACT → SLIDE_CASSETTE → FLANGE_DESCEND → LOCK_GRIP → RETRACT_DOCK → COMPLETE`
10. **Animation Loop** (`animate`) — `requestAnimationFrame` loop. Interpolates joint angles with smoothing factor for motor physics feel. Updates Bowden tubes, spool rotation, LED states

### Critical global state
- `tcpTargetPos` (Vector3) — desired tool-center-point position; IK solver runs against this every frame
- `targetTheta[0–5]` / `actualTheta[0–5]` — desired vs. interpolated joint angles; the lerp between them creates motor-smoothing feel
- `printQueue[]` / `queueIndex` — array of `{type, x, y, z, colorIdx}` commands (TRAVEL/PRINT/TOOLCHANGE) produced by the slicer; print execution steps through sequentially
- `currentToolIdx` / `targetToolIdx` — active and pending toolhead index (0–3, or -1 for none)
- `atcState` — current step in the tool-change state machine (string enum)
- `SLOT_LOCAL_XS = [-3, -1, 1, 3]` maps to toolheads `[T0_RED, T1_GREEN, T2_CYAN, T3_AMBER]`

### Robot joint hierarchy
The arm is a nested Three.js Group chain — each joint's transform is **relative to its parent**:
`robotBase → joint1(Y-rot) → joint2(X-rot) → joint3(X-rot) → joint4(Y-rot) → joint5(X-rot) → joint6(Z-rot) → flange → clawL/clawR`
Adding geometry to any joint means positioning in that joint's local space, not world space.

### Coordinate conventions
- Robot base: (0, 0.5, -6.0) — behind the 8×8 build bed
- Cassette rail: along X-axis at Z=-12.5, Y=4.5
- Spool rack: Z=-14.9 to -15.5
- Dock position: (0, 3.0, -11.0), approach at Z=-9.0
- Build bed: centered at origin, Y=0.1, 8×8 units

### UI ↔ JS interaction
All UI updates go through DOM element IDs (e.g., `stat-toolhead`, `stat-progress`, `console-log`). The `setupUIEventListeners` function wires buttons and sliders. Console messages via `logConsole(msg, type)` where type is "success"/"error"/"action"/default.

## Key constraints
- Three.js r128 loaded from CDN — no ES modules, all globals (`THREE.Scene`, `THREE.Mesh`, etc.)
- No bundler, no transpiler, no package.json — raw browser JS
- `InstancedMesh` filament rendering has a hard cap at `MAX_FILAMENT_SEGMENTS = 120000`
- IK solver assumes fixed arm segment lengths; changing `L1`–`L4` requires re-tuning reach checks
- ATC tool change is a sequential state machine — states must not be skipped or reordered
- Animation loop lerps `actualTheta` toward `targetTheta` with a smoothing factor — never set `actualTheta` directly or the arm will teleport
- The `Toolhead` constructor function (not a class) builds geometry imperatively; toolheads are reparented between cassette slots and the flange during tool changes
