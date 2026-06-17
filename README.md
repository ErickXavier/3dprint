# 6-Axis Robotic 3D Printer Simulator

Browser-based simulator of a 6-axis robotic arm with automatic tool changer (ATC), multi-color filament system, and real-time print simulation.

![Three.js](https://img.shields.io/badge/Three.js-r128-blue) ![License](https://img.shields.io/badge/license-MIT-green)

## Features

- **6-DOF Robot Arm** with analytical inverse kinematics (Puma-style IK solver)
- **Automatic Tool Changer** — 10-slot cassette with sequential dock/undock state machine
- **Multi-Color Printing** — 10 toolheads with independent Bowden tubes and filament spools
- **Real-Time Slicer** — progressive async slicer with contour extraction and layer preview hologram
- **STL Import** — drag-and-drop or browse for custom models with auto-orientation (flattest face down)
- **Built-In Models** — parametric vase, gear, and trefoil knot generators
- **Live Telemetry** — TCP coordinates, joint angles, ATC status, print progress
- **Configurable** — position, rotation, scale, nozzle size, speed, resolution, color mode

## Running

Open `index.html` in a browser. No build step, no dependencies to install.

For local development:

```bash
python3 -m http.server 8080
```

Then open [localhost:8080](http://localhost:8080).

## Controls

| Control | Action |
|---------|--------|
| Left drag | Orbit camera |
| Right drag | Pan camera |
| Scroll | Zoom |
| Model buttons | Load vase / gear / knot |
| Browse | Import STL file |
| Play/Pause | Start or pause print |
| Reset | Clear print and reset arm |
| T0–T9 | Manual tool change |
| Speed slider | Adjust simulation speed |
| Resolution slider | Set slice layer height |

## Architecture

Two files, zero build tools:

- **`index.html`** — UI layout, CSS theming, dashboard grid
- **`app.js`** — all simulation logic: IK solver, scene construction, slicer, ATC state machine, animation loop

External dependencies loaded from CDN:
- Three.js r128
- OrbitControls
- STLLoader

## License

MIT
