// System Constants
const L1 = 3.5; // Base column height (increased for reach)
const L2 = 7.0; // Upper arm length (increased to reach all 8 corners)
const L3 = 6.0; // Forearm length (increased to reach all 8 corners)
const L4 = 1.5; // Toolhead length

// Robot mounting offset (base is placed behind the print bed at Z = -6.0 to be completely out of the bed)
const BASE_OFFSET_Z = -6.0;

const DOCK_X = 0.0; // Docking insertion X
const DOCK_Y = 3.0;  // Docking insertion Y
const DOCK_Z = -11.0; // Docking insertion Z (the slot center)
const APPROACH_Z = -9.0; // Docking approach Z (in front of slot)

// Slots relative X coordinates on carriage
const SLOT_LOCAL_XS = [-3.0, -1.0, 1.0, 3.0];
const COLOR_HEXS = [0xff2a2a, 0x4af626, 0x00e5ff, 0xffb300]; // Red, Green, Cyan/Blue, Amber/Yellow
const COLOR_NAMES = ["T0_RED", "T1_GREEN", "T2_CYAN", "T3_AMBER"];

// App state variables
let scene, camera, renderer, controls;
let robotBase, joint1, joint2, joint3, joint4, joint5, joint6, flange;
let clawL, clawR;
let cassetteCarriage, cassetteRail, spools = [];
let toolheads = []; // instances
let activeToolhead = null;
let currentCarriageX = 0;
let targetCarriageX = 0;

// Print and Simulation variables
let simSpeed = 5.0;
let isSlicing = false;
let isPrinting = false;
let isPaused = true;
let slicingProgress = 0;
let currentModelType = 'vase'; // 'vase', 'gear', 'knot', 'custom'
let currentGeometry = null;
let baseTriangles = []; // Master copy of normalized STL triangles
let stlTriangles = []; // Configured/transformed triangles for slicing
let lastSlicedLayers = []; // Cache of last computed slice layers for color mode regeneration

// Model configuration parameters
let modelConfig = {
  x: 0.0,
  z: 0.0,
  rotX: 0.0, // in radians
  rotY: 0.0, // in radians
  rotZ: 0.0, // in radians
  scale: 1.0,
  nozzleSize: 0.2, // in mm (default is minimum for high resolution)
  colorMode: 'single', // 'single' or 'multi'
  snapRot: false // Snap rotation to 90 degrees
};

let hologramMesh = null; // Glowing hologram group representation

// Filament variables
let filamentMesh;
const MAX_FILAMENT_SEGMENTS = 120000; // Increased buffer size for higher resolutions
let currentFilamentCount = 0;
let printQueue = [];
let queueIndex = 0;

// Filament Line tracing the dynamic nozzle flow
let activeExtruderLine;

// Target coordinate for the TCP (Tool Center Point)
const tcpTargetPos = new THREE.Vector3(0, 3.0, 0); 

// Robot actual angles (interpolated for motor physics)
let actualTheta = [0, 0, 0, 0, 0, 0];
let targetTheta = [0, 0, 0, 0, 0, 0];

// Toolchange state variables
let atcState = 'IDLE'; // IDLE, DOCK_APPROACH, DOCK_INSERT, DOCK_RELEASE, FLANGE_RETRACT, SLIDE_CASSETTE, FLANGE_DESCEND, LOCK_GRIP, RETRACT_DOCK, COMPLETE
let atcProgress = 0;
let targetToolIdx = -1;
let currentToolIdx = -1; // -1 means none
let toolchangePrevPos = new THREE.Vector3();
let toolchangeReturnIndex = 0;

// Dynamic Bowden tubes/filament paths
let bowdenTubes = [];

// System clock
setInterval(() => {
  const now = new Date();
  document.getElementById('systime').innerText = now.toTimeString().split(' ')[0];
}, 1000);

// Initialize Terminal Logger
const logQueue = [];
function logConsole(message, type = '') {
  const logContainer = document.getElementById('console-log');
  const timeStr = new Date().toTimeString().split(' ')[0] + '.' + String(new Date().getMilliseconds()).padStart(3, '0');
  
  const lineDiv = document.createElement('div');
  lineDiv.className = 'console-line';
  
  const timeSpan = document.createElement('span');
  timeSpan.className = 'console-time';
  timeSpan.innerText = `[${timeStr}]`;
  
  const textSpan = document.createElement('span');
  textSpan.className = `console-text ${type}`;
  textSpan.innerText = message;
  
  lineDiv.appendChild(timeSpan);
  lineDiv.appendChild(textSpan);
  
  logContainer.appendChild(lineDiv);
  logContainer.scrollTop = logContainer.scrollHeight;
  
  // Limit log entries inside DOM
  while (logContainer.childNodes.length > 150) {
    logContainer.removeChild(logContainer.firstChild);
  }
}

// ----------------------------------------------------
// INVERSE KINEMATICS SOLVER
// Calculates joints: J1(RotY), J2(RotX), J3(RotX), J5(RotX)
// ----------------------------------------------------
function solveIK(tx, ty, tz) {
  let result = {
    theta1: 0,
    theta2: 0,
    theta3: 0,
    theta4: 0,
    theta5: 0,
    theta6: 0,
    outOfRange: false
  };

  // Wrist center coordinates relative to robot base:
  // Robot base is at X=0, Z=BASE_OFFSET_Z. The base group sits at Y = 0.5.
  const rx = tx;
  const ry = ty + L4 - 0.5; // Subtract 0.5 offset to account for base group Y position
  const rz = tz - BASE_OFFSET_Z;

  // Shoulder position is at (0, L1, 0) relative to robot base
  const dx = rx;
  const dy = ry - L1;
  const dz = rz;

  // Base angle theta1 (inverted to correct mirroring in Three.js coordinate system)
  result.theta1 = -Math.atan2(dz, dx);

  // Horizontal reach distance
  const r = Math.sqrt(dx * dx + dz * dz);
  const h = dy;

  // Diagonal reach distance from shoulder to wrist center
  const dSq = r * r + h * h;
  const d = Math.sqrt(dSq);

  // Law of Cosines for Joint 3
  const num = dSq - L2 * L2 - L3 * L3;
  const den = 2 * L2 * L3;
  let cosT3 = num / den;

  if (cosT3 > 1.0 || cosT3 < -1.0) {
    result.outOfRange = true;
    cosT3 = Math.max(-1.0, Math.min(1.0, cosT3));
  }

  // We choose Elbow-Up configuration (negative theta3 for elbow-up deflection in 2D plane)
  result.theta3 = -Math.acos(cosT3);

  // Angle theta2
  const angle1 = Math.atan2(h, r);
  const angle2 = Math.atan2(L3 * Math.sin(Math.abs(result.theta3)), L2 + L3 * Math.cos(result.theta3));
  
  // Physical joint rotation offset: mathematical theta2 plus angle2 minus PI/2
  result.theta2 = (angle1 + angle2) - Math.PI / 2;

  // To keep toolhead vertical, cancel out pitch of shoulder and elbow
  // Cumulative rotation: theta2 + theta3 + theta5 = 0
  result.theta5 = -(result.theta2 + result.theta3);

  // Roll joints locked during standard printing
  result.theta4 = 0;
  result.theta6 = 0;

  return result;
}

// ----------------------------------------------------
// INITIALIZATION & GEOMETRY GENERATORS
// ----------------------------------------------------
function init() {
  const container = document.getElementById('viewport-container');
  const w = container.clientWidth;
  const h = container.clientHeight;

  // Three.js Scene
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x060608);
  scene.fog = new THREE.FogExp2(0x060608, 0.015);

  // Camera
  camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 100);
  resetCamera();

  // Renderer
  renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('canvas3d'), antialias: true });
  renderer.setSize(w, h);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  // Controls
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.maxPolarAngle = Math.PI / 2 - 0.02; // Don't go below ground
  controls.minDistance = 5;
  controls.maxDistance = 35;
  controls.target.set(0, 2.0, 0);

  // Lighting
  const ambientLight = new THREE.AmbientLight(0x1a1a24);
  scene.add(ambientLight);

  const mainLight = new THREE.DirectionalLight(0xffffff, 1.2);
  mainLight.position.set(5, 12, 6);
  mainLight.castShadow = true;
  mainLight.shadow.mapSize.width = 2048;
  mainLight.shadow.mapSize.height = 2048;
  mainLight.shadow.camera.near = 0.5;
  mainLight.shadow.camera.far = 40;
  const d = 12;
  mainLight.shadow.camera.left = -d;
  mainLight.shadow.camera.right = d;
  mainLight.shadow.camera.top = d;
  mainLight.shadow.camera.bottom = -d;
  scene.add(mainLight);

  const fillLight = new THREE.DirectionalLight(0x00e5ff, 0.3); // Cyber blue fill
  fillLight.position.set(-8, 5, -5);
  scene.add(fillLight);

  const rimLight = new THREE.DirectionalLight(0xff2a2a, 0.2); // Hazard red rim
  rimLight.position.set(0, 2, -10);
  scene.add(rimLight);

  // Floor & Build Area
  createGroundGrid();

  // Filament InstancedMesh
  createFilamentExtruder();

  // Build the robot arm geometry (placed behind bed at Z = -6.0)
  buildRobot();

  // Build the cassette ATC system (placed at side at X = -7.0, Z = -6.0)
  buildCassette();

  // Listeners
  window.addEventListener('resize', onWindowResize);
  setupUIEventListeners();

  // Initialize default model
  loadBuiltInModel('vase');

  logConsole("System Initialization complete.", "success");
  logConsole("Robot mounted behind the print bed at (0, 0.5, -6.0).");
  logConsole("Active Kinematic Engine: Puma-Analytical-IK");

  // Start loop
  animate();
}

// ----------------------------------------------------
// CAMERA & WINDOW RESIZE
// ----------------------------------------------------
function resetCamera() {
  camera.position.set(11, 8, 12);
  if (controls) {
    controls.target.set(0, 2.0, 0);
  }
}

function onWindowResize() {
  const container = document.getElementById('viewport-container');
  const w = container.clientWidth;
  const h = container.clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}

// ----------------------------------------------------
// BUILD THREE.JS GRAPHICS
// ----------------------------------------------------
function createGroundGrid() {
  // Floor plane (heavy mechanical grid)
  const floorGeo = new THREE.PlaneGeometry(50, 50);
  const floorMat = new THREE.MeshStandardMaterial({ 
    color: 0x0a0a0c, 
    roughness: 0.9, 
    metalness: 0.1 
  });
  const floor = new THREE.Mesh(floorGeo, floorMat);
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);

  // Grid helper
  const grid = new THREE.GridHelper(40, 40, 0xff2a2a, 0x181822);
  grid.position.y = 0.01;
  scene.add(grid);

  // Printer build bed
  const bedGeo = new THREE.BoxGeometry(8, 0.2, 8);
  const bedMat = new THREE.MeshStandardMaterial({ 
    color: 0x121215, 
    roughness: 0.4, 
    metalness: 0.7 
  });
  const bed = new THREE.Mesh(bedGeo, bedMat);
  bed.position.y = 0.1;
  bed.receiveShadow = true;
  scene.add(bed);

  // Build plate border glowing lines
  const bedBorderGeo = new THREE.BoxGeometry(8.1, 0.02, 8.1);
  const bedBorderMat = new THREE.MeshBasicMaterial({ color: 0x22222d, wireframe: true });
  const bedBorder = new THREE.Mesh(bedBorderGeo, bedBorderMat);
  bedBorder.position.y = 0.21;
  scene.add(bedBorder);

  // Printer Bed Grid
  const bedGrid = new THREE.GridHelper(8, 16, 0x4af626, 0x113311);
  bedGrid.position.y = 0.21;
  scene.add(bedGrid);

  // Bed corner clamps
  const clampGeo = new THREE.BoxGeometry(0.3, 0.2, 0.6);
  const clampMat = new THREE.MeshStandardMaterial({ color: 0x333339, metalness: 0.9 });
  const clampPositions = [
    [-3.9, 0.2, -2], [-3.9, 0.2, 2], [3.9, 0.2, -2], [3.9, 0.2, 2]
  ];
  clampPositions.forEach(p => {
    const c = new THREE.Mesh(clampGeo, clampMat);
    c.position.set(p[0], p[1], p[2]);
    c.rotation.y = p[0] < 0 ? 0 : Math.PI;
    scene.add(c);
  });
  
  // Heating light under bed
  const bedLight = new THREE.PointLight(0xff3300, 0.8, 10);
  bedLight.position.set(0, -1, 0);
  scene.add(bedLight);
}

function buildRobot() {
  // Materials
  const ironMat = new THREE.MeshStandardMaterial({ color: 0x28282e, metalness: 0.8, roughness: 0.25 });
  const brightYellowMat = new THREE.MeshStandardMaterial({ color: 0xe59800, metalness: 0.5, roughness: 0.3 }); // Hazard Orange/Yellow
  const jointCapMat = new THREE.MeshStandardMaterial({ color: 0xff2a2a, metalness: 0.6, roughness: 0.1 });
  const steelMat = new THREE.MeshStandardMaterial({ color: 0x888890, metalness: 0.9, roughness: 0.15 });

  // Base pedestal (placed behind at Z = BASE_OFFSET_Z)
  const baseGeo = new THREE.CylinderGeometry(1.6, 1.8, 0.5, 32);
  const baseMesh = new THREE.Mesh(baseGeo, ironMat);
  baseMesh.position.set(0, 0.25, BASE_OFFSET_Z);
  baseMesh.castShadow = true;
  baseMesh.receiveShadow = true;
  scene.add(baseMesh);

  // Robot Base coordinate frame (contains joint1 group)
  robotBase = new THREE.Group();
  robotBase.position.set(0, 0.5, BASE_OFFSET_Z);
  scene.add(robotBase);

  // Joint 1: Rotates about vertical Y
  joint1 = new THREE.Group();
  robotBase.add(joint1);

  // Link 1: Turret body
  const l1BaseGeo = new THREE.CylinderGeometry(1.3, 1.3, 0.8, 24);
  const l1Base = new THREE.Mesh(l1BaseGeo, ironMat);
  l1Base.position.y = 0.4;
  l1Base.castShadow = true;
  joint1.add(l1Base);

  const l1BodyGeo = new THREE.BoxGeometry(1.6, L1 - 0.8, 1.6);
  const l1Body = new THREE.Mesh(l1BodyGeo, ironMat);
  l1Body.position.set(0, 0.8 + (L1 - 0.8) / 2, 0);
  l1Body.castShadow = true;
  joint1.add(l1Body);

  // Hazard stripe decals
  const stripeGeo = new THREE.BoxGeometry(1.64, 0.15, 0.8);
  const stripeDecal = new THREE.Mesh(stripeGeo, brightYellowMat);
  stripeDecal.position.set(0, L1 - 1.2, 0);
  joint1.add(stripeDecal);

  // Joint 2: Rotates about horizontal X at Y = L1
  joint2 = new THREE.Group();
  joint2.position.set(0, L1, 0);
  joint1.add(joint2);

  // Joint 2 Cap/motor cylinder
  const j2MotorGeo = new THREE.CylinderGeometry(0.6, 0.6, 1.8, 24);
  j2MotorGeo.rotateZ(Math.PI / 2);
  const j2Motor = new THREE.Mesh(j2MotorGeo, jointCapMat);
  joint2.add(j2Motor);

  // Link 2: Upper arm (Truss style with parallel columns)
  const l2UpperArm = new THREE.Group();
  joint2.add(l2UpperArm);

  // Left support column
  const colLGeo = new THREE.BoxGeometry(0.3, L2, 0.4);
  const colL = new THREE.Mesh(colLGeo, ironMat);
  colL.position.set(-0.5, L2 / 2, 0);
  colL.castShadow = true;
  l2UpperArm.add(colL);

  // Right support column
  const colRGeo = new THREE.BoxGeometry(0.3, L2, 0.4);
  const colR = new THREE.Mesh(colRGeo, ironMat);
  colR.position.set(0.5, L2 / 2, 0);
  colR.castShadow = true;
  l2UpperArm.add(colR);

  // Cross braces
  const braceGeo = new THREE.BoxGeometry(0.8, 0.1, 0.1);
  for (let yOffset = 0.8; yOffset < L2; yOffset += 1.2) {
    const brace = new THREE.Mesh(braceGeo, brightYellowMat);
    brace.position.set(0, yOffset, 0);
    l2UpperArm.add(brace);
  }

  // Joint 3: Rotates about horizontal X at end of Link 2
  joint3 = new THREE.Group();
  joint3.position.set(0, L2, 0);
  joint2.add(joint3);

  // Joint 3 Cap/motor cylinder
  const j3MotorGeo = new THREE.CylinderGeometry(0.45, 0.45, 1.4, 24);
  j3MotorGeo.rotateZ(Math.PI / 2);
  const j3Motor = new THREE.Mesh(j3MotorGeo, jointCapMat);
  joint3.add(j3Motor);

  // Link 3: Forearm
  const l3Forearm = new THREE.Group();
  joint3.add(l3Forearm);

  // Forearm body (tapered cylindrical tube)
  const col3Geo = new THREE.CylinderGeometry(0.28, 0.4, L3, 16);
  const col3 = new THREE.Mesh(col3Geo, ironMat);
  col3.position.y = L3 / 2;
  col3.castShadow = true;
  l3Forearm.add(col3);

  // Joint 4: Forearm Roll (Rotates about Y in local frame)
  joint4 = new THREE.Group();
  joint4.position.set(0, L3, 0);
  joint3.add(joint4);

  // Joint 5: Wrist Pitch (Rotates about horizontal X)
  joint5 = new THREE.Group();
  joint4.add(joint5);

  const j5MotorGeo = new THREE.CylinderGeometry(0.3, 0.3, 0.8, 16);
  j5MotorGeo.rotateZ(Math.PI / 2);
  const j5Motor = new THREE.Mesh(j5MotorGeo, steelMat);
  joint5.add(j5Motor);

  // Joint 6: Tool flange roll (Rotates about Z local axis)
  joint6 = new THREE.Group();
  joint5.add(joint6);

  // Flange disc
  const flangeGeo = new THREE.CylinderGeometry(0.35, 0.35, 0.15, 24);
  flangeGeo.rotateX(Math.PI / 2);
  const flangeMesh = new THREE.Mesh(flangeGeo, steelMat);
  joint6.add(flangeMesh);

  // Gripper mechanism parent group
  flange = new THREE.Group();
  flange.position.set(0, 0, 0);
  joint6.add(flange);

  // Gripper claws (2 sliding L-shapes)
  const clawBaseGeo = new THREE.BoxGeometry(0.8, 0.1, 0.2);
  const clawBase = new THREE.Mesh(clawBaseGeo, steelMat);
  clawBase.position.y = -0.05;
  flange.add(clawBase);

  const fingerGeo = new THREE.BoxGeometry(0.1, 0.4, 0.15);
  
  clawL = new THREE.Mesh(fingerGeo, ironMat);
  clawL.position.set(-0.25, -0.25, 0);
  flange.add(clawL);

  clawR = new THREE.Mesh(fingerGeo, ironMat);
  clawR.position.set(0.25, -0.25, 0);
  flange.add(clawR);
}

function buildCassette() {
  // The cassette is a sliding track system behind the arm at Z = -12.5, sliding along X
  const trackMat = new THREE.MeshStandardMaterial({ color: 0x33333e, metalness: 0.8 });
  const carriageMat = new THREE.MeshStandardMaterial({ color: 0x222227, metalness: 0.8 });
  
  // Linear rail support pillars
  const pillarGeo = new THREE.CylinderGeometry(0.2, 0.2, 4.5, 16);
  const pillar1 = new THREE.Mesh(pillarGeo, trackMat);
  pillar1.position.set(-4.5, 2.25, -12.5);
  scene.add(pillar1);
  
  const pillar2 = new THREE.Mesh(pillarGeo, trackMat);
  pillar2.position.set(4.5, 2.25, -12.5);
  scene.add(pillar2);

  // The linear rail bar (along X-axis)
  const railGeo = new THREE.CylinderGeometry(0.18, 0.18, 9.8, 16);
  railGeo.rotateZ(Math.PI / 2);
  cassetteRail = new THREE.Mesh(railGeo, trackMat);
  cassetteRail.position.set(0.0, 4.5, -12.5);
  scene.add(cassetteRail);

  // Spool Rack Frame
  const rackMat = new THREE.MeshStandardMaterial({ color: 0x1e1e24, metalness: 0.9 });
  const rackGeo = new THREE.BoxGeometry(8.5, 6.0, 0.2);
  const rack = new THREE.Mesh(rackGeo, rackMat);
  rack.position.set(0.0, 3.0, -14.3);
  scene.add(rack);

  // Spool spindles / holders
  const spindleGeo = new THREE.CylinderGeometry(0.1, 0.1, 1.0, 12);
  spindleGeo.rotateZ(Math.PI / 2); // along X-axis

  // Build 4 filament spools (fixed rack behind Z = -12.5)
  for (let i = 0; i < 4; i++) {
    const sx = SLOT_LOCAL_XS[i];
    
    // Spindle
    const spin = new THREE.Mesh(spindleGeo, rackMat);
    spin.position.set(sx, 5.5, -13.7);
    scene.add(spin);

    // Filament Spool wheel
    const spoolGroup = new THREE.Group();
    spoolGroup.position.set(sx, 5.5, -13.7);
    
    // Inner core
    const coreGeo = new THREE.CylinderGeometry(0.6, 0.6, 0.4, 24);
    coreGeo.rotateZ(Math.PI / 2); // along X-axis
    const core = new THREE.Mesh(coreGeo, rackMat);
    spoolGroup.add(core);

    // Filament winding (colored cylinder)
    const filGeo = new THREE.CylinderGeometry(1.2, 1.2, 0.38, 24);
    filGeo.rotateZ(Math.PI / 2); // along X-axis
    const filMat = new THREE.MeshStandardMaterial({ 
      color: COLOR_HEXS[i], 
      roughness: 0.7 
    });
    const fil = new THREE.Mesh(filGeo, filMat);
    spoolGroup.add(fil);

    // Outer rims
    const rimGeo = new THREE.CylinderGeometry(1.35, 1.35, 0.02, 24);
    rimGeo.rotateZ(Math.PI / 2); // along X-axis
    const rimL = new THREE.Mesh(rimGeo, carriageMat);
    rimL.position.x = -0.21;
    spoolGroup.add(rimL);

    const rimR = new THREE.Mesh(rimGeo, carriageMat);
    rimR.position.x = 0.21;
    spoolGroup.add(rimR);

    // Dynamic rotation marker/label sticker on the outer rim face
    const labelGeo = new THREE.BoxGeometry(0.02, 0.5, 0.12);
    const labelMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const label = new THREE.Mesh(labelGeo, labelMat);
    label.position.set(0.22, 0.7, 0);
    spoolGroup.add(label);

    scene.add(spoolGroup);
    spools.push(spoolGroup); // to animate rotation
  }

  // The sliding carriage that moves along the rail (centered initially at X = 0)
  cassetteCarriage = new THREE.Group();
  cassetteCarriage.position.set(0.0, 4.5, -12.5);
  scene.add(cassetteCarriage);

  const carriageGeo = new THREE.BoxGeometry(7.8, 0.4, 0.8);
  const carriageMesh = new THREE.Mesh(carriageGeo, carriageMat);
  cassetteCarriage.add(carriageMesh);

  // Add 4 brackets/holders spaced at local X coords
  const bracketMat = new THREE.MeshStandardMaterial({ color: 0x44444c, metalness: 0.8 });
  const forkGeo = new THREE.BoxGeometry(0.7, 0.08, 0.5); // fork flat in X-Z
  
  for (let i = 0; i < 4; i++) {
    const slotX = SLOT_LOCAL_XS[i];
    
    // Create holder group
    const slotGroup = new THREE.Group();
    slotGroup.position.set(slotX, -0.2, 0); // sits at Y = 4.3 relative to carriage base
    cassetteCarriage.add(slotGroup);
    
    // Support arm from rail to fork: local Z = 0.75
    const supportArmGeo = new THREE.BoxGeometry(0.2, 0.08, 1.5);
    const supportArm = new THREE.Mesh(supportArmGeo, bracketMat);
    supportArm.position.z = 0.75;
    slotGroup.add(supportArm);
    
    // Visual fork at local Z = 1.5 (perfectly aligned with robot docking position when aligned)
    const fork = new THREE.Mesh(forkGeo, bracketMat);
    fork.position.z = 1.5; // extends towards robot
    slotGroup.add(fork);

    const colorTabGeo = new THREE.BoxGeometry(0.72, 0.1, 0.1);
    const colorTabMat = new THREE.MeshBasicMaterial({ color: COLOR_HEXS[i] });
    const colorTab = new THREE.Mesh(colorTabGeo, colorTabMat);
    colorTab.position.set(0, 0, 1.75);
    slotGroup.add(colorTab);

    // Instantiate toolhead object and place it initially in the slot group at local Z = 1.5
    const toolhead = new Toolhead(i, COLOR_HEXS[i]);
    toolhead.group.position.set(0, 0, 1.5);
    slotGroup.add(toolhead.group);
    
    toolheads.push(toolhead);
  }

  // Initialize Bowden tubes
  createBowdenTubes();
}

// ----------------------------------------------------
// TOOLHEAD MODEL CLASS
// ----------------------------------------------------
function Toolhead(colorIdx, colorHex) {
  this.colorIdx = colorIdx;
  this.colorHex = colorHex;
  this.group = new THREE.Group();

  const baseMat = new THREE.MeshStandardMaterial({ color: 0x1c1c20, metalness: 0.8, roughness: 0.3 });
  const steelMat = new THREE.MeshStandardMaterial({ color: 0x77777e, metalness: 0.9, roughness: 0.15 });

  // Collar (slides into cassette fork slot)
  const collarGeo = new THREE.CylinderGeometry(0.25, 0.25, 0.1, 16);
  const collar = new THREE.Mesh(collarGeo, steelMat);
  collar.position.y = -0.05;
  this.group.add(collar);

  // Flange connector pin (locks with robot claws)
  const pinGeo = new THREE.CylinderGeometry(0.15, 0.15, 0.12, 16);
  const pin = new THREE.Mesh(pinGeo, steelMat);
  pin.position.y = 0.06;
  this.group.add(pin);

  // Toolhead main block
  const bodyGeo = new THREE.BoxGeometry(0.5, 0.7, 0.5);
  const body = new THREE.Mesh(bodyGeo, baseMat);
  body.position.y = -0.45;
  body.castShadow = true;
  this.group.add(body);

  // Extruder fan casing
  const fanGeo = new THREE.BoxGeometry(0.2, 0.4, 0.4);
  const fan = new THREE.Mesh(fanGeo, new THREE.MeshStandardMaterial({ color: 0x333, roughness: 0.5 }));
  fan.position.set(0.2, -0.45, 0);
  this.group.add(fan);

  // Color stripe decal
  const stripeGeo = new THREE.BoxGeometry(0.52, 0.08, 0.52);
  const stripeMat = new THREE.MeshBasicMaterial({ color: colorHex });
  const stripe = new THREE.Mesh(stripeGeo, stripeMat);
  stripe.position.y = -0.45;
  this.group.add(stripe);

  // Brass heater block (at y = -0.85)
  const heaterGeo = new THREE.BoxGeometry(0.25, 0.15, 0.25);
  const heaterMat = new THREE.MeshStandardMaterial({ color: 0xd4af37, metalness: 0.9, roughness: 0.1 });
  const heater = new THREE.Mesh(heaterGeo, heaterMat);
  heater.position.y = -0.875;
  this.group.add(heater);

  // Glowing nozzle heater light (tiny sphere)
  const bulbGeo = new THREE.SphereGeometry(0.03, 8, 8);
  const bulbMat = new THREE.MeshBasicMaterial({ color: 0xff4400 });
  this.bulb = new THREE.Mesh(bulbGeo, bulbMat);
  this.bulb.position.set(0.1, -0.875, 0.1);
  this.group.add(this.bulb);

  // Copper nozzle (ends exactly at Y = -1.5)
  const nozzleGeo = new THREE.ConeGeometry(0.06, 0.2, 8);
  const nozzleMat = new THREE.MeshStandardMaterial({ color: 0xb87333, metalness: 0.9, roughness: 0.1 });
  
  const neckGeo = new THREE.CylinderGeometry(0.06, 0.06, 0.4, 8);
  const neck = new THREE.Mesh(neckGeo, steelMat);
  neck.position.y = -1.1; // down to -1.3
  this.group.add(neck);

  const nozzleCone = new THREE.Mesh(nozzleGeo, nozzleMat);
  nozzleCone.rotation.x = Math.PI; // point down
  nozzleCone.position.y = -1.4; // tip sits at local -1.5!
  this.group.add(nozzleCone);
}

// ----------------------------------------------------
// DYNAMIC FILAMENT BOWDEN TUBES
// ----------------------------------------------------
function createBowdenTubes() {
  const tubeMat0 = new THREE.LineBasicMaterial({ color: 0xff2a2a, linewidth: 2 });
  const tubeMat1 = new THREE.LineBasicMaterial({ color: 0x4af626, linewidth: 2 });
  const tubeMat2 = new THREE.LineBasicMaterial({ color: 0x00e5ff, linewidth: 2 });
  const tubeMat3 = new THREE.LineBasicMaterial({ color: 0xffb300, linewidth: 2 });
  const materials = [tubeMat0, tubeMat1, tubeMat2, tubeMat3];

  for (let i = 0; i < 4; i++) {
    const geom = new THREE.BufferGeometry();
    const positions = new Float32Array(20 * 3);
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const line = new THREE.Line(geom, materials[i]);
    scene.add(line);
    bowdenTubes.push(line);
  }
}

function updateBowdenTubes() {
  for (let i = 0; i < 4; i++) {
    const tube = bowdenTubes[i];
    const toolhead = toolheads[i];
    
    // Start point is the spool outlet (outer front-top edge of the winding)
    const pStart = new THREE.Vector3(SLOT_LOCAL_XS[i], 6.35, -12.85);

    // End point is the top of the toolhead
    const pEnd = new THREE.Vector3();
    toolhead.group.getWorldPosition(pEnd);
    pEnd.y += 0.2; // top entrance

    // Quadratic Bezier control point for droop
    const pMid = new THREE.Vector3().addVectors(pStart, pEnd).multiplyScalar(0.5);
    pMid.y += 2.0; // arch upwards
    pMid.z += 1.0; // arch along Z towards front

    // Generate points along curve
    const curve = new THREE.QuadraticBezierCurve3(pStart, pMid, pEnd);
    const points = curve.getPoints(19);

    const positionAttr = tube.geometry.attributes.position;
    for (let j = 0; j < 20; j++) {
      positionAttr.setXYZ(j, points[j].x, points[j].y, points[j].z);
    }
    positionAttr.needsUpdate = true;
  }
}

// ----------------------------------------------------
// FILAMENT PRINT EXTRUDER (InstancedMesh)
// ----------------------------------------------------
function createFilamentExtruder() {
  const cylGeo = new THREE.CylinderGeometry(1.0, 1.0, 1.0, 6);
  cylGeo.translate(0, 0.5, 0); // Origin at the bottom
  cylGeo.rotateX(Math.PI / 2);  // Align along local Z axis
  const cylMat = new THREE.MeshStandardMaterial({ 
    roughness: 0.6, 
    metalness: 0.1 
  });
  
  filamentMesh = new THREE.InstancedMesh(cylGeo, cylMat, MAX_FILAMENT_SEGMENTS);
  filamentMesh.castShadow = true;
  filamentMesh.receiveShadow = true;

  // Initialize instance colors so the shader compiles with instanced color support
  const defaultColor = new THREE.Color(0xffffff);
  for (let i = 0; i < MAX_FILAMENT_SEGMENTS; i++) {
    filamentMesh.setColorAt(i, defaultColor);
  }

  scene.add(filamentMesh);

  // Filament Line representing the dynamic extrudee segment currently printing
  const lineGeo = new THREE.BufferGeometry();
  const linePos = new Float32Array(2 * 3);
  lineGeo.setAttribute('position', new THREE.BufferAttribute(linePos, 3));
  const lineMat = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 3 });
  activeExtruderLine = new THREE.Line(lineGeo, lineMat);
  scene.add(activeExtruderLine);
}

function addPrintedSegment(p1, p2, colorHex) {
  if (currentFilamentCount >= MAX_FILAMENT_SEGMENTS) {
    logConsole("WARNING: Filament memory buffer full!", "error");
    return;
  }

  const dir = new THREE.Vector3().subVectors(p2, p1);
  const len = dir.length();
  if (len < 0.001) return;

  const center = p1.clone();
  const dirNorm = dir.clone().normalize();

  const dummy = new THREE.Object3D();
  dummy.position.copy(center);
  
  // Rotate cylinder (along Z-axis) to match target direction
  const alignQuat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), dirNorm);
  dummy.quaternion.copy(alignQuat);
  
  // Scale based on configured nozzle size (scaled down to 1/5th diameter)
  const nozzleRadius = modelConfig.nozzleSize * 0.125 * 0.2;
  dummy.scale.set(nozzleRadius, nozzleRadius, len);
  dummy.updateMatrix();

  filamentMesh.setMatrixAt(currentFilamentCount, dummy.matrix);
  filamentMesh.setColorAt(currentFilamentCount, new THREE.Color(colorHex));
  
  currentFilamentCount++;
  filamentMesh.instanceMatrix.needsUpdate = true;
  if (filamentMesh.instanceColor) {
    filamentMesh.instanceColor.needsUpdate = true;
  }

  document.getElementById('stat-filaments').innerText = `${currentFilamentCount} segments`;
}

function updateActiveLine(p1, p2, colorHex) {
  const positionAttr = activeExtruderLine.geometry.attributes.position;
  positionAttr.setXYZ(0, p1.x, p1.y, p1.z);
  positionAttr.setXYZ(1, p2.x, p2.y, p2.z);
  positionAttr.needsUpdate = true;
  activeExtruderLine.material.color.setHex(colorHex);
}

function hideActiveLine() {
  const positionAttr = activeExtruderLine.geometry.attributes.position;
  positionAttr.setXYZ(0, 0, -10, 0);
  positionAttr.setXYZ(1, 0, -10, 0);
  positionAttr.needsUpdate = true;
}

function clearFilament() {
  currentFilamentCount = 0;
  filamentMesh.count = 0;
  filamentMesh.count = MAX_FILAMENT_SEGMENTS;
  
  const dummy = new THREE.Object3D();
  dummy.position.set(0, -100, 0);
  dummy.updateMatrix();
  for (let i = 0; i < MAX_FILAMENT_SEGMENTS; i++) {
    filamentMesh.setMatrixAt(i, dummy.matrix);
  }
  filamentMesh.instanceMatrix.needsUpdate = true;
  
  document.getElementById('stat-filaments').innerText = `0 segments`;
  hideActiveLine();
}

// ----------------------------------------------------
// SLICER ENGINE
// ----------------------------------------------------
function getTrianglesFromGeometry(geo) {
  const posAttr = geo.attributes.position;
  const indexAttr = geo.index;
  const tris = [];

  if (indexAttr) {
    for (let i = 0; i < indexAttr.count; i += 3) {
      const i0 = indexAttr.getX(i);
      const i1 = indexAttr.getX(i + 1);
      const i2 = indexAttr.getX(i + 2);
      
      tris.push({
        a: new THREE.Vector3(posAttr.getX(i0), posAttr.getY(i0), posAttr.getZ(i0)),
        b: new THREE.Vector3(posAttr.getX(i1), posAttr.getY(i1), posAttr.getZ(i1)),
        c: new THREE.Vector3(posAttr.getX(i2), posAttr.getY(i2), posAttr.getZ(i2))
      });
    }
  } else {
    for (let i = 0; i < posAttr.count; i += 3) {
      tris.push({
        a: new THREE.Vector3(posAttr.getX(i), posAttr.getY(i), posAttr.getZ(i)),
        b: new THREE.Vector3(posAttr.getX(i + 1), posAttr.getY(i + 1), posAttr.getZ(i + 1)),
        c: new THREE.Vector3(posAttr.getX(i + 2), posAttr.getY(i + 2), posAttr.getZ(i + 2))
      });
    }
  }
  return tris;
}

function findFlattestNormal(tris) {
  if (tris.length === 0) return new THREE.Vector3(0, 1, 0);

  // 1. Calculate Center of Mass (CoM)
  const com = new THREE.Vector3(0, 0, 0);
  let totalArea = 0;
  for (const t of tris) {
    const ab = new THREE.Vector3().subVectors(t.b, t.a);
    const ac = new THREE.Vector3().subVectors(t.c, t.a);
    const area = 0.5 * new THREE.Vector3().crossVectors(ab, ac).length();
    if (area > 0.0001) {
      const center = new THREE.Vector3().addVectors(t.a, t.b).add(t.c).divideScalar(3);
      com.addScaledVector(center, area);
      totalArea += area;
    }
  }
  if (totalArea > 0) {
    com.divideScalar(totalArea);
  }

  // 2. Cluster normal axes to find dominant flat planes
  const clusters = [];
  const eps = 0.98; // ~11 degrees threshold
  for (const t of tris) {
    const ab = new THREE.Vector3().subVectors(t.b, t.a);
    const ac = new THREE.Vector3().subVectors(t.c, t.a);
    const cross = new THREE.Vector3().crossVectors(ab, ac);
    const area = 0.5 * cross.length();
    if (area < 0.0001) continue;
    const normal = cross.clone().normalize();

    let found = false;
    for (const c of clusters) {
      if (Math.abs(c.axis.dot(normal)) > eps) {
        c.area += area;
        found = true;
        break;
      }
    }
    if (!found) {
      clusters.push({
        axis: normal.clone(),
        area: area
      });
    }
  }

  // Sort clusters by area descending and take top 12 to keep search fast
  clusters.sort((a, b) => b.area - a.area);
  const activeClusters = clusters.slice(0, 12);

  // 3. Generate candidate directions to evaluate
  const candidates = [];
  for (const c of activeClusters) {
    candidates.push(c.axis.clone());
    candidates.push(c.axis.clone().negate());
  }

  // Add fallback standard directions
  const defaults = [
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(-1, 0, 0),
    new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, -1, 0),
    new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, -1)
  ];
  for (const d of defaults) {
    let near = false;
    for (const cand of candidates) {
      if (cand.dot(d) > 0.99) {
        near = true;
        break;
      }
    }
    if (!near) candidates.push(d);
  }

  // 4. Evaluate each candidate direction
  let bestDirection = null;
  let maxScore = -1;

  for (const d of candidates) {
    // Project vertices along candidate direction to find bounding range
    let minProj = Infinity;
    let maxProj = -Infinity;
    for (const t of tris) {
      for (const v of [t.a, t.b, t.c]) {
        const proj = v.dot(d);
        if (proj < minProj) minProj = proj;
        if (proj > maxProj) maxProj = proj;
      }
    }
    const modelHeight = maxProj - minProj;
    if (modelHeight < 0.001) continue;

    // Contact Area is area of triangles aligned with d and close to maxProj (bottom)
    let contactArea = 0;
    for (const t of tris) {
      const ab = new THREE.Vector3().subVectors(t.b, t.a);
      const ac = new THREE.Vector3().subVectors(t.c, t.a);
      const area = 0.5 * new THREE.Vector3().crossVectors(ab, ac).length();
      if (area < 0.0001) continue;
      const normal = new THREE.Vector3().crossVectors(ab, ac).normalize();

      if (normal.dot(d) > 0.98) {
        const centerProj = (t.a.dot(d) + t.b.dot(d) + t.c.dot(d)) / 3;
        // Check if the face is within 5% of the bottom-most boundary in this orientation
        if (maxProj - centerProj < 0.05 * modelHeight) {
          contactArea += area;
        }
      }
    }

    // CoM height above bottom (larger projection = closer to bottom/maxProj)
    const comHeight = maxProj - com.dot(d);
    const comHeightNorm = Math.max(0, Math.min(1.0, comHeight / modelHeight));

    // Physics Score: maximizes contact area, minimizes center of mass height
    const score = contactArea / (comHeightNorm + 0.05) + 0.001 / (comHeightNorm + 0.05);

    if (score > maxScore) {
      maxScore = score;
      bestDirection = d;
    }
  }

  return bestDirection ? bestDirection : new THREE.Vector3(0, 1, 0);
}

function processAndNormalizeTriangles(tris) {
  if (tris.length === 0) return [];
  
  const nBest = findFlattestNormal(tris);
  const targetNormal = new THREE.Vector3(0, -1, 0);
  const quat = new THREE.Quaternion().setFromUnitVectors(nBest, targetNormal);
  
  for (const t of tris) {
    t.a.applyQuaternion(quat);
    t.b.applyQuaternion(quat);
    t.c.applyQuaternion(quat);
  }
  
  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;
  
  for (const t of tris) {
    for (const v of [t.a, t.b, t.c]) {
      minX = Math.min(minX, v.x); maxX = Math.max(maxX, v.x);
      minY = Math.min(minY, v.y); maxY = Math.max(maxY, v.y);
      minZ = Math.min(minZ, v.z); maxZ = Math.max(maxZ, v.z);
    }
  }
  
  const sizeX = maxX - minX;
  const sizeY = maxY - minY;
  const sizeZ = maxZ - minZ;
  const maxDim = Math.max(sizeX, sizeY, sizeZ);
  
  const targetMaxDim = 5.0;
  const scale = targetMaxDim / (maxDim || 1.0);
  
  const centerX = (minX + maxX) / 2;
  const centerZ = (minZ + maxZ) / 2;

  for (const t of tris) {
    for (const v of [t.a, t.b, t.c]) {
      v.x = (v.x - centerX) * scale;
      v.y = (v.y - minY) * scale + 0.22; // Offset slightly above the print bed
      v.z = (v.z - centerZ) * scale;
    }
  }
  return tris;
}

function intersectTrianglePlane(tri, sliceY) {
  const pts = [];
  const edges = [
    [tri.a, tri.b],
    [tri.b, tri.c],
    [tri.c, tri.a]
  ];

  for (const edge of edges) {
    const v0 = edge[0];
    const v1 = edge[1];
    
    if ((v0.y < sliceY && v1.y >= sliceY) || (v1.y < sliceY && v0.y >= sliceY)) {
      const t = (sliceY - v0.y) / (v1.y - v0.y);
      const iPt = new THREE.Vector3().lerpVectors(v0, v1, t);
      pts.push(iPt);
    }
  }

  if (pts.length === 2) {
    return { a: pts[0], b: pts[1] };
  }
  return null;
}

function connectSegments(segments) {
  const paths = [];
  const eps = 0.08;
  const segs = [...segments];

  while (segs.length > 0) {
    const currPath = [segs[0].a, segs[0].b];
    segs.splice(0, 1);
    let found = true;

    while (found) {
      found = false;
      const endPoint = currPath[currPath.length - 1];
      
      for (let i = 0; i < segs.length; i++) {
        const s = segs[i];
        if (s.a.distanceTo(endPoint) < eps) {
          currPath.push(s.b);
          segs.splice(i, 1);
          found = true;
          break;
        } else if (s.b.distanceTo(endPoint) < eps) {
          currPath.push(s.a);
          segs.splice(i, 1);
          found = true;
          break;
        }
      }
    }
    paths.push(currPath);
  }
  return paths;
}

// Progressive Slicing Routine
function startProgressiveSlice(numLayers, onComplete) {
  isSlicing = true;
  isPrinting = false;
  document.getElementById('slicing-modal').style.display = 'flex';
  
  let layerIdx = 0;
  const slicedLayers = [];
  
  logConsole(`Slicing sequence initialized. Triangles: ${stlTriangles.length}. layers: ${numLayers}`, "action");

  function sliceNextBatch() {
    if (!isSlicing) return; // aborted

    const batchSize = 2; // slice 2 layers per frame to keep UI active
    for (let b = 0; b < batchSize && layerIdx < numLayers; b++) {
      const heightProgress = (layerIdx + 0.5) / numLayers;
      const sliceY = 0.22 + heightProgress * 5.0; 
      
      const rawSegments = [];
      for (const tri of stlTriangles) {
        const seg = intersectTrianglePlane(tri, sliceY);
        if (seg) rawSegments.push(seg);
      }

      const contours = connectSegments(rawSegments);
      slicedLayers.push({ y: sliceY, paths: contours });
      
      layerIdx++;
    }

    const pct = Math.floor((layerIdx / numLayers) * 100);
    document.getElementById('slicing-bar-fill').style.width = `${pct}%`;
    document.getElementById('slicing-percentage').innerText = `${pct}%`;

    if (layerIdx < numLayers) {
      requestAnimationFrame(sliceNextBatch);
    } else {
      isSlicing = false;
      document.getElementById('slicing-modal').style.display = 'none';
      logConsole("Geometry slicing complete. Generating printqueue...", "success");
      
      // Generate G-code queue
      lastSlicedLayers = slicedLayers;
      generatePrintQueue(slicedLayers);
      onComplete();
    }
  }
  
  sliceNextBatch();
}

function generatePrintQueue(slicedLayers) {
  printQueue = [];
  let lastColorIdx = -1;

  const singleColorIdx = (currentToolIdx !== -1) ? currentToolIdx : 0;

  for (let l = 0; l < slicedLayers.length; l++) {
    const layer = slicedLayers[l];
    
    let colorIdx;
    if (modelConfig.colorMode === 'single') {
      colorIdx = singleColorIdx;
    } else {
      colorIdx = Math.floor((l / slicedLayers.length) * 4);
      colorIdx = Math.min(3, Math.max(0, colorIdx));
    }

    if (colorIdx !== lastColorIdx) {
      printQueue.push({
        type: 'TOOLCHANGE',
        colorIdx: colorIdx
      });
      lastColorIdx = colorIdx;
    }

    for (const path of layer.paths) {
      if (path.length < 2) continue;

      printQueue.push({
        type: 'TRAVEL',
        pos: path[0].clone()
      });

      for (let p = 1; p < path.length; p++) {
        printQueue.push({
          type: 'PRINT',
          pos: path[p].clone(),
          colorIdx: colorIdx
        });
      }
    }
  }

  queueIndex = 0;
  logConsole(`PrintQueue compiled: ${printQueue.length} operations. Ready.`, "success");
  document.getElementById('stat-progress').innerText = "0.0%";
}

// ----------------------------------------------------
// MODEL CONFIGURATION & HOLOGRAM TRANSFORMATIONS
// ----------------------------------------------------
function getTransformedTriangles() {
  const transformed = [];
  const s = modelConfig.scale;
  const tx = modelConfig.x;
  const tz = modelConfig.z;
  const euler = new THREE.Euler(modelConfig.rotX, modelConfig.rotY, modelConfig.rotZ, 'XYZ');
  
  for (const t of baseTriangles) {
    const a = t.a.clone();
    const b = t.b.clone();
    const c = t.c.clone();
    
    for (const v of [a, b, c]) {
      const vCopy = new THREE.Vector3(v.x, v.y - 0.22, v.z);
      vCopy.applyEuler(euler);
      vCopy.multiplyScalar(s);
      v.x = vCopy.x + tx;
      v.y = vCopy.y + 0.22;
      v.z = vCopy.z + tz;
    }
    
    transformed.push({ a, b, c });
  }
  return transformed;
}

function updateHologramGeometry(geom) {
  if (hologramMesh) {
    scene.remove(hologramMesh);
  }
  if (!geom) return;
  
  const normalizedGeom = geom.clone();
  
  // 1. Find flattest normal of raw geometry
  const rawTris = getTrianglesFromGeometry(geom);
  const nBest = findFlattestNormal(rawTris);
  const targetNormal = new THREE.Vector3(0, -1, 0);
  const quat = new THREE.Quaternion().setFromUnitVectors(nBest, targetNormal);
  
  // 2. Rotate geometry to align flattest face to bottom (FIXED: using applyMatrix4 instead of applyQuaternion)
  normalizedGeom.applyMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(quat));
  
  // 3. Normalize bounding box and position
  normalizedGeom.computeBoundingBox();
  const bbox = normalizedGeom.boundingBox;
  const sizeX = bbox.max.x - bbox.min.x;
  const sizeY = bbox.max.y - bbox.min.y;
  const sizeZ = bbox.max.z - bbox.min.z;
  const maxDim = Math.max(sizeX, sizeY, sizeZ);
  const scale = 5.0 / (maxDim || 1.0);
  
  const centerX = (bbox.min.x + bbox.max.x) / 2;
  const centerZ = (bbox.min.z + bbox.max.z) / 2;
  
  normalizedGeom.translate(-centerX, -bbox.min.y, -centerZ);
  normalizedGeom.scale(scale, scale, scale);
  
  hologramMesh = new THREE.Group();
  
  const wireframeMat = new THREE.MeshBasicMaterial({
    color: 0x00e5ff,
    wireframe: true,
    transparent: true,
    opacity: 0.15,
    blending: THREE.AdditiveBlending
  });
  const wireMesh = new THREE.Mesh(normalizedGeom, wireframeMat);
  hologramMesh.add(wireMesh);
  
  const solidMat = new THREE.MeshBasicMaterial({
    color: 0x00e5ff,
    transparent: true,
    opacity: 0.04,
    depthWrite: false,
    blending: THREE.AdditiveBlending
  });
  const solidMesh = new THREE.Mesh(normalizedGeom, solidMat);
  hologramMesh.add(solidMesh);
  
  applyConfigTransformsToHologram();
  scene.add(hologramMesh);
}

function applyConfigTransformsToHologram() {
  if (!hologramMesh) return;
  hologramMesh.position.set(modelConfig.x, 0.22, modelConfig.z);
  hologramMesh.rotation.set(modelConfig.rotX, modelConfig.rotY, modelConfig.rotZ, 'XYZ');
  hologramMesh.scale.set(modelConfig.scale, modelConfig.scale, modelConfig.scale);
}

function reSliceModel() {
  if (baseTriangles.length === 0) return;
  
  clearFilament();
  stlTriangles = getTransformedTriangles();
  
  const numLayers = parseInt(document.getElementById('slider-resolution').value);
  startProgressiveSlice(numLayers, () => {
    pausePrint();
  });
}

function updateUIStates() {
  const isEditable = (atcState === 'IDLE' && !isPrinting && currentFilamentCount === 0);
  
  document.getElementById('slider-pos-x').disabled = !isEditable;
  document.getElementById('slider-pos-z').disabled = !isEditable;
  document.getElementById('slider-rot-x').disabled = !isEditable;
  document.getElementById('slider-rot-y').disabled = !isEditable;
  document.getElementById('slider-rot-z').disabled = !isEditable;
  document.getElementById('btn-toggle-snap').disabled = !isEditable;
  document.getElementById('slider-scale').disabled = !isEditable;
  document.getElementById('slider-nozzle').disabled = !isEditable;
  document.getElementById('btn-color-single').disabled = !isEditable;
  document.getElementById('btn-color-multi').disabled = !isEditable;
  
  const canSlice = (atcState === 'IDLE' && !isPrinting);
  document.getElementById('slider-resolution').disabled = !canSlice;
  document.getElementById('btn-browse').disabled = !canSlice;
  document.getElementById('btn-model-vase').disabled = !canSlice;
  document.getElementById('btn-model-gear').disabled = !canSlice;
  document.getElementById('btn-model-knot').disabled = !canSlice;
}

// ----------------------------------------------------
// BUILT-IN PROCEDURAL MODELS GENERATOR
// ----------------------------------------------------
function loadBuiltInModel(type) {
  currentModelType = type;
  clearFilament();
  
  document.getElementById('btn-model-vase').className = type === 'vase' ? 'active' : '';
  document.getElementById('btn-model-gear').className = type === 'gear' ? 'active' : '';
  document.getElementById('btn-model-knot').className = type === 'knot' ? 'active' : '';

  let geom;

  if (type === 'vase') {
    geom = new THREE.CylinderGeometry(1.6, 1.3, 5.0, 32, 40, false);
    const pos = geom.attributes.position;
    for (let i = 0; i < pos.count; i++) {
      let x = pos.getX(i);
      let y = pos.getY(i);
      let z = pos.getZ(i);
      
      let h = y + 2.5; 
      let theta = Math.atan2(z, x);
      
      let radFactor = 1.0 + 0.3 * Math.sin((h / 5.0) * Math.PI * 1.5) + 0.08 * Math.sin(7 * theta);
      pos.setXYZ(i, x * radFactor, y, z * radFactor);
    }
    geom.computeVertexNormals();
    logConsole("Vase prototype generated programmatically.");
  } 
  else if (type === 'gear') {
    geom = new THREE.CylinderGeometry(1.5, 1.5, 5.0, 48, 40, false);
    const pos = geom.attributes.position;
    for (let i = 0; i < pos.count; i++) {
      let x = pos.getX(i);
      let y = pos.getY(i);
      let z = pos.getZ(i);
      
      let h = y + 2.5; 
      let theta = Math.atan2(z, x);
      
      let radFactor = 0.95 + 0.2 * Math.cos(6 * theta - (h / 5.0) * Math.PI * 2.0);
      pos.setXYZ(i, x * radFactor, y, z * radFactor);
    }
    geom.computeVertexNormals();
    logConsole("Gear pillar prototype generated programmatically.");
  } 
  else if (type === 'knot') {
    geom = new THREE.TorusKnotGeometry(1.2, 0.45, 150, 16);
    geom.computeBoundingBox();
    const yMin = geom.boundingBox.min.y;
    geom.translate(0, -yMin, 0);
    logConsole("TorusKnot prototype generated programmatically.");
  }

  currentGeometry = geom;
  const rawTris = getTrianglesFromGeometry(geom);
  baseTriangles = processAndNormalizeTriangles(rawTris);
  
  updateHologramGeometry(geom);
  reSliceModel();
  updateUIStates();
}

// ----------------------------------------------------
// SIMULATION STATE MACHINE (TICK & PHYSICS)
// ----------------------------------------------------
let currentSegmentStart = null;
let targetMovePos = null;
let moveProgress = 0;
let currentMoveType = 'TRAVEL';
let currentMoveColorIdx = 0;

function animate() {
  requestAnimationFrame(animate);

  if (isPrinting && !isPaused && atcState === 'IDLE') {
    processPrintExecution();
  }

  if (atcState !== 'IDLE') {
    processToolChangeSequence();
  }

  interpolateJoints();

  if (Math.abs(currentCarriageX - targetCarriageX) > 0.005) {
    currentCarriageX += (targetCarriageX - currentCarriageX) * 0.08 * simSpeed;
    cassetteCarriage.position.x = currentCarriageX;
    document.getElementById('stat-atc-pos').innerText = `X: ${currentCarriageX.toFixed(2)}`;
  }

  updateBowdenTubes();

  controls.update();
  renderer.render(scene, camera);
}

function processPrintExecution() {
  const loopsPerFrame = Math.max(1, Math.floor(simSpeed));
  
  for (let run = 0; run < loopsPerFrame; run++) {
    if (queueIndex >= printQueue.length) {
      isPrinting = false;
      logConsole("PRINT OPERATION COMPLETE.", "success");
      logConsole("Parking robot flange.", "action");
      
      tcpTargetPos.set(0, 4.0, 0.0);
      document.getElementById('btn-play-pause').innerText = '► START PRINT';
      break;
    }

    if (targetMovePos === null) {
      const cmd = printQueue[queueIndex];
      
      if (cmd.type === 'TOOLCHANGE') {
        const started = triggerToolChange(cmd.colorIdx);
        queueIndex++;
        if (started) {
          break; 
        }
      }
      
      currentMoveType = cmd.type;
      currentMoveColorIdx = cmd.colorIdx;
      currentSegmentStart = tcpTargetPos.clone();
      targetMovePos = cmd.pos.clone();
      moveProgress = 0;
      
      queueIndex++;
      
      const progressPct = ((queueIndex / printQueue.length) * 100).toFixed(1);
      document.getElementById('stat-progress').innerText = `${progressPct}%`;
    }

    const stepSize = currentMoveType === 'TRAVEL' ? 0.4 : 0.08;
    const dist = currentSegmentStart.distanceTo(targetMovePos);
    
    if (dist < 0.01) {
      tcpTargetPos.copy(targetMovePos);
      targetMovePos = null;
    } else {
      moveProgress += (stepSize * Math.min(2.0, simSpeed)) / dist;
      if (moveProgress >= 1.0) {
        tcpTargetPos.copy(targetMovePos);
        
        if (currentMoveType === 'PRINT' && activeToolhead) {
          const colorIdx = (modelConfig.colorMode === 'single' && currentToolIdx !== -1) ? currentToolIdx : currentMoveColorIdx;
          addPrintedSegment(currentSegmentStart, targetMovePos, COLOR_HEXS[colorIdx]);
          spools[colorIdx].rotation.x += 0.05;
        }
        
        targetMovePos = null;
        hideActiveLine();
      } else {
        const interpPos = new THREE.Vector3().lerpVectors(currentSegmentStart, targetMovePos, moveProgress);
        tcpTargetPos.copy(interpPos);
        
        if (currentMoveType === 'PRINT' && activeToolhead) {
          const colorIdx = (modelConfig.colorMode === 'single' && currentToolIdx !== -1) ? currentToolIdx : currentMoveColorIdx;
          updateActiveLine(currentSegmentStart, tcpTargetPos, COLOR_HEXS[colorIdx]);
        }
      }
      break; 
    }
  }
}

// ----------------------------------------------------
// TOOL CHANGE AUTOMATED SEQUENCE
// ----------------------------------------------------
function triggerToolChange(colorIdx) {
  if (currentToolIdx === colorIdx) {
    logConsole(`Toolhead T${colorIdx} (${COLOR_NAMES[colorIdx]}) is already active. Skipping toolchange.`);
    return false;
  }
  
  targetToolIdx = colorIdx;
  atcState = 'DOCK_APPROACH';
  atcProgress = 0;
  toolchangePrevPos.copy(tcpTargetPos);
  toolchangeReturnIndex = queueIndex;
  
  logConsole(`Color change requested -> Toolhead T${colorIdx} (${COLOR_NAMES[colorIdx]})`, "action");
  hideActiveLine(); // Hide extruder line during tool change
  return true;
}

function processToolChangeSequence() {
  const speedCoeff = 0.06 * Math.min(2.0, simSpeed);

  switch (atcState) {
    case 'DOCK_APPROACH':
      if (currentToolIdx === -1) {
        targetCarriageX = -SLOT_LOCAL_XS[targetToolIdx];
        atcState = 'SLIDE_CASSETTE';
        logConsole(`Initial tool pickup. Aligning Slot ${targetToolIdx} (${COLOR_NAMES[targetToolIdx]}).`);
        break;
      }

      const curSlotX = SLOT_LOCAL_XS[currentToolIdx];
      const tApproach = new THREE.Vector3(DOCK_X, DOCK_Y, APPROACH_Z);
      
      tcpTargetPos.lerp(tApproach, speedCoeff);
      if (tcpTargetPos.distanceTo(tApproach) < 0.05) {
        tcpTargetPos.copy(tApproach);
        atcState = 'DOCK_INSERT';
        logConsole("Dock approach complete. Inserting toolhead into bracket.");
      }
      break;

    case 'DOCK_INSERT':
      const tInsert = new THREE.Vector3(DOCK_X, DOCK_Y, DOCK_Z);
      
      tcpTargetPos.lerp(tInsert, speedCoeff);
      if (tcpTargetPos.distanceTo(tInsert) < 0.05) {
        tcpTargetPos.copy(tInsert);
        atcState = 'DOCK_RELEASE';
        atcProgress = 0;
      }
      break;

    case 'DOCK_RELEASE':
      atcProgress += speedCoeff * 5;
      clawL.position.z = -0.25 - 0.15 * Math.min(1.0, atcProgress);
      clawR.position.z = 0.25 + 0.15 * Math.min(1.0, atcProgress);
      
      if (atcProgress >= 1.0) {
        dockActiveToolhead();
        atcState = 'FLANGE_RETRACT';
      }
      break;

    case 'FLANGE_RETRACT':
      const tLift = new THREE.Vector3(DOCK_X, 4.0, DOCK_Z);
      
      tcpTargetPos.lerp(tLift, speedCoeff);
      if (tcpTargetPos.distanceTo(tLift) < 0.05) {
        tcpTargetPos.copy(tLift);
        atcState = 'SLIDE_CASSETTE';
        targetCarriageX = -SLOT_LOCAL_XS[targetToolIdx];
        logConsole(`Sliding ATC cassette. Aligning Slot ${targetToolIdx} (${COLOR_NAMES[targetToolIdx]}) to pickup axis.`, "action");
      }
      break;

    case 'SLIDE_CASSETTE':
      if (Math.abs(currentCarriageX - targetCarriageX) < 0.01) {
        currentCarriageX = targetCarriageX;
        atcState = 'FLANGE_DESCEND';
        logConsole("ATC Cassette aligned. Descending flange to engage.");
      }
      break;

    case 'FLANGE_DESCEND':
      const tDescend = new THREE.Vector3(DOCK_X, DOCK_Y, DOCK_Z);
      
      tcpTargetPos.lerp(tDescend, speedCoeff);
      if (tcpTargetPos.distanceTo(tDescend) < 0.05) {
        tcpTargetPos.copy(tDescend);
        atcState = 'LOCK_GRIP';
        atcProgress = 0;
      }
      break;

    case 'LOCK_GRIP':
      atcProgress += speedCoeff * 5;
      clawL.position.z = -0.4 + 0.15 * Math.min(1.0, atcProgress);
      clawR.position.z = 0.4 - 0.15 * Math.min(1.0, atcProgress);
      
      if (atcProgress >= 1.0) {
        pickupTargetToolhead();
        atcState = 'RETRACT_DOCK';
      }
      break;

    case 'RETRACT_DOCK':
      const tRetract = new THREE.Vector3(DOCK_X, DOCK_Y, APPROACH_Z);
      
      tcpTargetPos.lerp(tRetract, speedCoeff);
      if (tcpTargetPos.distanceTo(tRetract) < 0.05) {
        tcpTargetPos.copy(tRetract);
        atcState = 'COMPLETE';
      }
      break;

    case 'COMPLETE':
      tcpTargetPos.copy(toolchangePrevPos);
      atcState = 'IDLE';
      logConsole("ATC Toolchange cycle complete. Resuming print.", "success");
      break;
  }
}

function dockActiveToolhead() {
  if (currentToolIdx === -1) return;
  const th = toolheads[currentToolIdx];
  const slotGroup = cassetteCarriage.children[1 + currentToolIdx];

  flange.remove(th.group);
  slotGroup.add(th.group);
  th.group.position.set(0, 0, 1.5);
  th.group.rotation.set(0, 0, 0);
  
  activeToolhead = null;
  currentToolIdx = -1;
  
  document.getElementById('stat-toolhead').innerText = "NONE";
  document.getElementById('stat-toolhead').style.color = "var(--border-color)";

  // Remove active highlight from all manual toolhead buttons
  for (let i = 0; i < 4; i++) {
    const btn = document.getElementById(`btn-tool-${i}`);
    if (btn) {
      btn.style.backgroundColor = '';
      btn.style.color = '';
      btn.style.fontWeight = '';
    }
  }
}

function pickupTargetToolhead() {
  currentToolIdx = targetToolIdx;
  const th = toolheads[currentToolIdx];
  const slotGroup = cassetteCarriage.children[1 + currentToolIdx];

  slotGroup.remove(th.group);
  flange.add(th.group);
  th.group.position.set(0, 0, 0);
  th.group.rotation.set(0, 0, 0);

  activeToolhead = th;

  document.getElementById('stat-toolhead').innerText = COLOR_NAMES[currentToolIdx];
  document.getElementById('stat-toolhead').style.color = `#${COLOR_HEXS[currentToolIdx].toString(16).padStart(6, '0')}`;
  logConsole(`Flange coupled with toolhead T${currentToolIdx}.`, "success");

  // Highlight the active manual toolhead button with its filament color
  for (let i = 0; i < 4; i++) {
    const btn = document.getElementById(`btn-tool-${i}`);
    if (btn) {
      if (i === currentToolIdx) {
        btn.style.backgroundColor = `#${COLOR_HEXS[i].toString(16).padStart(6, '0')}`;
        btn.style.color = '#000';
        btn.style.fontWeight = 'bold';
      } else {
        btn.style.backgroundColor = '';
        btn.style.color = '';
        btn.style.fontWeight = '';
      }
    }
  }

  // Prevent resetting print queue index mid-print
  if (modelConfig.colorMode === 'single' && lastSlicedLayers.length > 0 && !isPrinting) {
    generatePrintQueue(lastSlicedLayers);
  }
}

// ----------------------------------------------------
// ROBOT JOINT SMOOTHING PHYSICS
// ----------------------------------------------------
function interpolateJoints() {
  const ik = solveIK(tcpTargetPos.x, tcpTargetPos.y, tcpTargetPos.z);
  
  const warningBadge = document.getElementById('warning-badge');
  if (ik.outOfRange) {
    warningBadge.style.display = 'flex';
    document.getElementById('warning-text').innerText = "KINEMATIC LIMIT EXCEEDED";
  } else {
    warningBadge.style.display = 'none';
  }

  targetTheta[0] = ik.theta1;
  targetTheta[1] = ik.theta2;
  targetTheta[2] = ik.theta3;
  targetTheta[3] = ik.theta4;
  targetTheta[4] = ik.theta5;
  targetTheta[5] = ik.theta6;

  const lerpFactor = 0.15 * Math.min(2.0, simSpeed);

  for (let i = 0; i < 6; i++) {
    actualTheta[i] += (targetTheta[i] - actualTheta[i]) * lerpFactor;
  }

  joint1.rotation.y = actualTheta[0];
  joint2.rotation.z = actualTheta[1]; 
  joint3.rotation.z = actualTheta[2];
  joint4.rotation.y = actualTheta[3]; 
  joint5.rotation.z = actualTheta[4]; 
  joint6.rotation.x = actualTheta[5]; 

  document.getElementById('j1-val').innerText = `${(actualTheta[0] * 180 / Math.PI).toFixed(1)}°`;
  document.getElementById('j2-val').innerText = `${(actualTheta[1] * 180 / Math.PI).toFixed(1)}°`;
  document.getElementById('j3-val').innerText = `${(actualTheta[2] * 180 / Math.PI).toFixed(1)}°`;
  document.getElementById('j4-val').innerText = `${(actualTheta[3] * 180 / Math.PI).toFixed(1)}°`;
  document.getElementById('j5-val').innerText = `${(actualTheta[4] * 180 / Math.PI).toFixed(1)}°`;
  document.getElementById('j6-val').innerText = `${(actualTheta[5] * 180 / Math.PI).toFixed(1)}°`;

  document.getElementById('j1-bar').style.width = `${Math.min(100, Math.max(0, (actualTheta[0] + Math.PI) / (2 * Math.PI) * 100))}%`;
  document.getElementById('j2-bar').style.width = `${Math.min(100, Math.max(0, (actualTheta[1] + Math.PI) / (2 * Math.PI) * 100))}%`;
  document.getElementById('j3-bar').style.width = `${Math.min(100, Math.max(0, (actualTheta[2] + Math.PI) / (2 * Math.PI) * 100))}%`;
  document.getElementById('j4-bar').style.width = `${Math.min(100, Math.max(0, (actualTheta[3] + Math.PI) / (2 * Math.PI) * 100))}%`;
  document.getElementById('j5-bar').style.width = `${Math.min(100, Math.max(0, (actualTheta[4] + Math.PI) / (2 * Math.PI) * 100))}%`;
  document.getElementById('j6-bar').style.width = `${Math.min(100, Math.max(0, (actualTheta[5] + Math.PI) / (2 * Math.PI) * 100))}%`;

  document.getElementById('coord-x').innerText = tcpTargetPos.x.toFixed(2);
  document.getElementById('coord-y').innerText = tcpTargetPos.y.toFixed(2);
  document.getElementById('coord-z').innerText = tcpTargetPos.z.toFixed(2);
}

// ----------------------------------------------------
// UI HANDLERS & LISTENERS
// ----------------------------------------------------
function setupUIEventListeners() {
  const posXSlider = document.getElementById('slider-pos-x');
  const posZSlider = document.getElementById('slider-pos-z');
  const posVal = document.getElementById('pos-val');

  function updateModelPos() {
    modelConfig.x = parseFloat(posXSlider.value);
    modelConfig.z = parseFloat(posZSlider.value);
    posVal.innerText = `X: ${modelConfig.x.toFixed(1)}, Z: ${modelConfig.z.toFixed(1)}`;
    applyConfigTransformsToHologram();
  }

  posXSlider.addEventListener('input', updateModelPos);
  posZSlider.addEventListener('input', updateModelPos);
  
  posXSlider.addEventListener('change', () => {
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });
  posZSlider.addEventListener('change', () => {
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });

  const rotXSlider = document.getElementById('slider-rot-x');
  const rotYSlider = document.getElementById('slider-rot-y');
  const rotZSlider = document.getElementById('slider-rot-z');
  const rotVal = document.getElementById('rot-val');

  function updateModelRotations() {
    const degX = parseInt(rotXSlider.value);
    const degY = parseInt(rotYSlider.value);
    const degZ = parseInt(rotZSlider.value);
    
    modelConfig.rotX = degX * Math.PI / 180;
    modelConfig.rotY = degY * Math.PI / 180;
    modelConfig.rotZ = degZ * Math.PI / 180;
    
    rotVal.innerText = `X: ${degX}°, Y: ${degY}°, Z: ${degZ}°`;
    applyConfigTransformsToHologram();
  }

  rotXSlider.addEventListener('input', updateModelRotations);
  rotYSlider.addEventListener('input', updateModelRotations);
  rotZSlider.addEventListener('input', updateModelRotations);
  
  rotXSlider.addEventListener('change', () => {
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });
  rotYSlider.addEventListener('change', () => {
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });
  rotZSlider.addEventListener('change', () => {
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });

  const snapBtn = document.getElementById('btn-toggle-snap');
  snapBtn.addEventListener('click', () => {
    modelConfig.snapRot = !modelConfig.snapRot;
    if (modelConfig.snapRot) {
      snapBtn.className = 'active';
      snapBtn.innerText = 'SNAP ON';
      
      rotXSlider.setAttribute('step', '90');
      rotYSlider.setAttribute('step', '90');
      rotZSlider.setAttribute('step', '90');
      
      const snapValue = (val) => Math.round(val / 90) * 90;
      rotXSlider.value = snapValue(parseInt(rotXSlider.value));
      rotYSlider.value = snapValue(parseInt(rotYSlider.value));
      rotZSlider.value = snapValue(parseInt(rotZSlider.value));
    } else {
      snapBtn.className = '';
      snapBtn.innerText = 'SNAP OFF';
      
      rotXSlider.setAttribute('step', '5');
      rotYSlider.setAttribute('step', '5');
      rotZSlider.setAttribute('step', '5');
    }
    updateModelRotations();
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });

  const scaleSlider = document.getElementById('slider-scale');
  const scaleVal = document.getElementById('scale-val');
  
  scaleSlider.addEventListener('input', () => {
    modelConfig.scale = parseFloat(scaleSlider.value);
    scaleVal.innerText = `${modelConfig.scale.toFixed(2)}x`;
    applyConfigTransformsToHologram();
  });
  
  scaleSlider.addEventListener('change', () => {
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });

  const nozzleSlider = document.getElementById('slider-nozzle');
  const nozzleVal = document.getElementById('nozzle-val');
  
  nozzleSlider.addEventListener('input', () => {
    modelConfig.nozzleSize = parseFloat(nozzleSlider.value);
    nozzleVal.innerText = `${modelConfig.nozzleSize.toFixed(1)} mm`;

    // Automatically adjust resolution layers based on nozzle size (thinner nozzle -> more layers)
    const calculatedLayers = Math.round(100 / modelConfig.nozzleSize);
    const resSlider = document.getElementById('slider-resolution');
    resSlider.max = calculatedLayers; // Dynamically adjust max limit as needed
    resSlider.value = calculatedLayers;
    document.getElementById('resolution-val').innerText = `${calculatedLayers} Layers`;
  });
  
  nozzleSlider.addEventListener('change', () => {
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });

  const btnColorSingle = document.getElementById('btn-color-single');
  const btnColorMulti = document.getElementById('btn-color-multi');

  btnColorSingle.addEventListener('click', () => {
    if (modelConfig.colorMode === 'single') return;
    modelConfig.colorMode = 'single';
    btnColorSingle.className = 'active';
    btnColorMulti.className = '';
    logConsole("Color mode set to SINGLE COLOR.");
    if (lastSlicedLayers.length > 0) {
      generatePrintQueue(lastSlicedLayers);
    }
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });

  btnColorMulti.addEventListener('click', () => {
    if (modelConfig.colorMode === 'multi') return;
    modelConfig.colorMode = 'multi';
    btnColorMulti.className = 'active';
    btnColorSingle.className = '';
    logConsole("Color mode set to MULTI-COLOR.");
    if (!isPrinting && atcState === 'IDLE') reSliceModel();
  });

  const hologramBtn = document.getElementById('btn-toggle-hologram');
  hologramBtn.addEventListener('click', () => {
    if (hologramMesh) {
      hologramMesh.visible = !hologramMesh.visible;
      hologramBtn.className = hologramMesh.visible ? 'active' : '';
      hologramBtn.innerText = hologramMesh.visible ? 'HOLOGRAM ON' : 'HOLOGRAM OFF';
    }
  });

  const playBtn = document.getElementById('btn-play-pause');
  playBtn.addEventListener('click', () => {
    if (!isPrinting) {
      isPrinting = true;
      isPaused = false;
      playBtn.innerText = '⏸ PAUSE PRINT';
      logConsole("Extrusion stream print session started.", "success");
    } else {
      isPaused = !isPaused;
      playBtn.innerText = isPaused ? '▶ RESUME PRINT' : '⏸ PAUSE PRINT';
      logConsole(isPaused ? "Print simulation suspended." : "Print simulation resumed.");
    }
    updateUIStates();
  });

  document.getElementById('btn-reset').addEventListener('click', () => {
    resetPrint();
  });

  const speedSlider = document.getElementById('slider-speed');
  speedSlider.addEventListener('input', () => {
    simSpeed = parseFloat(speedSlider.value);
    document.getElementById('speed-val').innerText = `${simSpeed.toFixed(1)}x`;
  });

  const resSlider = document.getElementById('slider-resolution');
  resSlider.addEventListener('input', () => {
    const layers = parseInt(resSlider.value);
    document.getElementById('resolution-val').innerText = `${layers} Layers`;
    
    if (!isPrinting && atcState === 'IDLE') {
      loadBuiltInModel(currentModelType);
    }
  });

  document.getElementById('btn-model-vase').addEventListener('click', () => {
    if (atcState === 'IDLE') loadBuiltInModel('vase');
  });
  document.getElementById('btn-model-gear').addEventListener('click', () => {
    if (atcState === 'IDLE') loadBuiltInModel('gear');
  });
  document.getElementById('btn-model-knot').addEventListener('click', () => {
    if (atcState === 'IDLE') loadBuiltInModel('knot');
  });

  document.getElementById('btn-cam-reset').addEventListener('click', () => {
    resetCamera();
  });

  for (let i = 0; i < 4; i++) {
    document.getElementById(`btn-tool-${i}`).addEventListener('click', () => {
      if (atcState === 'IDLE') {
        triggerToolChange(i);
      } else {
        logConsole("WARNING: ATC active. Clear sequence first.", "error");
      }
    });
  }

  const fileInput = document.getElementById('stl-file');
  document.getElementById('btn-browse').addEventListener('click', () => {
    if (atcState !== 'IDLE') {
      logConsole("WARNING: Clear ATC sequence first.", "error");
      return;
    }
    fileInput.click();
  });

  fileInput.addEventListener('change', (e) => {
    const file = e.target.files[0];
    if (!file) return;

    logConsole(`Custom STL loaded: ${file.name}. Reading...`, "action");

    const reader = new FileReader();
    reader.onload = function (event) {
      try {
        const loader = new THREE.STLLoader();
        const geom = loader.parse(event.target.result);
        
        clearFilament();
        currentModelType = 'custom';
        
        document.getElementById('btn-model-vase').className = '';
        document.getElementById('btn-model-gear').className = '';
        document.getElementById('btn-model-knot').className = '';

        currentGeometry = geom;
        baseTriangles = processAndNormalizeTriangles(getTrianglesFromGeometry(geom));
        
        logConsole(`Parsed ${baseTriangles.length} triangles successfully. Triggering slicer...`, "success");

        updateHologramGeometry(geom);
        reSliceModel();
        updateUIStates();
        
      } catch (err) {
        logConsole(`STL parsing error: ${err.message}`, "error");
      }
    };
    reader.readAsArrayBuffer(file);
  });
}

function pausePrint() {
  isPrinting = false;
  isPaused = true;
  document.getElementById('btn-play-pause').innerText = '► START PRINT';
  updateUIStates();
}

function resetPrint() {
  pausePrint();
  clearFilament();
  queueIndex = 0;
  tcpTargetPos.set(0, 3.0, 0);
  atcState = 'IDLE';
  
  if (currentToolIdx !== -1) {
    dockActiveToolhead();
  }
  
  targetCarriageX = 0;
  currentCarriageX = 0;
  cassetteCarriage.position.x = 0;
  
  clawL.position.z = -0.4;
  clawR.position.z = 0.4;

  document.getElementById('stat-progress').innerText = "0.0%";
  logConsole("Simulation workspace reset complete.", "action");
}

// ----------------------------------------------------
// WINDOW ONLOAD
// ----------------------------------------------------
window.onload = () => {
  init();
};
