// 3D models shared by the simulator window and the control window: bee, robot, arena and
// the tracking cameras with their id labels. Display only; sizes are not measured, except where a comment says so.
// Soft, matte, rounded shapes. Each model is merged into a few meshes per material and its
// geometry is shared, so a window with dozens of bees still needs only a few draw calls.
//
// Frame: the models are built in the arena frame (x along the rail, y into the arena, z up).
import * as THREE from "three";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";
import { RoundedBoxGeometry } from "three/addons/geometries/RoundedBoxGeometry.js";

// ---- shared helpers ----

// Merge geometries into one. three.js can only merge geometries that are all indexed or all
// non-indexed (rounded boxes are non-indexed, boxes, spheres and cylinders are indexed), so
// every part is made non-indexed first. The parts are disposed.
function mergeParts(geometries) {
  const flat = geometries.map(g => (g.index ? g.toNonIndexed() : g));
  const merged = mergeGeometries(flat);
  new Set([...geometries, ...flat]).forEach(g => g.dispose());
  if (!merged) throw new Error("models.js: parts with different attributes cannot be merged");
  return merged;
}

// Collects geometries by material key, then merges each key into one mesh.
class PartList {
  constructor() { this.byKey = new Map(); }

  // Add a geometry placed at (x, y, z) with Euler rotation (rx, ry, rz) in rad.
  add(key, geometry, [x, y, z] = [0, 0, 0], [rx, ry, rz] = [0, 0, 0]) {
    const matrix = new THREE.Matrix4().compose(new THREE.Vector3(x, y, z),
      new THREE.Quaternion().setFromEuler(new THREE.Euler(rx, ry, rz)), new THREE.Vector3(1, 1, 1));
    const placed = geometry.clone().applyMatrix4(matrix);
    geometry.dispose();
    if (!this.byKey.has(key)) this.byKey.set(key, []);
    this.byKey.get(key).push(placed);
  }

  // One mesh per material key, in a group.
  toGroup(materials) {
    const group = new THREE.Group();
    for (const [key, geometries] of this.byKey) {
      group.add(new THREE.Mesh(mergeParts(geometries), materials[key]));
    }
    return group;
  }
}

const rounded = (w, h, d, r) => new RoundedBoxGeometry(w, h, d, 2, r);
// Cylinder along y, as the lens barrels and the rail are drawn.
const cylinderY = (radius, length, segments = 20) => new THREE.CylinderGeometry(radius, radius, length, segments);
// Colour every vertex of a geometry with a function of its position.
function paint(geometry, colourAt) {
  const position = geometry.attributes.position, colours = new Float32Array(position.count * 3);
  const c = new THREE.Color();
  for (let i = 0; i < position.count; i++) {
    colourAt(c, position.getX(i), position.getY(i), position.getZ(i));
    c.toArray(colours, i * 3);
  }
  geometry.setAttribute("color", new THREE.BufferAttribute(colours, 3));
  return geometry;
}
// Solid colour on a geometry, so it can be merged with painted ones.
const solid = (geometry, hex) => paint(geometry, c => c.setHex(hex));
// Ellipsoid with the given radii, centred at the origin.
const ellipsoid = (rx, ry, rz, widthSegments = 14, heightSegments = 10) =>
  new THREE.SphereGeometry(1, widthSegments, heightSegments).scale(rx, ry, rz);

// ---- bee ----
// The bee is modelled with its length along +x (nose forward) and a body length of 1, then
// scaled to the display length. Colours follow a buff-tailed bumblebee: yellow collar,
// black waist, yellow band, buff tail.
const BEE_YELLOW = 0xf0b72f, BEE_BLACK = 0x2e2118, BEE_BUFF = 0xeadcc0, BEE_DARK = 0x3b2a1f;
const BEE_FLAP_HZ = 14;          // display only; real wings beat about 200 Hz, far too fast to show
const WING_FOLDED_SWEEP_RAD = 1.2;
const WING_FLYING_SWEEP_RAD = 0.25;
const WING_FLAP_AMPLITUDE_RAD = 0.55;
const WING_FLAP_CENTRE_RAD = 0.3;
const WING_OPACITY = 0.55;
const MAX_PITCH_RAD = 0.6;
const MIN_HEADING_SPEED_M_S = 0.03;   // slower than this, the bee keeps its last heading

// Smoothly blend colour a to colour b when v goes from edge - width to edge + width.
function blend(out, a, b, v, edge, width) {
  const t = THREE.MathUtils.smoothstep(v, edge - width, edge + width);
  return out.setHex(a).lerp(new THREE.Color(b), t);
}

// Abdomen colours along the body: buff tail, black band, yellow band at the front.
function abdomenColour(c, x) {
  const t = (x + 0.5) / 0.6;                       // 0 at the tail, 1 at the waist
  blend(c, BEE_BUFF, BEE_BLACK, t, 0.3, 0.05);
  if (t > 0.3) blend(c, BEE_BLACK, BEE_YELLOW, t, 0.72, 0.05);
}

// Thorax colours: yellow with a black patch on the back between the wings.
function thoraxColour(c, x, y, z) {
  const patch = z > 0.07 && Math.abs(x - 0.12) < 0.1 && Math.abs(y) < 0.1;
  c.setHex(patch ? BEE_BLACK : BEE_YELLOW);
}

// All the parts of a bee body that do not move, merged into one geometry.
function buildBeeBody() {
  const parts = [];
  const add = (geometry, [x, y, z], rotation) => {
    if (rotation) geometry.applyQuaternion(rotation);
    parts.push(geometry.translate(x, y, z));
  };
  add(paint(ellipsoid(0.3, 0.2, 0.19), (c, x, y, z) => abdomenColour(c, x)), [-0.2, 0, 0]);
  add(paint(ellipsoid(0.19, 0.17, 0.17), (c, x, y, z) => thoraxColour(c, x + 0.12, y, z)), [0.12, 0, 0.02]);
  add(solid(ellipsoid(0.09, 0.1, 0.09), BEE_BLACK), [0.34, 0, 0]);
  for (const side of [-1, 1]) {
    add(solid(ellipsoid(0.03, 0.025, 0.035, 8, 6), 0x0d0a08), [0.4, side * 0.06, 0.035]);   // eye
    // Antenna: a thin cylinder leaning forward, up and outwards.
    const dir = new THREE.Vector3(0.5, side * 0.35, 0.6).normalize();
    const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    add(solid(new THREE.CylinderGeometry(0.01, 0.01, 0.16, 6), BEE_DARK),
      [0.41 + dir.x * 0.08, side * 0.04 + dir.y * 0.08, 0.05 + dir.z * 0.08], q);
    // Three legs on each side, reaching down and outwards.
    for (const legX of [0.2, 0.05, -0.1]) {
      const leg = new THREE.Vector3(0, side * 0.45, -0.9).normalize();
      const lq = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), leg);
      add(solid(new THREE.CylinderGeometry(0.014, 0.01, 0.2, 6), BEE_DARK),
        [legX, side * 0.09 + leg.y * 0.1, -0.12 + leg.z * 0.1], lq);
    }
  }
  return mergeParts(parts);
}

// One wing: a flat ellipse whose root is at the origin and which reaches out along +y.
function buildBeeWing() {
  return ellipsoid(0.09, 0.24, 0.006, 12, 6).translate(-0.04, 0.24, 0);
}

let beeParts = null;   // geometries and materials shared by all bees, created on first use
function sharedBeeParts() {
  if (!beeParts) {
    beeParts = {
      body: buildBeeBody(),
      wing: buildBeeWing(),
      bodyMaterial: new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.95, metalness: 0 }),
      wingMaterial: new THREE.MeshStandardMaterial({ color: 0xf4f1e8, transparent: true,
        opacity: WING_OPACITY, roughness: 0.4, side: THREE.DoubleSide, depthWrite: false }),
    };
  }
  return beeParts;
}

/**
 * Create one bee. Geometry and materials are shared between bees.
 * @param {number} lengthM Display body length in m.
 * @returns {THREE.Group} The bee with `userData.wings` (left, right) and `userData.heading`.
 */
export function createBee(lengthM) {
  const parts = sharedBeeParts();
  const bee = new THREE.Group();
  bee.rotation.order = "ZYX";   // yaw about z, then pitch about y
  const model = new THREE.Group();
  model.scale.setScalar(lengthM);
  model.add(new THREE.Mesh(parts.body, parts.bodyMaterial));
  const wings = [-1, 1].map(side => {
    const wing = new THREE.Mesh(parts.wing, parts.wingMaterial);
    wing.position.set(0.12, side * 0.06, 0.15);
    wing.scale.y = side;      // mirror the left wing
    wing.rotation.order = "ZXY";
    model.add(wing);
    return wing;
  });
  bee.add(model);
  bee.userData = { wings, heading: 0 };
  return bee;
}

/**
 * Place a bee, turn it along its velocity and pose its wings.
 * @param {THREE.Group} bee A bee made by `createBee`.
 * @param {number[]} pos Position [x, y, z] in m.
 * @param {number[]} vel Velocity [x, y, z] in m/s.
 * @param {boolean} flying True: wings flap; false: wings folded back.
 * @param {number} phase Wing phase in rad, one value per bee so they do not flap in step.
 */
export function poseBee(bee, pos, vel, flying, phase) {
  bee.position.fromArray(pos);
  const horizontal = Math.hypot(vel[0], vel[1]);
  if (horizontal > MIN_HEADING_SPEED_M_S) bee.userData.heading = Math.atan2(vel[1], vel[0]);
  bee.rotation.z = bee.userData.heading;
  const pitch = Math.atan2(vel[2], Math.max(horizontal, MIN_HEADING_SPEED_M_S));
  bee.rotation.y = -THREE.MathUtils.clamp(pitch, -MAX_PITCH_RAD, MAX_PITCH_RAD);
  const sweep = flying ? WING_FLYING_SWEEP_RAD : WING_FOLDED_SWEEP_RAD;
  const lift = flying ? WING_FLAP_CENTRE_RAD + WING_FLAP_AMPLITUDE_RAD * Math.sin(phase) : 0.1;
  bee.userData.wings.forEach((wing, i) => {
    const side = i === 0 ? -1 : 1;
    wing.rotation.z = side * sweep;
    wing.rotation.x = side * lift;
  });
}

/** Wing phase in rad for a bee id at a wall-clock time in ms (display only). */
export function wingPhase(id, nowMs) {
  return nowMs * 0.001 * 2 * Math.PI * BEE_FLAP_HZ + id * 1.7;
}

// ---- robot ----
// Camera robot: a belt-driven linear rail, a carriage with a yoke, and a tilting head with a
// cinema camera, a macro lens, a stereo pair on top, and NEMA 23 stepper motors with gear
// boxes for the tilt and the focus ring. The carriage origin is the tilt axis.
const ROBOT_MATERIALS = {
  silver: new THREE.MeshStandardMaterial({ color: 0xb9c0c8, metalness: 0.55, roughness: 0.45 }),
  black: new THREE.MeshStandardMaterial({ color: 0x262b31, metalness: 0.2, roughness: 0.6 }),
  grey: new THREE.MeshStandardMaterial({ color: 0x59616b, metalness: 0.3, roughness: 0.5 }),
  accent: new THREE.MeshStandardMaterial({ color: 0x8b5cf6, metalness: 0.1, roughness: 0.55 }),
  glass: new THREE.MeshStandardMaterial({ color: 0x10202e, metalness: 0.6, roughness: 0.1 }),
};
const SIGHT_COLOR = 0x8b5cf6;
const RAIL_PROFILE_M = [0.03, 0.022];          // width (y) and height (z) of the aluminium profile
const PLATE_SIZE_M = [0.09, 0.07, 0.008];
const YOKE_HEIGHT_M = 0.05;                    // tilt axis above the plate
const YOKE_THICKNESS_M = 0.007;
const BODY_SIZE_M = [0.05, 0.09, 0.06];        // camera body; width along the tilt axis
const MOTOR_SIZE_M = 0.056;                    // NEMA 23 frame is 57 mm square
const MOTOR_LENGTH_M = 0.05;
const GEARBOX_RADIUS_M = 0.03;
const GEARBOX_LENGTH_M = 0.04;
const LENS_RADIUS_M = 0.02;
const LENS_LENGTH_M = 0.1;
const HOOD_RADIUS_M = 0.024;
const HOOD_LENGTH_M = 0.015;
const STEREO_BASELINE_M = 0.05;                // video

// The rail does not move: profile with T-slots, end caps and the drive motor at the +x end.
function buildRail(robot) {
  const [x0, y, z] = robot.rail_start_m, travel = robot.rail_length_m;
  const length = travel + PLATE_SIZE_M[0] + 0.02;
  const centreX = x0 + travel / 2;
  const top = z - YOKE_HEIGHT_M - PLATE_SIZE_M[2] - 0.012;     // just below the carriage block
  const parts = new PartList();
  parts.add("silver", rounded(length, RAIL_PROFILE_M[0], RAIL_PROFILE_M[1], 0.003),
    [centreX, y, top - RAIL_PROFILE_M[1] / 2]);
  for (const side of [-1, 1]) {
    parts.add("black", new THREE.BoxGeometry(length, 0.004, 0.002), [centreX, y + side * 0.008, top + 0.0005]);
  }
  for (const side of [-1, 1]) {
    parts.add("black", rounded(0.012, RAIL_PROFILE_M[0] + 0.006, RAIL_PROFILE_M[1] + 0.006, 0.003),
      [centreX + side * (length / 2 + 0.006), y, top - RAIL_PROFILE_M[1] / 2]);
  }
  const motorX = centreX + length / 2 + 0.012 + MOTOR_LENGTH_M / 2;
  parts.add("black", rounded(MOTOR_LENGTH_M, MOTOR_SIZE_M, MOTOR_SIZE_M, 0.006),
    [motorX, y, top - RAIL_PROFILE_M[1] / 2]);
  return parts.toGroup(ROBOT_MATERIALS);
}

// Carriage: block on the rail, plate, two yoke uprights, and the tilt motor with its gearbox
// on the +x side.
function buildCarriage() {
  const parts = new PartList();
  const plateZ = -YOKE_HEIGHT_M - PLATE_SIZE_M[2] / 2;
  parts.add("black", rounded(PLATE_SIZE_M[0], 0.045, 0.02, 0.003), [0, 0, plateZ - 0.014]);
  parts.add("accent", rounded(...PLATE_SIZE_M, 0.003), [0, 0, plateZ]);
  const upright = BODY_SIZE_M[0] / 2 + 0.004 + YOKE_THICKNESS_M / 2;
  const uprightHeight = YOKE_HEIGHT_M + 0.024;
  for (const side of [-1, 1]) {
    parts.add("accent", rounded(YOKE_THICKNESS_M, 0.05, uprightHeight, 0.003),
      [side * upright, 0, -YOKE_HEIGHT_M + uprightHeight / 2]);
  }
  const gearX = upright + YOKE_THICKNESS_M / 2 + GEARBOX_LENGTH_M / 2;
  parts.add("silver", cylinderY(GEARBOX_RADIUS_M, GEARBOX_LENGTH_M), [gearX, 0, 0], [0, 0, Math.PI / 2]);
  parts.add("black", rounded(MOTOR_LENGTH_M, MOTOR_SIZE_M, MOTOR_SIZE_M, 0.006),
    [gearX + GEARBOX_LENGTH_M / 2 + MOTOR_LENGTH_M / 2, 0, 0]);
  return parts.toGroup(ROBOT_MATERIALS);
}

// Head: camera body behind the tilt axis, lens with focus ring and hood along +y, focus
// motor on the side of the lens, stereo pair on top.
function buildHead() {
  const parts = new PartList();
  parts.add("black", rounded(...BODY_SIZE_M, 0.008), [0, -BODY_SIZE_M[1] / 2 + 0.015, 0]);
  parts.add("grey", cylinderY(LENS_RADIUS_M, LENS_LENGTH_M), [0, 0.015 + LENS_LENGTH_M / 2, 0]);
  parts.add("black", cylinderY(LENS_RADIUS_M + 0.003, 0.025), [0, 0.04, 0]);          // focus ring
  parts.add("black", cylinderY(HOOD_RADIUS_M, HOOD_LENGTH_M), [0, 0.015 + LENS_LENGTH_M + HOOD_LENGTH_M / 2, 0]);
  parts.add("glass", cylinderY(LENS_RADIUS_M - 0.004, 0.004), [0, 0.015 + LENS_LENGTH_M + HOOD_LENGTH_M - 0.002, 0]);
  parts.add("silver", cylinderY(0.016, 0.03), [-0.045, 0.04, 0.0]);                    // focus gearbox
  parts.add("black", rounded(MOTOR_SIZE_M * 0.7, MOTOR_LENGTH_M * 0.7, MOTOR_SIZE_M * 0.7, 0.005), [-0.045, 0.075, 0]);
  const stereoZ = BODY_SIZE_M[2] / 2 + 0.008;
  parts.add("black", rounded(STEREO_BASELINE_M + 0.02, 0.016, 0.012, 0.003), [0, 0.0, stereoZ]);
  for (const side of [-1, 1]) {
    parts.add("glass", cylinderY(0.0045, 0.006, 12), [side * STEREO_BASELINE_M / 2, 0.009, stereoZ]);
  }
  return parts.toGroup(ROBOT_MATERIALS);
}

/**
 * Build the robot from the robot geometry of a scene message.
 * @param {{rail_start_m: number[], rail_length_m: number, lens_offset_m: number}} robot
 * @returns {{group: THREE.Group, carriage: THREE.Group, tiltHead: THREE.Group,
 *            sightLine: THREE.Line}} `carriage` moves along the rail, `tiltHead` rotates about
 *   x, `sightLine` is a unit line along +y that the caller scales to the focus distance.
 */
export function buildRobotModel(robot) {
  const group = new THREE.Group();
  group.add(buildRail(robot));
  const carriage = new THREE.Group();
  carriage.add(buildCarriage());
  const tiltHead = new THREE.Group();
  tiltHead.add(buildHead());
  const sightLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 1, 0)]),
    new THREE.LineBasicMaterial({ color: SIGHT_COLOR }));
  sightLine.position.y = robot.lens_offset_m;
  tiltHead.add(sightLine);
  carriage.add(tiltHead);
  group.add(carriage);
  return { group, carriage, tiltHead, sightLine };
}

/**
 * Move the robot model to a pose.
 * @param {object} model Result of `buildRobotModel`.
 * @param {{rail_start_m: number[]}} robot Robot geometry.
 * @param {{rail_m: number, tilt_rad: number, focus_m: number}} pose Axis positions.
 */
export function poseRobotModel(model, robot, pose) {
  model.carriage.position.set(robot.rail_start_m[0] + pose.rail_m, robot.rail_start_m[1],
                              robot.rail_start_m[2]);
  model.tiltHead.rotation.x = pose.tilt_rad;
  model.sightLine.scale.y = pose.focus_m;   // the line starts at the lens reference point
}

// ---- arena ----
const FLOOR_COLOUR = 0xece3cf, BASE_COLOUR = 0xf7f0e0, FRAME_COLOUR = 0xa7aeb6;
const FRAME_RADIUS_M = 0.004;
const GLASS_OPACITY = 0.06;
const BASE_MARGIN_M = 0.05, BASE_HEIGHT_M = 0.02;

/**
 * Build the arena: a soft base plate, a floor, faint glass walls and a metal frame.
 * @param {number[]} size Arena size [x, y, z] in m; the origin is the corner nearest the robot.
 * @returns {THREE.Group} The arena model.
 */
export function buildArenaModel(size) {
  const [sx, sy, sz] = size;
  const group = new THREE.Group();
  const base = new THREE.Mesh(
    new RoundedBoxGeometry(sx + 2 * BASE_MARGIN_M, sy + 2 * BASE_MARGIN_M, BASE_HEIGHT_M, 3, 0.012),
    new THREE.MeshStandardMaterial({ color: BASE_COLOUR, roughness: 0.9 }));
  base.position.set(sx / 2, sy / 2, -BASE_HEIGHT_M / 2 - 0.001);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(sx, sy),
    new THREE.MeshStandardMaterial({ color: FLOOR_COLOUR, roughness: 1 }));
  floor.position.set(sx / 2, sy / 2, 0);
  const glass = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz),
    new THREE.MeshBasicMaterial({ color: 0xbcd4e0, transparent: true, opacity: GLASS_OPACITY,
                                  side: THREE.BackSide, depthWrite: false }));
  glass.position.set(sx / 2, sy / 2, sz / 2);
  const edges = new PartList();
  for (const [a, b] of boxEdges(size)) {
    const dir = new THREE.Vector3().subVectors(b, a);
    const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
    const g = new THREE.CylinderGeometry(FRAME_RADIUS_M, FRAME_RADIUS_M, dir.length(), 8);
    g.applyQuaternion(q).translate((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2);
    edges.add("frame", g);
  }
  const frame = edges.toGroup({ frame: new THREE.MeshStandardMaterial({ color: FRAME_COLOUR,
    metalness: 0.5, roughness: 0.5 }) });
  group.add(base, floor, glass, frame);
  return group;
}

// The 12 edges of the arena box as pairs of end points.
function boxEdges([sx, sy, sz]) {
  const corner = (i, j, k) => new THREE.Vector3(i * sx, j * sy, k * sz);
  const edges = [];
  for (const [i, j] of [[0, 0], [0, 1], [1, 0], [1, 1]]) edges.push([corner(i, j, 0), corner(i, j, 1)]);
  for (const [j, k] of [[0, 0], [0, 1], [1, 0], [1, 1]]) edges.push([corner(0, j, k), corner(1, j, k)]);
  for (const [i, k] of [[0, 0], [0, 1], [1, 0], [1, 1]]) edges.push([corner(i, 0, k), corner(i, 1, k)]);
  return edges;
}

// ---- tracking camera ----
// A Raspberry Pi camera module: green board, black sensor module with a round lens, a short
// ribbon cable and a status LED. Built with the lens along +y (the view axis) and the board
// in the x-z plane. Display only; a real module is 25 x 24 mm.
const PI_BOARD_M = [0.025, 0.0016, 0.024];     // width (x), thickness (y), height (z)
const PI_MODULE_M = 0.0085;                    // side of the square sensor module
const PI_MODULE_DEPTH_M = 0.005;
const PI_LENS_RADIUS_M = 0.003;
const PI_RIBBON_M = [0.016, 0.0004, 0.022];    // hangs down from the lower edge of the board
const PI_LED_RADIUS_M = 0.0022;
const PI_GREEN = 0x1f8a4c, PI_BLACK = 0x1c1c1e, PI_LENS = 0x0c1a26, PI_RIBBON = 0xd9d2c0;
const LED_RUNNING = 0x22c55e, LED_STOPPED = 0xef4444;

let cameraParts = null;   // geometry and materials shared by all tracking cameras
function sharedCameraParts() {
  if (!cameraParts) {
    const front = PI_BOARD_M[1] / 2;
    const body = mergeParts([
      solid(new THREE.BoxGeometry(...PI_BOARD_M), PI_GREEN),
      solid(new THREE.BoxGeometry(PI_MODULE_M, PI_MODULE_DEPTH_M, PI_MODULE_M)
        .translate(0, front + PI_MODULE_DEPTH_M / 2, 0), PI_BLACK),
      solid(cylinderY(PI_LENS_RADIUS_M, 0.002, 16)
        .translate(0, front + PI_MODULE_DEPTH_M + 0.001, 0), PI_LENS),
      solid(new THREE.BoxGeometry(...PI_RIBBON_M)
        .translate(0, 0, -(PI_BOARD_M[2] + PI_RIBBON_M[2]) / 2), PI_RIBBON),
    ]);
    cameraParts = {
      body,
      led: new THREE.SphereGeometry(PI_LED_RADIUS_M, 10, 8)
        .translate(PI_BOARD_M[0] / 2 - 0.004, front + PI_LED_RADIUS_M / 2, PI_BOARD_M[2] / 2 - 0.004),
      bodyMaterial: new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.7 }),
      ledMaterials: {
        true: new THREE.MeshBasicMaterial({ color: LED_RUNNING }),
        false: new THREE.MeshBasicMaterial({ color: LED_STOPPED }),
      },
    };
  }
  return cameraParts;
}

/**
 * Create one tracking camera, looking from its position towards a point. Geometry and
 * materials are shared between cameras, so the caller must not dispose them.
 * @param {number[]} positionM Camera position [x, y, z] in m.
 * @param {number[]} lookAtM Point on the view axis [x, y, z] in m.
 * @returns {THREE.Group} The camera with `userData.led`; its LED shows "running".
 */
export function createCameraModel(positionM, lookAtM) {
  const parts = sharedCameraParts();
  const cam = new THREE.Group();
  const led = new THREE.Mesh(parts.led, parts.ledMaterials[true]);
  cam.add(new THREE.Mesh(parts.body, parts.bodyMaterial), led);
  cam.position.fromArray(positionM);
  // The model looks along +y; z (the ribbon side is -z) stays as upright as the view allows.
  const forward = new THREE.Vector3().fromArray(lookAtM).sub(cam.position).normalize();
  const right = new THREE.Vector3().crossVectors(forward, new THREE.Vector3(0, 0, 1)).normalize();
  const up = new THREE.Vector3().crossVectors(right, forward);
  cam.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(right, forward, up));
  cam.userData = { led };
  return cam;
}

/**
 * Show on a camera's LED whether its node is running (green) or stopped (red).
 * @param {THREE.Group} cam A camera made by `createCameraModel`.
 * @param {boolean} running True for a running node.
 */
export function setCameraRunning(cam, running) {
  cam.userData.led.material = sharedCameraParts().ledMaterials[!!running];
}

// ---- camera id labels ----
// Text labels next to the cameras, as HTML elements over the 3D view. They are created
// hidden; the caller repositions them only while they are shown, so they cost nothing
// otherwise.
const LABEL_OFFSET_PX = [8, -8];

/**
 * Create one hidden label per camera inside a layer element that covers the 3D view.
 * @param {HTMLElement} layer Element the labels are added to; old camera labels are removed.
 * @param {{id: string, position_m: number[]}[]} cameras Cameras of the scene message.
 * @returns {{element: HTMLElement, position: THREE.Vector3}[]} The labels.
 */
export function createCameraLabels(layer, cameras) {
  layer.querySelectorAll(".camera-label").forEach(element => element.remove());
  return cameras.map(cam => {
    const element = document.createElement("div");
    element.className = "camera-label";
    element.textContent = cam.id;
    element.hidden = true;
    layer.appendChild(element);
    return { element, position: new THREE.Vector3().fromArray(cam.position_m) };
  });
}

/**
 * Show the labels at their cameras' places on the screen, or hide them all.
 * @param {{element: HTMLElement, position: THREE.Vector3}[]} labels From `createCameraLabels`.
 * @param {boolean} shown Whether the labels are switched on.
 * @param {THREE.Camera} viewCamera The camera the 3D view is drawn with.
 * @param {HTMLCanvasElement} canvas The canvas of the 3D view (same place as the layer).
 */
export function placeCameraLabels(labels, shown, viewCamera, canvas) {
  const point = new THREE.Vector3();
  for (const label of labels) {
    point.copy(label.position).project(viewCamera);
    // z above 1 means behind the viewer.
    label.element.hidden = !shown || point.z > 1;
    if (label.element.hidden) continue;
    const x = (point.x * 0.5 + 0.5) * canvas.clientWidth + LABEL_OFFSET_PX[0];
    const y = (-point.y * 0.5 + 0.5) * canvas.clientHeight + LABEL_OFFSET_PX[1];
    label.element.style.transform = `translate(${x}px, ${y}px)`;
  }
}
