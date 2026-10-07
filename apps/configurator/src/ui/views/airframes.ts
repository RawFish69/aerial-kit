import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';

/**
 * The airframe models the attitude view draws, built from primitives.
 *
 * Axes are three.js's: +X is the right wing, +Y is up, -Z is the nose. The
 * firmware's `airframe` parameter picks the shape (its help text is the source
 * of the numbering: 0 quadx, 1 elevonwing, 2 quadx1234, 3 quadp, 4 y4,
 * 5 vtail4, 6 tri, 7 elevonwingsingle).
 *
 * Every model returns its propellers in motor order, so the view can spin each
 * one at the speed the board reports for that output.
 */

export interface AirframeModel {
  readonly group: THREE.Group;
  /** Propeller discs in motor order (motor 1 first). */
  readonly props: readonly THREE.Object3D[];
  /** Each prop's turning direction seen from above: 1 counter-clockwise, -1 clockwise. */
  readonly spins: readonly (1 | -1)[];
  readonly label: string;
}

export const AIRFRAME_NAMES: Readonly<Record<number, string>> = {
  0: 'Quad X',
  1: 'Flying wing',
  2: 'Quad X (1234)',
  3: 'Quad +',
  4: 'Y4',
  5: 'V-tail quad',
  6: 'Tricopter',
  7: 'Flying wing, single motor',
};

// Blueprint's palette, so the model matches the page around it.
const COLORS = {
  body: 0x5f6b7c, // GRAY1
  bodyEdge: 0xc5cbd3, // GRAY5
  arm: 0x404854, // DARK_GRAY5
  nose: 0xec9a3c, // ORANGE4
  motor: 0x2f343c, // DARK_GRAY3
  prop: 0x4c90f0, // BLUE4
  elevon: 0x4c90f0, // BLUE4
};

function material(color: number, opts: Partial<THREE.MeshStandardMaterialParameters> = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.25, ...opts });
}

function outline(mesh: THREE.Mesh, color = COLORS.bodyEdge): THREE.LineSegments {
  const lines = new THREE.LineSegments(
    new THREE.EdgesGeometry(mesh.geometry, 25),
    new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.55 }),
  );
  lines.position.copy(mesh.position);
  lines.rotation.copy(mesh.rotation);
  return lines;
}

function propeller(radius: number): THREE.Group {
  const prop = new THREE.Group();
  const disc = new THREE.Mesh(
    new THREE.CircleGeometry(radius, 40),
    new THREE.MeshBasicMaterial({ color: COLORS.prop, transparent: true, opacity: 0.12, side: THREE.DoubleSide }),
  );
  disc.rotation.x = -Math.PI / 2;
  prop.add(disc);
  const blade = new THREE.Mesh(
    new THREE.BoxGeometry(radius * 2, 0.012, 0.07),
    material(COLORS.prop, { emissive: COLORS.prop, emissiveIntensity: 0.25 }),
  );
  prop.add(blade);
  return prop;
}

// ---- the FPV quad ---------------------------------------------------------
//
// Built in millimetres at the proportions of a real 5-inch freestyle quad
// (225 mm wheelbase, 2207 motors, 5.1" tri-blades, a 1500 mAh pack on top) and
// scaled into scene units once, so the parts stay in proportion to each other.

/** Scene units per millimetre: puts the Quad X motors at the view's arm reach. */
const MM = 0.0088;

const FPV = {
  aluminium: 0xb8c0c8,
  gunmetal: 0x2a2d33,
  bellAccent: 0xd13913,
  copper: 0xb5652b,
  pcb: 0x15181c,
  chip: 0x2a2e35,
  gold: 0xc9a227,
  lipo: 0x1b1d21,
  strap: 0x0d0e10,
  tpu: 0x2d72d2,
  lens: 0x0a0c0f,
  xt60: 0xf2c230,
  frontProp: 0xff8a1f, // orange at the front, the FPV convention
  rearProp: 0x4c90f0,
};

let carbonMap: THREE.Texture | null = null;

/** A 2x2 twill weave, drawn once. UVs on the plates are in millimetres. */
function carbonTexture(): THREE.Texture | null {
  if (carbonMap !== null) return carbonMap;
  if (typeof document === 'undefined') return null;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 128;
  const ctx = canvas.getContext('2d');
  if (ctx === null) return null;
  const cell = 32;
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      const across = (i + j) % 4 < 2;
      const x = i * cell;
      const y = j * cell;
      const g = across ? ctx.createLinearGradient(x, y, x, y + cell) : ctx.createLinearGradient(x, y, x + cell, y);
      g.addColorStop(0, '#0e0f11');
      g.addColorStop(0.5, '#34383e');
      g.addColorStop(1, '#0e0f11');
      ctx.fillStyle = g;
      ctx.fillRect(x, y, cell, cell);
    }
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(1 / 10, 1 / 10);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 4;
  carbonMap = texture;
  return texture;
}

function carbonMaterial(): THREE.Material {
  return new THREE.MeshPhysicalMaterial({
    color: 0xffffff,
    map: carbonTexture(),
    roughness: 0.38,
    metalness: 0.1,
    clearcoat: 1,
    clearcoatRoughness: 0.12,
  });
}

function metal(color: number, roughness = 0.3): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({ color, metalness: 0.85, roughness });
}

function plastic(color: number, roughness = 0.6): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({ color, metalness: 0, roughness });
}

/** Extrudes a shape drawn in plan (shape x -> world x, shape y -> world z), bottom at y = 0. */
function slab(shape: THREE.Shape, thickness: number, material: THREE.Material, bevel = 0.5): THREE.Mesh {
  const geometry = new THREE.ExtrudeGeometry(shape, {
    depth: thickness - 2 * bevel,
    bevelEnabled: bevel > 0,
    bevelThickness: bevel,
    bevelSize: bevel,
    bevelSegments: 2,
    curveSegments: 24,
  });
  geometry.rotateX(Math.PI / 2);
  geometry.translate(0, thickness - bevel, 0);
  return new THREE.Mesh(geometry, material);
}

function roundedRect(width: number, length: number, radius: number, cx = 0, cz = 0): THREE.Shape {
  const shape = new THREE.Shape();
  const w = width / 2;
  const l = length / 2;
  shape.moveTo(cx - w + radius, cz - l);
  shape.lineTo(cx + w - radius, cz - l);
  shape.quadraticCurveTo(cx + w, cz - l, cx + w, cz - l + radius);
  shape.lineTo(cx + w, cz + l - radius);
  shape.quadraticCurveTo(cx + w, cz + l, cx + w - radius, cz + l);
  shape.lineTo(cx - w + radius, cz + l);
  shape.quadraticCurveTo(cx - w, cz + l, cx - w, cz + l - radius);
  shape.lineTo(cx - w, cz - l + radius);
  shape.quadraticCurveTo(cx - w, cz - l, cx - w + radius, cz - l);
  return shape;
}

function hole(x: number, z: number, r: number): THREE.Path {
  const path = new THREE.Path();
  path.absarc(x, z, r, 0, Math.PI * 2, true);
  return path;
}

/**
 * One 5 mm arm, root at the centre and motor at `length` along +z in its own
 * frame: 24 mm wide at the root easing to 22 mm, with a round end that is the
 * arm's own width, so the outline never bulges. The 12 mm motor pattern sits
 * on the diagonal, the way 2207 arms are drilled.
 */
function frameArm(length: number): THREE.Mesh {
  const root = 12;
  const tip = 11;
  const shape = new THREE.Shape();
  shape.moveTo(-root, 0);
  shape.lineTo(root, 0);
  shape.lineTo(tip, length);
  shape.absarc(0, length, tip, 0, Math.PI, false);
  shape.lineTo(-root, 0);
  shape.holes.push(hole(0, length, 3.2));
  for (const [dx, dz] of [[8.5, 0], [-8.5, 0], [0, 8.5], [0, -8.5]]) shape.holes.push(hole(dx!, length + dz!, 1.6));
  // A lightening slot down the middle of the exposed part of the arm.
  const a0 = 44;
  const a1 = length - 22;
  if (a1 > a0) {
    const slot = new THREE.Path();
    slot.absarc(0, a1, 3, 0, Math.PI, false);
    slot.absarc(0, a0, 3, Math.PI, Math.PI * 2, false);
    shape.holes.push(slot);
  }
  return slab(shape, 5, carbonMaterial(), 0.6);
}

/** A 2207 motor, base at y = 0, 26 mm tall to the top of the bell. */
function motor2207(): THREE.Group {
  const group = new THREE.Group();
  const add = (geometry: THREE.BufferGeometry, mat: THREE.Material, y: number) => {
    const mesh = new THREE.Mesh(geometry, mat);
    mesh.position.y = y;
    group.add(mesh);
    return mesh;
  };
  add(new THREE.CylinderGeometry(13.5, 14, 4, 36), metal(FPV.gunmetal), 2);
  add(new THREE.CylinderGeometry(11.6, 11.6, 3.2, 36), metal(FPV.copper, 0.35), 5.6);
  add(new THREE.CylinderGeometry(14, 14, 14, 36, 1, true), metal(FPV.gunmetal, 0.28), 14.2);
  add(new THREE.CylinderGeometry(14.15, 14.15, 3, 36, 1, true), metal(FPV.bellAccent, 0.35), 19.2);
  add(new THREE.CylinderGeometry(13.6, 14, 1.8, 36), metal(FPV.gunmetal, 0.28), 22.1);
  add(new THREE.CylinderGeometry(6, 6, 0.8, 24), metal(FPV.aluminium, 0.2), 23.3);
  // Bell vents, so the bell reads as a bell and not a can.
  for (let i = 0; i < 6; i++) {
    const vent = add(new THREE.BoxGeometry(5, 5, 0.6), plastic(0x050505, 0.9), 12);
    const a = (i / 6) * Math.PI * 2;
    vent.position.set(Math.sin(a) * 14.05, 12, Math.cos(a) * 14.05);
    vent.rotation.y = a;
  }
  add(new THREE.CylinderGeometry(2.5, 2.5, 12, 12), metal(FPV.aluminium, 0.15), 28);
  return group;
}

/**
 * One blade, root at the hub and tip at +x, with a pitch that twists toward
 * flat at the tip. `hand` = 1 for a prop that turns counter-clockwise seen from
 * above; -1 mirrors it for the other direction.
 */
function blade(radius: number, hand: 1 | -1): THREE.BufferGeometry {
  const shape = new THREE.Shape();
  // Planform: chord along y, leading edge at +y.
  shape.moveTo(5, 5);
  shape.bezierCurveTo(radius * 0.3, 10, radius * 0.75, 9, radius, 2.5);
  shape.quadraticCurveTo(radius + 1, 0, radius - 2, -2);
  shape.bezierCurveTo(radius * 0.7, -4, radius * 0.35, -9, 5, -5);
  shape.closePath();
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: 1.1, bevelEnabled: false, curveSegments: 28 });
  geometry.translate(0, 0, -0.55);
  // Twist about the blade's own axis: steep at the root, flatter at the tip.
  const pitchMm = 109; // 4.3" pitch
  const pos = geometry.attributes.position as THREE.BufferAttribute;
  for (let i = 0; i < pos.count; i++) {
    const r = Math.max(6, pos.getX(i));
    const a = Math.min(0.6, Math.atan(pitchMm / (2 * Math.PI * r)));
    const y = pos.getY(i);
    const z = pos.getZ(i);
    pos.setY(i, y * Math.cos(a) - z * Math.sin(a));
    pos.setZ(i, y * Math.sin(a) + z * Math.cos(a));
  }
  // Chord into the horizontal plane, leading edge up: y -> -z, z -> y.
  geometry.rotateX(-Math.PI / 2);
  if (hand === -1) geometry.scale(1, 1, -1);
  geometry.computeVertexNormals();
  return geometry;
}

/** A 5.1" tri-blade, hub at the origin, spinning about +Y. */
function triBlade(radius: number, color: number, hand: 1 | -1): THREE.Group {
  const prop = new THREE.Group();
  const bladeMaterial = new THREE.MeshPhysicalMaterial({
    color,
    transparent: true,
    opacity: 0.88,
    roughness: 0.25,
    clearcoat: 0.6,
    side: THREE.DoubleSide,
  });
  const geometry = blade(radius, hand);
  for (let i = 0; i < 3; i++) {
    const mesh = new THREE.Mesh(geometry, bladeMaterial);
    mesh.rotation.y = (i * 2 * Math.PI) / 3;
    prop.add(mesh);
  }
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(6.5, 6.5, 7, 24), plastic(color, 0.4));
  prop.add(hub);
  const nut = new THREE.Mesh(new THREE.CylinderGeometry(4.6, 4.6, 6, 6), metal(FPV.aluminium, 0.2));
  nut.position.y = 6.5;
  prop.add(nut);
  const disc = new THREE.Mesh(
    new THREE.RingGeometry(8, radius, 64),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.05, side: THREE.DoubleSide, depthWrite: false }),
  );
  disc.rotation.x = -Math.PI / 2;
  prop.add(disc);
  prop.userData.disc = disc;
  return prop;
}

/** A plate in a vertical plane: shape x is distance forward (world -z), shape y is height. */
function sidePlate(shape: THREE.Shape, x: number, thickness: number): THREE.Mesh {
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: thickness, bevelEnabled: false, curveSegments: 16 });
  geometry.rotateY(Math.PI / 2); // shape x -> -z, extrusion -> +x
  geometry.translate(x - thickness / 2, 0, 0);
  return new THREE.Mesh(geometry, carbonMaterial());
}

/**
 * A 5-inch freestyle quad, put together the way real frames are: separate 5 mm
 * arms sandwiched between a bottom plate and a press plate, standoffs on the
 * plate corners carrying the top plate, the FC/ESC stack on its 30.5 mm posts,
 * a camera between two side plates at the front, and the pack strapped to the
 * top plate. Motors sit at the given (x, z) positions in the mixer's order.
 */
function multirotor(
  positions: ReadonlyArray<readonly [number, number]>,
  spins: readonly (1 | -1)[],
  label: string,
): AirframeModel {
  const frame = new THREE.Group();
  carbonMap = null; // one weave per model, so disposing a model frees its own
  const carbon = carbonMaterial();
  const motorsMm = positions.map(([x, z]) => [x / MM, z / MM] as const);

  // Bottom plate (y -2..0), arms (0..5), press plate (5..7).
  const bottom = slab(roundedRect(44, 112, 10), 2, carbon);
  bottom.position.y = -2;
  frame.add(bottom);
  const seen = new Set<string>();
  for (const [x, z] of motorsMm) {
    const key = `${x.toFixed(1)},${z.toFixed(1)}`;
    if (seen.has(key)) continue;
    seen.add(key);
    const arm = frameArm(Math.hypot(x, z));
    arm.rotation.y = Math.atan2(x, z);
    frame.add(arm);
  }
  const pressShape = roundedRect(44, 76, 10);
  for (const [sx, sz] of STACK) pressShape.holes.push(hole(sx, sz, 1.7));
  const press = slab(pressShape, 2, carbon);
  press.position.y = 5;
  frame.add(press);

  // Frame standoffs on the plate corners, 25 mm, carrying the top plate.
  for (const [sx, sz] of [[-17, -46], [17, -46], [-17, 46], [17, 46]] as const) {
    const standoff = new THREE.Mesh(new THREE.CylinderGeometry(2.6, 2.6, 25, 6), metal(0x7a2a2a, 0.35));
    standoff.position.set(sx, 7 + 12.5, sz);
    frame.add(standoff);
  }

  // The FC/ESC stack on its own posts.
  for (const [sx, sz] of STACK) {
    const post = new THREE.Mesh(new THREE.CylinderGeometry(1.6, 1.6, 18, 10), metal(FPV.aluminium, 0.3));
    post.position.set(sx, 7 + 9, sz);
    frame.add(post);
  }
  for (const [y, name] of [[12, 'esc'], [21, 'fc']] as const) {
    const board = new THREE.Mesh(new THREE.BoxGeometry(36, 1.6, 36), plastic(FPV.pcb, 0.5));
    board.position.set(0, y, 0);
    frame.add(board);
    const chip = new THREE.Mesh(new THREE.BoxGeometry(name === 'fc' ? 10 : 7, 1.4, name === 'fc' ? 10 : 7), plastic(FPV.chip, 0.4));
    chip.position.set(name === 'fc' ? 4 : -6, y + 1.4, name === 'fc' ? 2 : -4);
    frame.add(chip);
    for (const [px, pz] of [[-13, -13], [13, -13], [-13, 13], [13, 13]] as const) {
      const pad = new THREE.Mesh(new THREE.BoxGeometry(5, 0.4, 3), metal(FPV.gold, 0.3));
      pad.position.set(px, y + 1, pz);
      frame.add(pad);
    }
  }
  const led = new THREE.Mesh(
    new THREE.BoxGeometry(2, 1, 2),
    new THREE.MeshStandardMaterial({ color: 0x4c90f0, emissive: 0x4c90f0, emissiveIntensity: 2 }),
  );
  led.position.set(-10, 22.3, 10);
  frame.add(led);

  // Motor wires along each arm, from the stack to the motor.
  for (const [x, z] of motorsMm) {
    const length = Math.hypot(x, z);
    const wires = new THREE.Mesh(new THREE.BoxGeometry(5, 1.6, length - 40), plastic(0x16181c, 0.7));
    wires.rotation.y = Math.atan2(x, z);
    const mid = (length + 18) / 2 / length;
    wires.position.set(x * mid, 5.8, z * mid);
    frame.add(wires);
  }

  // Top plate, on the corner standoffs.
  const top = slab(roundedRect(40, 104, 9), 2, carbon);
  top.position.y = 32;
  frame.add(top);

  // Camera cage: two side plates between the plates, and a 19 mm camera tilted
  // up 30 degrees between them.
  const cage = new THREE.Shape();
  cage.moveTo(30, 0);
  cage.lineTo(52, 0);
  cage.quadraticCurveTo(56, 0, 56, 6);
  cage.lineTo(55, 28);
  cage.quadraticCurveTo(54, 32, 50, 32);
  cage.lineTo(30, 32);
  cage.closePath();
  cage.holes.push(hole(46, 19, 2));
  frame.add(sidePlate(cage, -13, 2), sidePlate(cage, 13, 2));
  const camera = new THREE.Group();
  const housing = new THREE.Mesh(new THREE.BoxGeometry(19, 19, 18), plastic(0x1d2025, 0.45));
  const barrel = new THREE.Mesh(new THREE.CylinderGeometry(7, 7.5, 9, 28), plastic(0x111316, 0.35));
  barrel.rotation.x = Math.PI / 2;
  barrel.position.z = -13;
  const ring = new THREE.Mesh(new THREE.TorusGeometry(7.1, 0.7, 10, 28), metal(FPV.bellAccent, 0.3));
  ring.position.z = -17.6;
  const glass = new THREE.Mesh(
    new THREE.CircleGeometry(5.6, 28),
    new THREE.MeshPhysicalMaterial({ color: 0x0b1726, metalness: 0.2, roughness: 0.05, clearcoat: 1, iridescence: 0.8 }),
  );
  glass.position.z = -17.6;
  glass.rotation.y = Math.PI;
  camera.add(housing, barrel, ring, glass);
  camera.rotation.x = 30 * (Math.PI / 180);
  camera.position.set(0, 19, -44);
  frame.add(camera);

  // The pack in its shrink-wrap, with a printed band, and the strap over it.
  const packY = 34;
  const pack = new THREE.Mesh(new RoundedBoxGeometry(35, 33, 76, 4, 3.5), plastic(FPV.lipo, 0.45));
  pack.position.set(0, packY + 16.5, 0);
  const band = new THREE.Mesh(new RoundedBoxGeometry(35.6, 33.6, 30, 4, 3.6), plastic(FPV.tpu, 0.45));
  band.position.set(0, packY + 16.5, -16);
  const strap = new THREE.Mesh(new RoundedBoxGeometry(36.4, 34.4, 14, 2, 3.8), plastic(FPV.strap, 0.9));
  strap.position.set(0, packY + 16.5, 12);
  frame.add(pack, band, strap);

  // XT60 lead out of the back of the pack, and the capacitor under the stack.
  const xt60 = new THREE.Mesh(new RoundedBoxGeometry(16, 8, 16, 2, 1.5), plastic(FPV.xt60, 0.5));
  xt60.position.set(-9, 38, 47);
  for (const [color, dx] of [[0xc62828, -12], [0x16181c, -6]] as const) {
    const lead = new THREE.Mesh(new THREE.CylinderGeometry(2, 2, 12, 10), plastic(color, 0.5));
    lead.rotation.x = Math.PI / 2;
    lead.position.set(dx, 38, 38);
    frame.add(lead);
  }
  const cap = new THREE.Mesh(new THREE.CylinderGeometry(5, 5, 16, 20), plastic(0x16181c, 0.4));
  cap.rotation.z = Math.PI / 2;
  cap.position.set(0, 13, 40);
  frame.add(xt60, cap);

  // TPU antenna mount on the back of the top plate, and its whip.
  const mount = new THREE.Mesh(new RoundedBoxGeometry(14, 10, 10, 2, 2), plastic(FPV.tpu, 0.7));
  mount.position.set(10, 39, 46);
  const whip = new THREE.Mesh(new THREE.CylinderGeometry(1.8, 2.2, 40, 10), plastic(0x4a515c, 0.5));
  whip.rotation.x = 0.7;
  whip.position.set(10, 55, 58);
  const tip = new THREE.Mesh(new THREE.CapsuleGeometry(3, 9, 4, 12), plastic(FPV.tpu, 0.5));
  tip.rotation.x = 0.7;
  tip.position.set(10, 70, 71);
  frame.add(mount, whip, tip);

  // Motors and props, in motor order.
  const props: THREE.Object3D[] = [];
  motorsMm.forEach(([x, z], i) => {
    const m = motor2207();
    m.position.set(x, 5, z);
    frame.add(m);
    const prop = triBlade(64, z < 0 ? FPV.frontProp : FPV.rearProp, spins[i] ?? 1);
    prop.position.set(x, 5 + 26.5, z);
    frame.add(prop);
    props.push(prop);
  });

  frame.scale.setScalar(MM);
  // Centre the model on its body rather than on the arm plane.
  frame.position.y = -30 * MM;
  const group = new THREE.Group();
  group.add(frame);
  return { group, props, spins, label };
}

/** The 30.5 mm stack pattern. */
const STACK = [[-15.25, -15.25], [15.25, -15.25], [-15.25, 15.25], [15.25, 15.25]] as const;

/** A swept flying wing with elevons and one or two pusher motors. */
function flyingWing(motors: 1 | 2, label: string): AirframeModel {
  const group = new THREE.Group();

  // Plan view in (x, z): nose at -z, wingtips swept back.
  const shape = new THREE.Shape();
  shape.moveTo(0, -0.75);
  shape.lineTo(1.5, 0.35);
  shape.lineTo(1.5, 0.62);
  shape.lineTo(0.22, 0.3);
  shape.lineTo(0, 0.42);
  shape.lineTo(-0.22, 0.3);
  shape.lineTo(-1.5, 0.62);
  shape.lineTo(-1.5, 0.35);
  shape.closePath();
  const wingGeometry = new THREE.ExtrudeGeometry(shape, { depth: 0.07, bevelEnabled: true, bevelSize: 0.02, bevelThickness: 0.02, bevelSegments: 1 });
  wingGeometry.rotateX(Math.PI / 2);
  wingGeometry.translate(0, 0.035, 0);
  const wing = new THREE.Mesh(wingGeometry, material(COLORS.body));
  group.add(wing, outline(wing));

  const pod = new THREE.Mesh(new THREE.CapsuleGeometry(0.11, 0.6, 6, 16), material(COLORS.arm));
  pod.rotation.x = Math.PI / 2;
  pod.position.set(0, 0.08, -0.15);
  group.add(pod);

  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.2, 16), material(COLORS.nose, { emissive: COLORS.nose, emissiveIntensity: 0.35 }));
  nose.rotation.x = -Math.PI / 2;
  nose.position.set(0, 0.08, -0.62);
  group.add(nose);

  for (const side of [-1, 1]) {
    const elevon = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.03, 0.12), material(COLORS.elevon, { emissive: COLORS.elevon, emissiveIntensity: 0.15 }));
    elevon.position.set(side * 0.95, 0.01, 0.55);
    elevon.rotation.y = side * -0.22;
    group.add(elevon);
  }

  const props: THREE.Object3D[] = [];
  const xs = motors === 2 ? [-0.55, 0.55] : [0];
  for (const x of xs) {
    const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.14, 16), material(COLORS.motor));
    motor.rotation.x = Math.PI / 2;
    motor.position.set(x, 0.06, 0.5);
    group.add(motor);
    const prop = propeller(0.26);
    prop.rotation.x = Math.PI / 2; // a pusher: the disc faces backwards
    prop.position.set(x, 0.06, 0.6);
    group.add(prop);
    props.push(prop);
  }
  return { group, props, spins: motors === 2 ? [1, -1] : [1], label };
}

const D = 0.75; // arm reach

export function buildAirframe(airframe: number | null): AirframeModel {
  const label = airframe === null ? 'Airframe unknown — drawn as a quad X' : AIRFRAME_NAMES[airframe] ?? `Airframe ${airframe}`;
  switch (airframe) {
    case 1:
      return flyingWing(2, label);
    case 7:
      return flyingWing(1, label);
    // Motor order below is the firmware's mixer row order (ak_mixer.c).
    case 3:
      // Quad +: rear, right, left, front.
      return multirotor([[0, D * 1.2], [D * 1.2, 0], [-D * 1.2, 0], [0, -D * 1.2]], [-1, 1, 1, -1], label);
    case 4:
      // Y4: rear top, front right, rear bottom, front left.
      return multirotor([[0, D * 1.3], [D, -D * 0.7], [0, D * 1.3], [-D, -D * 0.7]], [-1, 1, 1, -1], label);
    case 6:
      // Tri: rear, right, left.
      return multirotor([[0, D * 1.3], [D, -D * 0.7], [-D, -D * 0.7]], [1, -1, 1], label);
    case 2:
      // Quad X numbered clockwise from front left: 1 FL, 2 FR, 3 RR, 4 RL.
      return multirotor([[-D, -D], [D, -D], [D, D], [-D, D]], [-1, 1, -1, 1], label);
    case 0:
    case 5:
    default:
      // Quad X in the Betaflight order the firmware uses: 1 RR, 2 FR, 3 RL, 4 FL,
      // props in: 1 and 4 clockwise, 2 and 3 counter-clockwise.
      return multirotor([[D, D], [D, -D], [-D, D], [-D, -D]], [-1, 1, 1, -1], label);
  }
}
