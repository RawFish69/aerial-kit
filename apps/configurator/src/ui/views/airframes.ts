import * as THREE from 'three';

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

const FPV = {
  carbon: 0x1f2227,
  carbonEdge: 0x4a5058,
  aluminium: 0x9aa3ad,
  bell: 0x2b2f36,
  bellAccent: 0xd13913, // a coloured motor bell, as most 2306s have
  lipo: 0x30343b,
  lipoLabel: 0xf0b726,
  strap: 0x111316,
  lens: 0x0a0c0f,
  camera: 0x25282d,
  frontProp: 0xec9a3c, // orange props at the front, the FPV convention
  rearProp: 0x4c90f0,
};

function carbon(): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({ color: FPV.carbon, roughness: 0.35, metalness: 0.45 });
}

/** A rounded rectangle plate in the (x, z) plane, `thickness` tall. */
function plate(width: number, length: number, radius: number, thickness: number): THREE.Mesh {
  const shape = new THREE.Shape();
  const w = width / 2;
  const l = length / 2;
  shape.moveTo(-w + radius, -l);
  shape.lineTo(w - radius, -l);
  shape.quadraticCurveTo(w, -l, w, -l + radius);
  shape.lineTo(w, l - radius);
  shape.quadraticCurveTo(w, l, w - radius, l);
  shape.lineTo(-w + radius, l);
  shape.quadraticCurveTo(-w, l, -w, l - radius);
  shape.lineTo(-w, -l + radius);
  shape.quadraticCurveTo(-w, -l, -w + radius, -l);
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: thickness, bevelEnabled: false });
  geometry.rotateX(Math.PI / 2);
  return new THREE.Mesh(geometry, carbon());
}

/** A tapered arm from the centre to (x, z): wide at the root, narrow at the motor. */
function arm(x: number, z: number, thickness: number): THREE.Mesh {
  const length = Math.hypot(x, z);
  const shape = new THREE.Shape();
  shape.moveTo(-0.085, 0);
  shape.lineTo(0.085, 0);
  shape.lineTo(0.06, length);
  shape.absarc(0, length, 0.06, 0, Math.PI, false);
  shape.lineTo(-0.085, 0);
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: thickness, bevelEnabled: false });
  geometry.rotateX(Math.PI / 2); // shape y -> +z
  const mesh = new THREE.Mesh(geometry, carbon());
  mesh.rotation.y = Math.atan2(x, z);
  return mesh;
}

/** A 2306-style brushless motor, base at y = 0. */
function motor(): THREE.Group {
  const group = new THREE.Group();
  const base = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.08, 0.025, 24), material(FPV.aluminium, { metalness: 0.8, roughness: 0.3 }));
  base.position.y = 0.0125;
  const bell = new THREE.Mesh(new THREE.CylinderGeometry(0.074, 0.074, 0.075, 24), material(FPV.bell, { metalness: 0.7, roughness: 0.35 }));
  bell.position.y = 0.0625;
  const ring = new THREE.Mesh(new THREE.CylinderGeometry(0.0755, 0.0755, 0.018, 24), material(FPV.bellAccent, { metalness: 0.6, roughness: 0.4 }));
  ring.position.y = 0.09;
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.06, 10), material(FPV.aluminium, { metalness: 0.9, roughness: 0.2 }));
  shaft.position.y = 0.125;
  group.add(base, bell, ring, shaft);
  return group;
}

/** A tri-blade prop, hub at the origin, spinning about +Y. */
function triBlade(radius: number, color: number): THREE.Group {
  const prop = new THREE.Group();
  const bladeMaterial = new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.85, roughness: 0.4, side: THREE.DoubleSide });
  for (let i = 0; i < 3; i++) {
    const shape = new THREE.Shape();
    shape.moveTo(0.02, -0.018);
    shape.quadraticCurveTo(radius * 0.55, -0.055, radius, -0.012);
    shape.quadraticCurveTo(radius * 1.01, 0.012, radius * 0.9, 0.02);
    shape.quadraticCurveTo(radius * 0.5, 0.04, 0.02, 0.018);
    shape.closePath();
    const geometry = new THREE.ShapeGeometry(shape, 12);
    geometry.rotateX(-Math.PI / 2);
    const blade = new THREE.Mesh(geometry, bladeMaterial);
    blade.rotation.y = (i * 2 * Math.PI) / 3;
    blade.rotation.z = 0.12; // a little pitch, so the blades read as blades
    prop.add(blade);
  }
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.03, 16), material(color));
  prop.add(hub);
  const nut = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.03, 6), material(FPV.aluminium, { metalness: 0.9 }));
  nut.position.y = 0.025;
  prop.add(nut);
  const disc = new THREE.Mesh(
    new THREE.CircleGeometry(radius, 48),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.07, side: THREE.DoubleSide }),
  );
  disc.rotation.x = -Math.PI / 2;
  prop.add(disc);
  return prop;
}

/**
 * A 5-inch freestyle quad: true-X carbon arms, a stack between two plates, a
 * LiPo strapped on top, a tilted FPV camera at the front and a VTX antenna at
 * the back. Motors sit at the given (x, z) positions in the mixer's order.
 */
function multirotor(positions: ReadonlyArray<readonly [number, number]>, label: string): AirframeModel {
  const group = new THREE.Group();
  const armThickness = 0.045;

  for (const [x, z] of positions) group.add(arm(x, z, armThickness));

  const bottom = plate(0.32, 0.62, 0.06, 0.02);
  bottom.position.y = armThickness + 0.02;
  group.add(bottom);

  // Standoffs and the FC/ESC stack between the plates.
  for (const [sx, sz] of [[-0.12, -0.24], [0.12, -0.24], [-0.12, 0.24], [0.12, 0.24]]) {
    const standoff = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.16, 8), material(FPV.aluminium, { metalness: 0.85, roughness: 0.3 }));
    standoff.position.set(sx!, 0.15, sz!);
    group.add(standoff);
  }
  for (const [y, color] of [[0.1, 0x1e5a2f], [0.15, 0x1a3f7a]] as const) {
    const board = new THREE.Mesh(new THREE.BoxGeometry(0.17, 0.012, 0.17), material(color, { roughness: 0.6 }));
    board.position.set(0, y, 0.02);
    group.add(board);
  }

  const top = plate(0.3, 0.56, 0.05, 0.018);
  top.position.y = 0.245;
  group.add(top);

  // The LiPo and its strap.
  const lipo = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.17, 0.46), material(FPV.lipo, { roughness: 0.7 }));
  lipo.position.set(0, 0.335, 0.03);
  const label1 = new THREE.Mesh(new THREE.BoxGeometry(0.202, 0.06, 0.3), material(FPV.lipoLabel, { roughness: 0.6 }));
  label1.position.set(0, 0.34, 0.03);
  const strap = new THREE.Mesh(new THREE.BoxGeometry(0.23, 0.2, 0.05), material(FPV.strap, { roughness: 0.9 }));
  strap.position.set(0, 0.33, 0.03);
  group.add(lipo, label1, strap);

  // FPV camera in its cage, tilted up 25 degrees, at the front (-z).
  const cameraMount = new THREE.Group();
  const camera = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 0.11), material(FPV.camera));
  const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.045, 0.06, 20), material(FPV.lens, { metalness: 0.2, roughness: 0.1 }));
  lens.rotation.x = Math.PI / 2;
  lens.position.z = -0.08;
  const glass = new THREE.Mesh(new THREE.CircleGeometry(0.032, 20), material(0x3a6ea5, { metalness: 0.9, roughness: 0.05, emissive: 0x1a3352 }));
  glass.position.z = -0.111;
  glass.rotation.y = Math.PI;
  cameraMount.add(camera, lens, glass);
  cameraMount.rotation.x = 25 * (Math.PI / 180);
  cameraMount.position.set(0, 0.17, -0.31);
  group.add(cameraMount);

  // VTX antenna at the back.
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.26, 8), material(0x222222));
  mast.rotation.x = -0.6;
  mast.position.set(0, 0.24, 0.38);
  const cap = new THREE.Mesh(new THREE.SphereGeometry(0.03, 12, 12), material(0xd13913));
  cap.position.set(0, 0.345, 0.455);
  group.add(mast, cap);

  const props: THREE.Object3D[] = [];
  positions.forEach(([x, z]) => {
    const m = motor();
    m.position.set(x, armThickness, z);
    group.add(m);
    const prop = triBlade(0.36, z < 0 ? FPV.frontProp : FPV.rearProp);
    prop.position.set(x, armThickness + 0.145, z);
    group.add(prop);
    props.push(prop);
  });

  // Centre the model on its body rather than on the arm plane.
  group.position.y = -0.17;
  return { group, props, label };
}

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
  return { group, props, label };
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
      return multirotor([[0, D * 1.2], [D * 1.2, 0], [-D * 1.2, 0], [0, -D * 1.2]], label);
    case 4:
      // Y4: rear top, front right, rear bottom, front left.
      return multirotor([[0, D * 1.3], [D, -D * 0.7], [0, D * 1.3], [-D, -D * 0.7]], label);
    case 6:
      // Tri: rear, right, left.
      return multirotor([[0, D * 1.3], [D, -D * 0.7], [-D, -D * 0.7]], label);
    case 2:
      // Quad X numbered clockwise from front left: 1 FL, 2 FR, 3 RR, 4 RL.
      return multirotor([[-D, -D], [D, -D], [D, D], [-D, D]], label);
    case 0:
    case 5:
    default:
      // Quad X in the Betaflight order the firmware uses: 1 RR, 2 FR, 3 RL, 4 FL.
      return multirotor([[D, D], [D, -D], [-D, D], [-D, -D]], label);
  }
}
