"""MuJoCo MJCF for the single-motor flying wing.

Wing, pod, fins, elevons, hinge rods and nav lights come from the twin-wing
procedural generator, fed a planform view of ``single_wing.yaml`` (the
single wing's YAML stays the only source; :func:`twin_geometry_view` only
re-labels it). The twin's two nacelles and props are dropped, and a
centreline pusher motor pod, bell, spinner and prop are added.

Body tree (body FRD, root origin = reference CG):

    body (freejoint)              wing, pod, fins, motor pod, hinge rods
      left_elevon / right_elevon  hinge about the swept hinge line, TE-up +
      prop (hinge +x)             blades, tips, hub, bell, spinner

Mass split: elevons (``elevons.mass_kg_each``) and the rotor
(``articulation.rotor.mass_kg``) are child bodies; the root takes the rest so
the composite is the spec's mass and inertia about the CG exactly.
"""

from __future__ import annotations

import math

import numpy as np

from aerial_kit.airframes.twin_wing.geometry import Mesh, TwinWingGeometry, revolve
from aerial_kit.airframes.twin_wing.mjcf import _FPV_TILT_AXES, _f, _full_inertia, _mesh_xml, _rgba, _vec
from aerial_kit.airframes.twin_wing.spec import TwinWingSpec

from .spec import SingleWingSpec

KEEP_TWIN_PARTS = ("center_body", "hatch", "camera_window", "wing_top", "wing_bottom",
                   "wingtip_fin", "nav_light", "hinge_rod", "elevon", "elevon_horn")


def twin_geometry_view(spec: SingleWingSpec) -> TwinWingSpec:
    """The single wing's planform in the shape the twin-wing generator reads."""
    d = spec.data
    art = d["articulation"]
    wing = dict(d["wing"])
    airfoil = dict(wing.get("airfoil", {}))
    wing["spanwise_sections"] = int(airfoil.pop("spanwise_sections", 8))
    wing["airfoil"] = airfoil
    body = dict(d["center_body"])
    body.setdefault("top_fraction", 0.66)
    nose, tail = float(body["nose_x_m"]), float(body["tail_x_m"])
    body.setdefault("hatch", {"from_x_m": nose - 0.11, "to_x_m": tail + 0.13, "width_fraction": 0.62})
    body.setdefault("camera_window_x_m", nose - 0.012)
    elevons = dict(d["elevons"])
    elevons.setdefault("servo", art["elevon_servo"])
    elevons.setdefault("command_index", {"left": 1, "right": 2})
    elevons["command_index"] = {"left": 2, "right": 3}   # twin validator's slots; unused here
    pv = art["prop_visual"]
    spin = int(d["propulsion"]["motor"]["spin"])
    materials = dict(d["materials"])
    materials.setdefault("center_body", materials.get("body", [0.3, 0.33, 0.37, 1.0]))
    materials.setdefault("prop_blade", materials.get("prop", [0.08, 0.08, 0.09, 1.0]))
    materials.update({k: v for k, v in art["visual_materials"].items() if k not in materials})
    view = {
        "name": d.get("name", "single_wing"),
        "mass_properties": d["mass_properties"],
        "aero_reference": d["aero_reference"],
        "wing": wing,
        "center_body": body,
        "fins": d["fins"],
        "elevons": elevons,
        "propulsion": {
            "max_thrust_n": d["propulsion"]["max_thrust_n"],
            "thrust_tau_s": d["propulsion"]["thrust_tau_s"],
            # Placeholder twin motors (their parts are discarded); the prop
            # blade generator only reads `spin` and the prop block.
            "motors": {"left": {"y_m": -0.3, "spin": -spin}, "right": {"y_m": 0.3, "spin": spin}},
            "thrust_line_z_m": 0.0,
            "prop_plane_ahead_of_le_m": 0.05,
            "prop": {"diameter_m": d["propulsion"]["propeller"]["diameter_m"],
                     "blades": d["propulsion"]["propeller"]["blades"], **pv},
            "rotor": {"mass_kg": art["rotor"]["mass_kg"], "physical_inertia_kg_m2": 1e-5,
                      "physical_max_speed_rad_s": 1.0, "visual_max_speed_rad_s": 1.0},
            "nacelle": {"radius_m": 0.02, "length_m": 0.1, "bell_radius_m": 0.014, "bell_length_m": 0.02},
        },
        "materials": materials,
        "collision": d["collision"],
    }
    return TwinWingSpec(view, source=f"{spec.source} (planform view)")


class SingleWingMass:
    """Root/child mass split so the composite matches the spec exactly."""

    def __init__(self, spec: SingleWingSpec, view: TwinWingSpec):
        self.spec = spec
        rotor = spec.data["articulation"]["rotor"]
        self.prop_pos = np.asarray(spec.motor_position, dtype=float)
        axial = float(spec.prop_inertia_kg_m2) + float(spec.motor_inertia_kg_m2)
        self.rotor_mass = float(rotor["mass_kg"])
        self.rotor_inertia = np.diag([axial, 0.5 * axial + 1e-8, 0.5 * axial + 1e-8])
        self.elevons = {side: view.elevon_mass(side) for side in ("left", "right")}
        children = [(m.mass_kg, m.com_m, m.inertia_kg_m2) for m in self.elevons.values()]
        children.append((self.rotor_mass, self.prop_pos, self.rotor_inertia))
        total = float(spec.total_mass_kg)
        m_root = total - sum(c[0] for c in children)
        com_root = -sum(c[0] * c[1] for c in children) / m_root

        def parallel(m, d):
            return m * (float(d @ d) * np.eye(3) - np.outer(d, d))

        inertia = np.diag(np.asarray(spec.inertia_diag, dtype=float))
        for m, c, i in children:
            inertia = inertia - i - parallel(m, c)
        inertia = inertia - parallel(m_root, com_root)
        eig = np.linalg.eigvalsh(inertia)
        if eig.min() <= 0.0 or eig.max() > eig.sum() - eig.max():
            raise ValueError(f"single wing root inertia not physical: {eig}")
        self.root_mass, self.root_com, self.root_inertia = m_root, com_root, 0.5 * (inertia + inertia.T)


def _pusher_parts(spec: SingleWingSpec, geo: TwinWingGeometry) -> dict[str, list]:
    """Motor pod on the root; bell, spinner and blades on the prop body."""
    art = spec.data["articulation"]
    pod = art["motor_pod"]
    pv = art["prop_visual"]
    R, L = float(pod["radius_m"]), float(pod["length_m"])
    hub = np.asarray(spec.motor_position, dtype=float)
    # Pod runs forward from just ahead of the prop into the wing root.
    x0 = hub[0] + 0.03
    pod_mesh = revolve([(x0 + L, 0.25 * R), (x0 + L - 0.02, 0.9 * R), (x0 + 0.3 * L, R),
                        (x0 + 0.004, 0.85 * R), (x0, 0.7 * R)], 18).translated([0.0, hub[1], hub[2]])
    bell = revolve([(0.028, 0.75 * R), (0.026, 0.8 * R), (0.008, 0.8 * R), (0.006, 0.5 * R)], 16)
    sr, sl = float(pv["spinner_radius_m"]), float(pv["spinner_length_m"])
    spinner = revolve([(0.005, sr)] + [(-sl * (k / 7.0), sr * math.sqrt(max(1.0 - (k / 7.0) ** 2, 0.0)) ** 0.9)
                                        for k in range(1, 8)], 18)
    blades, tips = geo.prop_blades("right")
    hub_mesh = revolve([(0.006, float(pv["hub_radius_m"])), (-0.006, float(pv["hub_radius_m"]))], 14)
    return {
        "body": [("motor_pod", "nacelle", pod_mesh)],
        "prop": [("prop_blades", "prop_blade", blades), ("prop_tips", "prop_tip", tips),
                 ("prop_hub", "prop_blade", hub_mesh), ("motor_bell", "motor_bell", bell),
                 ("spinner", "spinner", spinner)],
    }


def single_wing_parts(spec: SingleWingSpec):
    """[(name, body, material, Mesh)] for the whole aircraft."""
    view = twin_geometry_view(spec)
    geo = TwinWingGeometry(view)
    parts = []
    for p in geo.parts():
        base = p.name.split("_", 1)[1] if p.name.startswith(("left_", "right_")) else p.name
        if base in KEEP_TWIN_PARTS:
            parts.append((p.name, p.body, p.material, p.mesh))
    for body, items in _pusher_parts(spec, geo).items():
        for name, material, mesh in items:
            parts.append((name, body, material, mesh))
    return parts, view, geo


def single_wing_mjcf(spec: SingleWingSpec, timestep: float = 0.002, gravity_mps2: float = 9.81) -> str:
    parts, view, geo = single_wing_parts(spec)
    mass = SingleWingMass(spec, view)
    art = spec.data["articulation"]
    servo = art["elevon_servo"]
    rotor = art["rotor"]
    travel, limit = spec.max_elevon_rad, spec.elevon_limit_rad
    v_max = float(rotor["visual_max_speed_rad_s"])
    tau = max(float(spec.thrust_tau_s), 1e-3)
    motor_kv = float(mass.rotor_inertia[0, 0]) / tau
    spin = int(spec.prop_spin)

    cb = spec.data["center_body"]
    fpv_eye = [float(cb["nose_x_m"]) + 0.02, 0.0, -0.18 * float(cb["max_height_m"])]
    assets = [_mesh_xml(name, mesh) for name, _, _, mesh in parts]
    materials = [f'    <material name="{n}" rgba="{_rgba(c)}" specular="0.35" shininess="0.4"/>'
                 for n, c in view.materials.items()]

    def geoms(body, indent):
        return "\n".join(f'{indent}<geom name="{n}" type="mesh" mesh="{n}" material="{m}" class="visual"/>'
                         for n, b, m, _ in parts if b == body)

    children = []
    for side in ("left", "right"):
        name = f"{side}_elevon"
        origin = geo.body_origin(name)
        _, _, axis = view.elevon_hinge(side)
        c = mass.elevons[side]
        te = np.array([view.te_x(abs(origin[1])), origin[1], 0.0]) - origin
        children.append(f"""\
      <body name="{name}" pos="{_vec(origin)}">
        <joint name="{name}_joint" type="hinge" axis="{_vec(axis)}" range="{_f(-limit)} {_f(limit)}"
               damping="{_f(servo['joint_damping_nms'])}" armature="{_f(servo['armature_kg_m2'])}"/>
        <inertial pos="{_vec(c.com_m - origin)}" mass="{_f(c.mass_kg)}" fullinertia="{_full_inertia(c.inertia_kg_m2)}"/>
{geoms(name, '        ')}
        <site name="{name}_te_marker" pos="{_vec(te)}" size="0.004"/>
      </body>""")
    children.append(f"""\
      <body name="prop" pos="{_vec(mass.prop_pos)}">
        <joint name="prop_joint" type="hinge" axis="1 0 0" damping="{_f(rotor['bearing_damping_nms'])}"/>
        <inertial pos="0 0 0" mass="{_f(mass.rotor_mass)}" fullinertia="{_full_inertia(mass.rotor_inertia)}"/>
{geoms('prop', '        ')}
      </body>""")
    lo, hi = (0.0, v_max) if spin > 0 else (-v_max, 0.0)
    col = spec.data["collision"]
    cap_r = float(col["wing_capsule_radius_m"])
    collision = [f'      <geom name="collision_center" type="box" size="{_vec(col["center_box_half_m"])}" class="collision"/>']
    for side, sgn in (("left", -1.0), ("right", 1.0)):
        y = spec.half_span - cap_r
        a = [spec.le_x(0.0) - 0.5 * spec.chord(0.0), 0.0, 0.0]
        b = [spec.le_x(y) - 0.5 * spec.chord(y), sgn * y, spec.wing_z(y)]
        collision.append(f'      <geom name="collision_{side}_wing" type="capsule" fromto="{_vec(a)} {_vec(b)}" '
                         f'size="{_f(cap_r)}" class="collision"/>')
    return f"""\
<mujoco model="{spec.data.get('name', 'single_wing')}">
  <!-- Generated from {spec.source} by aerial_kit.airframes.single_wing.mjcf. Body FRD, world ENU. -->
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 -{_f(gravity_mps2)}" timestep="{_f(timestep)}" integrator="implicit"/>
  <visual>
    <global offwidth="1600" offheight="900" azimuth="150" elevation="-18"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
    <map znear="0.004" zfar="600" shadowclip="3"/>
  </visual>
  <statistic extent="2.5" center="0 0 0"/>
  <default>
    <default class="visual"><geom contype="0" conaffinity="0" group="1" density="0"/></default>
    <default class="collision"><geom contype="1" conaffinity="1" group="3" density="0" rgba="0.8 0.2 0.2 0.25"/></default>
    <site group="4" rgba="1 0 1 1"/>
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.62 0.75 0.90" rgb2="0.95 0.96 0.98" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.50 0.58 0.47" rgb2="0.46 0.53 0.43"
             width="512" height="512" mark="edge" markrgb="0.40 0.46 0.38"/>
    <material name="ground" texture="grid" texrepeat="200 200" reflectance="0.0"/>
{chr(10).join(materials)}
{chr(10).join(assets)}
  </asset>
  <worldbody>
    <light name="sun" pos="0 0 200" dir="-0.35 0.25 -1" directional="true" diffuse="0.75 0.75 0.72"
           specular="0.25 0.25 0.25" castshadow="true"/>
    <geom name="ground" type="plane" size="1000 1000 0.1" material="ground"/>
    <body name="body" pos="0 0 0">
      <freejoint name="root"/>
      <inertial pos="{_vec(mass.root_com)}" mass="{_f(mass.root_mass)}" fullinertia="{_full_inertia(mass.root_inertia)}"/>
      <camera name="chase" mode="fixed" pos="-2.4 0 -0.8" xyaxes="0 1 0 0.33 0 -0.94"/>
      <!-- On-board nose camera, 5 deg down (FRD body). -->
      <camera name="fpv" mode="fixed" pos="{_vec(fpv_eye)}" xyaxes="0 1 0 {_FPV_TILT_AXES}" fovy="100"/>
      <site name="cg" pos="0 0 0" size="0.006"/>
{geoms('body', '      ')}
{chr(10).join(collision)}
{chr(10).join(children)}
    </body>
  </worldbody>
  <actuator>
    <position name="left_elevon_servo" joint="left_elevon_joint" kp="{_f(servo['kp_nm_per_rad'])}"
              kv="{_f(servo['kv_nms_per_rad'])}" ctrlrange="{_f(-travel)} {_f(travel)}"
              forcerange="{_f(-float(servo['force_limit_nm']))} {_f(servo['force_limit_nm'])}"/>
    <position name="right_elevon_servo" joint="right_elevon_joint" kp="{_f(servo['kp_nm_per_rad'])}"
              kv="{_f(servo['kv_nms_per_rad'])}" ctrlrange="{_f(-travel)} {_f(travel)}"
              forcerange="{_f(-float(servo['force_limit_nm']))} {_f(servo['force_limit_nm'])}"/>
    <velocity name="prop_motor" joint="prop_joint" kv="{_f(motor_kv)}" ctrlrange="{_f(lo)} {_f(hi)}"/>
  </actuator>
</mujoco>
"""
