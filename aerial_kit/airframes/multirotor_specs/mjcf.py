"""MuJoCo MJCF for a multirotor spec.

Body tree (body FLU, root origin = composite CG):

    body (freejoint "root")         frame, arms, hub, battery, gear, camera
      <rotor>_rotor (hinge +z)      bell, hub, prop blades -- visual spin only
      [payload bodies, welded]      CAD-style payloads (battery)

Per rotor, ``<rotor>_thrust`` is a ``general`` actuator on a site fixed to the
*root* body at the rotor hub: ``gear = [0 0 1 0 0 k_yaw]`` so one control
(newtons) produces thrust along body +z **and** the reaction yaw torque, and
``dyntype="filter"`` gives the first-order motor lag (``motor_tau_s``). That is
MuJoCo's standard multirotor pattern: the rotor forces are integrated by
MuJoCo, not applied as an external wrench. ``data.act`` is the lagged thrust.

``<rotor>_spin`` is a velocity actuator turning the visual rotor at a
display-rate speed derived from the commanded thrust; its reaction torque
(rotor inertia times spin-up) acts on the frame like a real motor's.
"""

from __future__ import annotations

import math

import numpy as np

from aerial_kit.airframes.twin_wing.geometry import Mesh

from .geometry import MultirotorGeometry, rotor_body
from .spec import CAD_TO_FLU, MultirotorSpec


def _f(x: float) -> str:
    return f"{float(x):.9g}"


def _vec(v) -> str:
    return " ".join(_f(x) for x in np.asarray(v, dtype=float).ravel())


def _quat_of(rot: np.ndarray) -> np.ndarray:
    m = np.asarray(rot, float)
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    if w > 1e-6:
        return np.array([w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w),
                         (m[1, 0] - m[0, 1]) / (4 * w)])
    # 180-degree rotations (not used by the bundled specs).
    i = int(np.argmax(np.diag(m)))
    q = np.zeros(4)
    q[1 + i] = math.sqrt(max(0.0, 1.0 + 2 * m[i, i] - np.trace(m))) / 2.0
    return q


def _full_inertia(m: np.ndarray) -> str:
    return _vec([m[0, 0], m[1, 1], m[2, 2], m[0, 1], m[0, 2], m[1, 2]])


def _mesh_xml(name: str, mesh: Mesh) -> str:
    verts = " ".join(f"{x:.5f}" for x in mesh.vertices.ravel())
    faces = " ".join(str(int(i)) for i in mesh.faces.ravel())
    return f'    <mesh name="{name}" vertex="{verts}" face="{faces}"/>'


def multirotor_mjcf(spec: MultirotorSpec, timestep: float = 0.002,
                    ground: bool = True) -> str:
    geo = MultirotorGeometry(spec)
    parts = geo.parts()
    assets, geoms = [], {}
    for p in parts:
        if p.file is not None:
            assets.append(f'    <mesh name="{p.name}" file="{p.file.path}" '
                          f'scale="{_vec([p.file.scale] * 3)}"/>')
            geoms.setdefault(p.body, []).append(
                f'<geom name="{p.name}" type="mesh" mesh="{p.name}" material="{p.material}" '
                f'class="visual" pos="{_vec(p.file.pos)}" quat="{_vec(_quat_of(p.file.rot))}"/>')
        else:
            assets.append(_mesh_xml(p.name, p.mesh))
            geoms.setdefault(p.body, []).append(
                f'<geom name="{p.name}" type="mesh" mesh="{p.name}" material="{p.material}" class="visual"/>')
    materials = [
        f'    <material name="{name}" rgba="{" ".join(f"{float(c):.4g}" for c in rgba)}" '
        f'specular="0.3" shininess="0.4"/>'
        for name, rgba in spec.materials.items()
    ]
    if "stator" not in spec.materials:
        materials.append('    <material name="stator" rgba="0.55 0.55 0.58 1" specular="0.3"/>')

    root = spec.root_mass()
    rotor_masses = {m.name: m for m in spec.rotor_masses()}
    rcfg = spec.rotor_cfg
    v_max = float(rcfg["visual_max_speed_rad_s"])

    rotor_xml, sites, actuators = [], [], []
    for rotor in spec.rotors:
        body = rotor_body(rotor)
        mass = rotor_masses[body]
        hub = rotor.position_m
        rotor_xml.append(f"""\
      <body name="{body}" pos="{_vec(hub)}">
        <joint name="{rotor.name}_rotor_joint" type="hinge" axis="0 0 1"
               damping="{_f(rcfg.get('bearing_damping_nms', 1e-7))}"/>
        <inertial pos="{_vec(mass.com_m - hub)}" mass="{_f(mass.mass_kg)}"
                  fullinertia="{_full_inertia(mass.inertia_kg_m2)}"/>
        {chr(10).join('        ' + g for g in geoms.get(body, [])).lstrip()}
      </body>""")
        sites.append(f'      <site name="{rotor.name}_thrust_site" pos="{_vec(hub)}" size="0.004"/>')
        if spec.propulsion_model == "rpm":
            # The backend integrates motor speed, battery and ground effect and
            # sets the resulting thrust; MuJoCo applies it (no filter here).
            actuators.append(
                f'    <general name="{rotor.name}_thrust" site="{rotor.name}_thrust_site" '
                f'gear="0 0 1 0 0 {_f(rotor.body_yaw_torque_per_n)}" gainprm="1" '
                f'ctrlrange="0 {_f(1.5 * rotor.max_thrust_n)}"/>')
        else:
            actuators.append(
                f'    <general name="{rotor.name}_thrust" site="{rotor.name}_thrust_site" '
                f'gear="0 0 1 0 0 {_f(rotor.body_yaw_torque_per_n)}" dyntype="filter" '
                f'dynprm="{_f(rotor.motor_tau_s)}" gainprm="1" ctrlrange="0 {_f(rotor.max_thrust_n)}"/>')
        # Spin motor: time constant equal to the thrust lag.
        kv = spec.rotor_axial_inertia() / max(rotor.motor_tau_s, 1e-3)
        lo, hi = (0.0, v_max) if rotor.spin > 0 else (-v_max, 0.0)
        actuators.append(
            f'    <velocity name="{rotor.name}_spin" joint="{rotor.name}_rotor_joint" '
            f'kv="{_f(kv)}" ctrlrange="{_f(lo)} {_f(hi)}"/>')

    payload_xml = []
    for pm in spec.payload_masses():
        payload_xml.append(f"""\
      <body name="{pm.name}_mass" pos="0 0 0">
        <inertial pos="{_vec(pm.com_m)}" mass="{_f(pm.mass_kg)}" fullinertia="{_full_inertia(pm.inertia_kg_m2)}"/>
      </body>""")

    collision = _collision_xml(spec)
    camera = ""
    cam = spec.camera
    if cam:
        t = math.radians(float(cam["uptilt_deg"]))
        y_axis = [-math.sin(t), 0.0, math.cos(t)]
        # The eye sits just ahead of the modelled lens, not inside it.
        eye = np.asarray(cam["position_m"], float) + 0.02 * np.array([math.cos(t), 0.0, math.sin(t)])
        camera = (f'      <camera name="{cam["name"]}" mode="fixed" pos="{_vec(eye)}" '
                  f'xyaxes="0 -1 0 {_vec(y_axis)}" fovy="{_f(cam.get("fovy_deg", 90.0))}"/>')
    span = 2.0 * (spec.arm_radius_m() + spec.prop_radius_m)
    chase = max(0.8, 4.0 * span)
    ground_xml = ('    <geom name="ground" type="plane" size="500 500 0.1" material="ground" '
                  'friction="0.9 0.01 0.001"/>') if ground else ""

    return f"""\
<mujoco model="{spec.name}">
  <!-- Generated from {spec.source} by aerial_kit.airframes.multirotor_specs.mjcf.
       Edit the YAML (or the CAD it references), not this text. Body frame FLU, world ENU. -->
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 -{_f(spec.gravity_mps2)}" timestep="{_f(timestep)}" integrator="implicitfast"/>
  <visual>
    <global offwidth="1600" offheight="900" azimuth="160" elevation="-20"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
    <map znear="0.002" zfar="800" shadowclip="2"/>
  </visual>
  <statistic extent="{_f(max(span, 0.3) * 2.0)}" center="0 0 0"/>
  <default>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="1" density="0"/>
    </default>
    <default class="collision">
      <geom contype="1" conaffinity="1" group="3" density="0" rgba="0.8 0.2 0.2 0.3"
            friction="0.9 0.01 0.001"/>
    </default>
    <site group="4" rgba="1 0 1 1"/>
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.62 0.75 0.90"
             rgb2="0.95 0.96 0.98" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.50 0.58 0.47"
             rgb2="0.46 0.53 0.43" width="512" height="512" mark="edge" markrgb="0.40 0.46 0.38"/>
    <material name="ground" texture="grid" texrepeat="500 500" reflectance="0.0"/>
{chr(10).join(materials)}
{chr(10).join(assets)}
  </asset>
  <worldbody>
    <light name="sun" pos="0 0 100" dir="-0.35 0.25 -1" directional="true"
           diffuse="0.75 0.75 0.72" specular="0.25 0.25 0.25" castshadow="true"/>
{ground_xml}
    <body name="body" pos="0 0 0">
      <freejoint name="root"/>
      <inertial pos="{_vec(root.com_m)}" mass="{_f(root.mass_kg)}" fullinertia="{_full_inertia(root.inertia_kg_m2)}"/>
      <camera name="chase" mode="trackcom" pos="{_vec([-chase, 0.0, 0.35 * chase])}" xyaxes="0 -1 0 0.33 0 0.94"/>
{camera}
      <site name="cg" pos="0 0 0" size="0.004"/>
{chr(10).join('      ' + g for g in geoms.get('body', []))}
{collision}
{chr(10).join(sites)}
{chr(10).join(rotor_xml)}
{chr(10).join(payload_xml)}
    </body>
  </worldbody>
  <actuator>
{chr(10).join(actuators)}
  </actuator>
</mujoco>
"""


def _collision_xml(spec: MultirotorSpec) -> str:
    col = spec.collision
    lines = []
    if spec.style == "fpv_x":
        lines.append(f'      <geom name="collision_frame" type="box" pos="{_vec(col["frame_box_center_m"])}" '
                     f'size="{_vec(col["frame_box_half_m"])}" class="collision"/>')
        r = float(col["motor_sphere_radius_m"])
        for rotor in spec.rotors:
            lines.append(f'      <geom name="collision_{rotor.name}" type="sphere" '
                         f'pos="{_vec(rotor.position_m - [0, 0, 0.02])}" size="{_f(r)}" class="collision"/>')
    elif spec.style == "research_hub":
        hc = col["hub_cylinder"]
        lines.append(f'      <geom name="collision_hub" type="cylinder" size="{_f(hc["radius_m"])} '
                     f'{_f(hc["half_height_m"])}" class="collision"/>')
        g = spec.geometry
        lg = g["landing_gear"]
        bottom = -0.5 * float(g["hub"]["height_m"]) - float(lg["height_m"])
        for sy in (-1, 1):
            y = sy * 0.5 * float(lg["width_m"])
            half_l = 0.5 * float(lg["length_m"])
            lines.append(f'      <geom name="collision_skid_{"l" if sy > 0 else "r"}" type="capsule" '
                         f'fromto="{_vec([half_l, y, bottom])} {_vec([-half_l, y, bottom])}" '
                         f'size="{_f(lg["tube_radius_m"])}" class="collision"/>')
    else:
        cad = spec._cad
        from .spec import _load_yaml

        box = _load_yaml(cad.alignment_path)["frame_collision"]
        center = CAD_TO_FLU @ np.asarray(box["center_xyz_m"], float) - spec._raw_cg()
        size = np.abs(CAD_TO_FLU @ np.asarray(box["box_size_xyz_m"], float)) * 0.5
        lines.append(f'      <geom name="collision_frame" type="box" pos="{_vec(center)}" '
                     f'size="{_vec(size)}" class="collision"/>')
    return "\n".join(lines)
