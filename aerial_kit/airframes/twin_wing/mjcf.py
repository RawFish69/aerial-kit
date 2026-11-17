"""MuJoCo MJCF for the twin wing, generated from the shared spec.

Body tree (all body FRD, root origin = reference CG):

    body (freejoint "root")       wing, pod, fins, nacelles, hinge rods, nav lights
      left_elevon  (hinge)        elevon + horn, rotates about the swept hinge line
      right_elevon (hinge)
      left_prop    (hinge, +x)    blades, tips, hub, spinner, motor bell
      right_prop   (hinge, +x)

Meshes are embedded inline (``<mesh vertex=... face=...>``), so the model has
no file dependencies. Visual geoms never collide; a few primitives in group 3
carry ground contact. Masses and inertias come from ``TwinWingSpec`` so the
composite is the spec's 1.5 kg and ``inertia_kg_m2`` about the origin.
"""

from __future__ import annotations

import math

import numpy as np

from .geometry import Mesh, TwinWingGeometry
from .spec import SIDES, TwinWingSpec

BODY_NAMES = ("body", "left_elevon", "right_elevon", "left_prop", "right_prop")
ELEVON_JOINTS = ("left_elevon_joint", "right_elevon_joint")
PROP_JOINTS = ("left_prop_joint", "right_prop_joint")
ACTUATORS = (
    "left_elevon_servo", "right_elevon_servo", "left_prop_motor", "right_prop_motor")


#: y axis of a camera looking forward and 5 degrees down in an FRD body.
_FPV_TILT_AXES = "0.087156 0 -0.996195"


def _f(x: float) -> str:
    return f"{float(x):.6g}"


def _vec(v) -> str:
    return " ".join(_f(x) for x in np.asarray(v, dtype=float).ravel())


def _rgba(v) -> str:
    return " ".join(f"{float(x):.4g}" for x in v)


def _mesh_xml(name: str, mesh: Mesh) -> str:
    verts = " ".join(f"{x:.5f}" for x in mesh.vertices.ravel())
    faces = " ".join(str(int(i)) for i in mesh.faces.ravel())
    return f'    <mesh name="{name}" vertex="{verts}" face="{faces}"/>'


def _full_inertia(m: np.ndarray) -> str:
    return _vec([m[0, 0], m[1, 1], m[2, 2], m[0, 1], m[0, 2], m[1, 2]])


def twin_wing_mjcf(spec: TwinWingSpec, timestep: float = 0.002,
                   gravity_mps2: float = 9.81) -> str:
    geo = TwinWingGeometry(spec)
    parts = geo.parts()
    root = spec.root_mass()
    children = {c.name: c for c in spec.children_mass()}
    e = spec.elevons
    servo = e["servo"]
    rotor = spec.propulsion["rotor"]
    travel = spec.max_elevon_rad
    limit = spec.elevon_limit_rad
    v_max = float(rotor["visual_max_speed_rad_s"])
    # Motor time constant matched to the thrust lag, so spin-up and thrust agree.
    tau = max(float(spec.propulsion["thrust_tau_s"]), 1e-3)
    motor_kv = spec.rotor_axial_inertia() / tau

    body_cfg = spec.body
    fpv_eye = [float(body_cfg["nose_x_m"]) + 0.02, 0.0, -0.18 * float(body_cfg["max_height_m"])]
    assets = [_mesh_xml(p.name, p.mesh) for p in parts]
    materials = [
        f'    <material name="{name}" rgba="{_rgba(rgba)}" specular="0.35" shininess="0.4"/>'
        for name, rgba in spec.materials.items()
    ]

    def geoms_for(body: str, indent: str) -> str:
        lines = []
        for p in parts:
            if p.body != body:
                continue
            lines.append(
                f'{indent}<geom name="{p.name}" type="mesh" mesh="{p.name}" '
                f'material="{p.material}" class="visual"/>')
        return "\n".join(lines)

    col = spec.collision
    half = col["center_box_half_m"]
    cap_r = float(col["wing_capsule_radius_m"])
    collision = [
        f'      <geom name="collision_center" type="box" size="{_vec(half)}" class="collision"/>'
    ]
    for side in SIDES:
        sgn = -1.0 if side == "left" else 1.0
        y_tip = spec.half_span - cap_r
        mid_root = np.array([spec.le_x(0.0) - 0.5 * spec.chord(0.0), 0.0, 0.0])
        mid_tip = np.array([spec.le_x(y_tip) - 0.5 * spec.chord(y_tip), sgn * y_tip, spec.wing_z(y_tip)])
        collision.append(
            f'      <geom name="collision_{side}_wing" type="capsule" '
            f'fromto="{_vec(mid_root)} {_vec(mid_tip)}" size="{_f(cap_r)}" class="collision"/>')

    child_xml = []
    for side in SIDES:
        name = f"{side}_elevon"
        origin = geo.body_origin(name)
        _, _, axis = spec.elevon_hinge(side)
        c = children[name]
        child_xml.append(f"""\
      <body name="{name}" pos="{_vec(origin)}">
        <joint name="{name}_joint" type="hinge" axis="{_vec(axis)}"
               range="{_f(-limit)} {_f(limit)}" damping="{_f(servo['joint_damping_nms'])}"
               armature="{_f(servo['armature_kg_m2'])}"/>
        <inertial pos="{_vec(c.com_m - origin)}" mass="{_f(c.mass_kg)}"
                  fullinertia="{_full_inertia(c.inertia_kg_m2)}"/>
{geoms_for(name, '        ')}
        <site name="{name}_te_marker" pos="{_vec(np.array([spec.te_x(abs(origin[1])), origin[1], 0.0]) - origin)}" size="0.004"/>
      </body>""")
    for side in SIDES:
        name = f"{side}_prop"
        origin = geo.body_origin(name)
        c = children[name]
        child_xml.append(f"""\
      <body name="{name}" pos="{_vec(origin)}">
        <joint name="{name}_joint" type="hinge" axis="1 0 0"
               damping="{_f(rotor['bearing_damping_nms'])}"/>
        <inertial pos="{_vec(c.com_m - origin)}" mass="{_f(c.mass_kg)}"
                  fullinertia="{_full_inertia(c.inertia_kg_m2)}"/>
{geoms_for(name, '        ')}
      </body>""")

    actuators = []
    for side in SIDES:
        actuators.append(
            f'    <position name="{side}_elevon_servo" joint="{side}_elevon_joint" '
            f'kp="{_f(servo["kp_nm_per_rad"])}" kv="{_f(servo["kv_nms_per_rad"])}" '
            f'ctrlrange="{_f(-travel)} {_f(travel)}" '
            f'forcerange="{_f(-float(servo["force_limit_nm"]))} {_f(servo["force_limit_nm"])}"/>')
    for side in SIDES:
        spin = spec.motor_spin(side)
        lo, hi = (0.0, v_max) if spin > 0 else (-v_max, 0.0)
        actuators.append(
            f'    <velocity name="{side}_prop_motor" joint="{side}_prop_joint" '
            f'kv="{_f(motor_kv)}" ctrlrange="{_f(lo)} {_f(hi)}"/>')

    return f"""\
<mujoco model="{spec.data.get('name', 'twin_wing')}">
  <!-- Generated from {spec.source} by aerial_kit.airframes.twin_wing.mjcf.
       Edit the YAML, not this text. Body frame is FRD; world is ENU. -->
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 -{_f(gravity_mps2)}" timestep="{_f(timestep)}" integrator="implicitfast"/>
  <visual>
    <global offwidth="1600" offheight="900" azimuth="150" elevation="-18"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
    <!-- znear/zfar are relative to the statistic extent pinned below; without
         the pin the 1 km ground plane pushes the near plane through the wing. -->
    <map znear="0.004" zfar="600" shadowclip="3"/>
  </visual>
  <statistic extent="2.5" center="0 0 0"/>
  <default>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="1" density="0"/>
    </default>
    <default class="collision">
      <geom contype="1" conaffinity="1" group="3" density="0" rgba="0.8 0.2 0.2 0.25"/>
    </default>
    <site group="4" rgba="1 0 1 1"/>
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.62 0.75 0.90"
             rgb2="0.95 0.96 0.98" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.50 0.58 0.47"
             rgb2="0.46 0.53 0.43" width="512" height="512" mark="edge" markrgb="0.40 0.46 0.38"/>
    <material name="ground" texture="grid" texrepeat="200 200" reflectance="0.0"/>
{chr(10).join(materials)}
{chr(10).join(assets)}
  </asset>
  <worldbody>
    <light name="sun" pos="0 0 200" dir="-0.35 0.25 -1" directional="true"
           diffuse="0.75 0.75 0.72" specular="0.25 0.25 0.25" castshadow="true"/>
    <geom name="ground" type="plane" pos="0 0 0" size="1000 1000 0.1" material="ground"/>
    <body name="body" pos="0 0 0">
      <freejoint name="root"/>
      <inertial pos="{_vec(root.com_m)}" mass="{_f(root.mass_kg)}"
                fullinertia="{_full_inertia(root.inertia_kg_m2)}"/>
      <!-- Body-locked chase camera, and a trailing one that keeps world orientation. -->
      <camera name="chase" mode="fixed" pos="-2.6 0 -0.9" xyaxes="0 1 0 0.33 0 -0.94"/>
      <camera name="trail" mode="trackcom" pos="-2.6 0 -0.9" xyaxes="0 1 0 0.33 0 -0.94"/>
      <!-- On-board nose camera (FRD body: image right = +y, image up = -z), 5 deg down. -->
      <camera name="fpv" mode="fixed" pos="{_vec(fpv_eye)}" xyaxes="0 1 0 {_FPV_TILT_AXES}" fovy="100"/>
      <site name="cg" pos="0 0 0" size="0.006"/>
{geoms_for('body', '      ')}
{chr(10).join(collision)}
{chr(10).join(child_xml)}
    </body>
  </worldbody>
  <actuator>
{chr(10).join(actuators)}
  </actuator>
</mujoco>
"""
