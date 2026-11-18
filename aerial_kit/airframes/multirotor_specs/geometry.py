"""Procedural, license-free geometry for the multirotor airframes.

Styles:

* ``fpv_x``        carbon bottom plate and arms, standoffs, top plate,
                   battery and strap, tilted FPV camera in a TPU cage,
                   antenna, coloured front props.
* ``research_hub`` round hub with canopy, carbon arm tubes (front arms red),
                   motor mounts, landing-gear skids, GPS mast, payload gimbal,
                   battery.
* ``cad_meshes``   the lr_drone CAD frame and prop STLs, placed from the URDF.

Parts are in the frame of the body that carries them: ``body`` (root, origin
at the CG) or ``<rotor>_rotor`` (origin at the rotor hub, spin axis +z).
Body axes are FLU.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from aerial_kit.airframes.twin_wing.geometry import (
    Mesh,
    _axis_angle,
    _cosine_stations,
    _naca_thickness,
    box_mesh,
    cylinder_between,
    loft,
    revolve,
    uv_sphere,
)

from .spec import CAD_TO_FLU, MultirotorSpec, RotorSpec


@dataclass
class MeshFile:
    """A mesh read from disk (CAD), placed by pos/rotation in its body."""

    path: str
    scale: float
    pos: np.ndarray
    rot: np.ndarray


@dataclass
class VisualPart:
    name: str
    body: str
    material: str
    mesh: Mesh | None = field(default=None, repr=False)
    file: MeshFile | None = None


def rotor_body(rotor: RotorSpec) -> str:
    return f"{rotor.name}_rotor"


def _rot_z(angle: float) -> np.ndarray:
    return _axis_angle([0.0, 0.0, 1.0], angle)


def _rounded_rect(cx: float, cy: float, half_x: float, half_y: float, radius: float,
                  z: float, n_corner: int = 5) -> np.ndarray:
    pts = []
    radius = min(radius, half_x, half_y)
    corners = [(half_x - radius, half_y - radius, 0.0), (-half_x + radius, half_y - radius, 0.5 * math.pi),
               (-half_x + radius, -half_y + radius, math.pi), (half_x - radius, -half_y + radius, 1.5 * math.pi)]
    for x0, y0, a0 in corners:
        for k in range(n_corner):
            a = a0 + 0.5 * math.pi * k / (n_corner - 1)
            pts.append([cx + x0 + radius * math.cos(a), cy + y0 + radius * math.sin(a), z])
    return np.asarray(pts)


def plate(cx, cy, cz, length, width, thickness, radius) -> Mesh:
    rings = [_rounded_rect(cx, cy, 0.5 * length, 0.5 * width, radius, cz - 0.5 * thickness),
             _rounded_rect(cx, cy, 0.5 * length, 0.5 * width, radius, cz + 0.5 * thickness)]
    return loft(rings, "fan", "fan")


def oriented_box(center, half, yaw: float = 0.0, pitch: float = 0.0) -> Mesh:
    rot = _rot_z(yaw) @ _axis_angle([0.0, 1.0, 0.0], pitch)
    return box_mesh([0.0, 0.0, 0.0], half).transformed(rot, center)


def bar_between(p0, p1, width, thickness) -> Mesh:
    """Flat bar (arm) between two points in the xy plane."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    length = float(np.linalg.norm(d[:2]))
    yaw = math.atan2(d[1], d[0])
    return oriented_box(0.5 * (p0 + p1), [0.5 * length, 0.5 * width, 0.5 * thickness], yaw)


def z_cylinder(center, radius, height, segments=18) -> Mesh:
    c = np.asarray(center, float)
    return cylinder_between(c - [0, 0, 0.5 * height], c + [0, 0, 0.5 * height], radius, segments)


def prop_blades(spec: MultirotorSpec, rotor: RotorSpec, tip_paint: float = 0.84) -> tuple[Mesh, Mesh]:
    """Blades (and painted tips) about +z for a rotor, pitched for +z thrust.

    The leading edge faces the direction of travel of the blade for this
    rotor's spin, tilted toward +z, so a CW and a CCW rotor get mirrored props.
    """
    p = spec.prop
    radius = spec.prop_radius_m
    r_hub = float(p.get("hub_radius_m", 0.006))
    c_max = float(p.get("max_chord_m", 0.2 * radius))
    c_tip = float(p.get("tip_chord_m", 0.1 * radius))
    b0 = math.radians(float(p.get("root_pitch_deg", 30.0)))
    b1 = math.radians(float(p.get("tip_pitch_deg", 12.0)))
    t0 = float(p.get("thickness_m", 0.003))
    spin = rotor.spin

    def ring(rho: float) -> np.ndarray:
        rr = max(rho * radius, r_hub)
        if rho < 0.3:
            c = 0.6 * c_max + 0.4 * c_max * (rho / 0.3)
        else:
            c = c_max + (c_tip - c_max) * (rho - 0.3) / 0.7
        if rho > 0.97:
            c *= 0.7
        pitch = b0 + (b1 - b0) * rho
        thick = t0 * (1.0 - 0.55 * rho)
        radial = np.array([1.0, 0.0, 0.0])
        motion = spin * np.array([0.0, 1.0, 0.0])          # omega z x r for r along +x
        le_dir = motion * math.cos(pitch) + np.array([0.0, 0.0, 1.0]) * math.sin(pitch)
        n_dir = np.cross(le_dir, radial)
        u = _cosine_stations(8)
        half_t = _naca_thickness(u, 1.0) * thick
        centre = rr * radial
        upper = [centre + (0.35 - ui) * c * le_dir + ht * n_dir for ui, ht in zip(u, half_t)]
        lower = [centre + (0.35 - ui) * c * le_dir - ht * n_dir for ui, ht in zip(u, half_t)]
        return np.vstack([np.asarray(upper), np.asarray(lower)[::-1]])

    rho0 = r_hub / radius
    inner = loft([ring(float(r)) for r in np.linspace(rho0, tip_paint, 6)], "strip", "strip")
    tip = loft([ring(float(r)) for r in (*np.linspace(tip_paint, 0.97, 3), 1.0)], "strip", "strip")
    blades, tips = [], []
    count = int(p.get("blades", 2))
    for k in range(count):
        rot = _rot_z(2.0 * math.pi * k / count)
        blades.append(inner.transformed(rot))
        tips.append(tip.transformed(rot))
    return Mesh.merge(blades), Mesh.merge(tips)


def _is_front(rotor: RotorSpec) -> bool:
    return rotor.position_m[0] > 0.05 * np.linalg.norm(rotor.position_m[:2])


class MultirotorGeometry:
    def __init__(self, spec: MultirotorSpec) -> None:
        self.spec = spec

    def body_origin(self, body: str) -> np.ndarray:
        if body == "body":
            return np.zeros(3)
        for rotor in self.spec.rotors:
            if body == rotor_body(rotor):
                return rotor.position_m.copy()
        raise ValueError(f"unknown body {body!r}")

    # --------------------------------------------------------------- rotors
    def _rotor_parts(self, motor_material: str, prop_mat_front: str, prop_mat_rear: str,
                     tip_front: str | None = None, tip_rear: str | None = None,
                     stator_material: str = "stator") -> list[VisualPart]:
        s = self.spec
        g = s.geometry.get("motor", {})
        r_st = float(g.get("stator_radius_m", 0.012))
        h = float(g.get("height_m", 0.015))
        parts = []
        for rotor in s.rotors:
            body = rotor_body(rotor)
            front = _is_front(rotor)
            hub = rotor.position_m
            # Fixed stator + mount on the root body, below the prop plane.
            parts.append(VisualPart(f"{rotor.name}_stator", "body", stator_material,
                                    z_cylinder(hub - [0, 0, 0.004 + 0.75 * h], r_st * 0.92, 0.5 * h)))
            # Rotating bell, hub and props on the rotor body.
            parts.append(VisualPart(f"{rotor.name}_bell", body, motor_material,
                                    z_cylinder([0, 0, -0.004 - 0.3 * h], r_st, 0.6 * h)))
            parts.append(VisualPart(f"{rotor.name}_hub", body, stator_material,
                                    revolve([(0.0, 0.25 * r_st), (-0.004, 0.55 * r_st)], 12)
                                    .transformed(_axis_angle([0, 1, 0], -math.pi / 2), [0, 0, 0.004])))
            blades, tips = prop_blades(s, rotor)
            parts.append(VisualPart(f"{rotor.name}_prop", body,
                                    prop_mat_front if front else prop_mat_rear, blades))
            parts.append(VisualPart(f"{rotor.name}_prop_tips", body,
                                    (tip_front if front else tip_rear) or
                                    (prop_mat_front if front else prop_mat_rear), tips))
        return parts

    # ------------------------------------------------------------------ FPV
    def _fpv(self) -> list[VisualPart]:
        s, g = self.spec, self.spec.geometry
        rz = float(np.mean([r.position_m[2] for r in s.rotors]))
        motor_h = float(g["motor"]["height_m"])
        arm_t = float(g["arm"]["thickness_m"])
        arm_z = rz - motor_h - 0.004 - 0.5 * arm_t
        pl = g["plate"]
        parts = [VisualPart("bottom_plate", "body", "carbon",
                            plate(0.0, 0.0, arm_z, pl["length_m"], pl["width_m"], pl["thickness_m"], 0.012))]
        for rotor in s.rotors:
            hub = rotor.position_m
            parts.append(VisualPart(f"{rotor.name}_arm", "body", "carbon",
                                    bar_between([0.0, 0.0, arm_z], [hub[0], hub[1], arm_z],
                                                g["arm"]["width_m"], arm_t)))
            parts.append(VisualPart(f"{rotor.name}_motor_pad", "body", "carbon",
                                    z_cylinder([hub[0], hub[1], arm_z], 0.0145, arm_t)))
        bat = g["battery"]
        bsize = np.asarray(bat["size_m"], float)
        bpos = np.asarray(bat["position_m"], float)
        top = g["top_plate"]
        top_z = bpos[2] - 0.5 * bsize[2] - 0.0015
        parts.append(VisualPart("top_plate", "body", "carbon",
                                plate(0.004, 0.0, top_z, top["length_m"], top["width_m"], 0.003, 0.008)))
        for sx in (-1, 1):
            for sy in (-1, 1):
                p = np.array([sx * 0.032 + 0.004, sy * 0.014, 0.0])
                parts.append(VisualPart(f"standoff_{sx}_{sy}", "body", "stator",
                                        cylinder_between(p + [0, 0, arm_z], p + [0, 0, top_z], 0.0025, 8)))
        parts.append(VisualPart("battery", "body", "battery", box_mesh(bpos, 0.5 * bsize)))
        parts.append(VisualPart("battery_strap", "body", "strap",
                                box_mesh(bpos, [0.009, 0.5 * bsize[1] + 0.0015, 0.5 * bsize[2] + 0.0015])))
        # Camera: TPU side plates (front accent) and a tilted camera body.
        cam = s.camera
        cpos = np.asarray(cam["position_m"], float)
        tilt = math.radians(float(cam["uptilt_deg"]))
        cage = np.asarray(g["camera_cage"]["size_m"], float)
        for sy in (-1, 1):
            parts.append(VisualPart(f"camera_side_{'l' if sy > 0 else 'r'}", "body", "front_accent",
                                    box_mesh(cpos - [0.006, 0, 0] + [0, sy * 0.5 * cage[1], -0.004],
                                             [0.5 * cage[0], 0.0015, 0.5 * cage[2]])))
        parts.append(VisualPart("camera_body", "body", "tpu",
                                oriented_box(cpos, [0.0095, 0.0095, 0.0095], pitch=-tilt)))
        lens_dir = np.array([math.cos(tilt), 0.0, math.sin(tilt)])
        parts.append(VisualPart("camera_lens", "body", "lens",
                                cylinder_between(cpos + 0.008 * lens_dir, cpos + 0.016 * lens_dir, 0.0065, 14)))
        rear = -0.5 * float(pl["length_m"]) - 0.004
        parts.append(VisualPart("antenna", "body", "antenna",
                                cylinder_between([rear, 0.0, top_z], [rear - 0.035, 0.0, top_z + 0.045], 0.0018, 8)))
        parts.append(VisualPart("antenna_tip", "body", "front_accent",
                                uv_sphere([rear - 0.036, 0.0, top_z + 0.046], 0.004, 5, 8)))
        return parts + self._rotor_parts("motor_bell", "prop_front", "prop_rear")

    # -------------------------------------------------------------- research
    def _research(self) -> list[VisualPart]:
        s, g = self.spec, self.spec.geometry
        hub = g["hub"]
        R, H = float(hub["radius_m"]), float(hub["height_m"])
        parts = [
            VisualPart("hub_lower", "body", "carbon", z_cylinder([0, 0, -0.25 * H], R, 0.5 * H, 24)),
            VisualPart("hub_canopy", "body", "hub",
                       revolve([(0.5 * H + 0.04, 0.02), (0.5 * H + 0.03, 0.55 * R),
                                 (0.5 * H + 0.01, 0.9 * R), (0.0, R * 1.01), (-0.004, R * 1.01)], 24)
                       .transformed(_axis_angle([0, 1, 0], -math.pi / 2))),
        ]
        motor_h = float(g["motor"]["height_m"])
        arm_r = float(g["arm"]["radius_m"])
        for rotor in s.rotors:
            hubp = rotor.position_m
            arm_z = hubp[2] - 0.004 - motor_h - arm_r
            direction = hubp[:2] / np.linalg.norm(hubp[:2])
            start = np.array([*(0.8 * R * direction), 0.0])
            end = np.array([hubp[0], hubp[1], arm_z])
            parts.append(VisualPart(f"{rotor.name}_arm", "body",
                                    "front_arm" if _is_front(rotor) else "carbon",
                                    cylinder_between(start, end, arm_r, 12)))
            parts.append(VisualPart(f"{rotor.name}_mount", "body", "carbon",
                                    oriented_box(end, [0.025, 0.02, arm_r + 0.004],
                                                 yaw=math.atan2(direction[1], direction[0]))))
        lg = g["landing_gear"]
        w, L, h, tr = (float(lg[k]) for k in ("width_m", "length_m", "height_m", "tube_radius_m"))
        bottom = -0.5 * H
        for sy in (-1, 1):
            y = sy * 0.5 * w
            parts.append(VisualPart(f"skid_{'l' if sy > 0 else 'r'}", "body", "gear",
                                    cylinder_between([0.5 * L, y, bottom - h], [-0.5 * L, y, bottom - h], tr, 10)))
            for sx in (-1, 1):
                parts.append(VisualPart(f"leg_{sx}_{sy}", "body", "gear",
                                        cylinder_between([sx * 0.25 * L, y, bottom - h],
                                                         [sx * 0.35 * R, sy * 0.35 * R, bottom], tr, 10)))
        mast = g["gps_mast"]
        top = 0.5 * H + 0.04
        parts.append(VisualPart("gps_mast", "body", "carbon",
                                cylinder_between([-0.3 * R, 0, top - 0.01], [-0.3 * R, 0, top + mast["height_m"]], 0.006, 8)))
        parts.append(VisualPart("gps_puck", "body", "gps",
                                z_cylinder([-0.3 * R, 0, top + mast["height_m"] + 0.008],
                                           float(mast["puck_radius_m"]), 0.016, 20)))
        pay = g["payload"]
        psize, ppos = np.asarray(pay["size_m"], float), np.asarray(pay["position_m"], float)
        parts.append(VisualPart("payload", "body", "payload", box_mesh(ppos, 0.5 * psize)))
        parts.append(VisualPart("payload_mount", "body", "carbon",
                                cylinder_between([ppos[0], 0, bottom], ppos + [0, 0, 0.5 * psize[2]], 0.01, 8)))
        cam = s.camera
        if cam:
            cpos = np.asarray(cam["position_m"], float)
            tilt = math.radians(float(cam["uptilt_deg"]))
            d = np.array([math.cos(tilt), 0.0, math.sin(tilt)])
            parts.append(VisualPart("payload_lens", "body", "lens",
                                    cylinder_between(cpos - 0.01 * d, cpos + 0.012 * d, 0.02, 16)))
        bat = g["battery"]
        parts.append(VisualPart("battery", "body", "battery",
                                box_mesh(np.asarray(bat["position_m"], float) + [0, 0, 0.03],
                                         0.5 * np.asarray(bat["size_m"], float))))
        return parts + self._rotor_parts("motor_bell", "prop_front", "prop_rear",
                                         "prop_tip_front", "prop_tip_rear")

    # ------------------------------------------------------------------- CAD
    def _cad(self) -> list[VisualPart]:
        s = self.spec
        cad = s._cad
        cg = s._raw_cg()
        parts = [VisualPart("frame", "body", "frame", file=MeshFile(
            str(cad.mesh_dir / s.data["cad"]["frame_mesh"]), cad.mesh_scale, -cg, CAD_TO_FLU))]
        entries = {j: (child, entry) for j, child, _, _, entry in cad._rotor_entries()}
        names = s.data["cad"].get("rotor_names", {})
        for joint, (child, entry) in entries.items():
            rotor = next(r for r in s.rotors if r.name == names.get(joint, joint))
            overrides = s.data["cad"].get("mesh_origin_overrides", {}) or {}
            origin = overrides.get(joint, entry["mesh_origin_xyz_m"])
            mesh_pos = CAD_TO_FLU @ np.asarray(origin, float)
            parts.append(VisualPart(f"{rotor.name}_prop", rotor_body(rotor),
                                    "prop_front" if _is_front(rotor) else "prop_rear",
                                    file=MeshFile(str(cad.mesh_dir / f"{child}.stl"), cad.mesh_scale,
                                                  mesh_pos, CAD_TO_FLU)))
        for p in s.data.get("payloads", []):
            pos = np.asarray(p["position_m"], float) - cg
            parts.append(VisualPart(p["name"], "body", "battery",
                                    box_mesh(pos, 0.5 * np.asarray(p["size_m"], float))))
        cam = s.camera
        if cam:
            cpos = np.asarray(cam["position_m"], float)
            parts.append(VisualPart("front_marker", "body", "front_accent",
                                    uv_sphere(cpos + [0.004, 0, 0], 0.006, 6, 10)))
        return parts

    def parts(self) -> list[VisualPart]:
        style = self.spec.style
        if style == "fpv_x":
            return self._fpv()
        if style == "research_hub":
            return self._research()
        return self._cad()
