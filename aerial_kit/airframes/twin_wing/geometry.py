"""Procedural, license-free low-poly geometry for the twin wing.

Every part is built from :class:`~aerial_kit.airframes.twin_wing.spec.TwinWingSpec`
with numpy only: lofted airfoil sections for the wing, elevons, fins and prop
blades, revolved profiles for the nacelles and spinners, and a superellipse
loft for the equipment pod. The same :class:`VisualPart` list feeds both the
MuJoCo MJCF and the Gazebo SDF, so the two cannot drift apart.

Each mesh is closed and outward-wound. Vertices are in the frame of the body
that carries the part (root for fixed parts, hinge point for elevons, prop
hub for rotors), body-FRD axes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .spec import SIDES, TwinWingSpec, _side_sign


# --------------------------------------------------------------------- mesh
@dataclass
class Mesh:
    vertices: np.ndarray
    faces: np.ndarray

    def __post_init__(self) -> None:
        self.vertices = np.asarray(self.vertices, dtype=float).reshape(-1, 3)
        self.faces = np.asarray(self.faces, dtype=np.int64).reshape(-1, 3)

    def translated(self, offset) -> "Mesh":
        return Mesh(self.vertices + np.asarray(offset, dtype=float), self.faces.copy())

    def transformed(self, rot: np.ndarray, offset=(0.0, 0.0, 0.0)) -> "Mesh":
        rot = np.asarray(rot, dtype=float)
        verts = self.vertices @ rot.T + np.asarray(offset, dtype=float)
        faces = self.faces.copy()
        if np.linalg.det(rot) < 0.0:
            faces = faces[:, ::-1]
        return Mesh(verts, faces)

    @staticmethod
    def merge(meshes: list["Mesh"]) -> "Mesh":
        verts, faces, base = [], [], 0
        for mesh in meshes:
            verts.append(mesh.vertices)
            faces.append(mesh.faces + base)
            base += len(mesh.vertices)
        return Mesh(np.vstack(verts), np.vstack(faces))

    def signed_volume(self) -> float:
        v = self.vertices[self.faces]
        return float(np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])).sum() / 6.0)

    def face_normals(self) -> np.ndarray:
        v = self.vertices[self.faces]
        return np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        return self.vertices.min(axis=0), self.vertices.max(axis=0)

    def compact(self, tol: float = 1e-9) -> "Mesh":
        """Weld coincident vertices and drop the faces that collapse.

        Lofts of airfoil halves and noses have coincident points; welding
        them keeps the mesh topologically closed (every edge shared by two
        faces) once the degenerate triangles are removed.
        """
        keys = np.round(self.vertices / tol).astype(np.int64)
        _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
        order = np.argsort(first)
        remap = np.empty_like(order)
        remap[order] = np.arange(len(order))
        verts = self.vertices[first[order]]
        faces = remap[inverse.reshape(-1)][self.faces]
        distinct = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
        faces = faces[distinct]
        v = verts[faces]
        area = np.linalg.norm(np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0]), axis=1)
        return Mesh(verts, faces[area > 1e-14])


def _orient_cap(cap: list[tuple[int, int, int]], verts: np.ndarray,
                outward: np.ndarray) -> list[tuple[int, int, int]]:
    total = np.zeros(3)
    for a, b, c in cap:
        total += np.cross(verts[b] - verts[a], verts[c] - verts[a])
    if float(total @ outward) < 0.0:
        return [(a, c, b) for a, b, c in cap]
    return cap


def loft(rings: list[np.ndarray], cap_start: str = "fan", cap_end: str = "fan") -> Mesh:
    """Skin consecutive rings of equal length into a closed mesh.

    ``cap`` is ``"fan"`` (convex rings, fan from the centroid) or ``"strip"``
    (rings laid out as [upper LE->TE, lower TE->LE], paired by station).
    """
    rings = [np.asarray(r, dtype=float) for r in rings]
    n = len(rings[0])
    if any(len(r) != n for r in rings):
        raise ValueError("loft rings must all have the same point count")
    verts = list(np.vstack(rings))
    sides: list[tuple[int, int, int]] = []
    for i in range(len(rings) - 1):
        for j in range(n):
            a = i * n + j
            b = i * n + (j + 1) % n
            c = (i + 1) * n + (j + 1) % n
            d = (i + 1) * n + j
            sides.append((a, b, c))
            sides.append((a, c, d))
    verts_arr = np.asarray(verts)

    # Orient the side skin outward: the sum of (face normal . (face centre -
    # its ring centroid)) is positive for an outward skin.
    score = 0.0
    for a, b, c in sides:
        i = a // n
        centre = (verts_arr[a] + verts_arr[b] + verts_arr[c]) / 3.0
        ring_c = rings[i].mean(axis=0) * 0.5 + rings[min(i + 1, len(rings) - 1)].mean(axis=0) * 0.5
        nrm = np.cross(verts_arr[b] - verts_arr[a], verts_arr[c] - verts_arr[a])
        score += float(nrm @ (centre - ring_c))
    if score < 0.0:
        sides = [(a, c, b) for a, b, c in sides]

    faces = list(sides)
    for which, mode in ((0, cap_start), (len(rings) - 1, cap_end)):
        if mode is None:
            continue
        other = 1 if which == 0 else len(rings) - 2
        outward = rings[which].mean(axis=0) - rings[other].mean(axis=0)
        base = which * n
        cap: list[tuple[int, int, int]] = []
        if mode == "fan":
            verts.append(rings[which].mean(axis=0))
            centre = len(verts) - 1
            for j in range(n):
                cap.append((centre, base + j, base + (j + 1) % n))
        elif mode == "strip":
            h = n // 2
            for j in range(h - 1):
                a, b = base + j, base + j + 1
                c, d = base + n - 2 - j, base + n - 1 - j
                cap.append((a, b, c))
                cap.append((a, c, d))
        else:
            raise ValueError(f"unknown cap mode {mode!r}")
        faces.extend(_orient_cap(cap, np.asarray(verts), outward))
    mesh = Mesh(np.asarray(verts), np.asarray(faces)).compact()
    if mesh.signed_volume() < 0.0:
        mesh = Mesh(mesh.vertices, mesh.faces[:, ::-1])
    return mesh


def revolve(profile: list[tuple[float, float]], segments: int = 20) -> Mesh:
    """Revolve ``(x, r)`` pairs (front to back) about +x into a closed mesh."""
    angles = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    rings = []
    for x, r in profile:
        r = max(float(r), 1e-4)
        rings.append(np.column_stack([
            np.full(segments, float(x)), r * np.cos(angles), r * np.sin(angles)]))
    return loft(rings, "fan", "fan")


def cylinder_between(p0, p1, radius: float, segments: int = 12) -> Mesh:
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    length = float(np.linalg.norm(p1 - p0))
    base = revolve([(0.0, radius), (-length, radius)], segments)
    rot = _rotation_from_x(-(p1 - p0) / length)
    return base.transformed(rot, p0)


def uv_sphere(centre, radius: float, rings: int = 8, segments: int = 12) -> Mesh:
    profile = []
    for k in range(rings + 1):
        th = math.pi * k / rings
        profile.append((radius * math.cos(th), radius * math.sin(th)))
    return revolve(profile, segments).translated(centre)


def box_mesh(centre, half) -> Mesh:
    cx, cy, cz = centre
    hx, hy, hz = half
    v = np.array([[cx + sx * hx, cy + sy * hy, cz + sz * hz]
                  for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    f = np.array([
        [0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
        [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    mesh = Mesh(v, f)
    if mesh.signed_volume() < 0.0:
        mesh = Mesh(v, f[:, ::-1])
    return mesh


def _rotation_from_x(direction: np.ndarray) -> np.ndarray:
    """Rotation matrix that maps +x onto ``direction``."""
    d = np.asarray(direction, dtype=float)
    d = d / np.linalg.norm(d)
    x = np.array([1.0, 0.0, 0.0])
    v = np.cross(x, d)
    c = float(x @ d)
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else np.diag([-1.0, -1.0, 1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / float(v @ v))


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * k @ k


# ----------------------------------------------------------------- airfoil
def _naca_thickness(x: np.ndarray, t: float) -> np.ndarray:
    """NACA 4-digit half thickness, closed trailing edge, fraction of chord."""
    x = np.clip(x, 0.0, 1.0)
    return 5.0 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2
                      + 0.2843 * x ** 3 - 0.1036 * x ** 4)


def _reflex_camber(x: np.ndarray, m: float, reflex: float) -> np.ndarray:
    grid = np.linspace(0.0, 1.0, 401)
    raw = grid * (1.0 - grid) * (reflex - grid)
    scale = m / max(float(raw.max()), 1e-9)
    return scale * np.asarray(x) * (1.0 - np.asarray(x)) * (reflex - np.asarray(x))


def _cosine_stations(n: int, start: float = 0.0, end: float = 1.0) -> np.ndarray:
    beta = np.linspace(0.0, math.pi, n)
    s = 0.5 * (1.0 - np.cos(beta))
    return start + (end - start) * s


# -------------------------------------------------------------- parts list
@dataclass
class VisualPart:
    name: str
    body: str           # "body", "left_elevon", "right_elevon", "left_prop", "right_prop"
    material: str       # key into spec.materials
    mesh: Mesh = field(repr=False)


class TwinWingGeometry:
    """Builds every visual part of the twin wing from a spec."""

    CHORD_POINTS = 15

    def __init__(self, spec: TwinWingSpec) -> None:
        self.spec = spec
        a = spec.wing["airfoil"]
        self._camber_m = float(a["max_camber"])
        self._reflex = float(a["reflex_start"])

    # ---------------------------------------------------------- frames
    def body_origin(self, body: str) -> np.ndarray:
        """Origin of a child body in the root frame."""
        if body == "body":
            return np.zeros(3)
        side, kind = body.split("_", 1)
        if kind == "elevon":
            p0, p1, _ = self.spec.elevon_hinge(side)
            return 0.5 * (p0 + p1)
        if kind == "prop":
            return self.spec.prop_position(side)
        raise ValueError(f"unknown body {body!r}")

    # ---------------------------------------------------------- airfoil
    def _section_point(self, y: float, xc: np.ndarray, zc: np.ndarray) -> np.ndarray:
        """Map airfoil (x/c, z/c up) at span station y into root FRD."""
        s = self.spec
        c = s.chord(y)
        le = s.le_x(y)
        eta = min(abs(y) / s.half_span, 1.0)
        twist = math.radians(float(s.wing["washout_deg"])) * eta
        # Local airfoil frame: X aft of LE, Z up. Washout rotates nose down
        # about the quarter chord.
        X = xc * c - 0.25 * c
        Z = zc * c
        Xr = X * math.cos(twist) - Z * math.sin(twist)
        Zr = X * math.sin(twist) + Z * math.cos(twist)
        x_body = le - (Xr + 0.25 * c)
        z_body = s.wing_z(y) - Zr
        return np.column_stack([x_body, np.full_like(x_body, y), z_body])

    def _airfoil_half_rings(self, y: float, x_end_c: float):
        """Top-half and bottom-half rings split along the camber line."""
        t = self.spec.thickness_ratio(y)
        xc = _cosine_stations(self.CHORD_POINTS, 0.0, x_end_c)
        yc = _reflex_camber(xc, self._camber_m, self._reflex)
        yt = _naca_thickness(xc, t)
        upper = self._section_point(y, xc, yc + yt)
        camber = self._section_point(y, xc, yc)
        lower = self._section_point(y, xc, yc - yt)
        top = np.vstack([upper, camber[::-1]])
        bottom = np.vstack([camber, lower[::-1]])
        return top, bottom

    # -------------------------------------------------------------- wing
    def wing_segments(self, side: str) -> list[tuple[float, float, float]]:
        """(y_from, y_to, x_end_c) spans: full chord, cut-out, full chord."""
        s = self.spec
        e = s.elevons
        y0, y1 = float(e["inboard_y_m"]), float(e["outboard_y_m"])
        return [(0.0, y0, 1.0), (y0, y1, None), (y1, s.half_span, 1.0)]

    def _cut_fraction(self, y: float) -> float:
        """x/c where the wing stops in front of the elevon (gap + nose)."""
        s = self.spec
        c = s.chord(y)
        r_nose = self.elevon_nose_radius(y)
        gap = float(s.elevons["hinge_gap_m"])
        x_cut = s.hinge_x(y) + r_nose + gap
        return (s.le_x(y) - x_cut) / c

    def elevon_nose_radius(self, y: float) -> float:
        s = self.spec
        xh_c = 1.0 - float(s.elevons["chord_fraction"])
        return 0.5 * 2.0 * float(_naca_thickness(np.array([xh_c]), s.thickness_ratio(y))[0]) * s.chord(y) * 0.92

    def wing_half(self, side: str) -> tuple[Mesh, Mesh]:
        sgn = _side_sign(side)
        tops, bottoms = [], []
        n_span = int(self.spec.wing["spanwise_sections"])
        for y_from, y_to, x_end in self.wing_segments(side):
            count = max(2, int(round(n_span * (y_to - y_from) / self.spec.half_span)) + 1)
            top_rings, bot_rings = [], []
            for y in np.linspace(y_from, y_to, count):
                xe = x_end if x_end is not None else self._cut_fraction(y)
                top, bot = self._airfoil_half_rings(float(y), xe)
                if sgn < 0:
                    top[:, 1] *= -1.0
                    bot[:, 1] *= -1.0
                top_rings.append(top)
                bot_rings.append(bot)
            tops.append(loft(top_rings, "strip", "strip"))
            bottoms.append(loft(bot_rings, "strip", "strip"))
        return Mesh.merge(tops), Mesh.merge(bottoms)

    # ----------------------------------------------------------- elevon
    def elevon(self, side: str) -> Mesh:
        """Elevon in its hinge body frame (origin = hinge-line midpoint)."""
        s = self.spec
        sgn = _side_sign(side)
        e = s.elevons
        end_gap = 2.0 * float(e["hinge_gap_m"])
        y0 = float(e["inboard_y_m"]) + end_gap
        y1 = float(e["outboard_y_m"]) - end_gap
        rings = []
        n_arc, n_aft = 7, 8
        for y in np.linspace(y0, y1, 5):
            r = self.elevon_nose_radius(y)
            # Camber-line height at the hinge, so the surface sits in the
            # wing's contour at neutral.
            xh_c = (s.le_x(y) - s.hinge_x(y)) / s.chord(y)
            zc_c = float(_reflex_camber(np.array([xh_c]), self._camber_m, self._reflex)[0])
            hinge_pt = self._section_point(y, np.array([xh_c]), np.array([zc_c]))[0]
            te_pt = self._section_point(y, np.array([1.0]), np.array([0.0]))[0]
            te_half = 0.0006
            upper, lower = [], []
            # Rounded nose: quarter circle centred on the hinge line, then a
            # straight taper to the trailing edge; the lower side mirrors it.
            upper = []
            for th in np.linspace(0.0, math.pi / 2, n_arc)[:-1]:
                upper.append([hinge_pt[0] + r * math.cos(th), 0.0, hinge_pt[2] - r * math.sin(th)])
            for u in np.linspace(0.0, 1.0, n_aft):
                x = hinge_pt[0] + u * (te_pt[0] - hinge_pt[0])
                z_mid = hinge_pt[2] + u * (te_pt[2] - hinge_pt[2])
                half = r + u * (te_half - r)
                upper.append([x, 0.0, z_mid - half])
            for pt in upper:
                u = max((pt[0] - hinge_pt[0]) / (te_pt[0] - hinge_pt[0]), 0.0)
                z_mid = hinge_pt[2] + u * (te_pt[2] - hinge_pt[2])
                lower.append([pt[0], 0.0, 2.0 * z_mid - pt[2]])
            ring = np.vstack([np.asarray(upper), np.asarray(lower)[::-1]])
            ring[:, 1] = sgn * y
            rings.append(ring)
        mesh = loft(rings, "strip", "strip")
        return mesh.translated(-self.body_origin(f"{side}_elevon"))

    def elevon_horn(self, side: str) -> Mesh:
        """Control horn on the elevon underside near its inboard end."""
        s = self.spec
        sgn = _side_sign(side)
        e = s.elevons
        y = float(e["inboard_y_m"]) + 0.06
        xh = s.hinge_x(y)
        r = self.elevon_nose_radius(y)
        z_bottom = s.wing_z(y) + r * 0.6
        prof = [
            [xh - 0.004, z_bottom - 0.002],
            [xh - 0.032, z_bottom - 0.002],
            [xh - 0.024, z_bottom + 0.020],
            [xh - 0.010, z_bottom + 0.020],
        ]
        rings = []
        for dy in (-0.0012, 0.0012):
            rings.append(np.array([[x, sgn * y + dy, z] for x, z in prof]))
        mesh = loft(rings, "fan", "fan")
        return mesh.translated(-self.body_origin(f"{side}_elevon"))

    def hinge_rod(self, side: str) -> Mesh:
        p0, p1, _ = self.spec.elevon_hinge(side)
        d = (p1 - p0) / np.linalg.norm(p1 - p0)
        return cylinder_between(p0 - 0.004 * d, p1 + 0.004 * d, 0.0016, 10)

    # ---------------------------------------------------------- center pod
    def _pod_profile(self, t: np.ndarray) -> np.ndarray:
        # Rounded nose, long parallel middle, tapering boat-tail.
        nose = np.clip(t / 0.28, 0.0, 1.0)
        tail = np.clip((1.0 - t) / 0.42, 0.0, 1.0)
        return np.sqrt(np.sin(nose * math.pi / 2)) * (0.30 + 0.70 * np.sin(tail * math.pi / 2))

    def _superellipse(self, x: float, half_w: float, h_top: float, h_bot: float,
                      n: int = 28, p: float = 2.6) -> np.ndarray:
        th = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        c, s = np.cos(th), np.sin(th)
        y = half_w * np.sign(c) * np.abs(c) ** (2.0 / p)
        zf = np.sign(s) * np.abs(s) ** (2.0 / p)
        z = np.where(zf < 0.0, zf * h_top, zf * h_bot)
        return np.column_stack([np.full(n, x), y, z])

    def center_body(self) -> Mesh:
        b = self.spec.body
        x0, x1 = float(b["nose_x_m"]), float(b["tail_x_m"])
        W, H = float(b["max_width_m"]), float(b["max_height_m"])
        top = float(b["top_fraction"])
        rings = []
        for t in np.linspace(0.0, 1.0, 22)[1:]:
            t = float(t) ** 1.15
            f = float(self._pod_profile(np.array([t]))[0])
            x = x0 + t * (x1 - x0)
            rings.append(self._superellipse(x, 0.5 * W * f, H * top * f, H * (1 - top) * f * 0.85))
        # Tiny nose ring so the fan cap closes a smooth point.
        nose = self._superellipse(x0 + 0.002, 0.002, 0.002, 0.002)
        return loft([nose] + rings, "fan", "fan")

    def hatch(self) -> Mesh:
        b = self.spec.body
        x0, x1 = float(b["nose_x_m"]), float(b["tail_x_m"])
        W, H = float(b["max_width_m"]), float(b["max_height_m"])
        top = float(b["top_fraction"])
        h = b["hatch"]
        frac = float(h["width_fraction"])
        rings = []
        for x in np.linspace(float(h["from_x_m"]), float(h["to_x_m"]), 10):
            t = (x0 - x) / (x0 - x1)
            # Invert the t**1.15 remap used for the pod rings.
            f = float(self._pod_profile(np.array([t]))[0])
            hw, ht = 0.5 * W * f, H * top * f
            th = np.linspace(-0.5 * math.pi * frac, 0.5 * math.pi * frac, 9)
            p = 2.6
            arc = []
            for a in th:
                ca, sa = math.sin(a), -math.cos(a)  # around the top (-z)
                y = hw * math.copysign(abs(ca) ** (2 / p), ca)
                z = -ht * abs(sa) ** (2 / p)
                arc.append((y, z))
            outer = [[x, y * 1.02, z * 1.02 - 0.0025] for y, z in arc]
            inner = [[x, y * 0.97, z * 0.97] for y, z in arc]
            rings.append(np.array(outer + inner[::-1]))
        return loft(rings, "strip", "strip")

    def camera_window(self) -> Mesh:
        b = self.spec.body
        x = float(b["camera_window_x_m"])
        H = float(b["max_height_m"])
        return uv_sphere([x, 0.0, -0.18 * H], 0.012, 6, 12)

    # --------------------------------------------------------------- fins
    def fin(self, side: str) -> Mesh:
        s = self.spec
        f = s.fins
        sgn = _side_sign(side)
        y = sgn * s.half_span
        le0 = s.le_x(s.half_span) - float(f["root_le_offset_x_m"])
        cr, ct, h = float(f["root_chord_m"]), float(f["tip_chord_m"]), float(f["height_m"])
        sweep = math.tan(math.radians(float(f["le_sweep_deg"])))
        t = float(f["thickness"])
        z_root = s.wing_z(s.half_span) + 0.012  # a little below the wing plane
        rings = []
        for k, frac in enumerate(np.linspace(0.0, 1.0, 6)):
            hh = frac * (h + 0.012)
            c = cr + (ct - cr) * frac
            le = le0 - hh * sweep
            # Round the top: last ring shrinks the chord.
            if k == 5:
                c *= 0.75
                le -= 0.12 * c
            xc = _cosine_stations(self.CHORD_POINTS)
            yt = _naca_thickness(xc, t) * c
            upper = np.column_stack([le - xc * c, y + yt, np.full_like(xc, z_root - hh)])
            lower = np.column_stack([le - xc * c, y - yt, np.full_like(xc, z_root - hh)])
            rings.append(np.vstack([upper, lower[::-1]]))
        return loft(rings, "strip", "strip")

    def nav_light(self, side: str) -> Mesh:
        s = self.spec
        sgn = _side_sign(side)
        y = sgn * (s.half_span + 0.004)
        x = s.le_x(s.half_span) - 0.02
        return uv_sphere([x, y, s.wing_z(s.half_span) + 0.004], 0.008, 6, 10)

    # ---------------------------------------------------------- propulsion
    def nacelle(self, side: str) -> Mesh:
        s = self.spec
        n = s.propulsion["nacelle"]
        R, L = float(n["radius_m"]), float(n["length_m"])
        p = s.prop_position(side)
        x_front = p[0] - 0.004 - float(n["bell_length_m"]) - 0.002
        profile = [
            (x_front, 0.72 * R), (x_front - 0.004, 0.92 * R), (x_front - 0.015, R),
            (x_front - 0.45 * L, R), (x_front - 0.75 * L, 0.80 * R),
            (x_front - L, 0.35 * R),
        ]
        return revolve(profile, 20).translated([0.0, p[1], p[2]])

    def motor_bell(self, side: str) -> Mesh:
        n = self.spec.propulsion["nacelle"]
        r, length = float(n["bell_radius_m"]), float(n["bell_length_m"])
        prof = [(-0.004, 0.55 * r), (-0.006, r), (-0.004 - length, r), (-0.004 - length, 0.85 * r)]
        return revolve(prof, 16)

    def spinner(self, side: str) -> Mesh:
        p = self.spec.propulsion["prop"]
        r, length = float(p["spinner_radius_m"]), float(p["spinner_length_m"])
        prof = []
        for k in range(8):
            u = k / 7.0
            prof.append((0.004 + length * (1.0 - u), r * math.sin(0.5 * math.pi * u) ** 0.8))
        prof.append((-0.004, r))
        return revolve(prof, 18)

    def prop_hub(self, side: str) -> Mesh:
        r = float(self.spec.propulsion["prop"]["hub_radius_m"])
        return revolve([(0.006, r), (-0.006, r)], 14)

    def _blade_ring(self, radius: float, chord: float, pitch: float, thick: float,
                    spin: int) -> np.ndarray:
        """Blade section at a radius along +y; prop axis +x."""
        motion = np.array([0.0, 0.0, float(spin)])      # omega x r for r along +y
        le_dir = motion * math.cos(pitch) + np.array([1.0, 0.0, 0.0]) * math.sin(pitch)
        n_dir = np.cross(le_dir, [0.0, 1.0, 0.0])
        u = _cosine_stations(9)
        half_t = _naca_thickness(u, 1.0) * thick
        centre = np.array([0.0, radius, 0.0])
        upper = [centre + (0.35 - ui) * chord * le_dir + ht * n_dir for ui, ht in zip(u, half_t)]
        lower = [centre + (0.35 - ui) * chord * le_dir - ht * n_dir for ui, ht in zip(u, half_t)]
        # Tiny camber toward the thrust face keeps it from reading as a flat plate.
        camber = 0.04 * chord * np.sin(math.pi * u)
        upper = [pt + cm * np.array([1.0, 0.0, 0.0]) for pt, cm in zip(upper, camber)]
        lower = [pt + cm * np.array([1.0, 0.0, 0.0]) for pt, cm in zip(lower, camber)]
        return np.vstack([np.asarray(upper), np.asarray(lower)[::-1]])

    def prop_blades(self, side: str) -> tuple[Mesh, Mesh]:
        """(blades, painted tips) in the prop body frame."""
        p = self.spec.propulsion["prop"]
        spin = self.spec.motor_spin(side)
        R = 0.5 * float(p["diameter_m"])
        r_hub = float(p["hub_radius_m"]) * 0.8
        c_max, c_tip = float(p["max_chord_m"]), float(p["tip_chord_m"])
        b0, b1 = math.radians(float(p["root_pitch_deg"])), math.radians(float(p["tip_pitch_deg"]))
        t0 = float(p["thickness_m"])
        tip_paint = 0.86

        def section(rho: float) -> np.ndarray:
            rr = max(rho * R, r_hub)
            if rho < 0.3:
                c = 0.55 * c_max + (c_max - 0.55 * c_max) * (rho - r_hub / R) / (0.3 - r_hub / R)
            else:
                c = c_max + (c_tip - c_max) * (rho - 0.3) / 0.7
            if rho > 0.97:
                c *= 0.75
            pitch = b0 + (b1 - b0) * (rho - r_hub / R) / (1.0 - r_hub / R)
            thick = t0 * (1.0 - 0.55 * rho)
            return self._blade_ring(rr, c, pitch, thick, spin)

        inner_rhos = np.linspace(r_hub / R, tip_paint, 7)
        tip_rhos = np.concatenate([np.linspace(tip_paint, 0.97, 3), [1.0]])
        blade = loft([section(float(r)) for r in inner_rhos], "strip", "strip")
        tip = loft([section(float(r)) for r in tip_rhos], "strip", "strip")
        blades, tips = [], []
        count = int(p["blades"])
        for k in range(count):
            rot = _axis_angle([1.0, 0.0, 0.0], 2.0 * math.pi * k / count)
            blades.append(blade.transformed(rot))
            tips.append(tip.transformed(rot))
        return Mesh.merge(blades), Mesh.merge(tips)

    # --------------------------------------------------------------- all
    def parts(self) -> list[VisualPart]:
        parts: list[VisualPart] = []
        parts.append(VisualPart("center_body", "body", "center_body", self.center_body()))
        parts.append(VisualPart("hatch", "body", "hatch", self.hatch()))
        parts.append(VisualPart("camera_window", "body", "camera_window", self.camera_window()))
        for side in SIDES:
            top, bottom = self.wing_half(side)
            parts.append(VisualPart(f"{side}_wing_top", "body", "wing_top", top))
            parts.append(VisualPart(f"{side}_wing_bottom", "body", "wing_bottom", bottom))
            parts.append(VisualPart(f"{side}_wingtip_fin", "body", "fin", self.fin(side)))
            parts.append(VisualPart(f"{side}_nav_light", "body", f"nav_{side}", self.nav_light(side)))
            parts.append(VisualPart(f"{side}_hinge_rod", "body", "hinge", self.hinge_rod(side)))
            parts.append(VisualPart(f"{side}_nacelle", "body", "nacelle", self.nacelle(side)))
            parts.append(VisualPart(f"{side}_elevon", f"{side}_elevon", "elevon", self.elevon(side)))
            parts.append(VisualPart(f"{side}_elevon_horn", f"{side}_elevon", "nacelle", self.elevon_horn(side)))
            blades, tips = self.prop_blades(side)
            parts.append(VisualPart(f"{side}_prop_blades", f"{side}_prop", "prop_blade", blades))
            parts.append(VisualPart(f"{side}_prop_tips", f"{side}_prop", "prop_tip", tips))
            parts.append(VisualPart(f"{side}_prop_hub", f"{side}_prop", "prop_blade", self.prop_hub(side)))
            parts.append(VisualPart(f"{side}_spinner", f"{side}_prop", "spinner", self.spinner(side)))
            parts.append(VisualPart(f"{side}_motor_bell", f"{side}_prop", "motor_bell", self.motor_bell(side)))
        return parts
