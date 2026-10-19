#!/usr/bin/env python3
"""Build the AH-64D Apache pilot-station cockpit as a game-ready GLB.

The model is authored around the pilot's eye, which sits at the origin:

    Blender axes (what this script uses)
        +X  -> pilot's left
        -Y  -> nose / forward
        +Z  -> up

    glTF axes after export (Blender's Y-up conversion)
        +X  -> pilot's left
        +Y  -> up
        +Z  -> nose / forward

So a future `view::cockpit_spec("ah64d")` can use eye = (0, 0, 0) and place the
whole node with the usual offset/drop.  All dimensions are metres.

What is detailed (per the gamepressure AH-64D cockpit guide):
  * the flat-panel canopy frame: two windscreens, centre mullion, A-pillars,
    side windows, overhead skylights, chin windows, wipers, jettison handles;
  * the main instrument panel: two MPDs with their 20 softkeys, VID/BRT knobs,
    DAY/NT/MONO switches, EUFD, master caution/warning, fire detection,
    armament and IHADSS panels, standby attitude/airspeed/altimeter, compass,
    keyboard unit, wiper switch;
  * the avionics/side consoles, centre pedestal, radio stack, power-lever
    quadrant and the HDU stowage box;
  * HOCAS: cyclic with trigger guard, collective with twist grip, pedals.

Everything else (seat, floor, rear bulkhead, insulation quilts) is deliberately
simpler, as requested.

Run:
    blender -b --factory-startup --python tools/ah64_cockpit.py
    blender -b --factory-startup --python tools/ah64_cockpit.py -- --no-preview

Outputs:
    assets/ah64d_cockpit.glb
    /tmp/opencode/ah64_cockpit_preview/*.png
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from collections import defaultdict

import bmesh
import bpy
from mathutils import Euler, Matrix, Vector

# ---------------------------------------------------------------------------
# paths
# ---------------------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT_GLB = os.path.join(ROOT, "assets", "ah64d_cockpit.glb")
PREVIEW_DIR = "/tmp/opencode/ah64_cockpit_preview"

# ---------------------------------------------------------------------------
# small utilities
# ---------------------------------------------------------------------------


def srgb(hex_or_tuple):
    """sRGB hex/'#rrggbb' (or 0-255 tuple) -> linear RGB tuple."""
    if isinstance(hex_or_tuple, str):
        s = hex_or_tuple.lstrip("#")
        vals = [int(s[i : i + 2], 16) for i in (0, 2, 4)]
    else:
        vals = list(hex_or_tuple)

    def channel(c):
        c = c / 255.0
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

    return tuple(channel(c) for c in vals)


def make_material(name, colour, rough=0.6, metal=0.0, emission=None, strength=0.0, alpha=1.0):
    m = bpy.data.materials.get(name)
    if m:
        return m
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    m.diffuse_color = (*srgb(colour), alpha)
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*srgb(colour), 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if alpha < 1.0:
        bsdf.inputs["Alpha"].default_value = alpha
        # glTF wants BLEND; EEVEE Next calls the same thing DITHERED.
        try:
            m.surface_render_method = "DITHERED"
        except Exception:
            pass
        try:
            m.blend_method = "BLEND"
        except Exception:
            pass
        m.use_backface_culling = False
    if emission is not None:
        bsdf.inputs["Emission Color"].default_value = (*srgb(emission), 1.0)
        bsdf.inputs["Emission Strength"].default_value = strength
    return m


# ---------------------------------------------------------------------------
# materials
# ---------------------------------------------------------------------------

MAT = {}


def build_materials():
    MAT.update(
        {
            "frame": make_material("AH64_Frame", "#0f1113", rough=0.72, metal=0.08),
            "frame_edge": make_material("AH64_FrameEdge", "#181b1e", rough=0.62, metal=0.16),
            "panel": make_material("AH64_Panel", "#0d0f11", rough=0.84),
            "panel_alt": make_material("AH64_PanelAlt", "#15181b", rough=0.76),
            "panel_deep": make_material("AH64_PanelDeep", "#070809", rough=0.88),
            "rubber": make_material("AH64_Rubber", "#08090a", rough=0.95),
            "rubber_grey": make_material("AH64_RubberGrey", "#17191a", rough=0.88),
            "metal": make_material("AH64_Metal", "#7d838b", rough=0.36, metal=1.0),
            "dark_metal": make_material("AH64_DarkMetal", "#262a2f", rough=0.48, metal=0.8),
            "brass": make_material("AH64_Brass", "#a8843c", rough=0.35, metal=1.0),
            "glass": make_material("AH64_CanopyGlass", "#a3c2d0", rough=0.03, metal=0.0, alpha=0.16),
            "instrument_glass": make_material(
                "AH64_InstrumentGlass", "#bcd8e6", rough=0.05, metal=0.0, alpha=0.09
            ),
            "screen": make_material("AH64_Screen", "#040806", rough=0.5),
            "green": make_material(
                "AH64_ScreenGreen", "#31ff6a", rough=0.4, emission="#25ff5f", strength=3.2
            ),
            "green_dim": make_material(
                "AH64_ScreenGreenDim", "#1ca33f", rough=0.4, emission="#1cc94a", strength=1.25
            ),
            "map": make_material(
                "AH64_ScreenMap", "#b7ff7a", rough=0.4, emission="#a8ff5c", strength=2.2
            ),
            "amber": make_material(
                "AH64_Amber", "#ff9f24", rough=0.4, emission="#ff8c10", strength=2.8
            ),
            "red": make_material(
                "AH64_RedLight", "#ff2a16", rough=0.4, emission="#ff1a08", strength=2.8
            ),
            "white": make_material("AH64_White", "#d8dcd5", rough=0.55),
            "label": make_material(
                "AH64_Label", "#c7ccc3", rough=0.5, emission="#aeb4aa", strength=0.12
            ),
            "label_dim": make_material(
                "AH64_LabelDim", "#6f756c", rough=0.62, emission="#5d635a", strength=0.06
            ),
            "label_yellow": make_material(
                "AH64_LabelYellow", "#d9b52f", rough=0.5, emission="#c7a42a", strength=0.18
            ),
            "label_red": make_material(
                "AH64_LabelRed", "#d03428", rough=0.5, emission="#c02418", strength=0.2
            ),
            "yellow": make_material("AH64_Yellow", "#d0a91f", rough=0.6),
            "red_paint": make_material("AH64_RedPaint", "#8c1a12", rough=0.55),
            "olive": make_material("AH64_Olive", "#4c5139", rough=0.9),
            "olive_dark": make_material("AH64_OliveDark", "#343829", rough=0.95),
            "seat": make_material("AH64_Seat", "#3a3f31", rough=0.92),
            "canvas": make_material("AH64_Canvas", "#5a5e48", rough=0.95),
            "floor": make_material("AH64_Floor", "#25272a", rough=0.86),
            "black": make_material("AH64_Black", "#070809", rough=0.8),
            "wiper": make_material("AH64_Wiper", "#0a0b0c", rough=0.85),
            "white_hot": make_material(
                "AH64_WhiteHot", "#fff2df", rough=0.4, emission="#ffe8cc", strength=1.0
            ),
            "warm_lens": make_material(
                "AH64_WarmLens", "#d8b47a", rough=0.45, emission="#ffb45c", strength=0.55
            ),
            "att_sky": make_material("AH64_AttSky", "#0a1e30", rough=0.86),
            "att_ground": make_material("AH64_AttGround", "#24180b", rough=0.9),
            "screen_dark": make_material("AH64_ScreenDark", "#050806", rough=0.55),
        }
    )


# ---------------------------------------------------------------------------
# object construction helpers
# ---------------------------------------------------------------------------

GROUPS = defaultdict(list)
COCKPIT_OBJECTS = []


def _register(obj, group):
    obj.name = obj.name  # keep Blender's unique suffix
    if group:
        GROUPS[group].append(obj)
    COCKPIT_OBJECTS.append(obj)
    return obj


def _set_material(obj, mat):
    if mat is not None:
        obj.data.materials.append(mat)


def _mesh_object(name, bm, mat, group, smooth_all=False):
    me = bpy.data.meshes.new(name)
    if smooth_all:
        for face in bm.faces:
            face.smooth = True
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    _set_material(obj, mat)
    return _register(obj, group)


def _bevel(bm, width, segments=2):
    if width and width > 0.00002:
        bmesh.ops.bevel(
            bm,
            geom=list(bm.edges),
            offset=width,
            offset_type="OFFSET",
            segments=segments,
            profile=0.5,
            affect="EDGES",
            clamp_overlap=True,
        )


def cube(name, loc, size, mat, group, rot=(0.0, 0.0, 0.0), bevel=0.0012, segments=2):
    """Axis-aligned (optionally rotated) box, baked into world space."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    sx, sy, sz = size
    R = Euler(rot).to_matrix()
    T = Vector(loc)
    for v in bm.verts:
        v.co = R @ Vector((v.co.x * sx, v.co.y * sy, v.co.z * sz)) + T
    _bevel(bm, min(bevel, min(size) * 0.44), segments)
    return _mesh_object(name, bm, mat, group)


def cylinder(
    name,
    loc,
    radius,
    depth,
    mat,
    group,
    axis="Z",
    vertices=24,
    bevel=0.0005,
    smooth=True,
    direction=None,
):
    bm = bmesh.new()
    bmesh.ops.create_cone(
        bm,
        cap_ends=True,
        cap_tris=False,
        segments=vertices,
        radius1=radius,
        radius2=radius,
        depth=depth,
    )
    if direction is not None:
        R = Vector((0.0, 0.0, 1.0)).rotation_difference(Vector(direction).normalized()).to_matrix()
    else:
        R = {
            "Z": Matrix.Identity(3),
            "Y": Matrix.Rotation(math.radians(90.0), 3, "X"),
            "X": Matrix.Rotation(math.radians(90.0), 3, "Y"),
        }[axis]
    T = Vector(loc)
    for v in bm.verts:
        v.co = R @ v.co + T
    _bevel(bm, min(bevel, depth * 0.2))
    for face in bm.faces:
        face.smooth = smooth and len(face.verts) == 4
    return _mesh_object(name, bm, mat, group)


def sphere(name, loc, radius, mat, group, segments=20, rings=12):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(
        bm, u_segments=segments, v_segments=rings, radius=radius
    )
    T = Vector(loc)
    for v in bm.verts:
        v.co += T
    return _mesh_object(name, bm, mat, group, smooth_all=True)


def torus(
    name,
    loc,
    major,
    minor,
    mat,
    group,
    axis="Y",
    major_segments=32,
    minor_segments=8,
):
    bm = bmesh.new()
    ring = []
    for i in range(major_segments):
        theta = 2.0 * math.pi * i / major_segments
        row = []
        for j in range(minor_segments):
            phi = 2.0 * math.pi * j / minor_segments
            x = (major + minor * math.cos(phi)) * math.cos(theta)
            y = (major + minor * math.cos(phi)) * math.sin(theta)
            z = minor * math.sin(phi)
            row.append(bm.verts.new((x, y, z)))
        ring.append(row)
    for i in range(major_segments):
        for j in range(minor_segments):
            a = ring[i][j]
            b = ring[(i + 1) % major_segments][j]
            c = ring[(i + 1) % major_segments][(j + 1) % minor_segments]
            d = ring[i][(j + 1) % minor_segments]
            bm.faces.new((a, b, c, d))
    R = {
        "Z": Matrix.Identity(3),
        "Y": Matrix.Rotation(math.radians(90.0), 3, "X"),
        "X": Matrix.Rotation(math.radians(90.0), 3, "Y"),
    }[axis]
    T = Vector(loc)
    for v in bm.verts:
        v.co = R @ v.co + T
    bm.normal_update()
    return _mesh_object(name, bm, mat, group, smooth_all=True)


def bar(name, p1, p2, width, mat, group, depth=None, bevel=0.001):
    """Rectangular bar from p1 to p2. `width` is local X, `depth` local Y."""
    p1, p2 = Vector(p1), Vector(p2)
    delta = p2 - p1
    length = delta.length
    if length < 1e-7:
        return None
    depth = width if depth is None else depth
    rotation = Vector((0.0, 0.0, 1.0)).rotation_difference(delta.normalized())
    return cube(
        name,
        (p1 + p2) * 0.5,
        (width, depth, length),
        mat,
        group,
        rot=rotation.to_euler(),
        bevel=bevel,
        segments=2,
    )


def quad(name, points, mat, group, thickness=0.004, bevel=0.0015):
    bm = bmesh.new()
    verts = [bm.verts.new(tuple(p)) for p in points]
    bm.faces.new(verts)
    bm.normal_update()
    if thickness:
        bmesh.ops.solidify(bm, geom=list(bm.faces), thickness=thickness)
    if bevel:
        _bevel(bm, bevel, 2)
    return _mesh_object(name, bm, mat, group)


def prism_x(name, profile_yz, x0, x1, mat, group, bevel=0.004):
    """Extrude a closed (y,z) profile along X."""
    bm = bmesh.new()
    n = len(profile_yz)
    a = [bm.verts.new((x0, y, z)) for y, z in profile_yz]
    b = [bm.verts.new((x1, y, z)) for y, z in profile_yz]
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((a[i], a[j], b[j], b[i]))
    bm.faces.new(tuple(reversed(a)))
    bm.faces.new(tuple(b))
    bm.normal_update()
    _bevel(bm, bevel, 3)
    return _mesh_object(name, bm, mat, group)


def text(
    name,
    body,
    loc,
    size,
    mat,
    group,
    normal=(0.0, 1.0, 0.0),
    up=(0.0, 0.0, 1.0),
    align="CENTER",
    extrude=0.00022,
    font=None,
):
    cu = bpy.data.curves.new(name + "_curve", type="FONT")
    cu.body = body
    cu.size = size
    cu.align_x = align
    cu.align_y = "CENTER"
    cu.extrude = extrude
    cu.resolution_u = 2
    cu.space_character = 0.92
    if font:
        cu.font = font
    tmp = bpy.data.objects.new(name + "_tmp", cu)
    bpy.context.collection.objects.link(tmp)
    bpy.context.view_layer.update()

    n = Vector(normal).normalized()
    u = Vector(up).normalized()
    r = u.cross(n).normalized()
    M = Matrix.Translation(Vector(loc)) @ Matrix((r, u, n)).transposed().to_4x4()

    src = tmp.to_mesh()
    me = src.copy() if src is not None else bpy.data.meshes.new(name)
    if src is not None:
        tmp.to_mesh_clear()
    me.transform(M)
    me.materials.clear()
    me.materials.append(mat)
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.curves.remove(cu)
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    return _register(obj, group)


def screw(loc, normal, radius=0.0028, depth=0.0016, group="HARDWARE", mat=None):
    cylinder(
        "screw",
        loc,
        radius,
        depth,
        mat or MAT["dark_metal"],
        group,
        vertices=8,
        bevel=0.0002,
        smooth=False,
        direction=normal,
    )


def rivet_line(p1, p2, normal, spacing=0.075, radius=0.0022, group="HARDWARE"):
    p1, p2 = Vector(p1), Vector(p2)
    n = Vector(normal).normalized()
    length = (p2 - p1).length
    count = max(1, int(length / spacing))
    for i in range(count + 1):
        p = p1.lerp(p2, i / max(1, count)) + n * 0.001
        screw(p, n, radius=radius, depth=0.0012, group=group)


def knob(name, loc, radius, depth, mat, group, axis="Y", pointer=True, pointer_mat=None):
    cylinder(name, loc, radius, depth, mat, group, axis=axis, vertices=24)
    if pointer:
        if axis == "Y":
            p1 = Vector(loc) + Vector((0.0, depth * 0.55, 0.0))
            p2 = p1 + Vector((0.0, 0.0, radius * 0.78))
        elif axis == "Z":
            p1 = Vector(loc) + Vector((0.0, 0.0, depth * 0.55))
            p2 = p1 + Vector((0.0, radius * 0.78, 0.0))
        else:
            p1 = Vector(loc) + Vector((depth * 0.55, 0.0, 0.0))
            p2 = p1 + Vector((0.0, 0.0, radius * 0.78))
        bar(
            name + "_pointer",
            p1,
            p2,
            radius * 0.22,
            pointer_mat or MAT["label"],
            group,
            depth=radius * 0.18,
            bevel=0.00015,
        )


def _plane_basis(normal, up_hint=None):
    n = Vector(normal).normalized()
    up = Vector((0.0, 0.0, 1.0)) if up_hint is None else Vector(up_hint).normalized()
    if abs(up.dot(n)) > 0.95:
        up = Vector((0.0, -1.0, 0.0)) if abs(n.y) < 0.5 else Vector((0.0, 0.0, 1.0))
    up = (up - n * up.dot(n)).normalized()
    right = up.cross(n).normalized()
    return right, up, n


def toggle(name, loc, size, mat, group, angle=0.32, normal=(0, 1, 0), base=True):
    """A toggle switch on a panel with the given outward normal."""
    right, up, n = _plane_basis(normal)
    M = Matrix((right, up, n)).transposed()
    if base:
        cube(
            name + "_base",
            Vector(loc) - n * 0.004,
            (size * 2.4, size * 2.4, 0.008),
            MAT["dark_metal"],
            group,
            rot=M.to_euler(),
            bevel=0.001,
        )
    p1 = Vector(loc)
    p2 = p1 + n * (math.sin(angle) * size * 3.2) + up * (math.cos(angle) * size * 3.2)
    bar(name + "_lever", p1, p2, size * 0.55, MAT["metal"], group, depth=size * 0.55, bevel=0.0002)
    sphere(name + "_tip", p2, size * 0.75, MAT["black"], group, segments=12, rings=8)


def guarded_switch(name, loc, size, group, mat=None, normal=(0, 1, 0)):
    """Toggle with a yellow/black striped guard (engine/APU/fire panels)."""
    mat = mat or MAT["metal"]
    right, up, n = _plane_basis(normal)
    M = Matrix((right, up, n)).transposed()
    center = Vector(loc)
    cube(
        name + "_guard",
        center,
        (size * 3.1, size * 3.8, 0.010),
        MAT["yellow"],
        group,
        rot=M.to_euler(),
        bevel=0.0012,
    )
    for i in range(3):
        z = -size * 1.25 + i * size * 1.25
        p1 = center + M @ Vector((-size * 1.35, z, 0.006))
        p2 = center + M @ Vector((size * 1.35, z + size * 0.5, 0.006))
        bar(name + f"_stripe{i}", p1, p2, size * 0.3, MAT["black"], group, depth=0.0008, bevel=0.0)
    toggle(name, center + n * 0.010, size * 0.55, mat, group, angle=0.45, base=False, normal=normal)


def panel_plate(name, center, size, group, mat=None, bevel=0.003, screws=True):
    plate = cube(name, center, (size[0], 0.012, size[1]), mat or MAT["panel_alt"], group, bevel=bevel)
    if screws:
        x, y, z = center
        w, h = size
        for sx in (-1, 1):
            for sz in (-1, 1):
                screw(
                    (x + sx * (w / 2 - 0.008), y + 0.008, z + sz * (h / 2 - 0.008)),
                    (0, 1, 0),
                    radius=0.0024,
                )
    return plate


def top_plate(name, center, size, group, mat=None, bevel=0.003, screws=True):
    """A horizontal control panel lying on a console/pedestal."""
    plate = cube(name, center, (size[0], size[1], 0.010), mat or MAT["panel_alt"], group, bevel=bevel)
    if screws:
        x, y, z = center
        w, h = size
        for sx in (-1, 1):
            for sy in (-1, 1):
                screw(
                    (x + sx * (w / 2 - 0.008), y + sy * (h / 2 - 0.008), z + 0.006),
                    (0, 0, 1),
                    radius=0.0024,
                )
    return plate


def panel_plate(name, center, size, group, mat=None, bevel=0.003, screws=True):
    plate = cube(name, center, (size[0], 0.012, size[1]), mat or MAT["panel_alt"], group, bevel=bevel)
    if screws:
        x, y, z = center
        w, h = size
        for sx in (-1, 1):
            for sz in (-1, 1):
                screw(
                    (x + sx * (w / 2 - 0.008), y + 0.008, z + sz * (h / 2 - 0.008)),
                    (0, 1, 0),
                    radius=0.0024,
                )
    return plate


# ---------------------------------------------------------------------------
# cockpit geometry
# ---------------------------------------------------------------------------

PANEL_Y = -0.775
PANEL_Z = -0.16
MPD_CX = 0.140
MPD_CZ = -0.185
MPD_OUTER = 0.200
MPD_SCREEN = 0.152


def build_shell():
    g = "COCKPIT_SHELL"
    # floor and footwells
    cube("floor", (0.0, -0.28, -1.03), (1.04, 1.78, 0.045), MAT["floor"], g, bevel=0.006)
    for sx in (-1.0, 1.0):
        cube(
            "footwell_wall",
            (sx * 0.415, -0.780, -0.880),
            (0.040, 0.520, 0.260),
            MAT["panel_deep"],
            g,
            bevel=0.006,
        )
    # anti-slip strips on the floor
    for i in range(4):
        bar(
            "floor_grip",
            (-0.300, -0.980 + i * 0.095, -1.000),
            (0.300, -0.980 + i * 0.095, -1.000),
            0.018,
            MAT["rubber_grey"],
            g,
            depth=0.006,
            bevel=0.001,
        )

    # side walls / inner skins
    for side, sx in (("L", 1.0), ("R", -1.0)):
        cube(
            f"side_wall_{side}",
            (sx * 0.535, -0.32, -0.48),
            (0.055, 1.46, 0.92),
            MAT["panel"],
            g,
            bevel=0.006,
        )
        # quilted insulation: a grid of slightly raised pads
        for iz in range(4):
            for iy in range(7):
                cube(
                    f"quilt_{side}_{iz}_{iy}",
                    (
                        sx * 0.503,
                        -0.92 + iy * 0.19,
                        -0.82 + iz * 0.17,
                    ),
                    (0.012, 0.155, 0.13),
                    MAT["panel_alt"],
                    g,
                    bevel=0.008,
                )
        # a couple of cable runs
        cylinder(
            f"cable_{side}",
            (sx * 0.49, -0.45, 0.02),
            0.008,
            1.18,
            MAT["rubber"],
            g,
            axis="Y",
            vertices=10,
        )
        # side-wall placards and a stowed flashlight
        for i, label in enumerate(("HYD", "FUEL", "ELEC")):
            text(
                "side_placard",
                label,
                (sx * 0.505, 0.300 - i * 0.075, -0.360),
                0.0038,
                MAT["label_dim"],
                "LABELS",
                normal=(-sx, 0, 0),
                up=(0, 0, 1),
            )
        cylinder(
            "flashlight",
            (sx * 0.505, 0.050, -0.700),
            0.014,
            0.150,
            MAT["dark_metal"],
            "DETAILS",
            axis="Y",
            vertices=12,
        )
        cylinder(
            "flashlight_lens",
            (sx * 0.505, 0.128, -0.700),
            0.012,
            0.006,
            MAT["white_hot"],
            "DETAILS",
            axis="Y",
            vertices=12,
        )

        # vertical and horizontal structural ribs on the side wall
        for i in range(3):
            bar(
                "side_rib_v",
                (sx * 0.500, -0.760 + i * 0.310, -0.940),
                (sx * 0.500, -0.760 + i * 0.310, -0.120),
                0.024,
                MAT["panel_alt"],
                "DETAILS",
                depth=0.016,
                bevel=0.003,
            )
        for i in range(2):
            bar(
                "side_rib_h",
                (sx * 0.500, -0.930, -0.700 + i * 0.300),
                (sx * 0.500, 0.230, -0.700 + i * 0.300),
                0.024,
                MAT["panel_alt"],
                "DETAILS",
                depth=0.016,
                bevel=0.003,
            )

        # side-wall access panel seams and fasteners
        for i in range(3):
            bar(
                "side_seam",
                (sx * 0.507, -0.900 + i * 0.220, -0.930),
                (sx * 0.507, -0.900 + i * 0.220, -0.130),
                0.0025,
                MAT["frame_edge"],
                "DETAILS",
                depth=0.0025,
                bevel=0.0,
            )
        for iy in range(4):
            for iz in range(3):
                screw(
                    (sx * 0.505, -0.820 + iy * 0.220, -0.800 + iz * 0.220),
                    (-sx, 0, 0),
                    radius=0.0022,
                    depth=0.0012,
                )
        cube(
            "access_panel",
            (sx * 0.506, -0.740, -0.520),
            (0.012, 0.240, 0.180),
            MAT["panel_alt"],
            "DETAILS",
            bevel=0.003,
        )
        text(
            "access_label",
            "ACCESS",
            (sx * 0.500, -0.740, -0.440),
            0.0032,
            MAT["label_dim"],
            "LABELS",
            normal=(-sx, 0, 0),
            up=(0, 0, 1),
        )

        # canopy sill / coaming
        cube(
            f"sill_{side}",
            (sx * 0.455, -0.27, 0.275),
            (0.10, 1.44, 0.065),
            MAT["frame"],
            g,
            bevel=0.012,
        )
        # side console support structure
        cube(
            f"console_base_{side}",
            (sx * 0.412, -0.40, -0.60),
            (0.25, 1.05, 0.52),
            MAT["panel"],
            g,
            bevel=0.008,
        )

    # rear bulkhead + armoured seat tub
    cube("rear_bulkhead", (0.0, 0.49, -0.42), (1.06, 0.075, 1.18), MAT["panel"], g, bevel=0.008)
    cube("rear_armor", (0.0, 0.44, -0.24), (0.72, 0.06, 0.62), MAT["frame"], g, bevel=0.012)
    cube("rear_shelf", (0.0, 0.38, -0.92), (0.74, 0.24, 0.06), MAT["panel_alt"], g, bevel=0.005)

    # nose-side lower structure, kept below the chin windows
    cube("lower_nose", (0.0, -1.03, -0.48), (1.02, 0.20, 0.52), MAT["panel_deep"], g, bevel=0.01)
    cube("lower_nose_top", (0.0, -0.97, -0.215), (0.92, 0.28, 0.04), MAT["panel"], g, bevel=0.006)

    # canopy rear fairing
    cube("canopy_rear_fairing", (0.0, 0.43, 0.54), (0.98, 0.16, 0.22), MAT["frame"], g, bevel=0.012)

    # trim strips along the floor edges
    for sx in (-1.0, 1.0):
        bar(
            "floor_trim",
            (sx * 0.48, -1.05, -0.995),
            (sx * 0.48, 0.45, -0.995),
            0.02,
            MAT["dark_metal"],
            g,
            depth=0.012,
            bevel=0.002,
        )


def framed_window(name, pts, glass_mat=None, frame_group="CANOPY_FRAME", width=0.034, depth=0.040, glass_group="CANOPY_GLASS"):
    pts = [Vector(p) for p in pts]
    if glass_mat is not None:
        quad(name + "_glass", pts, glass_mat, glass_group, thickness=0.004, bevel=0.0018)
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        bar(
            f"{name}_frame{i}",
            p1,
            p2,
            width,
            MAT["frame"],
            frame_group,
            depth=depth,
            bevel=0.006,
        )
    # gasket line just inside the frame
    centre = sum(pts, Vector()) / 4.0
    inner = [p + (centre - p).normalized() * 0.010 for p in pts]
    for i in range(4):
        bar(
            f"{name}_gasket{i}",
            inner[i],
            inner[(i + 1) % 4],
            0.010,
            MAT["rubber"],
            frame_group,
            depth=0.012,
            bevel=0.0015,
        )
    # corner bolts
    for p in pts:
        sphere(name + "_bolt", p, 0.0055, MAT["dark_metal"], "HARDWARE", segments=10, rings=6)


def build_canopy():
    # ---- front windscreen: two panes split by a slim centre mullion ----
    z_bot, z_top = 0.255, 0.665
    y_bot, y_top = -0.995, -0.555
    x_bot_outer, x_top_outer = 0.442, 0.344
    x_inner = 0.020

    for side, sx in (("L", 1.0), ("R", -1.0)):
        pts = [
            (sx * x_inner, y_bot, z_bot),
            (sx * x_bot_outer, y_bot, z_bot),
            (sx * x_top_outer, y_top, z_top),
            (sx * x_inner, y_top, z_top),
        ]
        framed_window(f"windscreen_{side}", pts, MAT["glass"], width=0.038, depth=0.040)

    # centre mullion + A-pillars + header / sill
    bar("windscreen_mullion", (0.0, y_bot, z_bot), (0.0, y_top, z_top), 0.040, MAT["frame"], "CANOPY_FRAME", depth=0.045, bevel=0.008)
    bar("windscreen_sill", (-x_bot_outer, y_bot, z_bot), (x_bot_outer, y_bot, z_bot), 0.055, MAT["frame"], "CANOPY_FRAME", depth=0.05, bevel=0.008)
    bar("windscreen_header", (-x_top_outer, y_top, z_top), (x_top_outer, y_top, z_top), 0.055, MAT["frame"], "CANOPY_FRAME", depth=0.055, bevel=0.008)
    for sx in (-1.0, 1.0):
        bar("a_pillar", (sx * x_bot_outer, y_bot, z_bot), (sx * x_top_outer, y_top, z_top), 0.058, MAT["frame"], "CANOPY_FRAME", depth=0.055, bevel=0.009)
        # outer lip / rain gutter
        bar(
            "a_pillar_lip",
            (sx * (x_bot_outer + 0.026), y_bot - 0.018, z_bot),
            (sx * (x_top_outer + 0.022), y_top - 0.016, z_top),
            0.018,
            MAT["frame_edge"],
            "CANOPY_FRAME",
            depth=0.018,
            bevel=0.003,
        )
        # A-pillar inner padding
        bar(
            "a_pillar_pad",
            (sx * (x_bot_outer - 0.018), y_bot + 0.006, z_bot - 0.01),
            (sx * (x_top_outer - 0.012), y_top + 0.006, z_top + 0.01),
            0.026,
            MAT["rubber_grey"],
            "CANOPY_FRAME",
            depth=0.022,
            bevel=0.005,
        )

    # ---- side windows ----
    for side, sx in (("L", 1.0), ("R", -1.0)):
        x = sx * 0.474
        pts_main = [
            (x, -0.955, 0.300),
            (x, -0.655, 0.645),
            (x, -0.090, 0.625),
            (x, -0.090, 0.300),
        ]
        framed_window(f"side_window_main_{side}", pts_main, MAT["glass"], width=0.036, depth=0.040)
        pts_quarter = [
            (x, -0.090, 0.300),
            (x, -0.090, 0.625),
            (x, 0.360, 0.590),
            (x, 0.360, 0.310),
        ]
        framed_window(f"side_window_quarter_{side}", pts_quarter, MAT["glass"], width=0.036, depth=0.040)
        # lower sill trim and coaming
        bar("side_sill", (x, -0.995, 0.285), (x, 0.390, 0.300), 0.060, MAT["frame"], "CANOPY_FRAME", depth=0.075, bevel=0.010)
        # forward side-window post: separates the CPG window from the pilot's
        bar(
            "side_window_front_post",
            (x, -0.555, 0.300),
            (x, -0.555, 0.645),
            0.032,
            MAT["frame"],
            "CANOPY_FRAME",
            depth=0.045,
            bevel=0.006,
        )
        # diagonal brace from the A-pillar foot to that post
        bar(
            "side_window_brace",
            (x, -0.900, 0.310),
            (x, -0.555, 0.410),
            0.022,
            MAT["frame_edge"],
            "CANOPY_FRAME",
            depth=0.030,
            bevel=0.004,
        )

        # vertical divider between the two side windows
        bar("side_window_divider", (x, -0.090, 0.290), (x, -0.090, 0.625), 0.040, MAT["frame"], "CANOPY_FRAME", depth=0.045, bevel=0.007)
        # rear post
        bar("canopy_rear_post", (x, 0.360, 0.300), (x, 0.360, 0.590), 0.040, MAT["frame"], "CANOPY_FRAME", depth=0.045, bevel=0.007)

    # ---- overhead skylights ----
    z_roof = 0.657
    for side, sx in (("L", 1.0), ("R", -1.0)):
        pts = [
            (sx * 0.022, -0.535, z_roof),
            (sx * 0.440, -0.535, z_roof),
            (sx * 0.440, 0.330, z_roof - 0.020),
            (sx * 0.022, 0.330, z_roof - 0.020),
        ]
        framed_window(f"skylight_{side}", pts, MAT["glass"], width=0.030, depth=0.034)
    bar("roof_spine", (0.0, -0.555, z_roof), (0.0, 0.335, z_roof - 0.020), 0.045, MAT["frame"], "CANOPY_FRAME", depth=0.050, bevel=0.008)
    bar("roof_cross", (-0.445, -0.060, z_roof - 0.010), (0.445, -0.060, z_roof - 0.010), 0.038, MAT["frame"], "CANOPY_FRAME", depth=0.045, bevel=0.006)
    bar("roof_front", (-0.445, -0.545, z_roof), (0.445, -0.545, z_roof), 0.048, MAT["frame"], "CANOPY_FRAME", depth=0.052, bevel=0.008)
    bar("roof_rear", (-0.445, 0.328, z_roof - 0.020), (0.445, 0.328, z_roof - 0.020), 0.045, MAT["frame"], "CANOPY_FRAME", depth=0.048, bevel=0.007)
    for sx in (-1.0, 1.0):
        bar(
            "roof_side_rail",
            (sx * 0.452, -0.545, z_roof - 0.005),
            (sx * 0.452, 0.330, z_roof - 0.025),
            0.034,
            MAT["frame"],
            "CANOPY_FRAME",
            depth=0.040,
            bevel=0.006,
        )

    # ---- chin windows ----
    for side, sx in (("L", 1.0), ("R", -1.0)):
        pts = [
            (sx * 0.105, -0.865, 0.095),
            (sx * 0.335, -0.865, 0.095),
            (sx * 0.300, -1.120, -0.205),
            (sx * 0.120, -1.120, -0.205),
        ]
        framed_window(f"chin_window_{side}", pts, MAT["glass"], width=0.030, depth=0.034)

    # ---- frame fasteners and corner gussets ----
    for sx in (-1.0, 1.0):
        # bolts along the windshield sill
        for i in range(9):
            screw(
                (sx * (0.055 + i * 0.048), -0.985, 0.262),
                (0, 1, 0),
                radius=0.0026,
                depth=0.0014,
            )
        # bolts along the side sill (inner face)
        for i in range(10):
            screw(
                (sx * 0.407, -0.900 + i * 0.135, 0.292),
                (-sx, 0, 0),
                radius=0.0026,
                depth=0.0014,
            )
        # corner gussets
        cube("frame_gusset", (sx * 0.382, -0.615, 0.645), (0.055, 0.055, 0.040), MAT["frame_edge"], "CANOPY_FRAME", bevel=0.006)
        cube("frame_gusset", (sx * 0.430, -0.960, 0.285), (0.050, 0.050, 0.035), MAT["frame_edge"], "CANOPY_FRAME", bevel=0.006)

    # ---- panel flood lights under the glareshield ----
    for sx in (-0.20, 0.20):
        cylinder("flood_light", (sx, -0.750, 0.170), 0.009, 0.016, MAT["panel_deep"], "PANEL_HARDWARE", axis="Y", vertices=16)
        cylinder("flood_lens", (sx, -0.740, 0.170), 0.006, 0.003, MAT["warm_lens"], "PANEL_HARDWARE", axis="Y", vertices=16)

    # ---- windshield coaming / defog lip ----
    cube(
        "windshield_coaming",
        (0.0, -0.995, 0.213),
        (0.900, 0.070, 0.045),
        MAT["panel"],
        "CANOPY_FRAME",
        bevel=0.008,
    )
    for i in range(7):
        cube(
            "defog_slot",
            (-0.300 + i * 0.100, -1.028, 0.226),
            (0.055, 0.006, 0.010),
            MAT["black"],
            "CANOPY_FRAME",
            bevel=0.001,
        )

    # ---- side-window diagonal frame, door hinge and latch ----
    for side, sx in (("L", 1.0), ("R", -1.0)):
        x = sx * 0.474
        # diagonal post splitting the forward quarter from the pilot's window
        bar(
            "side_window_diag",
            (x, -0.350, 0.300),
            (x, -0.215, 0.630),
            0.030,
            MAT["frame"],
            "CANOPY_FRAME",
            depth=0.042,
            bevel=0.005,
        )
        bar(
            "side_window_lower",
            (x, -0.955, 0.300),
            (x, -0.350, 0.300),
            0.028,
            MAT["frame"],
            "CANOPY_FRAME",
            depth=0.036,
            bevel=0.005,
        )
        # door hinge and latch
        cube("canopy_hinge", (x, -0.930, 0.292), (0.028, 0.055, 0.045), MAT["dark_metal"], "CANOPY_DETAILS", bevel=0.004)
        cube("canopy_latch", (x, 0.315, 0.300), (0.030, 0.040, 0.060), MAT["dark_metal"], "CANOPY_DETAILS", bevel=0.004)
        bar("canopy_latch_lever", (x, 0.300, 0.300), (x, 0.255, 0.285), 0.014, MAT["metal"], "CANOPY_DETAILS", depth=0.014, bevel=0.002)
        # small map pocket on the side wall
        cube("map_pocket", (sx * 0.495, 0.180, -0.250), (0.020, 0.180, 0.140), MAT["canvas"], "DETAILS", bevel=0.008)
        text(
            "map_pocket_label",
            "MAP",
            (sx * 0.482, 0.180, -0.220),
            0.0035,
            MAT["label_dim"],
            "LABELS",
            normal=(-sx, 0, 0),
            up=(0, 0, 1),
        )

    # ---- canopy latches, handles, jettison ----
    # overhead grab handles
    for sx in (-1.0, 1.0):
        p1 = (sx * 0.30, 0.10, 0.625)
        p2 = (sx * 0.30, 0.10, 0.600)
        p3 = (sx * 0.30, 0.19, 0.600)
        bar("grab_handle", p1, p2, 0.012, MAT["dark_metal"], "CANOPY_DETAILS", depth=0.012, bevel=0.002)
        bar("grab_handle", p2, p3, 0.012, MAT["dark_metal"], "CANOPY_DETAILS", depth=0.012, bevel=0.002)
        bar("grab_handle", p3, (sx * 0.30, 0.19, 0.625), 0.012, MAT["dark_metal"], "CANOPY_DETAILS", depth=0.012, bevel=0.002)

    # canopy jettison handle (yellow/black, left rear)
    cube("jettison_base", (0.395, 0.235, 0.355), (0.055, 0.075, 0.035), MAT["panel_alt"], "CANOPY_DETAILS", bevel=0.004)
    for i in range(4):
        cube(
            "jettison_stripe",
            (0.395, 0.245 + i * 0.016, 0.372),
            (0.045, 0.007, 0.008),
            MAT["yellow"] if i % 2 == 0 else MAT["black"],
            "CANOPY_DETAILS",
            bevel=0.001,
        )
    text("canopy_jettison_text", "CANOPY JETTISON", (0.395, 0.205, 0.322), 0.0075, MAT["label_yellow"], "LABELS_WARN", normal=(0, 0, 1), up=(0, 1, 0))

    # emergency release (red) on the right rear
    cube("emergency_release", (-0.395, 0.230, 0.350), (0.040, 0.040, 0.028), MAT["red_paint"], "CANOPY_DETAILS", bevel=0.005)
    cube("emergency_release_top", (-0.395, 0.230, 0.372), (0.025, 0.055, 0.010), MAT["dark_metal"], "CANOPY_DETAILS", bevel=0.002)

    # canopy latches along the header
    for sx in (-0.22, 0.22):
        cube("canopy_latch", (sx, -0.545, 0.700), (0.055, 0.030, 0.020), MAT["dark_metal"], "CANOPY_DETAILS", bevel=0.003)
        cylinder("latch_pin", (sx, -0.560, 0.700), 0.010, 0.060, MAT["metal"], "CANOPY_DETAILS", axis="Y", vertices=12)

    # wire cutter above the centre mullion
    bar("wire_cutter", (0.0, -0.550, 0.700), (0.0, -0.480, 0.735), 0.018, MAT["metal"], "CANOPY_DETAILS", depth=0.030, bevel=0.003)


def wiper(side, sx):
    g = "CANOPY_DETAILS"
    motor = (sx * 0.205, -0.930, 0.305)
    cube("wiper_motor", motor, (0.060, 0.050, 0.035), MAT["wiper"], g, bevel=0.004)
    pivot = (sx * 0.205, -0.905, 0.330)
    elbow = (sx * 0.115, -0.885, 0.455)
    tip = (sx * 0.285, -0.875, 0.500)
    bar("wiper_arm", pivot, elbow, 0.010, MAT["dark_metal"], g, depth=0.008, bevel=0.0015)
    bar("wiper_arm2", elbow, tip, 0.010, MAT["dark_metal"], g, depth=0.008, bevel=0.0015)
    bar("wiper_blade", (sx * 0.240, -0.872, 0.492), (sx * 0.330, -0.868, 0.515), 0.014, MAT["rubber"], g, depth=0.010, bevel=0.002)
    cylinder("wiper_pivot", pivot, 0.014, 0.020, MAT["dark_metal"], g, axis="Y", vertices=16)
    text(
        f"wiper_label_{side}",
        "WIPER",
        (sx * 0.205, -0.940, 0.270),
        0.0065,
        MAT["label_dim"],
        "LABELS",
    )


def build_glareshield():
    g = "GLARESHIELD"
    profile = [
        (-0.745, 0.188),
        (-0.760, 0.225),
        (-0.880, 0.250),
        (-1.015, 0.230),
        (-1.045, 0.175),
        (-1.030, 0.120),
        (-0.860, 0.115),
        (-0.760, 0.145),
    ]
    prism_x("glareshield", profile, -0.438, 0.438, MAT["panel"], g, bevel=0.010)
    # anti-glare lip
    bar("glareshield_lip", (-0.438, -1.045, 0.175), (0.438, -1.045, 0.175), 0.020, MAT["rubber"], g, depth=0.035, bevel=0.005)
    # side cheeks / eyebrows
    for sx in (-1.0, 1.0):
        prism_x(
            "glareshield_cheek",
            [
                (-0.755, 0.180),
                (-0.890, 0.205),
                (-1.000, 0.140),
                (-0.990, 0.035),
                (-0.760, 0.035),
            ],
            sx * 0.438,
            sx * 0.470,
            MAT["panel_alt"],
            g,
            bevel=0.008,
        )
    # panel seams / access covers on the shroud
    for i in range(5):
        bar(
            "glareshield_seam",
            (-0.400, -0.860 + i * 0.035, 0.253 - i * 0.006),
            (0.400, -0.860 + i * 0.035, 0.253 - i * 0.006),
            0.0025,
            MAT["frame_edge"],
            g,
            depth=0.0025,
            bevel=0.0,
        )
    for sx in (-0.30, 0.0, 0.30):
        bar(
            "glareshield_seam_long",
            (sx, -1.030, 0.180),
            (sx, -0.760, 0.215),
            0.0025,
            MAT["frame_edge"],
            g,
            depth=0.0025,
            bevel=0.0,
        )

    # a few fasteners across the shroud
    for sx in (-0.36, -0.18, 0.0, 0.18, 0.36):
        screw((sx, -1.046, 0.195), (0, -0.2, 1), radius=0.0028, group="HARDWARE")


def build_panel():
    g = "INSTRUMENT_PANEL"
    # main panel slab and face plate
    cube("panel_slab", (0.0, PANEL_Y - 0.018, PANEL_Z), (0.720, 0.036, 0.740), MAT["panel"], g, bevel=0.008)
    cube("panel_face", (0.0, PANEL_Y + 0.002, PANEL_Z), (0.700, 0.008, 0.720), MAT["panel_alt"], g, bevel=0.004)
    # centre raised spine between the MPDs
    cube("panel_spine", (0.0, PANEL_Y + 0.010, -0.185), (0.062, 0.018, 0.395), MAT["panel"], g, bevel=0.005)
    # lower switch/breaker strip
    cube("panel_lower_strip", (0.0, PANEL_Y + 0.012, -0.472), (0.660, 0.022, 0.055), MAT["panel"], g, bevel=0.004)
    # side returns to the consoles
    for sx in (-1.0, 1.0):
        cube(
            "panel_cheek",
            (sx * 0.371, PANEL_Y + 0.005, -0.330),
            (0.075, 0.055, 0.500),
            MAT["panel"],
            g,
            bevel=0.006,
            rot=(0.0, math.radians(sx * 4.0), 0.0),
        )
    # top cap under the glareshield
    cube("panel_top_cap", (0.0, PANEL_Y + 0.004, 0.195), (0.710, 0.030, 0.055), MAT["panel"], g, bevel=0.004)

    # centre spine: mode / brightness switches between the two MPDs
    for i, label in enumerate(("MODE", "BRT", "VID", "DAY")):
        toggle("spine_sw", (0.0, PANEL_Y + 0.028, -0.275 + i * 0.055), 0.0055, MAT["metal"], g, angle=0.35)
        text("spine_label", label, (0.0, PANEL_Y + 0.033, -0.252 + i * 0.055), 0.0038, MAT["label_dim"], "LABELS")
    for i in range(3):
        cube("spine_led", (-0.012 + i * 0.012, PANEL_Y + 0.030, -0.300), (0.006, 0.002, 0.006), MAT["green_dim"], g, bevel=0.0003)

    text("station_label", "PLT STATION", (0.0, PANEL_Y + 0.030, -0.540), 0.0042, MAT["label_dim"], "LABELS")

    # master caution / warning buttons above the EUFD
    for sx, label, mat in (
        (-0.108, "MASTER WARN", MAT["red"]),
        (0.108, "MASTER CAUTION", MAT["amber"]),
    ):
        cube("master_button", (sx, PANEL_Y + 0.015, 0.118), (0.058, 0.020, 0.017), MAT["panel_deep"], g, bevel=0.003)
        cube("master_lens", (sx, PANEL_Y + 0.027, 0.118), (0.046, 0.004, 0.009), mat, g, bevel=0.0015)
        text(
            "master_label",
            label,
            (sx, PANEL_Y + 0.030, 0.132),
            0.0042,
            MAT["label_dim"],
            "LABELS",
        )

    # fire detection / extinguishing panel (left top)
    fire_c = (0.292, PANEL_Y + 0.014, 0.150)
    panel_plate("fire_panel", fire_c, (0.125, 0.072), g)
    text("fire_title", "FIRE DET/EXTG", (0.292, PANEL_Y + 0.029, 0.178), 0.0056, MAT["label"], "LABELS")
    for i, (dx, label) in enumerate(((0.0, "ENG 1"), (0.040, "APU"), (0.080, "ENG 2"))):
        guarded_switch(f"fire_sw_{i}", (0.257 + i * 0.034, PANEL_Y + 0.025, 0.144), 0.0065, g)
        text(
            "fire_sw_label",
            label,
            (0.257 + i * 0.034, PANEL_Y + 0.031, 0.164),
            0.0045,
            MAT["label_dim"],
            "LABELS",
        )
    for i, label in enumerate(("TEST", "RESET")):
        cylinder("fire_test", (0.238 + i * 0.030, PANEL_Y + 0.025, 0.121), 0.007, 0.006, MAT["dark_metal"], g, axis="Y", vertices=16)
        text("fire_test_label", label, (0.238 + i * 0.030, PANEL_Y + 0.030, 0.110), 0.0042, MAT["label_dim"], "LABELS")

    # armament panel (left, below the left MPD)
    arm_c = (0.300, PANEL_Y + 0.014, -0.350)
    panel_plate("armament_panel", arm_c, (0.115, 0.155), g)
    text("armament_title", "ARMAMENT", (0.300, PANEL_Y + 0.029, -0.287), 0.0056, MAT["label"], "LABELS")
    for i, (dz, label) in enumerate(((-0.012, "A/S"), (-0.050, "GND ORIDE"), (-0.088, "BURST"), (-0.122, "ACTION"))):
        toggle(f"arm_sw_{i}", (0.286, PANEL_Y + 0.026, -0.320 + dz), 0.007, MAT["metal"], g, angle=0.4)
        text("arm_label", label, (0.320, PANEL_Y + 0.030, -0.320 + dz), 0.0044, MAT["label_dim"], "LABELS")

    # IHADSS adjustment panel
    ih_c = (0.300, PANEL_Y + 0.014, -0.455)
    panel_plate("ihadss_panel", ih_c, (0.115, 0.060), g)
    text("ihadss_title", "IHADSS", (0.300, PANEL_Y + 0.029, -0.438), 0.0054, MAT["label"], "LABELS")
    for i, (dx, label) in enumerate(((-0.028, "BRT"), (0.028, "VID"))):
        knob("ihadss_knob", (0.300 + dx, PANEL_Y + 0.022, -0.468), 0.010, 0.012, MAT["dark_metal"], g)
        text("ihadss_knob_label", label, (0.300 + dx, PANEL_Y + 0.030, -0.492), 0.0040, MAT["label_dim"], "LABELS")
    toggle("ihadss_sw", (0.300, PANEL_Y + 0.026, -0.448), 0.0055, MAT["metal"], g, angle=0.3)

    # wiper switch
    wiper_c = (0.0, PANEL_Y + 0.014, -0.405)
    panel_plate("wiper_panel", wiper_c, (0.105, 0.048), g)
    text("wiper_title", "WIPER", (0.0, PANEL_Y + 0.029, -0.395), 0.0050, MAT["label"], "LABELS")
    toggle("wiper_sw", (0.0, PANEL_Y + 0.026, -0.415), 0.006, MAT["metal"], g, angle=0.4)

    # keyboard unit (KDU) on the left console, angled toward the pilot
    build_keyboard()

    # circuit breaker rows on the lower strip
    for i in range(11):
        x = -0.280 + i * 0.056
        cylinder("breaker", (x, PANEL_Y + 0.030, -0.472), 0.007, 0.010, MAT["dark_metal"], g, axis="Y", vertices=12)
        cylinder("breaker_cap", (x, PANEL_Y + 0.037, -0.472), 0.0045, 0.004, MAT["white"], g, axis="Y", vertices=12)
    text("breaker_label", "CB  ON  OFF", (0.0, PANEL_Y + 0.031, -0.500), 0.0045, MAT["label_dim"], "LABELS")

    # panel screws around the edge
    for sx in (-0.34, -0.17, 0.0, 0.17, 0.34):
        for sz in (-0.52, 0.20):
            screw((sx, PANEL_Y + 0.020, sz), (0, 1, 0), radius=0.0030)
    for sz in (-0.44, -0.32, -0.20, -0.08, 0.04):
        for sx in (-0.34, 0.34):
            screw((sx, PANEL_Y + 0.020, sz), (0, 1, 0), radius=0.0030)


def build_mpd(side, cx):
    g = "PANEL_HARDWARE"
    cz = MPD_CZ
    y = PANEL_Y
    # backplate and recessed screen
    cube("mpd_body", (cx, y + 0.014, cz), (MPD_OUTER + 0.018, 0.024, MPD_OUTER + 0.018), MAT["panel_deep"], g, bevel=0.005)
    cube("mpd_screen", (cx, y + 0.028, cz), (MPD_SCREEN, 0.004, MPD_SCREEN), MAT["screen"], "MPD_SCREEN_" + side, bevel=0.001)
    # raised bezel: four rails
    half = MPD_OUTER / 2
    inner = MPD_SCREEN / 2
    for i, (p1, p2) in enumerate(
        (
            ((cx - half, y + 0.035, cz - half), (cx + half, y + 0.035, cz - half)),
            ((cx - half, y + 0.035, cz + half), (cx + half, y + 0.035, cz + half)),
            ((cx - half, y + 0.035, cz - half), (cx - half, y + 0.035, cz + half)),
            ((cx + half, y + 0.035, cz - half), (cx + half, y + 0.035, cz + half)),
        )
    ):
        bar("mpd_bezel", p1, p2, 0.018, MAT["panel_alt"], g, depth=0.022, bevel=0.003)
    # inner screen lip
    for p1, p2 in (
        ((cx - inner - 0.006, y + 0.033, cz - inner - 0.006), (cx + inner + 0.006, y + 0.033, cz - inner - 0.006)),
        ((cx - inner - 0.006, y + 0.033, cz + inner + 0.006), (cx + inner + 0.006, y + 0.033, cz + inner + 0.006)),
        ((cx - inner - 0.006, y + 0.033, cz - inner - 0.006), (cx - inner - 0.006, y + 0.033, cz + inner + 0.006)),
        ((cx + inner + 0.006, y + 0.033, cz - inner - 0.006), (cx + inner + 0.006, y + 0.033, cz + inner + 0.006)),
    ):
        bar("mpd_screen_lip", p1, p2, 0.006, MAT["dark_metal"], g, depth=0.006, bevel=0.001)

    # 20 softkeys (6 per side, 4 top + 4 bottom) with a white legend bar
    keys = []
    for i in range(6):
        z = cz - 0.075 + i * 0.030
        keys.append((cx - 0.092, y + 0.040, z))
        keys.append((cx + 0.092, y + 0.040, z))
    for i in range(4):
        x = cx - 0.060 + i * 0.040
        keys.append((x, y + 0.040, cz - 0.092))
        keys.append((x, y + 0.040, cz + 0.092))
    for i, (x, yy, z) in enumerate(keys):
        cube("mpd_key", (x, yy, z), (0.019, 0.012, 0.014), MAT["panel_alt"], g, bevel=0.0018)
        cube("mpd_key_legend", (x, yy + 0.0065, z), (0.013, 0.0015, 0.004), MAT["label_dim"], g, bevel=0.0003)

    # VID / BRT knobs and DAY/NT/MONO switch on the bezel
    for dx, label in ((-0.086, "VID"), (0.086, "BRT")):
        knob("mpd_knob", (cx + dx, y + 0.042, cz + 0.104), 0.011, 0.014, MAT["dark_metal"], g)
        text("mpd_knob_label", label, (cx + dx, y + 0.048, cz + 0.124), 0.0046, MAT["label"], "LABELS")
    for i, label in enumerate(("DAY", "NT", "MONO")):
        toggle("mpd_mode", (cx - 0.026 + i * 0.026, y + 0.044, cz - 0.108), 0.0045, MAT["metal"], g, angle=0.25)
        text("mpd_mode_label", label, (cx - 0.026 + i * 0.026, y + 0.048, cz - 0.124), 0.0038, MAT["label_dim"], "LABELS")
    # corner screws
    for sx in (-1, 1):
        for sz in (-1, 1):
            screw((cx + sx * 0.094, y + 0.045, cz + sz * 0.094), (0, 1, 0), radius=0.0024)

    # screen content (separate nodes for future live pages)
    if side == "L":
        build_engine_page(cx, cz, y + 0.032, MPD_SCREEN)
    else:
        build_tsd_page(cx, cz, y + 0.032, MPD_SCREEN)


def _spt(cx, cz, y, size, u, v):
    """Screen-local coordinate: u,v in [-1,1], +u = pilot's left."""
    return (cx + u * size * 0.5, y, cz + v * size * 0.5)


def _screen_text(cx, cz, y, size, u, v, body, s=0.0055, mat=None, group=None):
    text(
        "screen_text",
        body,
        _spt(cx, cz, y, size, u, v),
        s,
        mat or MAT["green"],
        group or "SCREEN_CONTENT",
    )


def _screen_line(cx, cz, y, size, u1, v1, u2, v2, w=0.0012, mat=None, group=None):
    bar(
        "screen_line",
        _spt(cx, cz, y, size, u1, v1),
        _spt(cx, cz, y, size, u2, v2),
        w,
        mat or MAT["green_dim"],
        group or "SCREEN_CONTENT",
        depth=0.0012,
        bevel=0.0,
    )


def build_engine_page(cx, cz, y, size):
    g = "MPD_CONTENT_L"
    # frame border
    for u1, v1, u2, v2 in ((-0.95, -0.95, 0.95, -0.95), (0.95, -0.95, 0.95, 0.95), (0.95, 0.95, -0.95, 0.95), (-0.95, 0.95, -0.95, -0.95)):
        _screen_line(cx, cz, y + 0.001, size, u1, v1, u2, v2, w=0.0015, group=g)
    _screen_text(cx, cz, y + 0.0015, size, 0.62, 0.84, "ENG", 0.0075, MAT["green"], g)
    _screen_text(cx, cz, y + 0.0015, size, 0.14, 0.84, "1", 0.0075, MAT["green"], g)
    _screen_text(cx, cz, y + 0.0015, size, -0.34, 0.84, "2", 0.0075, MAT["green"], g)
    _screen_text(cx, cz, y + 0.0015, size, -0.73, 0.84, "TQ", 0.0055, MAT["green"], g)
    # two torque columns
    heights = [
        [0.18, 0.25, 0.33, 0.42, 0.50, 0.56, 0.62, 0.59, 0.54, 0.48, 0.40, 0.31],
        [0.20, 0.28, 0.35, 0.44, 0.52, 0.58, 0.64, 0.61, 0.56, 0.49, 0.41, 0.32],
    ]
    for col, u0 in enumerate((0.50, -0.12)):
        for i, h in enumerate(heights[col]):
            u = u0 + i * 0.021
            _screen_line(cx, cz, y + 0.001, size, u, -0.58, u, -0.58 + h, w=0.0065, mat=MAT["green"], group=g)
        _screen_line(cx, cz, y + 0.001, size, u0 - 0.02, -0.60, u0 + 0.28, -0.60, w=0.0018, group=g)
        # scale ticks
        for i in range(5):
            vv = -0.58 + i * 0.14
            _screen_line(cx, cz, y + 0.001, size, u0 - 0.02, vv, u0 - 0.005, vv, w=0.0012, group=g)
    _screen_text(cx, cz, y + 0.0015, size, 0.50, 0.62, "100", 0.0045, MAT["green_dim"], g)
    _screen_text(cx, cz, y + 0.0015, size, 0.50, 0.06, "50", 0.0045, MAT["green_dim"], g)
    _screen_text(cx, cz, y + 0.0015, size, -0.12, 0.62, "100", 0.0045, MAT["green_dim"], g)
    # lower data block
    _screen_text(cx, cz, y + 0.0015, size, 0.50, -0.74, "TQ 38%", 0.0055, MAT["green"], g)
    _screen_text(cx, cz, y + 0.0015, size, -0.12, -0.74, "TQ 40%", 0.0055, MAT["green"], g)
    _screen_text(cx, cz, y + 0.0015, size, 0.50, -0.86, "N1 92  TGT 620", 0.0045, MAT["green_dim"], g)
    _screen_text(cx, cz, y + 0.0015, size, -0.12, -0.86, "N1 93  TGT 630", 0.0045, MAT["green_dim"], g)
    # right-hand status column
    for i, label in enumerate(("APU", "FUEL", "HYD", "OIL", "TRQ")):
        _screen_text(cx, cz, y + 0.0015, size, -0.66, 0.62 - i * 0.20, label, 0.0043, MAT["green_dim"], g)


def build_tsd_page(cx, cz, y, size):
    g = "MPD_CONTENT_R"
    # border
    for u1, v1, u2, v2 in ((-0.95, -0.95, 0.95, -0.95), (0.95, -0.95, 0.95, 0.95), (0.95, 0.95, -0.95, 0.95), (-0.95, 0.95, -0.95, -0.95)):
        _screen_line(cx, cz, y + 0.001, size, u1, v1, u2, v2, w=0.0015, group=g)
    # heading tape
    _screen_line(cx, cz, y + 0.001, size, -0.82, 0.90, 0.82, 0.90, w=0.0015, group=g)
    for i in range(-8, 9):
        u = i * 0.10
        h = 0.030 if i % 2 == 0 else 0.016
        _screen_line(cx, cz, y + 0.001, size, u, 0.90, u, 0.90 - h, w=0.0012, group=g)
    for u, label in ((-0.70, "12"), (-0.42, "15"), (-0.14, "18"), (0.14, "21"), (0.42, "24"), (0.70, "27")):
        _screen_text(cx, cz, y + 0.0015, size, u, 0.80, label, 0.0045, MAT["green_dim"], g)
    # range rings and map grid
    for r in (0.24, 0.44, 0.64):
        torus("tsd_ring", _spt(cx, cz, y + 0.001, size, 0.0, -0.08), r * size * 0.5, 0.00065, MAT["green_dim"], g, axis="Y", major_segments=40, minor_segments=4)
    for i in range(-3, 4):
        _screen_line(cx, cz, y + 0.001, size, i * 0.22, -0.82, i * 0.22, 0.62, w=0.0007, group=g)
        _screen_line(cx, cz, y + 0.001, size, -0.82, -0.08 + i * 0.22, 0.82, -0.08 + i * 0.22, w=0.0007, group=g)
    # route
    route = [(-0.72, -0.62), (-0.48, -0.38), (-0.22, -0.16), (0.05, 0.02), (0.30, 0.30), (0.55, 0.56)]
    for i in range(len(route) - 1):
        _screen_line(cx, cz, y + 0.001, size, *route[i], *route[i + 1], w=0.0022, mat=MAT["green"], group=g)
    # ownship
    _screen_line(cx, cz, y + 0.001, size, -0.04, -0.08, 0.04, -0.08, w=0.0025, mat=MAT["green"], group=g)
    _screen_line(cx, cz, y + 0.001, size, 0.0, -0.12, 0.0, -0.04, w=0.0025, mat=MAT["green"], group=g)
    for u, v in route[::2]:
        _screen_line(cx, cz, y + 0.001, size, u - 0.018, v + 0.018, u + 0.018, v + 0.018, w=0.0016, mat=MAT["green"], group=g)
        _screen_line(cx, cz, y + 0.001, size, u + 0.018, v + 0.018, u + 0.018, v - 0.018, w=0.0016, mat=MAT["green"], group=g)
        _screen_line(cx, cz, y + 0.001, size, u + 0.018, v - 0.018, u - 0.018, v - 0.018, w=0.0016, mat=MAT["green"], group=g)
        _screen_line(cx, cz, y + 0.001, size, u - 0.018, v - 0.018, u - 0.018, v + 0.018, w=0.0016, mat=MAT["green"], group=g)
    _screen_text(cx, cz, y + 0.0015, size, -0.64, -0.70, "W01", 0.0042, MAT["green"], g)
    _screen_text(cx, cz, y + 0.0015, size, 0.36, 0.40, "W02", 0.0042, MAT["green"], g)
    # data blocks
    _screen_text(cx, cz, y + 0.0015, size, -0.76, 0.60, "TSD", 0.0058, MAT["green"], g)
    _screen_text(cx, cz, y + 0.0015, size, -0.76, 0.48, "10", 0.0050, MAT["green_dim"], g)
    _screen_text(cx, cz, y + 0.0015, size, -0.76, 0.36, "5", 0.0050, MAT["green_dim"], g)
    _screen_text(cx, cz, y + 0.0015, size, -0.76, 0.24, "ATT", 0.0042, MAT["green_dim"], g)
    _screen_text(cx, cz, y + 0.0015, size, 0.74, 0.60, "FCR", 0.0042, MAT["green_dim"], g)
    _screen_text(cx, cz, y + 0.0015, size, 0.74, 0.48, "ASE", 0.0042, MAT["green_dim"], g)
    _screen_text(cx, cz, y + 0.0015, size, -0.10, -0.86, "W01 91.6KM  89  0133", 0.0040, MAT["green"], g)


def build_eufd():
    g = "PANEL_HARDWARE"
    cx, cz = 0.0, 0.062
    y = PANEL_Y
    cube("eufd_body", (cx, y + 0.012, cz), (0.410, 0.020, 0.105), MAT["panel_deep"], g, bevel=0.004)
    cube("eufd_screen", (cx, y + 0.027, cz), (0.370, 0.004, 0.072), MAT["screen"], "EUFD_SCREEN", bevel=0.001)
    for dx in (-0.205, 0.205):
        bar("eufd_rail", (cx + dx, y + 0.030, cz - 0.052), (cx + dx, y + 0.030, cz + 0.052), 0.014, MAT["panel_alt"], g, depth=0.012, bevel=0.002)
    # side knobs / buttons
    for i, label in enumerate(("BRT", "DIM")):
        knob("eufd_knob", (0.222, y + 0.035, cz + 0.028 - i * 0.036), 0.009, 0.012, MAT["dark_metal"], g)
        text("eufd_knob_label", label, (0.222, y + 0.043, cz + 0.050 - i * 0.036), 0.0040, MAT["label_dim"], "LABELS")
    for i in range(5):
        cube("eufd_key", (-0.150 + i * 0.075, y + 0.030, cz - 0.064), (0.050, 0.010, 0.012), MAT["panel_alt"], g, bevel=0.0015)
    # content
    cg = "EUFD_CONTENT"
    lines = [
        ("VHF 121.500  UHF 251.000", 0.022, MAT["green"]),
        ("FM  30.000   IFF 0000", -0.001, MAT["green"]),
        ("WCA OFF   ICS 1", -0.024, MAT["green"]),
    ]
    for label, dz, mat in lines:
        text("eufd_text", label, (cx, y + 0.031, cz + dz), 0.0060, mat, cg)
    # a thin green frame and extra data
    for zz in (-0.032, 0.032):
        bar("eufd_border", (-0.180, y + 0.031, cz + zz), (0.180, y + 0.031, cz + zz), 0.0015, MAT["green_dim"], cg, depth=0.0012, bevel=0.0)
    for xx in (-0.180, 0.180):
        bar("eufd_border", (xx, y + 0.031, cz - 0.032), (xx, y + 0.031, cz + 0.032), 0.0015, MAT["green_dim"], cg, depth=0.0012, bevel=0.0)
    # four status blocks on the right edge
    for i in range(4):
        cube("eufd_status", (0.162, y + 0.031, cz + 0.018 - i * 0.012), (0.010, 0.0015, 0.005), MAT["green_dim"], cg, bevel=0.0002)
    screw((cx - 0.195, y + 0.030, cz + 0.048), (0, 1, 0), radius=0.0024)
    screw((cx + 0.195, y + 0.030, cz - 0.048), (0, 1, 0), radius=0.0024)
    text("eufd_title", "EUFD", (0.0, y + 0.031, cz + 0.046), 0.0046, MAT["label_dim"], "LABELS")


def build_caution_panel():
    g = "PANEL_HARDWARE"
    cx, cz = 0.0, 0.155
    y = PANEL_Y
    cube("caution_body", (cx, y + 0.012, cz), (0.330, 0.020, 0.052), MAT["panel_deep"], g, bevel=0.004)
    cube("caution_screen", (cx, y + 0.027, cz), (0.300, 0.004, 0.032), MAT["screen"], "CAUTION_SCREEN", bevel=0.0008)
    cg = "CAUTION_CONTENT"
    for i, (label, mat) in enumerate(
        (
            ("MASTER CAUTION", MAT["amber"]),
            ("ENG 1 OIL   HYD 1", MAT["green_dim"]),
            ("FUEL 1   APU", MAT["green_dim"]),
        )
    ):
        text("caution_text", label, (cx - 0.06 + i * 0.12, y + 0.031, cz + 0.004), 0.0044, mat, cg)
    for i in range(7):
        cube("caution_lamp", (-0.130 + i * 0.043, y + 0.032, cz - 0.018), (0.006, 0.0015, 0.004), MAT["amber"] if i in (1, 4) else MAT["green_dim"], cg, bevel=0.0002)


def gauge_disc(name, center, radius, mat, group, y, segments=40):
    """Disc in the XZ plane, facing +Y, at a given y."""
    verts = [(center[0], y, center[1])]
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        verts.append((center[0] + radius * math.cos(a), y, center[1] + radius * math.sin(a)))
    faces = [(0, i + 1, (i + 1) % segments + 1) for i in range(segments)]
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    _set_material(obj, mat)
    return _register(obj, group)


def half_disc(name, center, radius, y, upper, mat, group, segments=32):
    verts = [(center[0], y, center[1])]
    if upper:
        angles = [math.pi * i / segments for i in range(segments + 1)]
    else:
        angles = [math.pi + math.pi * i / segments for i in range(segments + 1)]
    for a in angles:
        verts.append((center[0] + radius * math.cos(a), y, center[1] + radius * math.sin(a)))
    faces = [(0, i + 1, i + 2) for i in range(segments)]
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    _set_material(obj, mat)
    return _register(obj, group)


def gauge(name, cx, cz, radius, y, kind):
    g = "STANDBY_INSTRUMENTS"
    # can and bezel
    cylinder(name + "_can", (cx, y - 0.018, cz), radius * 1.08, 0.034, MAT["panel_deep"], g, axis="Y", vertices=32)
    torus(name + "_bezel", (cx, y + 0.001, cz), radius * 1.04, 0.0042, MAT["dark_metal"], g, axis="Y", major_segments=40, minor_segments=8)
    # face
    if kind == "att":
        gauge_disc(name + "_face", (cx, cz), radius, MAT["att_ground"], g, y + 0.002)
        half_disc(name + "_sky", (cx, cz), radius * 0.99, y + 0.0035, True, MAT["att_sky"], g)
        bar(name + "_horizon", (cx - radius * 0.90, y + 0.0045, cz), (cx + radius * 0.90, y + 0.0045, cz), 0.0032, MAT["white"], g, depth=0.0012, bevel=0.0)
        # pitch ladder
        for i in (-2, -1, 1, 2):
            dz = i * radius * 0.20
            half = radius * (0.30 if i % 2 else 0.22)
            bar(name + "_pitch", (cx - half, y + 0.0045, cz + dz), (cx + half, y + 0.0045, cz + dz), 0.0022, MAT["white"], g, depth=0.0010, bevel=0.0)
        # bank pointer
        bar(name + "_bank", (cx, y + 0.005, cz + radius * 0.86), (cx, y + 0.005, cz + radius * 0.99), 0.0025, MAT["white"], g, depth=0.0010, bevel=0.0)
        # miniature aircraft
        cx2 = cx
        for dx, dz, w, h in ((0.0, 0.0, 0.012, 0.002), (0.0, -0.007, 0.002, 0.014)):
            cube(name + "_ac", (cx2 + dx, y + 0.006, cz + dz), (w, 0.001, h), MAT["yellow"], g, bevel=0.0002)
        text(name + "_label", "CLI  MB", (cx, y + 0.006, cz - radius * 0.55), radius * 0.16, MAT["label_dim"], "LABELS")
    elif kind == "asi":
        gauge_disc(name + "_face", (cx, cz), radius, MAT["screen_dark"], g, y + 0.002)
        for i in range(25):
            a = math.radians(-220 + i * (260 / 24))
            r1 = radius * 0.86
            r2 = radius * (0.96 if i % 5 == 0 else 0.91)
            p1 = (cx + r1 * math.cos(a), y + 0.0045, cz + r1 * math.sin(a))
            p2 = (cx + r2 * math.cos(a), y + 0.0045, cz + r2 * math.sin(a))
            bar(name + "_tick", p1, p2, 0.0018, MAT["white"], g, depth=0.0010, bevel=0.0)
        for i, label in enumerate(("0", "50", "100", "150", "200", "250")):
            a = math.radians(-220 + i * (260 / 5))
            r = radius * 0.70
            text(name + "_num", label, (cx + r * math.cos(a), y + 0.0055, cz + r * math.sin(a)), radius * 0.18, MAT["white"], "LABELS")
        needle_angle = math.radians(-220 + (89 / 250) * 260)
        p2 = (cx + radius * 0.76 * math.cos(needle_angle), y + 0.006, cz + radius * 0.76 * math.sin(needle_angle))
        bar(name + "_needle", (cx, y + 0.006, cz), p2, 0.0022, MAT["white"], g, depth=0.0012, bevel=0.0)
        cylinder(name + "_hub", (cx, y + 0.006, cz), radius * 0.08, 0.004, MAT["dark_metal"], g, axis="Y", vertices=12)
        text(name + "_unit", "KNOTS", (cx, y + 0.0055, cz - radius * 0.55), radius * 0.16, MAT["label_dim"], "LABELS")
    else:  # altimeter
        gauge_disc(name + "_face", (cx, cz), radius, MAT["screen_dark"], g, y + 0.002)
        for i in range(10):
            a = math.radians(-90 + i * 36)
            r = radius * 0.78
            text(name + "_num", str(i), (cx + r * math.cos(a), y + 0.0055, cz + r * math.sin(a)), radius * 0.20, MAT["white"], "LABELS")
        for i in range(50):
            a = math.radians(-90 + i * 7.2)
            r1, r2 = radius * 0.88, radius * 0.95
            bar(
                name + "_tick",
                (cx + r1 * math.cos(a), y + 0.0045, cz + r1 * math.sin(a)),
                (cx + r2 * math.cos(a), y + 0.0045, cz + r2 * math.sin(a)),
                0.0014,
                MAT["white"],
                g,
                depth=0.0010,
                bevel=0.0,
            )
        # long and short needles
        for length, angle, width in ((0.72, 150, 0.0022), (0.50, 30, 0.0030)):
            a = math.radians(angle)
            bar(
                name + "_needle",
                (cx, y + 0.006, cz),
                (cx + radius * length * math.cos(a), y + 0.006, cz + radius * length * math.sin(a)),
                width,
                MAT["white"],
                g,
                depth=0.0012,
                bevel=0.0,
            )
        cylinder(name + "_hub", (cx, y + 0.006, cz), radius * 0.07, 0.004, MAT["dark_metal"], g, axis="Y", vertices=12)
        text(name + "_unit", "ALT", (cx, y + 0.0055, cz - radius * 0.55), radius * 0.16, MAT["label_dim"], "LABELS")
    # black mask ring + glass
    if kind == "att":
        torus(name + "_mask", (cx, y + 0.0045, cz), radius * 1.01, 0.006, MAT["black"], g, axis="Y", major_segments=36, minor_segments=6)
    cylinder(name + "_glass", (cx, y + 0.009, cz), radius * 0.98, 0.0015, MAT["instrument_glass"], g, axis="Y", vertices=32, bevel=0.0)
    # mount screws
    for sx in (-1, 1):
        for sz in (-1, 1):
            screw((cx + sx * radius * 1.00, y + 0.003, cz + sz * radius * 1.00), (0, 1, 0), radius=0.0022)


def build_standby():
    g = "STANDBY_INSTRUMENTS"
    # backing plate
    cube("standby_plate", (-0.300, PANEL_Y + 0.010, -0.205), (0.135, 0.018, 0.370), MAT["panel"], g, bevel=0.005)
    gauge("attitude", -0.300, -0.105, 0.040, PANEL_Y + 0.020, "att")
    gauge("airspeed", -0.338, -0.340, 0.038, PANEL_Y + 0.020, "asi")
    gauge("altimeter", -0.258, -0.340, 0.038, PANEL_Y + 0.020, "alt")
    # labels
    text("standby_label", "STANDBY INSTRUMENTS", (-0.300, PANEL_Y + 0.034, -0.398), 0.0046, MAT["label_dim"], "LABELS")
    # spare compass and HDU alignment visor on the glareshield
    cylinder("compass_body", (0.020, -0.930, 0.285), 0.035, 0.030, MAT["panel_deep"], "PANEL_HARDWARE", axis="Y", vertices=24)
    torus("compass_ring", (0.020, -0.913, 0.285), 0.034, 0.0032, MAT["dark_metal"], "PANEL_HARDWARE", axis="Y", major_segments=32, minor_segments=6)
    gauge_disc("compass_face", (0.020, 0.285), 0.031, MAT["screen_dark"], "PANEL_HARDWARE", -0.912)
    for i, label in enumerate(("N", "E", "S", "W")):
        a = math.radians(90 - i * 90)
        text("compass_letter", label, (0.020 + 0.021 * math.cos(a), -0.910, 0.285 + 0.021 * math.sin(a)), 0.0055, MAT["white"], "LABELS")
    bar("compass_needle", (0.014, -0.909, 0.285), (0.026, -0.909, 0.285), 0.0020, MAT["red"], "PANEL_HARDWARE", depth=0.001, bevel=0.0)
    text("compass_label", "COMPASS", (0.020, -0.920, 0.244), 0.0040, MAT["label_dim"], "LABELS")
    # helmet-sight alignment visor
    torus("hdu_visor_ring", (-0.060, -0.935, 0.292), 0.023, 0.0035, MAT["dark_metal"], "PANEL_HARDWARE", axis="Y", major_segments=28, minor_segments=6)
    cylinder("hdu_visor_glass", (-0.060, -0.934, 0.292), 0.019, 0.0015, MAT["instrument_glass"], "PANEL_HARDWARE", axis="Y", vertices=24, bevel=0.0)
    text("hdu_visor_label", "HDU ALIGN", (-0.060, -0.918, 0.258), 0.0038, MAT["label_dim"], "LABELS")


def build_keyboard():
    g = "CONSOLE_LEFT"
    origin = Vector((0.405, -0.535, -0.315))
    rot = Euler((math.radians(-38.0), 0.0, math.radians(-12.0))).to_matrix()
    size = (0.190, 0.165, 0.022)
    cube("kdu_base", origin, size, MAT["panel"], g, rot=rot.to_euler(), bevel=0.004)
    # display strip
    cube(
        "kdu_display",
        origin + rot @ Vector((0.0, 0.055, 0.0125)),
        (0.140, 0.028, 0.003),
        MAT["screen_dark"],
        g,
        rot=rot.to_euler(),
        bevel=0.0008,
    )
    # 5 rows x 6 columns of keys
    for r in range(5):
        for c in range(6):
            local = Vector((-0.072 + c * 0.029, 0.030 - r * 0.023, 0.014))
            cube(
                "kdu_key",
                origin + rot @ local,
                (0.022, 0.017, 0.006),
                MAT["panel_alt"],
                g,
                rot=rot.to_euler(),
                bevel=0.0015,
            )
            cube(
                "kdu_key_face",
                origin + rot @ (local + Vector((0, 0, 0.0035))),
                (0.016, 0.011, 0.0015),
                MAT["panel_deep"],
                g,
                rot=rot.to_euler(),
                bevel=0.0003,
            )
            bar(
                "kdu_key_legend",
                origin + rot @ (local + Vector((-0.0045, 0.0, 0.0045))),
                origin + rot @ (local + Vector((0.0045, 0.0, 0.0045))),
                0.0012,
                MAT["label_dim"],
                g,
                depth=0.0010,
                bevel=0.0,
            )
    text("kdu_title", "KDU", origin + rot @ Vector((0.0, -0.075, 0.014)), 0.0050, MAT["label"], "LABELS", normal=rot @ Vector((0, 0, 1)), up=rot @ Vector((0, 1, 0)))
    # small green text in the display
    text("kdu_line", "W01 91.6", origin + rot @ Vector((0.0, 0.055, 0.0155)), 0.0042, MAT["green"], "CONSOLE_CONTENT", normal=rot @ Vector((0, 0, 1)), up=rot @ Vector((0, 1, 0)))


def build_consoles():
    for side, sx in (("L", 1.0), ("R", -1.0)):
        g = "CONSOLE_" + side
        # sloped top plate
        cube(
            "console_top",
            (sx * 0.402, -0.430, -0.365),
            (0.245, 1.020, 0.030),
            MAT["panel_alt"],
            g,
            bevel=0.005,
            rot=(0.0, math.radians(sx * -5.0), 0.0),
        )
        # inner rail
        bar(
            "console_rail",
            (sx * 0.286, -0.930, -0.340),
            (sx * 0.286, 0.070, -0.340),
            0.022,
            MAT["dark_metal"],
            g,
            depth=0.030,
            bevel=0.003,
        )
        # rows of small fasteners along the rail
        for i in range(9):
            screw((sx * 0.286, -0.90 + i * 0.115, -0.323), (0, 0, 1), radius=0.0020)

    # rear console tops: breaker panels and stowage (simplified, but keeps the
    # broad console surfaces from reading as empty slabs when you look sideways)
    for side, sx in (("L", 1.0), ("R", -1.0)):
        g = "CONSOLE_" + side
        top_plate("rear_cb_panel", (sx * 0.405, 0.005, -0.342), (0.130, 0.120), g, mat=MAT["panel"])
        for r in range(3):
            for c in range(4):
                cylinder(
                    "rear_cb",
                    (sx * 0.405 - 0.045 + c * 0.030, 0.005 - 0.035 + r * 0.035, -0.330),
                    0.006,
                    0.010,
                    MAT["dark_metal"],
                    g,
                    axis="Z",
                    vertices=10,
                )
                cylinder(
                    "rear_cb_cap",
                    (sx * 0.405 - 0.045 + c * 0.030, 0.005 - 0.035 + r * 0.035, -0.323),
                    0.0035,
                    0.003,
                    MAT["white"],
                    g,
                    axis="Z",
                    vertices=10,
                )
        text(
            "rear_cb_label",
            "CB",
            (sx * 0.405, 0.070, -0.330),
            0.0040,
            MAT["label_dim"],
            "LABELS",
            normal=(0, 0, 1),
            up=(0, -1, 0),
        )
        cube(
            "rear_stow",
            (sx * 0.415, 0.095, -0.342),
            (0.120, 0.145, 0.030),
            MAT["panel_alt"],
            g,
            bevel=0.005,
        )
        cube(
            "rear_stow_latch",
            (sx * 0.415, 0.025, -0.323),
            (0.040, 0.020, 0.010),
            MAT["dark_metal"],
            g,
            bevel=0.002,
        )

    # ---- left console: lighting, engine/APU, emergency, power levers ----
    g = "CONSOLE_L"
    top_plate("lighting_panel", (0.405, -0.825, -0.342), (0.150, 0.180), g, mat=MAT["panel"])
    text("lighting_title", "LIGHTING", (0.405, -0.832, -0.238), 0.0048, MAT["label"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))
    for i in range(3):
        knob("light_knob", (0.370 + i * 0.038, -0.780, -0.330), 0.010, 0.012, MAT["dark_metal"], g, axis="Z")
    for i in range(3):
        toggle("light_sw", (0.370 + i * 0.038, -0.860, -0.330), 0.0055, MAT["metal"], g, angle=0.4, normal=(0, 0, 1))
        text("light_label", ("EXT", "INT", "INST")[i], (0.370 + i * 0.038, -0.860, -0.318), 0.0038, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))

    top_plate("engine_panel", (0.405, -0.640, -0.342), (0.150, 0.200), g, mat=MAT["panel"])
    text("engine_title", "ENGINE / APU", (0.405, -0.648, -0.530), 0.0046, MAT["label"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))
    for i, label in enumerate(("ENG 1", "APU", "ENG 2")):
        guarded_switch("eng_sw", (0.350 + i * 0.055, -0.610, -0.320), 0.0075, g, normal=(0, 0, 1))
        text("eng_sw_label", label, (0.350 + i * 0.055, -0.610, -0.298), 0.0040, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))
    for i, label in enumerate(("RUN", "START", "IGN")):
        toggle("eng_toggle", (0.350 + i * 0.055, -0.680, -0.320), 0.0060, MAT["metal"], g, angle=0.4, normal=(0, 0, 1))
        text("eng_toggle_label", label, (0.350 + i * 0.055, -0.680, -0.298), 0.0038, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))

    top_plate("emergency_panel", (0.405, -0.430, -0.342), (0.150, 0.140), g, mat=MAT["panel"])
    text("emergency_title", "EMERGENCY", (0.405, -0.438, -0.478), 0.0046, MAT["label_red"], "LABELS_RED", normal=(0, 0, 1), up=(0, -1, 0))
    for i, label in enumerate(("HYD", "FUEL", "BATT")):
        toggle("emerg_sw", (0.355 + i * 0.050, -0.430, -0.320), 0.0065, MAT["metal"], g, angle=0.5, normal=(0, 0, 1))
        text("emerg_label", label, (0.355 + i * 0.050, -0.430, -0.300), 0.0038, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))

    # power lever quadrant
    cube("power_quadrant", (0.400, -0.255, -0.330), (0.120, 0.115, 0.030), MAT["panel"], g, bevel=0.004)
    for i, dx in enumerate((-0.025, 0.025)):
        bar("power_slot", (0.400 + dx, -0.300, -0.318), (0.400 + dx, -0.205, -0.318), 0.012, MAT["black"], g, depth=0.008, bevel=0.001)
        bar("power_lever", (0.400 + dx, -0.270, -0.310), (0.400 + dx, -0.235, -0.250), 0.018, MAT["dark_metal"], g, depth=0.015, bevel=0.003)
        cube("power_grip", (0.400 + dx, -0.230, -0.238), (0.035, 0.040, 0.030), MAT["rubber"], g, bevel=0.006)
        text("power_label", ("FLY", "IDLE")[i], (0.400 + dx, -0.300, -0.292), 0.0036, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))

    # stores jettison
    top_plate("jettison_panel", (0.405, -0.135, -0.342), (0.150, 0.100), g, mat=MAT["panel"])
    text("jettison_title", "STORES JETTISON", (0.405, -0.142, -0.105), 0.0040, MAT["label_yellow"], "LABELS_WARN", normal=(0, 0, 1), up=(0, -1, 0))
    guarded_switch("jettison_sw", (0.405, -0.135, -0.320), 0.008, g, normal=(0, 0, 1))

    # ---- right console: radios, processor, HDU stowage ----
    g = "CONSOLE_R"
    top_plate("radio_panel", (-0.405, -0.735, -0.342), (0.170, 0.260), g, mat=MAT["panel"])
    text("radio_title", "RADIO / ICS", (-0.405, -0.742, -0.578), 0.0048, MAT["label"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))
    for r in range(2):
        for c in range(5):
            knob("radio_knob", (-0.465 + c * 0.030, -0.790 + r * 0.075, -0.330), 0.0095, 0.012, MAT["dark_metal"], g, axis="Z")
            if r == 0:
                text("radio_label", ("VHF", "UHF", "FM", "IFF", "ICS")[c], (-0.465 + c * 0.030, -0.825, -0.318), 0.0036, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))
    # radio display
    cube("radio_display", (-0.405, -0.610, -0.334), (0.140, 0.022, 0.004), MAT["screen_dark"], g, bevel=0.001)
    text("radio_line", "121.500  251.000", (-0.405, -0.610, -0.330), 0.0040, MAT["green"], "CONSOLE_CONTENT", normal=(0, 0, 1), up=(0, -1, 0))

    top_plate("processor_panel", (-0.405, -0.370, -0.342), (0.170, 0.180), g, mat=MAT["panel"])
    text("processor_title", "SYSTEM PROCESSOR", (-0.405, -0.378, -0.278), 0.0042, MAT["label"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))
    for i in range(4):
        toggle("proc_sw", (-0.460 + i * 0.037, -0.360, -0.322), 0.0055, MAT["metal"], g, angle=0.35, normal=(0, 0, 1))
        text("proc_label", ("A", "B", "C", "D")[i], (-0.460 + i * 0.037, -0.360, -0.300), 0.0036, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))

    # HDU stowage compartment
    cube("hdu_box", (-0.410, -0.160, -0.330), (0.150, 0.190, 0.070), MAT["panel"], g, bevel=0.008)
    cube("hdu_lid", (-0.410, -0.160, -0.294), (0.140, 0.180, 0.012), MAT["panel_alt"], g, bevel=0.004)
    cube("hdu_latch", (-0.410, -0.245, -0.288), (0.050, 0.020, 0.010), MAT["dark_metal"], g, bevel=0.002)
    text("hdu_box_label", "HDU", (-0.410, -0.160, -0.283), 0.0070, MAT["label"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))
    # stowed monocle tube clipped into the lid
    cylinder("hdu_monocle", (-0.445, -0.140, -0.272), 0.011, 0.075, MAT["dark_metal"], g, axis="Y", vertices=16)
    torus("hdu_monocle_ring", (-0.445, -0.100, -0.272), 0.011, 0.0025, MAT["metal"], g, axis="Y", major_segments=20, minor_segments=5)
    text("hdu_stow_label", "IHADSS MONOCLE", (-0.410, -0.160, -0.365), 0.0034, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))

    # circuit breaker side panels (simplified)
    for side, sx in (("L", 1.0), ("R", -1.0)):
        g2 = "CONSOLE_" + side
        cube("cb_panel", (sx * 0.492, 0.100, -0.430), (0.012, 0.260, 0.230), MAT["panel"], g2, bevel=0.004)
        for r in range(5):
            for c in range(3):
                cylinder("cb", (sx * 0.480, 0.010 + c * 0.060, -0.520 + r * 0.040), 0.006, 0.010, MAT["dark_metal"], g2, axis="X", vertices=10)
                cylinder("cb_cap", (sx * 0.472, 0.010 + c * 0.060, -0.520 + r * 0.040), 0.0035, 0.003, MAT["white"], g2, axis="X", vertices=10)


def build_center_pedestal():
    g = "CENTER_PEDESTAL"
    cube("pedestal", (0.0, -0.230, -0.640), (0.190, 0.620, 0.400), MAT["panel"], g, bevel=0.008)
    cube("pedestal_top", (0.0, -0.230, -0.432), (0.200, 0.620, 0.025), MAT["panel_alt"], g, bevel=0.005)
    cube("pedestal_face", (0.0, -0.545, -0.550), (0.160, 0.030, 0.160), MAT["panel_deep"], g, bevel=0.004)
    # radio / ICS control head on the pedestal
    top_plate("pedestal_radio", (0.0, -0.365, -0.414), (0.150, 0.150), g, mat=MAT["panel"])
    for r in range(2):
        for c in range(4):
            knob("ped_knob", (-0.048 + c * 0.032, -0.400 + r * 0.070, -0.400), 0.0080, 0.010, MAT["dark_metal"], g, axis="Z")
    cube("ped_display", (0.0, -0.330, -0.401), (0.110, 0.020, 0.004), MAT["screen_dark"], g, bevel=0.001)
    text("ped_line", "ICS 1  VHF", (0.0, -0.330, -0.397), 0.0038, MAT["green"], "CONSOLE_CONTENT", normal=(0, 0, 1), up=(0, -1, 0))
    # a stowage box and a map case
    cube("ped_case", (0.0, -0.080, -0.408), (0.140, 0.120, 0.030), MAT["panel"], g, bevel=0.006)
    cube("ped_case_lid", (0.0, -0.080, -0.390), (0.130, 0.110, 0.010), MAT["panel_alt"], g, bevel=0.003)
    text("ped_case_label", "MAP CASE", (0.0, -0.080, -0.384), 0.0040, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))
    # trim / force trim switch on the pedestal
    toggle("trim_sw", (0.060, -0.475, -0.394), 0.0055, MAT["metal"], g, angle=0.35, normal=(0, 0, 1))
    text("trim_label", "TRIM", (0.060, -0.475, -0.376), 0.0036, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))


def build_controls():
    # ---- cyclic ----
    g = "CYCLIC"
    # curved boot/stick: sample a quadratic curve into cylinders
    p0 = Vector((0.0, -0.660, -0.965))
    p1 = Vector((0.0, -0.560, -0.690))
    p2 = Vector((0.0, -0.485, -0.500))
    pts = []
    for i in range(13):
        t = i / 12.0
        p = (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t * t * p2
        pts.append(p)
    for i in range(len(pts) - 1):
        cylinder(
            "cyclic_tube",
            (pts[i] + pts[i + 1]) * 0.5,
            0.017,
            (pts[i + 1] - pts[i]).length + 0.004,
            MAT["rubber"],
            g,
            vertices=12,
            direction=(pts[i + 1] - pts[i]),
        )
    # boot at the floor
    cylinder("cyclic_boot", (0.0, -0.665, -0.955), 0.060, 0.040, MAT["rubber"], g, axis="Z", vertices=20)
    # grip
    grip_c = Vector((0.0, -0.475, -0.435))
    grip_rot = Euler((math.radians(-16.0), 0.0, 0.0))
    cube("cyclic_grip", grip_c, (0.056, 0.052, 0.155), MAT["rubber"], g, rot=grip_rot, bevel=0.014)
    cube("cyclic_grip_back", grip_c + Vector((0.0, 0.028, -0.005)), (0.050, 0.018, 0.130), MAT["rubber_grey"], g, rot=grip_rot, bevel=0.006)
    # top cap
    cylinder("cyclic_top", grip_c + Vector((0.0, -0.004, 0.080)), 0.027, 0.014, MAT["dark_metal"], g, axis="Z", vertices=24)
    # four-way weapon/symbology/trim hat
    for dx, dy in ((0.0, -0.012), (0.0, 0.012), (-0.012, 0.0), (0.012, 0.0)):
        cube("cyclic_hat", grip_c + Vector((dx, dy, 0.090)), (0.010, 0.010, 0.010), MAT["black"], g, bevel=0.002)
    # thumb buttons
    for i, (dx, dz) in enumerate(((-0.030, 0.030), (-0.030, 0.000), (0.030, 0.030), (0.030, 0.000))):
        cylinder("cyclic_button", grip_c + Vector((dx, -0.020, dz)), 0.0075, 0.010, MAT["dark_metal"], g, axis="X", vertices=12)
    # trigger guard and trigger (relative to the grip, not the eye!)
    guard = [
        grip_c + Vector((-0.026, -0.040, -0.060)),
        grip_c + Vector((-0.026, -0.075, -0.060)),
        grip_c + Vector((0.026, -0.075, -0.060)),
        grip_c + Vector((0.026, -0.040, -0.060)),
    ]
    for i in range(3):
        bar("trigger_guard", guard[i], guard[i + 1], 0.008, MAT["dark_metal"], g, depth=0.008, bevel=0.0015)
    bar(
        "trigger",
        grip_c + Vector((-0.014, -0.055, -0.060)),
        grip_c + Vector((0.014, -0.055, -0.060)),
        0.010,
        MAT["red_paint"],
        g,
        depth=0.012,
        bevel=0.002,
    )
    # force trim / radio switches on the grip face
    for dz, label in ((0.050, "FT"), (0.020, "RTS")):
        cube("cyclic_face_sw", grip_c + Vector((0.0, -0.030, dz)), (0.020, 0.008, 0.010), MAT["panel_alt"], g, bevel=0.0015)
    # a couple of labels
    text("cyclic_label", "WEAPON", grip_c + Vector((0.0, -0.034, -0.035)), 0.0034, MAT["label_dim"], "LABELS")

    # ---- collective ----
    g = "COLLECTIVE"
    base = Vector((0.450, -0.520, -0.470))
    elbow = Vector((0.380, -0.430, -0.405))
    grip = Vector((0.315, -0.360, -0.390))
    bar("collective_arm", base, elbow, 0.030, MAT["rubber"], g, depth=0.030, bevel=0.005)
    bar("collective_arm2", elbow, grip, 0.030, MAT["rubber"], g, depth=0.030, bevel=0.005)
    cylinder("collective_pivot", base, 0.035, 0.110, MAT["dark_metal"], g, axis="Y", vertices=20)
    # twist grip and head
    cylinder("collective_grip", grip, 0.030, 0.150, MAT["rubber"], g, axis="Y", vertices=20)
    cylinder("collective_twist", grip + Vector((0.0, 0.070, 0.0)), 0.034, 0.040, MAT["dark_metal"], g, axis="Y", vertices=20)
    cube("collective_head", grip + Vector((0.0, 0.010, 0.042)), (0.060, 0.100, 0.035), MAT["rubber_grey"], g, bevel=0.008)
    for i, dx in enumerate((-0.020, 0.020)):
        cylinder("collective_button", grip + Vector((dx, 0.030, 0.058)), 0.008, 0.012, MAT["dark_metal"], g, axis="Z", vertices=12)
    toggle("collective_sw", grip + Vector((0.0, -0.040, 0.052)), 0.006, MAT["metal"], g, angle=0.3, normal=(0, 0, 1))
    text("collective_label", "COLLECTIVE", grip + Vector((0.0, 0.0, -0.045)), 0.0036, MAT["label_dim"], "LABELS", normal=(0, 0, 1), up=(0, -1, 0))

    # ---- pedals ----
    g = "PEDALS"
    for sx in (-1.0, 1.0):
        # rudder arm
        bar("pedal_arm", (sx * 0.140, -0.560, -0.780), (sx * 0.140, -0.820, -0.930), 0.018, MAT["dark_metal"], g, depth=0.018, bevel=0.003)
        cube("pedal_plate", (sx * 0.140, -0.900, -0.955), (0.095, 0.120, 0.025), MAT["rubber_grey"], g, bevel=0.008, rot=(math.radians(18), 0.0, 0.0))
        for i in range(3):
            bar("pedal_rib", (sx * 0.105, -0.890 + i * 0.030, -0.940), (sx * 0.175, -0.890 + i * 0.030, -0.940), 0.012, MAT["black"], g, depth=0.012, bevel=0.002)
        cube("heel_rest", (sx * 0.140, -0.800, -1.010), (0.110, 0.100, 0.020), MAT["dark_metal"], g, bevel=0.004)


def build_seat():
    g = "SEAT"
    # armoured bucket, deliberately simplified
    cube("seat_back", (0.0, 0.330, -0.400), (0.470, 0.140, 0.720), MAT["seat"], g, bevel=0.018, rot=(math.radians(-8), 0, 0))
    cube("seat_back_pad", (0.0, 0.245, -0.390), (0.380, 0.080, 0.560), MAT["olive_dark"], g, bevel=0.020)
    cube("seat_headrest", (0.0, 0.300, 0.100), (0.250, 0.130, 0.170), MAT["seat"], g, bevel=0.022)
    cube("seat_pan", (0.0, 0.110, -0.810), (0.460, 0.460, 0.080), MAT["seat"], g, bevel=0.016)
    cube("seat_cushion", (0.0, 0.080, -0.762), (0.390, 0.400, 0.055), MAT["olive_dark"], g, bevel=0.018)
    # side armour
    for sx in (-1.0, 1.0):
        cube("seat_armor", (sx * 0.275, 0.240, -0.420), (0.045, 0.440, 0.700), MAT["frame"], g, bevel=0.010)
        cube("seat_bolster", (sx * 0.245, 0.110, -0.700), (0.055, 0.330, 0.110), MAT["olive_dark"], g, bevel=0.014)
    # harness straps and buckles
    for sx in (-1.0, 1.0):
        bar("harness", (sx * 0.150, 0.230, 0.030), (sx * 0.095, -0.010, -0.560), 0.045, MAT["canvas"], g, depth=0.010, bevel=0.003)
        cube("harness_buckle", (sx * 0.095, -0.020, -0.565), (0.045, 0.025, 0.055), MAT["metal"], g, bevel=0.004)
    # a red fire extinguisher on the left rear wall
    cylinder("extinguisher", (0.400, 0.300, -0.520), 0.045, 0.220, MAT["red_paint"], g, axis="Z", vertices=20)
    cylinder("extinguisher_top", (0.400, 0.300, -0.395), 0.020, 0.050, MAT["dark_metal"], g, axis="Z", vertices=12)
    # first aid kit on the rear bulkhead
    cube("first_aid", (-0.300, 0.440, -0.280), (0.120, 0.050, 0.090), MAT["olive"], g, bevel=0.006)
    text("first_aid_label", "FIRST AID", (-0.300, 0.410, -0.280), 0.0038, MAT["label"], "LABELS", normal=(0, -1, 0), up=(0, 0, 1))


def build_misc_details():
    g = "DETAILS"
    # footwell pedals/interior lights
    for sx in (-1.0, 1.0):
        cube("dome_light", (sx * 0.420, 0.020, 0.560), (0.080, 0.060, 0.020), MAT["panel_alt"], g, bevel=0.008)
        cube("dome_lens", (sx * 0.420, 0.020, 0.547), (0.060, 0.040, 0.006), MAT["red"], g, bevel=0.002)
    # canvas bag / helmet bag behind the seat
    cube("helmet_bag", (0.0, 0.380, -0.720), (0.300, 0.180, 0.160), MAT["canvas"], g, bevel=0.030)
    # first aid kit on the right side wall
    cube("side_first_aid", (-0.505, -0.340, -0.330), (0.035, 0.120, 0.090), MAT["olive"], g, bevel=0.006)
    cube("side_first_aid_cross_v", (-0.484, -0.340, -0.330), (0.004, 0.018, 0.060), MAT["white"], g, bevel=0.001)
    cube("side_first_aid_cross_h", (-0.484, -0.340, -0.330), (0.004, 0.060, 0.018), MAT["white"], g, bevel=0.001)
    text(
        "side_first_aid_label",
        "FIRST AID",
        (-0.484, -0.340, -0.395),
        0.0032,
        MAT["label"],
        "LABELS",
        normal=(1, 0, 0),
        up=(0, 0, 1),
    )

    # stowed IHADSS helmet on the rear shelf, with its monocle and cable
    sphere("ihadss_helmet", (0.255, 0.300, -0.805), 0.105, MAT["olive_dark"], g, segments=24, rings=14)
    cube("helmet_visor", (0.255, 0.215, -0.815), (0.155, 0.075, 0.100), MAT["black"], g, bevel=0.018)
    # boom mic and monocle
    bar("helmet_boom", (0.205, 0.195, -0.790), (0.150, 0.080, -0.790), 0.010, MAT["dark_metal"], g, depth=0.010, bevel=0.002)
    cylinder("helmet_monocle", (0.175, 0.070, -0.780), 0.014, 0.050, MAT["dark_metal"], g, axis="Y", vertices=16)
    torus("helmet_monocle_ring", (0.175, 0.040, -0.780), 0.015, 0.0030, MAT["metal"], g, axis="Y", major_segments=20, minor_segments=6)
    cylinder("helmet_monocle_glass", (0.175, 0.038, -0.780), 0.012, 0.002, MAT["instrument_glass"], g, axis="Y", vertices=16, bevel=0.0)
    # cable down to the right console
    for i in range(5):
        bar(
            "hdu_cable",
            (0.320, 0.220 - i * 0.055, -0.700 - i * 0.055),
            (0.320, 0.165 - i * 0.055, -0.755 - i * 0.055),
            0.008,
            MAT["rubber"],
            g,
            depth=0.008,
            bevel=0.001,
        )

    # canopy jettison placard on the sill
    text("jettison_placard", "CANOPY JETTISON", (0.440, -0.020, 0.310), 0.0038, MAT["label_yellow"], "LABELS_WARN", normal=(-1, 0, 0), up=(0, 0, 1))
    # emergency exit placard
    text("exit_placard", "EMERGENCY EXIT", (-0.440, 0.020, 0.310), 0.0038, MAT["label_red"], "LABELS_RED", normal=(1, 0, 0), up=(0, 0, 1))


# ---------------------------------------------------------------------------
# join / export
# ---------------------------------------------------------------------------


def join_groups():
    scene = bpy.context.scene
    COCKPIT_OBJECTS.clear()
    bpy.ops.object.select_all(action="DESELECT")
    for group_name in list(GROUPS.keys()):
        objs = [o for o in GROUPS[group_name] if o.name in bpy.data.objects]
        if not objs:
            continue
        bpy.ops.object.select_all(action="DESELECT")
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        if len(objs) > 1:
            bpy.ops.object.join()
        obj = bpy.context.object
        obj.name = group_name
        # Keep the origin at the eye for static groups; give the movable ones a
        # sensible pivot.
        pivots = {
            "CYCLIC": (0.0, -0.660, -0.965),
            "COLLECTIVE": (0.450, -0.520, -0.470),
            "PEDALS": (0.0, -0.700, -0.930),
            "MPD_CONTENT_L": (MPD_CX, PANEL_Y + 0.033, MPD_CZ),
            "MPD_CONTENT_R": (-MPD_CX, PANEL_Y + 0.033, MPD_CZ),
            "EUFD_CONTENT": (0.0, PANEL_Y + 0.033, 0.062),
            "CAUTION_CONTENT": (0.0, PANEL_Y + 0.033, 0.155),
        }
        scene.cursor.location = pivots.get(group_name, (0.0, 0.0, 0.0))
        bpy.ops.object.origin_set(type="ORIGIN_CURSOR")
        # Freeze rotation/scale for static meshes; animated groups keep their
        # own local space but no scale.
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
        COCKPIT_OBJECTS.append(obj)
        print(f"  joined {group_name:20s} {len(obj.data.polygons):6d} faces")


def add_anchors():
    anchors = {
        "EYE_PILOT": (0.0, 0.0, 0.0),
        "MPD_LEFT_ANCHOR": (MPD_CX, PANEL_Y + 0.040, MPD_CZ),
        "MPD_RIGHT_ANCHOR": (-MPD_CX, PANEL_Y + 0.040, MPD_CZ),
        "EUFD_ANCHOR": (0.0, PANEL_Y + 0.040, 0.062),
        "IHADSS_ANCHOR": (0.0, -0.500, 0.350),
    }
    for name, loc in anchors.items():
        bpy.ops.object.empty_add(type="PLAIN_AXES", radius=0.03, location=loc)
        obj = bpy.context.object
        obj.name = name
        obj.empty_display_size = 0.02
        obj["role"] = name
        COCKPIT_OBJECTS.append(obj)


def export_glb():
    bpy.ops.object.select_all(action="DESELECT")
    for obj in COCKPIT_OBJECTS:
        if obj.name in bpy.data.objects:
            obj.select_set(True)
    bpy.context.view_layer.objects.active = next(
        (o for o in COCKPIT_OBJECTS if o.name in bpy.data.objects and o.type == "MESH"), None
    )
    bpy.ops.export_scene.gltf(
        filepath=OUT_GLB,
        export_format="GLB",
        use_selection=True,
        export_apply=True,
        export_yup=True,
        export_extras=True,
        export_cameras=False,
        export_lights=False,
        export_animations=False,
        export_skins=False,
        export_morph=False,
        export_tangents=False,
        export_texcoords=True,
        export_normals=True,
        export_materials="EXPORT",
    )
    print(f"exported {OUT_GLB} ({os.path.getsize(OUT_GLB) / 1e6:.2f} MB)")


# ---------------------------------------------------------------------------
# preview render (not exported)
# ---------------------------------------------------------------------------


def add_preview_environment():
    scene = bpy.context.scene
    # sky
    world = bpy.data.worlds.new("AH64_PreviewWorld")
    world.use_nodes = True
    nodes = world.node_tree.nodes
    links = world.node_tree.links
    bg = nodes.get("Background")
    sky = nodes.new("ShaderNodeTexSky")
    sky.sky_type = "MULTIPLE_SCATTERING"
    sky.sun_elevation = math.radians(32.0)
    sky.sun_rotation = math.radians(140.0)
    sky.sun_intensity = 1.2
    sky.altitude = 500.0
    links.new(sky.outputs["Color"], bg.inputs["Color"])
    bg.inputs["Strength"].default_value = 0.35
    scene.world = world

    # ground
    bpy.ops.mesh.primitive_plane_add(size=4000.0, location=(0.0, 0.0, -60.0))
    ground = bpy.context.object
    ground.name = "PREVIEW_ground"
    mat = make_material("AH64_PreviewGround", "#6f6249", rough=0.95)
    _set_material(ground, mat)

    # sun
    bpy.ops.object.light_add(type="SUN", location=(0.0, 0.0, 10.0))
    sun = bpy.context.object
    sun.data.energy = 3.0
    sun.data.angle = math.radians(2.5)
    sun.rotation_euler = (math.radians(55.0), 0.0, math.radians(35.0))

    # cockpit fill lights
    for loc, size, energy, color in (
        ((0.0, 0.55, 0.30), 1.2, 2.5, (1.0, 0.95, 0.88)),
        ((0.0, -0.10, 0.45), 0.5, 1.2, (0.85, 0.92, 1.0)),
        ((0.0, -2.50, 0.60), 3.0, 35.0, (1.0, 0.96, 0.88)),
    ):
        bpy.ops.object.light_add(type="AREA", location=loc)
        light = bpy.context.object
        light.data.size = size
        light.data.energy = energy
        light.data.color = color
        direction = Vector((0.0, 0.0, 0.0)) - Vector(loc)
        light.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()

    # camera
    bpy.ops.object.camera_add(location=(0.0, 0.0, 0.0))
    cam = bpy.context.object
    cam.name = "PREVIEW_camera"
    cam.data.lens = 20.0
    cam.data.sensor_width = 36.0
    cam.data.clip_start = 0.005
    cam.data.clip_end = 5000.0
    scene.camera = cam
    return cam


def render_previews():
    os.makedirs(PREVIEW_DIR, exist_ok=True)
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 900
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    try:
        scene.eevee.taa_render_samples = 48
    except Exception:
        pass
    try:
        scene.view_settings.view_transform = "AgX"
        scene.view_settings.look = "AgX - Medium High Contrast"
    except Exception:
        pass
    scene.view_settings.exposure = -0.2

    cam = add_preview_environment()
    views = {
        "forward": Vector((0.0, -5.0, -0.10)),
        "panel": Vector((0.0, -3.0, -0.16)),
        "canopy": Vector((0.0, -3.0, 0.42)),
        "left": Vector((2.6, -1.2, 0.55)),
        "right": Vector((-2.6, -1.2, 0.55)),
        "left_low": Vector((3.0, -2.4, -0.35)),
        "right_low": Vector((-3.0, -2.4, -0.35)),
        "up": Vector((0.0, -1.5, 3.0)),
        "down": Vector((0.0, -2.0, -2.2)),
        "back": Vector((0.0, 4.0, -0.15)),
        "wide": Vector((2.2, -2.2, 1.0)),
    }
    for name, target in views.items():
        if name == "wide":
            cam.location = (2.0, 2.6, 1.8)
            direction = Vector((0.0, -0.35, -0.25)) - Vector(cam.location)
            cam.data.lens = 32.0
        else:
            cam.location = (0.0, 0.0, 0.0)
            cam.data.lens = 35.0 if name == "panel" else 22.0
            direction = target - Vector((0.0, 0.0, 0.0))
        cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = os.path.join(PREVIEW_DIR, f"ah64_{name}.png")
        print(f"  rendering {name} ...")
        bpy.ops.render.render(write_still=True)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def build():
    print("materials", flush=True)
    build_materials()
    build_shell()
    build_canopy()
    wiper("L", 1.0)
    wiper("R", -1.0)
    build_glareshield()
    build_panel()
    build_mpd("L", MPD_CX)
    build_mpd("R", -MPD_CX)
    build_eufd()
    build_caution_panel()
    build_standby()
    build_consoles()
    build_center_pedestal()
    build_controls()
    build_seat()
    build_misc_details()
    join_groups()
    add_anchors()


def main():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-preview", action="store_true")
    args = parser.parse_args(argv)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    build()
    export_glb()
    if not args.no_preview:
        render_previews()
    print("done")


if __name__ == "__main__":
    main()
