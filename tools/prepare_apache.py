#!/usr/bin/env python3
"""Prepare the downloaded AH-64D Apache for the sim.

The Sketchfab GLB ships the whole helicopter as seven material batches; the
main and tail rotor blades are welded into the big `Object_8` batch together
with the airframe.  This script splits the rotor assemblies out as their own
nodes, gives each node a pivot on its hub, and exports:

    assets/ah64d.glb

Axes stay as downloaded: Blender +X nose / +Z up; after the glTF Y-up
conversion that is +X nose / +Y up.  The app scales the model in `view.rs`.

Run:
    blender -b --factory-startup --python tools/prepare_apache.py
"""

from __future__ import annotations

import math
import os
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.environ.get(
    "APACHE_SRC",
    "/home/nanbert/box/boeing_ah-64d_apache_combat_helicopter.glb",
)
OUT = os.path.join(ROOT, "assets", "ah64d.glb")

# Rotor hubs, in the imported model's Blender coordinates (metres at scale 1).
# The Apache tail rotor is side-mounted on the fin: its hub is at +Y ≈ 0.10,
# not on the fin centreline (that was the original pivot bug).
MAIN_PIVOT = Vector((0.270, 0.000, 0.155))
TAIL_PIVOT = Vector((-1.050, 0.100, -0.027))


def clear_and_bake():
    """Drop the Sketchfab empties and bake every mesh transform into world space."""
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    bpy.ops.object.select_all(action="DESELECT")
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    bpy.ops.object.parent_clear(type="CLEAR_KEEP_TRANSFORM")
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)


def components(obj):
    """Connected-component vertex-index sets for a mesh object."""
    me = obj.data
    parent = list(range(len(me.vertices)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for e in me.edges:
        union(e.vertices[0], e.vertices[1])

    groups = {}
    for v in me.vertices:
        groups.setdefault(find(v.index), []).append(v.index)
    return groups


def component_bbox(obj, verts):
    me = obj.data
    pts = [me.vertices[i].co for i in verts]
    mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return mn, mx


def main_rotor_vertices(obj):
    """Blades + the hub/swashplate parts around the mast, in model coordinates."""
    chosen = []
    for verts in components(obj).values():
        mn, mx = component_bbox(obj, verts)
        # All four blades: two run across Y and two across X, so test the
        # larger of the two horizontal extents. The old test only caught the
        # transverse pair and left the longitudinal pair in the airframe.
        wide_blade = max(mx.x - mn.x, mx.y - mn.y) > 0.40 and mn.z > 0.05
        near_hub = (
            mn.z > 0.10
            and mx.z < 0.22
            and (Vector(((mn.x + mx.x) * 0.5, (mn.y + mx.y) * 0.5, 0.0)) - Vector((MAIN_PIVOT.x, 0.0, 0.0))).length
            < 0.26
        )
        if wide_blade or near_hub:
            chosen.extend(verts)
    return set(chosen)


def tail_rotor_vertices(obj):
    """The tail rotor is mounted on the side of the fin at model y ≈ +0.10.

    Its blades are the thin plates in that side slab; the fin is at y ≈ 0 and
    the horizontal stabilizer at y ≈ ±0.30, so a y-band isolates the rotor.
    """
    chosen = []
    for verts in components(obj).values():
        mn, mx = component_bbox(obj, verts)
        if (
            mx.x < -0.80
            and mn.x > -1.32
            and mn.y > 0.06
            and mx.y < 0.20
            and mn.z > -0.25
            and mx.z < 0.20
        ):
            chosen.extend(verts)
    return set(chosen)


def subset_object(src, keep_verts, name, pivot):
    """Create one object from `keep_verts`, with its origin on `pivot`.

    Copying through bmesh preserves the material indices and UV layers, so the
    downloaded PBR textures survive the split.
    """
    bm = bmesh.new()
    bm.from_mesh(src.data)
    bm.verts.ensure_lookup_table()
    # Every face lies wholly inside one connected component, so a face is kept
    # only when all of its vertices are in the selected set.
    doomed = [f for f in bm.faces if any(v.index not in keep_verts for v in f.verts)]
    bmesh.ops.delete(bm, geom=doomed, context="FACES")
    loose = [v for v in bm.verts if not v.link_faces]
    if loose:
        bmesh.ops.delete(bm, geom=loose, context="VERTS")

    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for mat in src.data.materials:
        me.materials.append(mat)
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    me.transform(Matrix.Translation(-pivot))
    obj.location = pivot
    print(f"  {name}: {len(me.vertices)} verts, origin {tuple(round(v, 3) for v in pivot)}")
    return obj


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=SRC)
    clear_and_bake()

    body = bpy.data.objects["Object_8"]
    print("body bbox", tuple(round(v, 3) for v in body.dimensions))

    main_verts = main_rotor_vertices(body)
    tail_verts = tail_rotor_vertices(body)
    all_verts = set(range(len(body.data.vertices)))
    airframe_verts = all_verts - main_verts - tail_verts

    main = subset_object(body, main_verts, "AH64_MainRotor", MAIN_PIVOT)
    tail = subset_object(body, tail_verts, "AH64_TailRotor", TAIL_PIVOT)
    air = subset_object(body, airframe_verts, "AH64_Airframe", Vector((0.0, 0.0, 0.0)))
    bpy.data.objects.remove(body, do_unlink=True)

    # The other material batches are part of the airframe; give them readable
    # names but keep them separate (they carry different textures).
    keep = (main, tail, air)
    for i, o in enumerate(
        [o for o in bpy.data.objects if o.type == "MESH" and o not in keep]
    ):
        if o.name.startswith("Object_"):
            o.name = f"AH64_Body_{i:02d}"

    # A small sanity render of the split rotors, so a bad selection is obvious.
    for obj, euler in ((main, (0.0, 0.0, math.radians(20.0))), (tail, (0.0, math.radians(35.0), 0.0))):
        _ = obj
        obj.rotation_euler = euler
    bpy.context.scene.render.engine = "BLENDER_EEVEE"
    bpy.context.scene.render.resolution_x = 900
    bpy.context.scene.render.resolution_y = 700
    world = bpy.data.worlds.new("W")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.45, 0.6, 0.8, 1.0)
    bpy.context.scene.world = world
    bpy.ops.object.light_add(type="SUN", location=(3, -4, 6))
    bpy.context.object.data.energy = 4
    bpy.context.object.rotation_euler = (math.radians(45), 0, math.radians(40))
    bpy.ops.object.camera_add(location=(4, -4, 3))
    cam = bpy.context.object
    cam.data.lens = 50
    cam.rotation_euler = (Vector((0, 0, 0)) - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    bpy.context.scene.render.filepath = "/tmp/opencode/apache_rotor_split.png"
    bpy.ops.render.render(write_still=True)
    for obj in (main, tail):
        obj.rotation_euler = (0.0, 0.0, 0.0)

    # Keep only the aircraft meshes in the export.
    bpy.ops.object.select_all(action="DESELECT")
    for o in bpy.data.objects:
        if o.type == "MESH":
            o.select_set(True)
    bpy.context.view_layer.objects.active = air
    bpy.ops.export_scene.gltf(
        filepath=OUT,
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
        export_texcoords=True,
        export_normals=True,
        export_materials="EXPORT",
    )
    print(f"exported {OUT} ({os.path.getsize(OUT) / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
