#!/usr/bin/env python3
"""Export the IONIQ 5 Blender asset as a ROS-frame Collada mesh.

Run from this package with Blender, not system Python:
  blender --background source_assets/Project_Cars_Hyundai_Ioniq_5.blend --python tools/export_ioniq5_mesh.py
"""

import json
import os
import re
import shutil
from mathutils import Matrix, Vector

import bpy


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MESH_DIR = os.path.join(PACKAGE_DIR, "meshes", "ioniq5")
TEXTURE_DIR = os.path.join(MESH_DIR, "textures")
OUT_DAE = os.path.join(MESH_DIR, "ioniq5_base_link.dae")
OUT_REPORT = os.path.join(MESH_DIR, "ioniq5_mesh_measurements.json")
SOURCE_TEXTURE_DIR = os.path.join(PACKAGE_DIR, "source_assets", "textures")

TARGET = {
    "length_m": 4.635,
    # Visual bbox width with side mirrors unfolded. Body width without mirrors is 1.890 m.
    "width_m": 2.134,
    "body_width_m": 1.890,
    "height_m": 1.600,
    "wheelbase_m": 3.000,
    "front_overhang_m": 0.845,
    "rear_overhang_m": 0.790,
    "front_track_m": 1.641,
    "rear_track_m": 1.651,
    "tire_width_m": 0.235,
    "tire_outer_diameter_m": 0.741,
}


def mesh_objects():
    return [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]


def object_bbox(obj):
    mins = Vector((float("inf"), float("inf"), float("inf")))
    maxs = Vector((float("-inf"), float("-inf"), float("-inf")))
    for corner in obj.bound_box:
        point = obj.matrix_world @ Vector(corner)
        mins.x = min(mins.x, point.x)
        mins.y = min(mins.y, point.y)
        mins.z = min(mins.z, point.z)
        maxs.x = max(maxs.x, point.x)
        maxs.y = max(maxs.y, point.y)
        maxs.z = max(maxs.z, point.z)
    return mins, maxs


def bbox_from_points(points):
    mins = Vector((float("inf"), float("inf"), float("inf")))
    maxs = Vector((float("-inf"), float("-inf"), float("-inf")))
    for point in points:
        mins.x = min(mins.x, point.x)
        mins.y = min(mins.y, point.y)
        mins.z = min(mins.z, point.z)
        maxs.x = max(maxs.x, point.x)
        maxs.y = max(maxs.y, point.y)
        maxs.z = max(maxs.z, point.z)
    return mins, maxs


def scene_bbox(objects):
    points = []
    for obj in objects:
        for corner in obj.bound_box:
            points.append(obj.matrix_world @ Vector(corner))
    return bbox_from_points(points)


def vertex_bbox(objects):
    points = []
    for obj in objects:
        for vertex in obj.data.vertices:
            points.append(obj.matrix_world @ vertex.co)
    return bbox_from_points(points)


def as_tuple(vec):
    return [round(vec.x, 6), round(vec.y, 6), round(vec.z, 6)]


def build_wheel_measurements(objects):
    centers = {}
    front_y = []
    rear_y = []
    widths_x = []
    diameters_y = []
    diameters_z = []

    for obj in objects:
        if not obj.name.startswith("SM_Wheel_"):
            continue
        obj_min, obj_max = object_bbox(obj)
        center = (obj_min + obj_max) * 0.5
        centers[obj.name] = center
        if "_F" in obj.name:
            front_y.append(center.y)
        else:
            rear_y.append(center.y)
        widths_x.append(obj_max.x - obj_min.x)
        diameters_y.append(obj_max.y - obj_min.y)
        diameters_z.append(obj_max.z - obj_min.z)

    centers["front_axle_y"] = sum(front_y) / len(front_y)
    centers["rear_axle_y"] = sum(rear_y) / len(rear_y)
    centers["width_x_m"] = sum(widths_x) / len(widths_x)
    centers["diameter_y_m"] = sum(diameters_y) / len(diameters_y)
    centers["diameter_z_m"] = sum(diameters_z) / len(diameters_z)
    return centers


def body_lateral_to_ros(raw_x, original):
    x_min, x_max = original["bbox_min"].x, original["bbox_max"].x
    return -TARGET["width_m"] / 2.0 + (
        (raw_x - x_min) / (x_max - x_min)
    ) * TARGET["width_m"]


def transform_body(point, original, front_axle_y, rear_axle_y):
    y_front, y_rear = original["bbox_min"].y, original["bbox_max"].y
    z_min, z_max = original["bbox_min"].z, original["bbox_max"].z

    if point.y >= rear_axle_y:
        ros_x = -TARGET["rear_overhang_m"] * (
            (point.y - rear_axle_y) / (y_rear - rear_axle_y)
        )
    elif point.y <= front_axle_y:
        ros_x = TARGET["wheelbase_m"] + TARGET["front_overhang_m"] * (
            (front_axle_y - point.y) / (front_axle_y - y_front)
        )
    else:
        ros_x = TARGET["wheelbase_m"] * (
            (rear_axle_y - point.y) / (rear_axle_y - front_axle_y)
        )

    ros_y = body_lateral_to_ros(point.x, original)
    ros_z = ((point.z - z_min) / (z_max - z_min)) * TARGET["height_m"]
    return Vector((ros_x, ros_y, ros_z))


def transform_wheel(point, obj_name, wheel, original):
    center = wheel[obj_name]
    is_front = "_F" in obj_name
    is_left = obj_name.endswith("_BL") or obj_name.endswith("_FL")

    target_center_x = TARGET["wheelbase_m"] if is_front else 0.0
    target_track = TARGET["front_track_m"] if is_front else TARGET["rear_track_m"]
    target_center_y = target_track / 2.0 if is_left else -target_track / 2.0
    target_center_z = TARGET["tire_outer_diameter_m"] / 2.0

    ros_x = target_center_x + (center.y - point.y) * (
        TARGET["tire_outer_diameter_m"] / wheel["diameter_y_m"]
    )
    ros_y = target_center_y + (point.x - center.x) * (
        TARGET["tire_width_m"] / wheel["width_x_m"]
    )
    ros_z = target_center_z + (point.z - center.z) * (
        TARGET["tire_outer_diameter_m"] / wheel["diameter_z_m"]
    )
    return Vector((ros_x, ros_y, ros_z))


def rewrite_mesh_vertices(obj, original, wheel):
    obj.data = obj.data.copy()
    is_wheel = obj.name.startswith("SM_Wheel_")
    for vertex in obj.data.vertices:
        world_point = obj.matrix_world @ vertex.co
        if is_wheel:
            vertex.co = transform_wheel(world_point, obj.name, wheel, original)
        else:
            vertex.co = transform_body(
                world_point,
                original,
                wheel["front_axle_y"],
                wheel["rear_axle_y"],
            )
    obj.matrix_world = Matrix.Identity(4)
    obj.location = (0.0, 0.0, 0.0)
    obj.rotation_euler = (0.0, 0.0, 0.0)
    obj.scale = (1.0, 1.0, 1.0)
    obj.data.update()


def copy_textures():
    os.makedirs(TEXTURE_DIR, exist_ok=True)
    for name in os.listdir(SOURCE_TEXTURE_DIR):
        if name.lower().endswith(".png"):
            shutil.copy2(
                os.path.join(SOURCE_TEXTURE_DIR, name),
                os.path.join(TEXTURE_DIR, name),
            )

    for image in bpy.data.images:
        if not image.filepath:
            continue
        basename = os.path.basename(bpy.path.abspath(image.filepath))
        if basename:
            image.filepath = os.path.join("textures", basename)


def rewrite_collada_texture_paths():
    with open(OUT_DAE, "r", encoding="utf-8") as handle:
        data = handle.read()

    texture_names = [name for name in os.listdir(TEXTURE_DIR) if name.lower().endswith(".png")]
    for name in texture_names:
        data = re.sub(r"[^<>\"]*" + re.escape(name), "textures/" + name, data)

    with open(OUT_DAE, "w", encoding="utf-8") as handle:
        handle.write(data)


def write_report(report):
    with open(OUT_REPORT, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")


def main():
    os.makedirs(MESH_DIR, exist_ok=True)
    copy_textures()

    objects = mesh_objects()
    raw_min, raw_max = scene_bbox(objects)
    original = {
        "bbox_min": raw_min,
        "bbox_max": raw_max,
        "bbox_size": raw_max - raw_min,
        "bbox_center": (raw_min + raw_max) * 0.5,
    }
    wheel = build_wheel_measurements(objects)

    for obj in objects:
        rewrite_mesh_vertices(obj, original, wheel)

    ros_min, ros_max = vertex_bbox(objects)
    report = {
        "source_blend": bpy.data.filepath,
        "export_mode": "ros_rear_axle_ground_projection_baked_vertices",
        "ros_frame_convention": {
            "mesh_origin": "rear axle ground projection",
            "urdf_base_link": "rear axle center after applying -wheel_radius visual offset",
            "x": "forward",
            "y": "left",
            "z": "up",
        },
        "target_dimensions_m": TARGET,
        "wheel_lateral_policy": "use_official_track_with_mirror_width_visual_bbox",
        "raw_scene_bbox_m": {
            "min": as_tuple(raw_min),
            "max": as_tuple(raw_max),
            "size": as_tuple(raw_max - raw_min),
            "center": as_tuple((raw_min + raw_max) * 0.5),
        },
        "raw_wheel_centers_m": {
            key: round(value, 6) if isinstance(value, float) else as_tuple(value)
            for key, value in wheel.items()
        },
        "ros_bbox_m": {
            "min": as_tuple(ros_min),
            "max": as_tuple(ros_max),
            "size": as_tuple(ros_max - ros_min),
            "center": as_tuple((ros_min + ros_max) * 0.5),
        },
        "mesh_object_count": len(objects),
        "mesh_objects": sorted(obj.name for obj in objects),
    }
    write_report(report)

    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]

    bpy.ops.wm.collada_export(
        filepath=OUT_DAE,
        selected=True,
        apply_modifiers=True,
        include_animations=False,
        triangulate=True,
        use_texture_copies=False,
        use_object_instantiation=False,
    )
    rewrite_collada_texture_paths()

    write_report(report)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
