#!/usr/bin/env python3
"""Build and render a procedural USD sliding station asset for Isaac Sim.

The geometry is generated from front-view pixel landmarks so the validation
camera can match the supplied reference composition without using a billboard.
"""

from __future__ import annotations

import argparse
import asyncio
import math
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image


RENDER_WIDTH = 1536
RENDER_HEIGHT = 889

TARGET_BBOX = (62, 303, 1384, 658)
TARGET_WHITE_PANELS = {
    "left": (69, 513, 115, 646),
    "right": (1325, 516, 1374, 651),
}
TARGET_WARNINGS = {
    "left": (75, 453, 105, 478),
    "right": (1339, 455, 1370, 481),
}

ROOT_PATH = "/World/SlidingStation"
CAMERA_PATH = "/World/Camera_Front_Reference"

SHELF_CART_B_SOURCE_USD = Path(
    "/home/mh/.cache/ov/client/https/a8d3d53d6d261a11c594ba76e56592ea8c4627be784cac6efb07deb20cb18b99.usd"
)
SHELF_CART_B_SOURCE_PRIM = "/Root/Visual/FOF_Mesh_Shelf_Cart_B_LOD0"
SHELF_CART_B_MESH_MIN = np.array([-0.4250573829565197, -0.6307055321486181, 0.21031635396131768])
SHELF_CART_B_MESH_MAX = np.array([0.4253401099755604, 0.6287146812698621, 0.4414757414392642])
SHELF_CART_B_MESH_SIZE = SHELF_CART_B_MESH_MAX - SHELF_CART_B_MESH_MIN
SHELF_CART_B_ROTATE_Z_DEGREES = 90.0
SHELF_CART_B_ROTATED_MIN = np.array(
    [-SHELF_CART_B_MESH_MAX[1], SHELF_CART_B_MESH_MIN[0], SHELF_CART_B_MESH_MIN[2]]
)
SHELF_CART_B_ROTATED_MAX = np.array(
    [-SHELF_CART_B_MESH_MIN[1], SHELF_CART_B_MESH_MAX[0], SHELF_CART_B_MESH_MAX[2]]
)
SHELF_CART_B_ROTATED_SIZE = SHELF_CART_B_ROTATED_MAX - SHELF_CART_B_ROTATED_MIN

GEOMETRY_REFERENCE_WIDTH_M = 8.80
TARGET_WIDTH_M = float(SHELF_CART_B_MESH_SIZE[0])
PIXELS_PER_METER = (TARGET_BBOX[2] - TARGET_BBOX[0]) / GEOMETRY_REFERENCE_WIDTH_M
CAMERA_PIXELS_PER_METER = (TARGET_BBOX[2] - TARGET_BBOX[0]) / TARGET_WIDTH_M
REFERENCE_CENTER_X_PX = (TARGET_BBOX[0] + TARGET_BBOX[2]) * 0.5
REFERENCE_BOTTOM_Y_PX = TARGET_BBOX[3]
ORTHO_WIDTH_M = RENDER_WIDTH / CAMERA_PIXELS_PER_METER
ORTHO_HEIGHT_M = RENDER_HEIGHT / CAMERA_PIXELS_PER_METER
CAMERA_CENTER_Z = (REFERENCE_BOTTOM_Y_PX - 480.0) / CAMERA_PIXELS_PER_METER

FRONT_Y = -0.66
BACK_Y = 0.54
DETAIL_Y = FRONT_Y - 0.018

GLOSSY_BLACK = "GlossyBlack"
DARK_BLACK = "DarkBlackVariation"
SILVER_RAIL = "SilverRail"
WHITE_PANEL = "WhitePanel"
WARNING_YELLOW = "WarningYellow"
WARNING_BLACK = "WarningBlack"
RAIL_GROOVE = "RailGroove"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd-path", default="sliding_station.usd")
    parser.add_argument("--preview-path", default="sliding_station_preview.png")
    parser.add_argument("--reference-path", default=None, help="Optional reference PNG used only for validation.")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--show", action="store_true", help="Run with a visible Isaac Sim window.")
    parser.add_argument("--no-render", action="store_true", help="Create only the USD file.")
    parser.add_argument("--keep-open", action="store_true", help="Keep the Isaac Sim preview window open after creation.")
    parser.add_argument("--max-frame-iterations", type=int, default=5)
    return parser.parse_args()


def px_to_x(px: float) -> float:
    return (float(px) - REFERENCE_CENTER_X_PX) / PIXELS_PER_METER


def py_to_z(py: float) -> float:
    return (REFERENCE_BOTTOM_Y_PX - float(py)) / PIXELS_PER_METER


def p2m(point_px: tuple[float, float]) -> tuple[float, float]:
    return px_to_x(point_px[0]), py_to_z(point_px[1])


def create_material(stage, name, color, *, metallic=0.0, roughness=0.2, specular=0.75):
    from pxr import Gf, Sdf, UsdShade

    material = UsdShade.Material.Define(stage, f"{ROOT_PATH}/Materials/{name}")
    shader = UsdShade.Shader.Define(stage, f"{ROOT_PATH}/Materials/{name}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(specular, specular, specular))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def bind_material(prim, material) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def add_collision(prim) -> None:
    from pxr import PhysxSchema, UsdPhysics

    UsdPhysics.CollisionAPI.Apply(prim)
    if prim.GetTypeName() == "Mesh":
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_api.CreateApproximationAttr().Set("convexHull")
    try:
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
    except Exception:
        pass


def create_custom_mesh(
    stage,
    path: str,
    points: Iterable,
    face_counts: Iterable[int],
    face_indices: Iterable[int],
    material,
    *,
    collision: bool = True,
    double_sided: bool = True,
):
    from pxr import Gf, UsdGeom

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in points])
    mesh.CreateFaceVertexCountsAttr(list(face_counts))
    mesh.CreateFaceVertexIndicesAttr(list(face_indices))
    mesh.CreateDoubleSidedAttr(bool(double_sided))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    if material is not None:
        bind_material(mesh.GetPrim(), material)
    if collision:
        add_collision(mesh.GetPrim())
    return mesh


def create_extruded_profile_mesh(
    stage,
    path: str,
    points_xz: list[tuple[float, float]],
    *,
    y_min: float,
    y_max: float,
    material,
    collision: bool = True,
):
    points = [(x, y_min, z) for x, z in points_xz] + [(x, y_max, z) for x, z in points_xz]
    n = len(points_xz)
    face_counts = [n, n]
    face_indices = list(range(n)) + list(range(2 * n - 1, n - 1, -1))
    for idx in range(n):
        nxt = (idx + 1) % n
        face_counts.append(4)
        face_indices.extend([idx, nxt, nxt + n, idx + n])
    return create_custom_mesh(stage, path, points, face_counts, face_indices, material, collision=collision)


def create_front_polygon(stage, path: str, points_xz: list[tuple[float, float]], y: float, material):
    points = [(x, y, z) for x, z in points_xz]
    face_counts = [len(points_xz)]
    face_indices = list(range(len(points_xz)))
    return create_custom_mesh(stage, path, points, face_counts, face_indices, material, collision=False)


def create_extruded_profile_mesh_from_px(stage, path: str, points_px, *, y_min, y_max, material, collision=True):
    return create_extruded_profile_mesh(
        stage,
        path,
        [p2m(point) for point in points_px],
        y_min=y_min,
        y_max=y_max,
        material=material,
        collision=collision,
    )


def chamfered_rect_xz(x0: float, x1: float, z0: float, z1: float, bevel: float) -> list[tuple[float, float]]:
    bevel = min(float(bevel), abs(x1 - x0) * 0.45, abs(z1 - z0) * 0.45)
    if bevel <= 0.0:
        return [(x0, z0), (x1, z0), (x1, z1), (x0, z1)]
    return [
        (x0 + bevel, z0),
        (x1 - bevel, z0),
        (x1, z0 + bevel),
        (x1, z1 - bevel),
        (x1 - bevel, z1),
        (x0 + bevel, z1),
        (x0, z1 - bevel),
        (x0, z0 + bevel),
    ]


def create_beveled_box(stage, path: str, center, size, material, *, bevel=0.035, collision=True):
    cx, cy, cz = [float(v) for v in center]
    sx, sy, sz = [float(v) for v in size]
    x0, x1 = cx - sx * 0.5, cx + sx * 0.5
    z0, z1 = cz - sz * 0.5, cz + sz * 0.5
    return create_extruded_profile_mesh(
        stage,
        path,
        chamfered_rect_xz(x0, x1, z0, z1, bevel),
        y_min=cy - sy * 0.5,
        y_max=cy + sy * 0.5,
        material=material,
        collision=collision,
    )


def create_rotated_box_between_points(
    stage,
    path: str,
    start_xz: tuple[float, float],
    end_xz: tuple[float, float],
    *,
    center_y: float,
    depth: float,
    thickness: float,
    material,
    collision: bool = True,
):
    sx, sz = start_xz
    ex, ez = end_xz
    dx, dz = ex - sx, ez - sz
    length = math.hypot(dx, dz)
    if length <= 1e-6:
        raise ValueError(f"Cannot create rotated box with zero length: {path}")
    ux, uz = dx / length, dz / length
    nx, nz = -uz, ux
    half_t = thickness * 0.5
    points_xz = [
        (sx + nx * half_t, sz + nz * half_t),
        (ex + nx * half_t, ez + nz * half_t),
        (ex - nx * half_t, ez - nz * half_t),
        (sx - nx * half_t, sz - nz * half_t),
    ]
    return create_extruded_profile_mesh(
        stage,
        path,
        points_xz,
        y_min=center_y - depth * 0.5,
        y_max=center_y + depth * 0.5,
        material=material,
        collision=collision,
    )


def mirror_xz(points_xz: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-x, z) for x, z in reversed(list(points_xz))]


def create_chamfered_base_housing(stage, side: str, materials):
    sign = -1.0 if side == "Left" else 1.0
    left_profile_px = [
        (62, 402),
        (66, 645),
        (77, 652),
        (122, 651),
        (219, 561),
        (269, 558),
        (275, 462),
        (285, 452),
        (333, 453),
        (343, 445),
        (343, 381),
        (349, 367),
        (366, 356),
        (329, 304),
        (118, 392),
        (96, 385),
    ]
    points = [p2m(p) for p in left_profile_px]
    if side == "Right":
        points = mirror_xz(points)

    create_extruded_profile_mesh(
        stage,
        f"{ROOT_PATH}/{side}Base",
        points,
        y_min=FRONT_Y,
        y_max=BACK_Y,
        material=materials[GLOSSY_BLACK],
    )

    outer_x = sign * 4.235
    create_beveled_box(
        stage,
        f"{ROOT_PATH}/{side}BaseOuterGlossFace",
        center=(outer_x, FRONT_Y - 0.004, 0.86),
        size=(0.42, 0.045, 1.34),
        material=materials[DARK_BLACK],
        bevel=0.045,
    )
    create_rotated_box_between_points(
        stage,
        f"{ROOT_PATH}/{side}BaseLowerSlopedFairing",
        (sign * 3.95, 0.22),
        (sign * 3.35, 0.78),
        center_y=FRONT_Y + 0.10,
        depth=0.42,
        thickness=0.26,
        material=materials[DARK_BLACK],
    )
    create_rotated_box_between_points(
        stage,
        f"{ROOT_PATH}/{side}BaseUpperShoulderChamfer",
        (sign * 4.18, 1.78),
        (sign * 3.63, 1.94),
        center_y=FRONT_Y + 0.10,
        depth=0.52,
        thickness=0.16,
        material=materials[GLOSSY_BLACK],
    )
    create_beveled_box(
        stage,
        f"{ROOT_PATH}/{side}InnerHangingSkirt",
        center=(sign * 2.60, FRONT_Y + 0.055, 1.03),
        size=(0.40, 0.22, 0.78),
        material=materials[DARK_BLACK],
        bevel=0.030,
    )
    return points


def create_angled_arm(stage, side: str, materials):
    sign = -1.0 if side == "Left" else 1.0
    start = (sign * 4.03, 1.74)
    end = (sign * 2.58, 2.28)
    if side == "Left":
        start, end = start, end
    create_rotated_box_between_points(
        stage,
        f"{ROOT_PATH}/{side}AngledArm",
        start,
        end,
        center_y=FRONT_Y + 0.22,
        depth=0.68,
        thickness=0.26,
        material=materials[GLOSSY_BLACK],
    )
    create_rotated_box_between_points(
        stage,
        f"{ROOT_PATH}/{side}AngledArmGlossLowerFacet",
        (sign * 4.10, 1.59),
        (sign * 2.66, 2.13),
        center_y=FRONT_Y + 0.04,
        depth=0.23,
        thickness=0.13,
        material=materials[DARK_BLACK],
    )


def create_silver_rail(stage, side: str, materials):
    sign = -1.0 if side == "Left" else 1.0
    if side == "Left":
        start = p2m((116, 385))
        end = p2m((346, 306))
    else:
        start = (-p2m((346, 306))[0], p2m((346, 306))[1])
        end = (-p2m((116, 385))[0], p2m((116, 385))[1])

    create_rotated_box_between_points(
        stage,
        f"{ROOT_PATH}/{side}Rail",
        start,
        end,
        center_y=FRONT_Y - 0.025,
        depth=0.24,
        thickness=0.115,
        material=materials[SILVER_RAIL],
    )

    sx, sz = start
    ex, ez = end
    dx, dz = ex - sx, ez - sz
    length = math.hypot(dx, dz)
    ux, uz = dx / length, dz / length
    nx, nz = -uz, ux
    for idx, t in enumerate(np.linspace(0.15, 0.85, 5)):
        cx, cz = sx + dx * float(t), sz + dz * float(t)
        create_rotated_box_between_points(
            stage,
            f"{ROOT_PATH}/{side}RailDarkGroove{idx}",
            (cx - nx * 0.055, cz - nz * 0.055),
            (cx + nx * 0.055, cz + nz * 0.055),
            center_y=FRONT_Y - 0.050,
            depth=0.026,
            thickness=0.018,
            material=materials[RAIL_GROOVE],
            collision=False,
        )
    for cap_idx, t in enumerate((0.0, 1.0)):
        cx, cz = sx + dx * t, sz + dz * t
        create_rotated_box_between_points(
            stage,
            f"{ROOT_PATH}/{side}RailBlackEndCap{cap_idx}",
            (cx - ux * 0.055, cz - uz * 0.055),
            (cx + ux * 0.055, cz + uz * 0.055),
            center_y=FRONT_Y - 0.025,
            depth=0.27,
            thickness=0.15,
            material=materials[GLOSSY_BLACK],
        )


def create_stopper_blocks(stage, side: str, materials):
    from pxr import UsdGeom

    group = UsdGeom.Xform.Define(stage, f"{ROOT_PATH}/{side}StopperBlocks")
    sign = -1.0 if side == "Left" else 1.0
    if side == "Left":
        base = p2m((176, 410))
        direction_start = p2m((150, 418))
        direction_end = p2m((208, 394))
    else:
        left_base = p2m((176, 410))
        left_start = p2m((150, 418))
        left_end = p2m((208, 394))
        base = (-left_base[0], left_base[1])
        direction_start = (-left_end[0], left_end[1])
        direction_end = (-left_start[0], left_start[1])

    dx, dz = direction_end[0] - direction_start[0], direction_end[1] - direction_start[1]
    length = math.hypot(dx, dz)
    ux, uz = dx / length, dz / length
    for label, half_len, z_lift, thick in (
        ("Lower", 0.19, 0.00, 0.120),
        ("Upper", 0.13, 0.095, 0.105),
        ("RearStep", 0.15, -0.095, 0.090),
    ):
        cx, cz = base[0], base[1] + z_lift
        create_rotated_box_between_points(
            stage,
            f"{ROOT_PATH}/{side}StopperBlocks/{label}",
            (cx - ux * half_len, cz - uz * half_len),
            (cx + ux * half_len, cz + uz * half_len),
            center_y=FRONT_Y - 0.055,
            depth=0.22,
            thickness=thick,
            material=materials[GLOSSY_BLACK],
        )
    return group


def create_white_panel(stage, side: str, materials):
    box = TARGET_WHITE_PANELS["left" if side == "Left" else "right"]
    x0, z1 = p2m((box[0], box[1]))
    x1, z0 = p2m((box[2], box[3]))
    center = ((x0 + x1) * 0.5, DETAIL_Y, (z0 + z1) * 0.5)
    size = (abs(x1 - x0), 0.026, abs(z1 - z0))
    create_beveled_box(stage, f"{ROOT_PATH}/{side}WhitePanel", center, size, materials[WHITE_PANEL], bevel=0.015, collision=False)
    border = 0.024
    cx, _, cz = center
    sx, sy, sz = size
    create_beveled_box(
        stage,
        f"{ROOT_PATH}/{side}WhitePanelBorderTop",
        (cx, DETAIL_Y - 0.006, cz + sz * 0.5 + border * 0.5),
        (sx + border * 1.4, 0.014, border),
        materials[DARK_BLACK],
        bevel=0.004,
        collision=False,
    )
    create_beveled_box(
        stage,
        f"{ROOT_PATH}/{side}WhitePanelBorderBottom",
        (cx, DETAIL_Y - 0.006, cz - sz * 0.5 - border * 0.5),
        (sx + border * 1.4, 0.014, border),
        materials[DARK_BLACK],
        bevel=0.004,
        collision=False,
    )
    create_beveled_box(
        stage,
        f"{ROOT_PATH}/{side}WhitePanelBorderOuter",
        (cx + math.copysign(sx * 0.5 + border * 0.5, x0 + x1), DETAIL_Y - 0.006, cz),
        (border, 0.014, sz + border * 1.6),
        materials[DARK_BLACK],
        bevel=0.004,
        collision=False,
    )
    create_beveled_box(
        stage,
        f"{ROOT_PATH}/{side}WhitePanelBorderInner",
        (cx - math.copysign(sx * 0.5 + border * 0.5, x0 + x1), DETAIL_Y - 0.006, cz),
        (border, 0.014, sz + border * 1.6),
        materials[DARK_BLACK],
        bevel=0.004,
        collision=False,
    )


def circle_polygon(center_xz: tuple[float, float], radius: float, segments: int = 24) -> list[tuple[float, float]]:
    cx, cz = center_xz
    return [
        (cx + math.cos(2.0 * math.pi * i / segments) * radius, cz + math.sin(2.0 * math.pi * i / segments) * radius)
        for i in range(segments)
    ]


def sector_polygon(center_xz: tuple[float, float], inner_radius: float, outer_radius: float, angle_deg: float, spread_deg: float):
    cx, cz = center_xz
    a0 = math.radians(angle_deg - spread_deg * 0.5)
    a1 = math.radians(angle_deg + spread_deg * 0.5)
    return [
        (cx + math.cos(a0) * inner_radius, cz + math.sin(a0) * inner_radius),
        (cx + math.cos(a0) * outer_radius, cz + math.sin(a0) * outer_radius),
        (cx + math.cos(a1) * outer_radius, cz + math.sin(a1) * outer_radius),
        (cx + math.cos(a1) * inner_radius, cz + math.sin(a1) * inner_radius),
    ]


def create_radiation_symbol(stage, root_path: str, center_xz: tuple[float, float], scale: float, y: float, material):
    create_front_polygon(stage, f"{root_path}/TrefoilCenter", circle_polygon(center_xz, scale * 0.13, 18), y, material)
    for idx, angle in enumerate((90.0, 210.0, 330.0)):
        create_front_polygon(
            stage,
            f"{root_path}/TrefoilBlade{idx}",
            sector_polygon(center_xz, scale * 0.20, scale * 0.46, angle, 58.0),
            y,
            material,
        )


def create_triangle_warning_decal(stage, side: str, materials):
    from pxr import UsdGeom

    key = "left" if side == "Left" else "right"
    box = TARGET_WARNINGS[key]
    x0, z1 = p2m((box[0], box[1]))
    x1, z0 = p2m((box[2], box[3]))
    cx = (x0 + x1) * 0.5
    width = abs(x1 - x0)
    height = abs(z1 - z0)
    root = UsdGeom.Xform.Define(stage, f"{ROOT_PATH}/{side}WarningDecal")
    root_path = str(root.GetPath())

    border = [
        (cx, z1 + height * 0.07),
        (cx - width * 0.58, z0 - height * 0.07),
        (cx + width * 0.58, z0 - height * 0.07),
    ]
    fill = [
        (cx, z1),
        (cx - width * 0.50, z0),
        (cx + width * 0.50, z0),
    ]
    create_front_polygon(stage, f"{root_path}/BlackTriangleBorder", border, DETAIL_Y - 0.026, materials[WARNING_BLACK])
    create_front_polygon(stage, f"{root_path}/YellowTriangle", fill, DETAIL_Y - 0.030, materials[WARNING_YELLOW])
    create_radiation_symbol(stage, root_path, (cx, (z0 + z1) * 0.5 - height * 0.03), height * 0.78, DETAIL_Y - 0.034, materials[WARNING_BLACK])


def create_central_bridge(stage, materials):
    top_l = p2m((297, 303))
    top_r = p2m((1155, 303))
    bot_r = p2m((1090, 358))
    bot_l = p2m((366, 356))
    points_xz = [top_l, top_r, bot_r, bot_l]
    create_extruded_profile_mesh(
        stage,
        f"{ROOT_PATH}/CenterBridge",
        points_xz,
        y_min=FRONT_Y + 0.015,
        y_max=FRONT_Y + 0.62,
        material=materials[GLOSSY_BLACK],
    )
    lip_z = py_to_z(359)
    create_beveled_box(
        stage,
        f"{ROOT_PATH}/CenterBridgeLowerLip",
        center=(0.0, FRONT_Y - 0.015, lip_z - 0.035),
        size=(abs(bot_r[0] - bot_l[0]), 0.040, 0.055),
        material=materials[DARK_BLACK],
        bevel=0.015,
    )
    create_beveled_box(
        stage,
        f"{ROOT_PATH}/CenterBridgeTopHighlight",
        center=(0.0, FRONT_Y - 0.020, py_to_z(307)),
        size=(abs(top_r[0] - top_l[0]) * 0.96, 0.024, 0.026),
        material=materials[DARK_BLACK],
        bevel=0.010,
        collision=False,
    )


def setup_lighting(stage):
    from pxr import Gf, UsdGeom, UsdLux

    def place_xform(prim, translate):
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*translate))

    key = UsdLux.RectLight.Define(stage, "/World/SlidingStationStudioKeyLight")
    key.CreateIntensityAttr().Set(7600.0)
    key.CreateWidthAttr().Set(5.2)
    key.CreateHeightAttr().Set(1.8)
    place_xform(key.GetPrim(), (0.0, -3.8, 3.5))

    fill_left = UsdLux.SphereLight.Define(stage, "/World/SlidingStationFillLightLeft")
    fill_left.CreateIntensityAttr().Set(1800.0)
    fill_left.CreateRadiusAttr().Set(2.3)
    place_xform(fill_left.GetPrim(), (-3.8, -2.8, 1.5))

    fill_right = UsdLux.SphereLight.Define(stage, "/World/SlidingStationFillLightRight")
    fill_right.CreateIntensityAttr().Set(1800.0)
    fill_right.CreateRadiusAttr().Set(2.3)
    place_xform(fill_right.GetPrim(), (3.8, -2.8, 1.5))

    rim = UsdLux.SphereLight.Define(stage, "/World/SlidingStationRimLight")
    rim.CreateIntensityAttr().Set(4200.0)
    rim.CreateRadiusAttr().Set(1.7)
    place_xform(rim.GetPrim(), (0.0, 1.4, 2.6))


def setup_camera(stage, *, camera_x=0.0, camera_z=CAMERA_CENTER_Z, ortho_width_m=ORTHO_WIDTH_M):
    from pxr import Gf, UsdGeom

    camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    camera.CreateProjectionAttr().Set(UsdGeom.Tokens.orthographic)
    aperture_unit = Gf.Camera.APERTURE_UNIT
    camera.CreateHorizontalApertureAttr().Set(float(ortho_width_m / aperture_unit))
    camera.CreateVerticalApertureAttr().Set(float((ortho_width_m * RENDER_HEIGHT / RENDER_WIDTH) / aperture_unit))
    camera.CreateFocalLengthAttr().Set(50.0)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))

    eye = Gf.Vec3d(float(camera_x), -8.0, float(camera_z))
    target = Gf.Vec3d(float(camera_x), 0.0, float(camera_z))
    up = Gf.Vec3d(0.0, 0.0, 1.0)
    view = Gf.Matrix4d().SetLookAt(eye, target, up)
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(view.GetInverse())
    return camera


def set_active_viewport_camera(camera_path: str) -> None:
    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport:
            viewport.camera_path = camera_path
    except Exception as exc:
        print(f"[SlidingStation] viewport camera setup skipped: {exc}", flush=True)


def save_stage(stage, output_path: Path) -> None:
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(output_path))
    print(f"[SlidingStation] wrote USD: {output_path}", flush=True)


def measure_bbox(image_path: Path, *, threshold=5) -> tuple[int, int, int, int] | None:
    image = Image.open(image_path).convert("RGB")
    arr = np.asarray(image)
    mask = np.max(arr, axis=2) > threshold
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())


def bbox_error(actual, target) -> tuple[int, int, int, int]:
    return tuple(int(actual[i] - target[i]) for i in range(4))


def mask_bbox(arr, mask) -> tuple[int, int, int, int] | None:
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())


def validate_preview(preview_path: Path, reference_path: Path | None = None) -> dict:
    image = Image.open(preview_path).convert("RGB")
    arr = np.asarray(image)
    result = {"resolution": image.size}
    result["object_bbox"] = measure_bbox(preview_path)
    result["object_bbox_error"] = bbox_error(result["object_bbox"], TARGET_BBOX) if result["object_bbox"] else None

    white_mask = (arr[:, :, 0] > 145) & (arr[:, :, 1] > 145) & (arr[:, :, 2] > 130)
    yellow_mask = (arr[:, :, 0] > 135) & (arr[:, :, 1] > 80) & (arr[:, :, 2] < 80)
    result["white_left_bbox"] = mask_bbox(arr, white_mask & (np.indices(arr.shape[:2])[1] < RENDER_WIDTH // 2))
    result["white_right_bbox"] = mask_bbox(arr, white_mask & (np.indices(arr.shape[:2])[1] > RENDER_WIDTH // 2))
    result["warning_left_bbox"] = mask_bbox(arr, yellow_mask & (np.indices(arr.shape[:2])[1] < RENDER_WIDTH // 2))
    result["warning_right_bbox"] = mask_bbox(arr, yellow_mask & (np.indices(arr.shape[:2])[1] > RENDER_WIDTH // 2))

    if reference_path:
        ref = Image.open(reference_path).convert("RGBA")
        result["reference_resolution"] = ref.size
    print("[SlidingStation] validation:", result, flush=True)
    return result


async def capture_rgb(stage, camera_path: str):
    import omni.replicator.core as rep

    render_product = rep.create.render_product(camera_path, (RENDER_WIDTH, RENDER_HEIGHT))
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(render_product)
    try:
        for _ in range(8):
            await rep.orchestrator.step_async()
        data = annotator.get_data()
    finally:
        annotator.detach()
        render_product.destroy()
    return np.asarray(data)


def render_preview(simulation_app, stage, preview_path: Path) -> None:
    preview_path = preview_path.expanduser().resolve()
    preview_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

    task = loop.create_task(capture_rgb(stage, CAMERA_PATH))
    for _ in range(220):
        simulation_app.update()
        if not loop.is_running():
            loop.run_until_complete(asyncio.sleep(0))
        if task.done():
            break
    if not task.done():
        task.cancel()
        if not loop.is_running():
            loop.run_until_complete(asyncio.sleep(0))
        raise RuntimeError("Timed out while rendering sliding station preview.")
    rgb = task.result()
    if rgb.ndim != 3 or rgb.shape[2] not in (3, 4):
        raise RuntimeError(f"Unexpected render shape: {rgb.shape}")
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    Image.fromarray(rgb[:, :, :3]).save(preview_path)
    print(f"[SlidingStation] wrote preview: {preview_path}", flush=True)


def adjust_camera_for_bbox(stage, actual_bbox, target_bbox, current_width_m):
    if actual_bbox is None:
        return current_width_m
    from pxr import Gf, UsdGeom

    actual_w = max(1, actual_bbox[2] - actual_bbox[0])
    target_w = target_bbox[2] - target_bbox[0]
    new_width_m = current_width_m * actual_w / target_w

    actual_cx = (actual_bbox[0] + actual_bbox[2]) * 0.5
    actual_cy = (actual_bbox[1] + actual_bbox[3]) * 0.5
    target_cx = (target_bbox[0] + target_bbox[2]) * 0.5
    target_cy = (target_bbox[1] + target_bbox[3]) * 0.5
    current_ppm = actual_w / TARGET_WIDTH_M
    camera = UsdGeom.Camera(stage.GetPrimAtPath(CAMERA_PATH))
    aperture_unit = Gf.Camera.APERTURE_UNIT
    camera.GetHorizontalApertureAttr().Set(float(new_width_m / aperture_unit))
    camera.GetVerticalApertureAttr().Set(float((new_width_m * RENDER_HEIGHT / RENDER_WIDTH) / aperture_unit))

    xformable = UsdGeom.Xformable(camera.GetPrim())
    local = xformable.GetLocalTransformation()
    eye = local.ExtractTranslation()
    dx_world = (actual_cx - target_cx) / current_ppm
    dz_world = -(actual_cy - target_cy) / current_ppm
    setup_camera(stage, camera_x=eye[0] + dx_world, camera_z=eye[2] + dz_world, ortho_width_m=new_width_m)
    return new_width_m


def fit_authored_mesh_points_to_fof_mesh_size(stage) -> None:
    from pxr import Gf, Usd, UsdGeom

    root = stage.GetPrimAtPath(ROOT_PATH)
    mesh_points = []
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "Mesh":
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        if points:
            mesh_points.append((mesh, points))

    if not mesh_points:
        raise RuntimeError(f"No authored mesh points found under {ROOT_PATH}")

    all_points = np.array([[float(p[0]), float(p[1]), float(p[2])] for _, points in mesh_points for p in points])
    current_min = all_points.min(axis=0)
    current_max = all_points.max(axis=0)
    current_size = current_max - current_min
    target_size = SHELF_CART_B_MESH_SIZE.astype(float)
    target_min = np.array([-target_size[0] * 0.5, -target_size[1] * 0.5, 0.0], dtype=float)
    scale = np.divide(target_size, current_size, out=np.ones(3, dtype=float), where=current_size > 1e-9)

    for mesh, points in mesh_points:
        transformed = []
        for point in points:
            p = np.array([float(point[0]), float(point[1]), float(point[2])], dtype=float)
            fitted = target_min + (p - current_min) * scale
            transformed.append(Gf.Vec3f(float(fitted[0]), float(fitted[1]), float(fitted[2])))
        mesh.GetPointsAttr().Set(transformed)

    print(
        "[SlidingStation] fitted image-shaped station to FOF_Mesh_Shelf_Cart_B_LOD0 "
        f"from_size={tuple(float(v) for v in current_size)} "
        f"target_size={tuple(float(v) for v in target_size)}",
        flush=True,
    )


def create_asset(stage):
    from pxr import UsdGeom

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    root = UsdGeom.Xform.Define(stage, ROOT_PATH)
    UsdGeom.Xform.Define(stage, f"{ROOT_PATH}/Materials")

    materials = {
        GLOSSY_BLACK: create_material(stage, GLOSSY_BLACK, (0.004, 0.004, 0.004), roughness=0.62, specular=0.035),
        DARK_BLACK: create_material(stage, DARK_BLACK, (0.020, 0.020, 0.019), roughness=0.68, specular=0.025),
        SILVER_RAIL: create_material(stage, SILVER_RAIL, (0.66, 0.66, 0.63), metallic=0.42, roughness=0.48, specular=0.18),
        WHITE_PANEL: create_material(stage, WHITE_PANEL, (0.90, 0.90, 0.88), roughness=0.30, specular=0.42),
        WARNING_YELLOW: create_material(stage, WARNING_YELLOW, (1.0, 0.70, 0.02), roughness=0.38, specular=0.20),
        WARNING_BLACK: create_material(stage, WARNING_BLACK, (0.0, 0.0, 0.0), roughness=0.30, specular=0.20),
        RAIL_GROOVE: create_material(stage, RAIL_GROOVE, (0.08, 0.08, 0.08), roughness=0.26, specular=0.35),
    }

    create_central_bridge(stage, materials)
    for side in ("Left", "Right"):
        create_chamfered_base_housing(stage, side, materials)
        create_angled_arm(stage, side, materials)
        create_silver_rail(stage, side, materials)
        create_stopper_blocks(stage, side, materials)
        create_white_panel(stage, side, materials)
        create_triangle_warning_decal(stage, side, materials)

    fit_authored_mesh_points_to_fof_mesh_size(stage)

    setup_lighting(stage)
    setup_camera(stage)
    stage.SetDefaultPrim(root.GetPrim())
    return materials


def main() -> None:
    args = parse_args()
    if args.show:
        args.headless = False

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": bool(args.headless),
            "width": RENDER_WIDTH,
            "height": RENDER_HEIGHT,
            "renderer": "RayTracedLighting",
        }
    )

    import carb
    import omni.usd

    settings = carb.settings.get_settings()
    settings.set("/app/renderer/resolution/width", RENDER_WIDTH)
    settings.set("/app/renderer/resolution/height", RENDER_HEIGHT)
    settings.set("/app/window/width", RENDER_WIDTH)
    settings.set("/app/window/height", RENDER_HEIGHT)
    settings.set("/rtx/post/backgroundZeroAlpha", False)
    settings.set("/rtx/rendermode", "RayTracedLighting")

    context = omni.usd.get_context()
    context.new_stage()
    for _ in range(6):
        simulation_app.update()
    stage = context.get_stage()
    create_asset(stage)
    set_active_viewport_camera(CAMERA_PATH)
    for _ in range(12):
        simulation_app.update()

    usd_path = Path(args.usd_path)
    preview_path = Path(args.preview_path)
    save_stage(stage, usd_path)

    if not args.no_render:
        current_width_m = ORTHO_WIDTH_M
        for attempt in range(max(1, args.max_frame_iterations)):
            render_preview(simulation_app, stage, preview_path)
            bbox = measure_bbox(preview_path)
            print(f"[SlidingStation] render attempt {attempt + 1} bbox={bbox}", flush=True)
            if bbox and all(abs(bbox[i] - TARGET_BBOX[i]) <= 3 for i in range(4)):
                break
            current_width_m = adjust_camera_for_bbox(stage, bbox, TARGET_BBOX, current_width_m)
            for _ in range(4):
                simulation_app.update()
        save_stage(stage, usd_path)
        validate_preview(preview_path, Path(args.reference_path) if args.reference_path else None)

    if args.keep_open:
        print("[SlidingStation] preview is running; close Isaac Sim or press Ctrl-C to stop.", flush=True)
        try:
            while simulation_app.is_running():
                simulation_app.update()
        except KeyboardInterrupt:
            pass

    simulation_app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
