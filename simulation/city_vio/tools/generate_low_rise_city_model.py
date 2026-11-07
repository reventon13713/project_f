#!/usr/bin/env python3
"""Generate the separate, full-size low-rise city Gazebo asset deterministically."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "src/VIO_pkg/gazebo_models/vio_low_rise_city"
WORLD = ROOT / "src/VIO_pkg/gazebo_worlds/vio_low_rise_city.world"
TEXTURES = MODEL / "materials/textures"
ROAD_X = (-180.0, -90.0, 0.0, 90.0, 180.0)
ROAD_Y = (-140.0, -70.0, 0.0, 70.0, 140.0)
ROAD_WIDTH = 10.0
BLOCK_X = tuple(zip(ROAD_X[:-1], ROAD_X[1:]))
BLOCK_Y = tuple(zip(ROAD_Y[:-1], ROAD_Y[1:]))
FACADE_MATERIALS = (
    "VioLowRiseCity/FacadeBrickShop",
    "VioLowRiseCity/FacadeBlueStone",
    "VioLowRiseCity/FacadeTimberStore",
    "VioLowRiseCity/FacadeBakeryStucco",
    "VioLowRiseCity/FacadeSageClinic",
    "VioLowRiseCity/FacadeOchreWorkshop",
)
TREE_MATERIALS = (
    "VioLowRiseCity/TreeMaple",
    "VioLowRiseCity/TreeHornbeam",
    "VioLowRiseCity/TreeCherry",
)
BODY_COLORS = (
    "0.67 0.48 0.34 1", "0.48 0.58 0.55 1", "0.72 0.65 0.50 1",
    "0.54 0.49 0.45 1", "0.70 0.56 0.52 1", "0.50 0.56 0.67 1",
)
ROOF_COLORS = (
    "0.22 0.24 0.25 1", "0.34 0.25 0.20 1", "0.28 0.31 0.28 1",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def pose(x: float, y: float, z: float, yaw: float = 0.0) -> str:
    return f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.9f}"


def box_geometry(size_x: float, size_y: float, size_z: float) -> str:
    return f"<box><size>{size_x:.3f} {size_y:.3f} {size_z:.3f}</size></box>"


def color_material(color: str) -> str:
    return ("<material><ambient>" + color + "</ambient><diffuse>" + color +
            "</diffuse></material>")


def scripted_material(name: str) -> str:
    return ("<material><script><uri>model://vio_low_rise_city/materials/"
            f"scripts</uri><uri>model://vio_low_rise_city/materials/textures</uri>"
            f"<name>{name}</name></script></material>")


def add_box(parts: list[str], name: str, center: tuple[float, float, float],
            size: tuple[float, float, float], material: str,
            collision: bool = False, yaw: float = 0.0) -> None:
    parts.append(f'<visual name="{name}"><pose>{pose(*center, yaw)}</pose>')
    parts.append(f"<geometry>{box_geometry(*size)}</geometry>{material}</visual>")
    if collision:
        parts.append(f'<collision name="{name}_collision"><pose>{pose(*center, yaw)}</pose>')
        parts.append(f"<geometry>{box_geometry(*size)}</geometry></collision>")


def add_collision(parts: list[str], name: str, center: tuple[float, float, float],
                  size: tuple[float, float, float], yaw: float = 0.0) -> None:
    parts.append(f'<collision name="{name}"><pose>{pose(*center, yaw)}</pose>')
    parts.append(f"<geometry>{box_geometry(*size)}</geometry></collision>")


def add_face_box(parts: list[str], name: str, x: float, y: float,
                 sx: float, sy: float, face: str, along: float, z: float,
                 along_size: float, depth: float, height: float, material: str,
                 outset: float = 0.04) -> None:
    """Place real architectural geometry on a street-facing wall."""
    if face in ("north", "south"):
        sign = 1.0 if face == "north" else -1.0
        center = (x + along, y + sign * (sy / 2 + outset + depth / 2), z)
        size = (along_size, depth, height)
    else:
        sign = 1.0 if face == "east" else -1.0
        center = (x + sign * (sx / 2 + outset + depth / 2), y + along, z)
        size = (depth, along_size, height)
    add_box(parts, name, center, size, material)


def add_building(parts: list[str], records: list[dict], block_id: int,
                 slot: int, x: float, y: float, sx: float, sy: float,
                 height: float, face: str) -> None:
    name = f"b{block_id:02d}_{slot}_{face}"
    index = block_id * 6 + slot
    body = BODY_COLORS[index % len(BODY_COLORS)]
    material = FACADE_MATERIALS[index % len(FACADE_MATERIALS)]
    add_box(parts, f"{name}_body", (x, y, height / 2), (sx, sy, height),
            color_material(body), collision=True)
    add_box(parts, f"{name}_roof", (x, y, height + 0.16),
            (sx + 0.35, sy + 0.35, 0.32),
            scripted_material("VioLowRiseCity/RoofWeatheredMembrane"))
    # Real rooftop furniture is visible to the oblique near-nadir D435i even
    # when the vertical shopfront is near the image edge. Its position and
    # proportions vary per building, avoiding a repeated blank-roof pattern.
    roof_offset_x = ((index % 3) - 1) * min(3.0, sx * 0.18)
    roof_offset_y = (((index // 3) % 3) - 1) * min(2.2, sy * 0.18)
    rooftop_height = 0.38 + 0.10 * (index % 3)
    add_box(parts, f"{name}_rooftop_unit",
            (x + roof_offset_x, y + roof_offset_y,
             height + 0.32 + rooftop_height / 2),
            (2.0 + 0.35 * (index % 2), 1.35 + 0.25 * ((index // 2) % 2),
             rooftop_height),
            color_material("0.76 0.77 0.72 1" if index % 2 else
                           "0.19 0.23 0.24 1"), collision=True)
    add_box(parts, f"{name}_skylight",
            (x - roof_offset_x * 0.55, y - roof_offset_y * 0.55, height + 0.39),
            (2.8, 0.9, 0.14), color_material("0.20 0.34 0.42 1"))

    inset = 0.025
    if face in ("north", "south"):
        facade_size = (sx - 0.7, 0.05, height - 0.45)
        facade_y = y + (sy / 2 + inset) * (1 if face == "north" else -1)
        facade_center = (x, facade_y, height / 2)
    else:
        facade_size = (0.05, sy - 0.7, height - 0.45)
        facade_x = x + (sx / 2 + inset) * (1 if face == "east" else -1)
        facade_center = (facade_x, y, height / 2)
    add_box(parts, f"{name}_facade", facade_center, facade_size,
            scripted_material(material))

    # Slim awnings and parapets add real street-scale silhouette variation.
    awning_color = ("0.18 0.31 0.45 1" if index % 2 == 0 else
                    "0.53 0.24 0.20 1")
    if face in ("north", "south"):
        awning_center = (x, facade_center[1] + (0.35 if face == "north" else -0.35),
                          min(3.15, height * 0.58))
        awning_size = (max(5.0, sx * 0.55), 0.75, 0.16)
    else:
        awning_center = (facade_center[0] + (0.35 if face == "east" else -0.35), y,
                          min(3.15, height * 0.58))
        awning_size = (0.75, max(5.0, sy * 0.55), 0.16)
    add_box(parts, f"{name}_awning", awning_center, awning_size,
            color_material(awning_color))
    # The photographic facade is backed by ordinary city architecture, not a
    # calibration target: projecting window bays, a doorway, lintels, corner
    # returns and occasional balconies create stable corners and depth changes
    # at the same places a real low-rise streetscape provides them.
    extent = sx if face in ("north", "south") else sy
    masonry = color_material("0.73 0.70 0.63 1" if index % 2 else
                             "0.43 0.39 0.34 1")
    glazing = color_material("0.08 0.12 0.14 1")
    door = color_material("0.20 0.16 0.12 1" if index % 2 else
                          "0.12 0.24 0.29 1")
    for edge_index, along in enumerate((-extent / 2 + 0.28, extent / 2 - 0.28)):
        add_face_box(parts, f"{name}_corner_return_{edge_index}", x, y, sx, sy,
                     face, along, height / 2, 0.34, 0.22, height - 0.30, masonry)
    add_face_box(parts, f"{name}_entry_door", x, y, sx, sy, face,
                 extent * 0.30, 1.25, min(1.15, extent * 0.12), 0.12, 2.30, door)
    for window_index, fraction in enumerate((-0.25, 0.02)):
        add_face_box(parts, f"{name}_shop_window_{window_index}", x, y, sx, sy,
                     face, extent * fraction, 1.45, min(3.0, extent * 0.24),
                     0.10, 1.65, glazing)
    add_face_box(parts, f"{name}_ground_lintel", x, y, sx, sy, face,
                 -extent * 0.03, 2.48, extent * 0.78, 0.16, 0.20, masonry)

    if height >= 5.5:
        upper_z = min(height - 1.25, 4.85)
        for window_index, fraction in enumerate((-0.30, 0.0, 0.30)):
            window_width = min(2.25, extent * 0.18)
            add_face_box(parts, f"{name}_upper_window_{window_index}", x, y, sx, sy,
                         face, extent * fraction, upper_z, window_width,
                         0.11, 1.48, glazing)
            add_face_box(parts, f"{name}_upper_sill_{window_index}", x, y, sx, sy,
                         face, extent * fraction, upper_z - 0.82,
                         window_width + 0.28, 0.24, 0.13, masonry)
        if index % 3 == 0:
            add_face_box(parts, f"{name}_balcony_slab", x, y, sx, sy, face,
                         0.0, 3.62, min(4.8, extent * 0.42), 0.92, 0.16, masonry,
                         outset=0.32)
            add_face_box(parts, f"{name}_balcony_rail", x, y, sx, sy, face,
                         0.0, 4.04, min(4.6, extent * 0.40), 0.10, 0.72,
                         color_material("0.16 0.18 0.18 1"), outset=0.82)
    records.append({"name": name, "block": block_id, "floor_count": 1 if height < 5.5 else 2,
                    "height_m": height, "center_east_north_m": [x, y],
                    "size_east_north_m": [sx, sy], "street_face": face,
                    "facade_material": material,
                    "street_architecture": "windows_doors_lintels_corner_returns_balconies"})


def add_tree(parts: list[str], records: list[dict], name: str,
             x: float, y: float, height: float, species_index: int) -> None:
    material = TREE_MATERIALS[species_index % len(TREE_MATERIALS)]
    width = height * (0.46 if species_index % 3 == 1 else 0.62)
    # The canopy is a transparent photographic cutout on crossed planes. The
    # only primitive is an invisible, narrow trunk collision proxy.
    parts.append(f'<visual name="{name}_canopy_ns"><pose>{pose(x, y, height / 2)}</pose>')
    parts.append(f"<geometry><plane><normal>0 1 0</normal><size>{width:.3f} {height:.3f}</size>"
                 f"</plane></geometry>{scripted_material(material)}<cast_shadows>true</cast_shadows></visual>")
    parts.append(f'<visual name="{name}_canopy_ew"><pose>{pose(x, y, height / 2)}</pose>')
    parts.append(f"<geometry><plane><normal>1 0 0</normal><size>{width:.3f} {height:.3f}</size>"
                 f"</plane></geometry>{scripted_material(material)}<cast_shadows>true</cast_shadows></visual>")
    parts.append(f'<collision name="{name}_trunk_collision"><pose>{pose(x, y, 1.4)}</pose>')
    parts.append("<geometry><cylinder><radius>0.24</radius><length>2.8</length>"
                 "</cylinder></geometry></collision>")
    records.append({"name": name, "species_material": material,
                    "center_east_north_m": [x, y], "height_m": height,
                    "visual_geometry": "crossed_transparent_photographic_planes",
                    "collision_geometry": "invisible_trunk_only"})


def add_parked_car(parts: list[str], records: list[dict], name: str,
                   x: float, y: float, yaw: float, color_index: int) -> None:
    colors = ("0.58 0.10 0.08 1", "0.10 0.25 0.48 1", "0.64 0.62 0.56 1",
              "0.20 0.38 0.24 1")
    # A solid, recognisable street vehicle rather than a billboard or a single
    # primitive. The body, bonnet and dark glazed cabin form several real
    # depth steps in the near-nadir stereo image. One vehicle in every pair is
    # a low delivery van, adding useful height diversity without tall map
    # structures; it remains 4.95 m below the aircraft.
    heading = (math.cos(yaw), math.sin(yaw))
    body_color = color_material(colors[color_index % len(colors)])
    # Each roadside pair contains one passenger car and one low delivery van.
    # This is ordinary low-rise street furniture, and it keeps a third depth
    # layer visible instead of placing every landmark at passenger-car height.
    is_van = color_index % 2 == 0
    if is_van:
        length, width, height = 4.8, 1.95, 2.55
        add_box(parts, f"{name}_lower_body", (x, y, 0.68),
                (length, width, 1.15), body_color, yaw=yaw)
        upper_x = x - 0.35 * heading[0]
        upper_y = y - 0.35 * heading[1]
        add_box(parts, f"{name}_upper_body", (upper_x, upper_y, 1.88),
                (3.55, 1.78, 1.34), body_color, yaw=yaw)
        windscreen_x = x + 1.47 * heading[0]
        windscreen_y = y + 1.47 * heading[1]
        add_box(parts, f"{name}_windscreen", (windscreen_x, windscreen_y, 1.84),
                (0.12, 1.60, 1.10), color_material("0.12 0.18 0.21 1"), yaw=yaw)
    else:
        length, width, height = 4.4, 1.85, 1.62
        add_box(parts, f"{name}_lower_body", (x, y, 0.54),
                (length, width, 0.82), body_color, yaw=yaw)
        bonnet_x = x + 1.38 * heading[0]
        bonnet_y = y + 1.38 * heading[1]
        add_box(parts, f"{name}_bonnet", (bonnet_x, bonnet_y, 1.02),
                (1.18, 1.76, 0.25), body_color, yaw=yaw)
        cabin_x = x - 0.25 * heading[0]
        cabin_y = y - 0.25 * heading[1]
        add_box(parts, f"{name}_glazed_cabin", (cabin_x, cabin_y, 1.26),
                (2.15, 1.55, 0.72), color_material("0.16 0.22 0.25 1"), yaw=yaw)
    add_collision(parts, f"{name}_body_collision", (x, y, height / 2),
                  (length, width, height), yaw=yaw)
    marker_shift = (-0.55, -0.15, 0.25, 0.60)[color_index % 4]
    marker_x = x + marker_shift * heading[0]
    marker_y = y + marker_shift * heading[1]
    add_box(parts, f"{name}_roof_marker", (marker_x, marker_y, height + 0.045),
            (1.05 + 0.30 * (color_index % 3), 0.48, 0.09),
            color_material("0.91 0.88 0.68 1" if color_index % 2 else
                           "0.08 0.10 0.11 1"), yaw=yaw)
    records.append({"name": name, "center_east_north_m": [x, y],
                    "size_east_north_m": [length, width], "height_m": height,
                    "vehicle_type": "low_van" if is_van else "passenger_car",
                    "geometry": "three-dimensional_body_bonnet_glazed_cabin"})


def add_corner_landmark(parts: list[str], records: list[dict], block_id: int,
                        slot: str, x: float, y: float, material_index: int) -> None:
    name = f"block_{block_id:02d}_corner_kiosk_{slot}"
    size = (2.4, 2.4, 3.55)
    add_box(parts, f"{name}_shop", (x, y, size[2] / 2), size,
            scripted_material(FACADE_MATERIALS[material_index % len(FACADE_MATERIALS)]),
            collision=True)
    add_box(parts, f"{name}_canopy", (x, y, 3.68), (3.25, 3.25, 0.26),
            color_material(ROOF_COLORS[material_index % len(ROOF_COLORS)]))
    sign_offset = -1.28 if slot in ("southwest", "northwest") else 1.28
    add_box(parts, f"{name}_sign", (x + sign_offset, y, 2.85),
            (0.16, 2.05, 0.72), color_material(
                "0.88 0.72 0.19 1" if material_index % 2 else "0.18 0.49 0.65 1"))
    records.append({"name": name, "center_east_north_m": [x, y],
                    "size_east_north_m": [size[0], size[1]], "height_m": size[2],
                    "geometry": "textured_three-dimensional_corner_kiosk"})


def build_sdf() -> tuple[str, dict]:
    parts = ['<?xml version="1.0"?>', '<sdf version="1.6">',
             '<model name="vio_low_rise_city"><static>true</static><link name="city">']
    buildings: list[dict] = []
    trees: list[dict] = []
    parked_cars: list[dict] = []
    corner_landmarks: list[dict] = []
    curb_segments: list[dict] = []

    add_box(parts, "city_ground", (0, 0, -0.18), (410, 370, 0.35),
            color_material("0.31 0.34 0.29 1"), collision=True)

    # Continuous collision corridors, with short visual tiles so a 390 m road
    # does not stretch one bitmap into low-detail streaks.
    for index, x in enumerate(ROAD_X):
        add_collision(parts, f"road_vertical_{index}_collision", (x, 0, 0.035),
                      (ROAD_WIDTH, 350, 0.07))
        for tile, y in enumerate(range(-155, 166, 40)):
            add_box(parts, f"road_vertical_{index}_tile_{tile}", (x, y, 0.075),
                    (ROAD_WIDTH, 40.0, 0.08),
                    scripted_material("VioLowRiseCity/AsphaltNonperiodic"))
    for index, y in enumerate(ROAD_Y):
        add_collision(parts, f"road_horizontal_{index}_collision", (0, y, 0.035),
                      (390, ROAD_WIDTH, 0.07))
        for tile, x in enumerate([*range(-175, 146, 40), 175]):
            add_box(parts, f"road_horizontal_{index}_tile_{tile}", (x, y, 0.115),
                    (40.0, ROAD_WIDTH, 0.08),
                    scripted_material("VioLowRiseCity/AsphaltNonperiodic"))

    # Pavements and 96 street-facing 1--2 storey buildings fill all 16 blocks.
    block_id = 0
    for x_roads in BLOCK_X:
        for y_roads in BLOCK_Y:
            x0, x1 = x_roads[0] + ROAD_WIDTH / 2, x_roads[1] - ROAD_WIDTH / 2
            y0, y1 = y_roads[0] + ROAD_WIDTH / 2, y_roads[1] - ROAD_WIDTH / 2
            add_box(parts, f"block_{block_id:02d}_pavement",
                    ((x0 + x1) / 2, (y0 + y1) / 2, 0.12),
                    (x1 - x0, y1 - y0, 0.24),
                    color_material("0.58 0.57 0.53 1"), collision=True)
            # Raised kerbs bound every block face. Their long edges and short
            # vertical faces are ubiquitous city geometry and remain visible
            # between parked cars without becoming artificial VIO targets.
            curb_material = color_material("0.69 0.68 0.64 1")
            curb_specs = (
                ("south", (x0 + x1) / 2, y0, x1 - x0, 0.28),
                ("north", (x0 + x1) / 2, y1, x1 - x0, 0.28),
                ("west", x0, (y0 + y1) / 2, 0.28, y1 - y0),
                ("east", x1, (y0 + y1) / 2, 0.28, y1 - y0),
            )
            for curb_name, curb_x, curb_y, curb_sx, curb_sy in curb_specs:
                add_box(parts, f"block_{block_id:02d}_curb_{curb_name}",
                        (curb_x, curb_y, 0.30), (curb_sx, curb_sy, 0.36),
                        curb_material, collision=True)
                curb_segments.append({"block": block_id, "face": curb_name})
            # Two shopfronts on each long edge, plus one on each short edge.
            height_pattern = (4.4, 7.0, 6.8, 4.6, 7.2, 4.3)
            heights = tuple(height_pattern[(slot + block_id) % 6] for slot in range(6))
            left, right = x0 + 18.0, x1 - 18.0
            add_building(parts, buildings, block_id, 0, left, y1 - 7.3,
                         24.0, 11.6, heights[0], "north")
            add_building(parts, buildings, block_id, 1, right, y1 - 7.3,
                         24.0, 11.6, heights[1], "north")
            add_building(parts, buildings, block_id, 2, left, y0 + 7.3,
                         24.0, 11.6, heights[2], "south")
            add_building(parts, buildings, block_id, 3, right, y0 + 7.3,
                         24.0, 11.6, heights[3], "south")
            add_building(parts, buildings, block_id, 4, x0 + 7.2, (y0 + y1) / 2,
                         11.4, 17.0, heights[4], "west")
            add_building(parts, buildings, block_id, 5, x1 - 7.2, (y0 + y1) / 2,
                         11.4, 17.0, heights[5], "east")

            # A small paved courtyard with planters breaks up the block interior.
            add_box(parts, f"block_{block_id:02d}_courtyard",
                    ((x0 + x1) / 2, (y0 + y1) / 2, 0.27),
                    (20.0, 12.0, 0.06), color_material("0.43 0.42 0.38 1"))

            add_corner_landmark(parts, corner_landmarks, block_id, "southwest",
                                x0 + 4.0, y0 + 4.0, block_id)
            add_corner_landmark(parts, corner_landmarks, block_id, "southeast",
                                x1 - 4.0, y0 + 4.0, block_id + 1)
            add_corner_landmark(parts, corner_landmarks, block_id, "northwest",
                                x0 + 4.0, y1 - 4.0, block_id + 2)
            add_corner_landmark(parts, corner_landmarks, block_id, "northeast",
                                x1 - 4.0, y1 - 4.0, block_id + 3)

            tree_positions = (
                ((x0 + x1) / 2 - 12, y0 + 2.7),
                ((x0 + x1) / 2 + 12, y1 - 2.7),
                (x0 + 2.7, (y0 + y1) / 2 - 10),
                (x1 - 2.7, (y0 + y1) / 2 + 10),
            )
            for tree_slot, (tx, ty) in enumerate(tree_positions):
                species = (block_id + tree_slot) % 3
                tree_height = (6.2, 7.4, 6.7)[species] + 0.15 * (block_id % 3)
                add_tree(parts, trees, f"tree_{block_id:02d}_{tree_slot}",
                         tx, ty, tree_height, species)
            block_id += 1

    # Irregular groups of recognisable parked cars and low delivery vans line
    # the streets used by the new urban circuit. Gaps, vehicle types and roof
    # profiles vary as they do on an ordinary city block; no fiducials exist.
    horizontal_corridors = ((70.0, 0.0, 180.0),
                            (-140.0, -180.0, 180.0))
    vehicle_index = 0
    for north, east_start, east_end in horizontal_corridors:
        boundaries = [value for value in ROAD_X
                      if east_start <= value <= east_end]
        for left, right in zip(boundaries, boundaries[1:]):
            station_offsets = (9, 22, 39, 57, 74)
            for offset in station_offsets:
                if offset >= right - left:
                    continue
                east = left + offset
                road_index = ROAD_Y.index(north)
                for side in (-1, 1):
                    add_parked_car(
                        parts, parked_cars,
                        f"road_horizontal_{road_index}_depth_car_{vehicle_index:03d}",
                        east, north + side * 3.0, 0.0, vehicle_index)
                    vehicle_index += 1
    vertical_corridors = ((180.0, -140.0, 70.0),
                          (0.0, 0.0, 70.0),
                          (-180.0, -140.0, 70.0))
    for east, north_start, north_end in vertical_corridors:
        boundaries = [value for value in ROAD_Y
                      if north_start <= value <= north_end]
        for lower, upper in zip(boundaries, boundaries[1:]):
            station_offsets = (8, 21, 37, 54, 66)
            for offset in station_offsets:
                if offset >= upper - lower:
                    continue
                north = lower + offset
                road_index = ROAD_X.index(east)
                for side in (-1, 1):
                    add_parked_car(
                        parts, parked_cars,
                        f"road_vertical_{road_index}_depth_car_{vehicle_index:03d}",
                        east + side * 3.0, north, 1.5707963267948966, vehicle_index)
                    vehicle_index += 1

    # Normal lane paint and four-sided zebra crossings provide familiar urban
    # corners. They are deliberately ordinary road markings, never checkerboards,
    # ArUco tags or estimator-only hints.
    marking = color_material("0.92 0.90 0.74 1")
    for road_index, x in enumerate(ROAD_X):
        for mark_index, y in enumerate(range(-164, 165, 12)):
            if min(abs(y - road_y) for road_y in ROAD_Y) < 10:
                continue
            add_box(parts, f"vline_{road_index}_{mark_index}", (x, y, 0.125),
                    (0.20, 5.0, 0.025), marking)
    for road_index, y in enumerate(ROAD_Y):
        for mark_index, x in enumerate(range(-188, 189, 12)):
            if min(abs(x - road_x) for road_x in ROAD_X) < 10:
                continue
            add_box(parts, f"hline_{road_index}_{mark_index}", (x, y, 0.13),
                    (5.0, 0.20, 0.025), marking)
    crosswalk = color_material("0.89 0.89 0.86 1")
    for xi, x in enumerate(ROAD_X):
        for yi, y in enumerate(ROAD_Y):
            for approach, offset_x, offset_y, yaw in (
                    ("south", 0.0, -6.2, 0.0), ("north", 0.0, 6.2, 0.0),
                    ("west", -6.2, 0.0, math.pi / 2),
                    ("east", 6.2, 0.0, math.pi / 2)):
                for stripe in range(-2, 3):
                    lateral_x = stripe * 1.35 * math.cos(yaw)
                    lateral_y = stripe * 1.35 * math.sin(yaw)
                    add_box(parts, f"crosswalk_{xi}_{yi}_{approach}_{stripe + 2}",
                            (x + offset_x + lateral_x, y + offset_y + lateral_y, 0.145),
                            (0.76, 3.0, 0.025), crosswalk, yaw=yaw)

    parts.extend(['</link></model>', '</sdf>'])
    layout = {
        "schema": "project_f.vio_low_rise_city_layout.v1",
        "model": "vio_low_rise_city",
        "map_footprint_east_west_m": 390.0,
        "map_footprint_north_south_m": 350.0,
        "road_width_m": ROAD_WIDTH,
        "road_centers_east_m": list(ROAD_X),
        "road_centers_north_m": list(ROAD_Y),
        "block_count": 16,
        "building_count": len(buildings),
        "tree_count": len(trees),
        "parked_car_count": len(parked_cars),
        "corner_landmark_count": len(corner_landmarks),
        "curb_segment_count": len(curb_segments),
        "buildings": buildings,
        "trees": trees,
        "parked_cars": parked_cars,
        "corner_landmarks": corner_landmarks,
        "curb_segments": curb_segments,
    }
    return "\n".join(parts) + "\n", layout


def material_script() -> str:
    declarations = (
        ("FacadeBrickShop", "facade_brick_shop.png", False),
        ("FacadeBlueStone", "facade_blue_stone.png", False),
        ("FacadeTimberStore", "facade_timber_store.png", False),
        ("FacadeBakeryStucco", "facade_bakery_stucco.png", False),
        ("FacadeSageClinic", "facade_sage_clinic.png", False),
        ("FacadeOchreWorkshop", "facade_ochre_workshop.png", False),
        ("AsphaltNonperiodic", "asphalt_nonperiodic.png", False),
        ("RoofWeatheredMembrane", "roof_weathered_membrane_v2.png", False),
        ("TreeMaple", "tree_maple.png", True),
        ("TreeHornbeam", "tree_hornbeam.png", True),
        ("TreeCherry", "tree_cherry.png", True),
    )
    blocks = []
    for name, texture, alpha in declarations:
        alpha_lines = ("scene_blend alpha_blend\n      alpha_rejection greater_equal 64\n"
                       "      depth_write off\n      cull_hardware none\n      cull_software none\n      ") if alpha else ""
        blocks.append(
            f"material VioLowRiseCity/{name}\n{{\n  technique\n  {{\n    pass\n    {{\n"
            f"      ambient 0.78 0.78 0.78 1\n      diffuse 0.94 0.94 0.94 1\n      {alpha_lines}"
            f"texture_unit {{ texture {texture} filtering anisotropic max_anisotropy 8 }}\n"
            "    }\n  }\n}\n")
    return "\n".join(blocks)


def world_sdf(model_names: list[str]) -> str:
    includes = "\n".join(
        f"    <include><uri>model://{name}</uri></include>"
        for name in model_names)
    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <plugin name="vio_gps_control" filename="libvio_gps_control.so"/>
    <scene><ambient>0.76 0.76 0.76 1</ambient><background>0.43 0.54 0.65 1</background><shadows>true</shadows></scene>
    <include><uri>model://sun</uri></include>
{includes}
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>37.413534</latitude_deg><longitude_deg>-121.996561</longitude_deg><elevation>1.3</elevation><heading_deg>0</heading_deg></spherical_coordinates>
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -9.8066</gravity>
      <ode><solver><type>quick</type><iters>10</iters><sor>1.3</sor><use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling></solver><constraints><cfm>0</cfm><erp>0.2</erp><contact_max_correcting_vel>100</contact_max_correcting_vel><contact_surface_layer>0.001</contact_surface_layer></constraints></ode>
      <max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>250</real_time_update_rate><magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>
  </world>
</sdf>
'''


def partition_name(element_name: str) -> str:
    """Choose a small Gazebo model so the GUI never receives 1,000 visuals at once."""
    block = re.match(r"(?:block_|b|tree_)(\d{2})", element_name)
    if block:
        return f"vio_low_rise_city_block_{block.group(1)}"
    vertical = re.match(r"(?:road_vertical_|vline_)(\d+)", element_name)
    if vertical:
        return f"vio_low_rise_city_road_v_{vertical.group(1)}"
    horizontal = re.match(r"(?:road_horizontal_|hline_)(\d+)", element_name)
    if horizontal:
        return f"vio_low_rise_city_road_h_{horizontal.group(1)}"
    crosswalk = re.match(r"crosswalk_\d+_(\d+)_", element_name)
    if crosswalk:
        return f"vio_low_rise_city_road_h_{crosswalk.group(1)}"
    return "vio_low_rise_city"


def partition_sdf(monolithic: str) -> dict[str, str]:
    root = ET.fromstring(monolithic)
    link = root.find("./model/link")
    grouped: dict[str, list[str]] = {}
    for element in list(link):
        name = element.get("name", "")
        grouped.setdefault(partition_name(name), []).append(
            ET.tostring(element, encoding="unicode"))
    outputs = {}
    for model_name, elements in grouped.items():
        outputs[model_name] = (
            '<?xml version="1.0"?>\n<sdf version="1.6">\n'
            f'  <model name="{model_name}"><static>true</static><link name="geometry">\n'
            + "\n".join(elements) + "\n  </link></model>\n</sdf>\n")
    return outputs


def model_config(model_name: str) -> str:
    label = model_name.replace("_", " ").title()
    return (f'<?xml version="1.0"?>\n<model><name>{label}</name><version>1.0</version>'
            '<sdf version="1.6">model.sdf</sdf><author><name>project_f</name>'
            '<email>noreply@example.invalid</email></author><description>'
            'A deterministic partition of the project-owned full-size low-rise city.'
            '</description></model>\n')


def main() -> None:
    sdf, layout = build_sdf()
    partitions = partition_sdf(sdf)
    MODEL.mkdir(parents=True, exist_ok=True)
    (MODEL / "materials/scripts").mkdir(parents=True, exist_ok=True)
    for model_name, model_sdf in partitions.items():
        model_root = ROOT / "src/VIO_pkg/gazebo_models" / model_name
        model_root.mkdir(parents=True, exist_ok=True)
        (model_root / "model.sdf").write_text(model_sdf)
        (model_root / "model.config").write_text(model_config(model_name))
    (MODEL / "materials/scripts/vio_low_rise_city.material").write_text(material_script())
    WORLD.write_text(world_sdf(sorted(partitions)))
    assets = []
    for path in sorted(TEXTURES.glob("*.png")):
        assets.append({"path": str(path.relative_to(MODEL)), "sha256": sha256(path),
                       "source": "project-owned asphalt low-pass derivative" if
                       path.name == "asphalt_nonperiodic.png" else
                       "OpenAI built-in image generation" if path.name not in
                       ("asphalt_nonperiodic.png", "facade_blue_stone.png",
                        "facade_brick_shop.png", "facade_timber_store.png") else
                       "reused project-owned low-rise audition asset"})
    manifest = {"schema": "project_f.vio_low_rise_city_assets.v1", "assets": assets}
    (MODEL / "asset_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    output = ROOT / "artifacts/low_rise_city_20260823/design"
    output.mkdir(parents=True, exist_ok=True)
    (output / "layout.json").write_text(json.dumps(layout, indent=2, sort_keys=True) + "\n")
    print(json.dumps({key: layout[key] for key in (
        "map_footprint_east_west_m", "map_footprint_north_south_m",
        "block_count", "building_count", "tree_count")}
        | {"gazebo_model_partition_count": len(partitions)}, indent=2))


if __name__ == "__main__":
    main()
