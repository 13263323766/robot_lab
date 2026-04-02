#!/usr/bin/env python3
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Build a MuJoCo scene that mimics Isaac Lab's tiled terrain generator layout.

Current support prioritizes the terrain families used by the Go2 target stairs-heavy training setup:

- pyramid_stairs
- pyramid_stairs_inv
- boxes
- random_rough
- hf_pyramid_slope
- hf_pyramid_slope_inv

The scene reproduces either:
- Isaac training-like tiled layout with curriculum-style column assignment
- Isaac play-like tiled layout, which in this repo currently overrides the terrain generator to:
  - `num_rows = 5`
  - `num_cols = 5`
  - `curriculum = False`

The geometry implementation is mixed:
- stairs / boxes are generated as MuJoCo-native box geoms
- random rough / pyramid slopes are generated as per-cell hfields
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image


SCENE_TEMPLATE = """<mujoco model="go2 isaac grid scene">
  <include file="{go2_xml}"/>

  <statistic center="{stat_center_x} {stat_center_y} {stat_center_z}" extent="{stat_extent}"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="12 12" reflectance="0.2"/>
    <material name="stairs" rgba="0.45 0.47 0.50 1"/>
    <material name="boxes" rgba="0.43 0.42 0.40 1"/>
    <material name="hfield_mat" rgba="0.48 0.50 0.53 1"/>
{assets}
  </asset>

  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" directional="true"/>
{floor_geom}
{geoms}
  </worldbody>
</mujoco>
"""


@dataclass(frozen=True)
class TerrainKindCfg:
    name: str
    proportion: float


def _write_resolved_go2_xml(go2_xml: Path, output_xml: Path) -> Path:
    text = go2_xml.read_text(encoding="utf-8")
    asset_dir = (go2_xml.parent / "assets").resolve()
    text = re.sub(r'<compiler[^>]*meshdir="[^"]*"[^>]*/>', '<compiler angle="radian" autolimits="true" />', text)
    text = re.sub(
        r'file="([^"]+)"',
        lambda m: f'file="{(asset_dir / m.group(1)).resolve().as_posix()}"',
        text,
    )
    resolved = output_xml.with_name(f"{output_xml.stem}_go2_resolved.xml")
    resolved.write_text(text, encoding="utf-8")
    return resolved


def _cell_center(row: int, col: int, num_rows: int, num_cols: int, size_x: float, size_y: float) -> tuple[float, float]:
    total_x = num_rows * size_x
    total_y = num_cols * size_y
    cx = (row + 0.5) * size_x - 0.5 * total_x
    cy = (col + 0.5) * size_y - 0.5 * total_y
    return cx, cy


def _column_assignment(num_cols: int, terrain_kinds: list[TerrainKindCfg]) -> list[str]:
    proportions = np.array([cfg.proportion for cfg in terrain_kinds], dtype=np.float64)
    proportions /= proportions.sum()
    cumulative = np.cumsum(proportions)
    names = [cfg.name for cfg in terrain_kinds]
    out: list[str] = []
    for index in range(num_cols):
        sub_index = int(np.min(np.where(index / num_cols + 0.001 < cumulative)[0]))
        out.append(names[sub_index])
    return out


def _random_column_assignment(rng: np.random.Generator, num_cols: int, terrain_kinds: list[TerrainKindCfg]) -> list[str]:
    proportions = np.array([cfg.proportion for cfg in terrain_kinds], dtype=np.float64)
    proportions /= proportions.sum()
    names = [cfg.name for cfg in terrain_kinds]
    indices = rng.choice(len(names), size=num_cols, p=proportions)
    return [names[int(i)] for i in indices]


def _difficulty_for_row(rng: np.random.Generator, row: int, num_rows: int, lower: float, upper: float) -> float:
    difficulty = (row + rng.uniform()) / num_rows
    return lower + (upper - lower) * difficulty


def _stair_height(difficulty: float, low: float, high: float) -> float:
    difficulty = min(max(difficulty, 0.0), 1.0)
    return low + difficulty * (high - low)


def _geom_box(
    name: str,
    center_x: float,
    center_y: float,
    center_z: float,
    half_x: float,
    half_y: float,
    half_z: float,
    material: str,
) -> str:
    return (
        f'    <geom name="{name}" pos="{center_x:.6f} {center_y:.6f} {center_z:.6f}" '
        f'type="box" size="{half_x:.6f} {half_y:.6f} {half_z:.6f}" material="{material}"/>\n'
    )


def _pyramid_stairs_cell(
    prefix: str,
    cell_cx: float,
    cell_cy: float,
    size_x: float,
    size_y: float,
    border_width: float,
    platform_width: float,
    step_width: float,
    step_height: float,
    inverted: bool,
) -> tuple[list[str], float]:
    terrain_size_x = size_x - 2.0 * border_width
    terrain_size_y = size_y - 2.0 * border_width
    num_steps_x = math.floor((size_x - 2.0 * border_width - platform_width) / (2.0 * step_width)) + 1
    num_steps_y = math.floor((size_y - 2.0 * border_width - platform_width) / (2.0 * step_width)) + 1
    num_steps = int(min(num_steps_x, num_steps_y))
    geoms: list[str] = []
    total_height = (num_steps + 1) * step_height

    if border_width > 0.0:
        border_half_z = step_height * 0.5
        inner_y = size_y - 2.0 * border_width
        inner_x = size_x - 2.0 * border_width
        geoms.extend(
            [
                _geom_box(f"{prefix}_border_top", cell_cx, cell_cy + size_y * 0.5 - border_width * 0.5, border_half_z, size_x * 0.5, border_width * 0.5, border_half_z, "stairs"),
                _geom_box(f"{prefix}_border_bottom", cell_cx, cell_cy - size_y * 0.5 + border_width * 0.5, border_half_z, size_x * 0.5, border_width * 0.5, border_half_z, "stairs"),
                _geom_box(f"{prefix}_border_right", cell_cx + size_x * 0.5 - border_width * 0.5, cell_cy, border_half_z, border_width * 0.5, inner_y * 0.5, border_half_z, "stairs"),
                _geom_box(f"{prefix}_border_left", cell_cx - size_x * 0.5 + border_width * 0.5, cell_cy, border_half_z, border_width * 0.5, inner_y * 0.5, border_half_z, "stairs"),
            ]
        )

    for k in range(num_steps):
        box_size_x = terrain_size_x - 2.0 * k * step_width
        box_size_y = terrain_size_y - 2.0 * k * step_width
        box_offset = (k + 0.5) * step_width
        if inverted:
            box_center_z = -total_height * 0.5 - (k + 1.0) * step_height * 0.5
            box_height = total_height - (k + 1.0) * step_height
        else:
            box_center_z = k * step_height * 0.5
            box_height = (k + 2.0) * step_height
        half_z = box_height * 0.5
        geoms.extend(
            [
                _geom_box(f"{prefix}_top_{k:02d}", cell_cx, cell_cy + terrain_size_y * 0.5 - box_offset, box_center_z, box_size_x * 0.5, step_width * 0.5, half_z, "stairs"),
                _geom_box(f"{prefix}_bottom_{k:02d}", cell_cx, cell_cy - terrain_size_y * 0.5 + box_offset, box_center_z, box_size_x * 0.5, step_width * 0.5, half_z, "stairs"),
                _geom_box(f"{prefix}_right_{k:02d}", cell_cx + terrain_size_x * 0.5 - box_offset, cell_cy, box_center_z, step_width * 0.5, max((box_size_y - 2.0 * step_width) * 0.5, 1e-4), half_z, "stairs"),
                _geom_box(f"{prefix}_left_{k:02d}", cell_cx - terrain_size_x * 0.5 + box_offset, cell_cy, box_center_z, step_width * 0.5, max((box_size_y - 2.0 * step_width) * 0.5, 1e-4), half_z, "stairs"),
            ]
        )

    middle_size_x = terrain_size_x - 2.0 * num_steps * step_width
    middle_size_y = terrain_size_y - 2.0 * num_steps * step_width
    if inverted:
        middle_height = step_height
        middle_center_z = -total_height - step_height * 0.5
        origin_z = -(num_steps + 1) * step_height
    else:
        middle_height = (num_steps + 2.0) * step_height
        middle_center_z = num_steps * step_height * 0.5
        origin_z = (num_steps + 1) * step_height
    geoms.append(
        _geom_box(
            f"{prefix}_center",
            cell_cx,
            cell_cy,
            middle_center_z,
            middle_size_x * 0.5,
            middle_size_y * 0.5,
            middle_height * 0.5,
            "stairs",
        )
    )
    return geoms, origin_z


def _boxes_cell(
    rng: np.random.Generator,
    prefix: str,
    cell_cx: float,
    cell_cy: float,
    size: float,
    difficulty: float,
    grid_width: float,
    grid_height_low: float,
    grid_height_high: float,
    platform_width: float,
) -> tuple[list[str], float]:
    grid_height = grid_height_low + difficulty * (grid_height_high - grid_height_low)
    num_boxes = int(size / grid_width)
    border_width = size - min(num_boxes, num_boxes) * grid_width
    terrain_height = 1.0
    geoms: list[str] = []

    if border_width <= 0:
        raise ValueError("grid_width too large for boxes terrain.")

    border_half_z = terrain_height * 0.5
    inner_size = size - border_width
    geoms.extend(
        [
            _geom_box(f"{prefix}_border_top", cell_cx, cell_cy + size * 0.5 - border_width * 0.25, 0.0, size * 0.5, border_width * 0.25, border_half_z, "boxes"),
            _geom_box(f"{prefix}_border_bottom", cell_cx, cell_cy - size * 0.5 + border_width * 0.25, 0.0, size * 0.5, border_width * 0.25, border_half_z, "boxes"),
            _geom_box(f"{prefix}_border_right", cell_cx + size * 0.5 - border_width * 0.25, cell_cy, 0.0, border_width * 0.25, inner_size * 0.5, border_half_z, "boxes"),
            _geom_box(f"{prefix}_border_left", cell_cx - size * 0.5 + border_width * 0.25, cell_cy, 0.0, border_width * 0.25, inner_size * 0.5, border_half_z, "boxes"),
        ]
    )

    start_x = cell_cx - size * 0.5 + border_width * 0.5
    start_y = cell_cy - size * 0.5 + border_width * 0.5
    for ix in range(num_boxes):
        for iy in range(num_boxes):
            h_noise = rng.uniform(-grid_height, grid_height)
            center_z = (-terrain_height + h_noise) * 0.5
            half_z = (terrain_height + h_noise) * 0.5
            geoms.append(
                _geom_box(
                    f"{prefix}_box_{ix:02d}_{iy:02d}",
                    start_x + (ix + 0.5) * grid_width,
                    start_y + (iy + 0.5) * grid_width,
                    center_z,
                    grid_width * 0.5,
                    grid_width * 0.5,
                    max(half_z, 1e-4),
                    "boxes",
                )
            )

    geoms.append(
        _geom_box(
            f"{prefix}_platform",
            cell_cx,
            cell_cy,
            (-terrain_height + grid_height) * 0.5,
            platform_width * 0.5,
            platform_width * 0.5,
            (terrain_height + grid_height) * 0.5,
            "boxes",
        )
    )
    return geoms, grid_height


def _hf_random_uniform(size_x: float, size_y: float, horizontal_scale: float, vertical_scale: float, noise_low: float, noise_high: float, noise_step: float, rng: np.random.Generator) -> np.ndarray:
    width_pixels = int(size_x / horizontal_scale)
    length_pixels = int(size_y / horizontal_scale)
    height_min = int(noise_low / vertical_scale)
    height_max = int(noise_high / vertical_scale)
    height_step = max(1, int(noise_step / vertical_scale))
    height_range = np.arange(height_min, height_max + height_step, height_step, dtype=np.int32)
    height_field = rng.choice(height_range, size=(width_pixels, length_pixels))
    return height_field.astype(np.int16) * vertical_scale


def _hf_pyramid_slope(size_x: float, size_y: float, horizontal_scale: float, vertical_scale: float, slope_low: float, slope_high: float, platform_width: float, difficulty: float, inverted: bool) -> np.ndarray:
    slope = slope_low + difficulty * (slope_high - slope_low)
    if inverted:
        slope = -slope
    width_pixels = int(size_x / horizontal_scale)
    length_pixels = int(size_y / horizontal_scale)
    height_max = int(slope * size_x / 2.0 / vertical_scale)
    center_x = int(width_pixels / 2)
    center_y = int(length_pixels / 2)
    x = np.arange(0, width_pixels)
    y = np.arange(0, length_pixels)
    xx, yy = np.meshgrid(x, y, sparse=True)
    xx = (center_x - np.abs(center_x - xx)) / center_x
    yy = (center_y - np.abs(center_y - yy)) / center_y
    xx = xx.reshape(width_pixels, 1)
    yy = yy.reshape(1, length_pixels)
    hf_raw = height_max * xx * yy
    platform_half = int(platform_width / horizontal_scale / 2)
    x_pf = width_pixels // 2 - platform_half
    y_pf = length_pixels // 2 - platform_half
    z_pf = hf_raw[x_pf, y_pf]
    hf_raw = np.clip(hf_raw, min(0, z_pf), max(0, z_pf))
    return np.rint(hf_raw).astype(np.int16) * vertical_scale


def _write_hfield_asset(asset_dir: Path, name: str, heights_m: np.ndarray) -> tuple[Path, float, float]:
    asset_dir.mkdir(parents=True, exist_ok=True)
    min_h = float(np.min(heights_m))
    max_h = float(np.max(heights_m))
    span = max(max_h - min_h, 1e-6)
    normalized = ((heights_m - min_h) / span * 65535.0).astype(np.uint16)
    img = Image.fromarray(normalized, mode="I;16")
    path = asset_dir / f"{name}.png"
    img.save(path)
    # MuJoCo requires all hfield size components to be strictly positive.
    # In our use case many cells start exactly at z=0, so `base` would otherwise
    # serialize to 0.000000 and fail model compilation.
    base = max(-min_h, 1e-4)
    ztop = max(span, 1e-4)
    return path, ztop, base


def _hfield_cell(
    asset_dir: Path,
    prefix: str,
    cell_cx: float,
    cell_cy: float,
    size_x: float,
    size_y: float,
    heights_m: np.ndarray,
    output_xml_dir: Path,
) -> tuple[str, str, float]:
    img_path, ztop, base = _write_hfield_asset(asset_dir, prefix, heights_m)
    rel = img_path.relative_to(output_xml_dir).as_posix()
    asset_xml = (
        f'    <hfield name="{prefix}_hf" size="{size_x * 0.5:.6f} {size_y * 0.5:.6f} {ztop:.6f} {base:.6f}" '
        f'file="{rel}"/>\n'
    )
    geom_xml = (
        f'    <geom name="{prefix}_geom" type="hfield" hfield="{prefix}_hf" '
        f'pos="{cell_cx:.6f} {cell_cy:.6f} 0.0" material="hfield_mat"/>\n'
    )
    origin_z = float(np.max(heights_m))
    return asset_xml, geom_xml, origin_z


def _preset_kinds(preset: str) -> list[TerrainKindCfg]:
    if preset == "stairs-heavy":
        return [
            TerrainKindCfg("pyramid_stairs", 0.35),
            TerrainKindCfg("pyramid_stairs_inv", 0.35),
            TerrainKindCfg("boxes", 0.10),
            TerrainKindCfg("random_rough", 0.10),
            TerrainKindCfg("hf_pyramid_slope", 0.05),
            TerrainKindCfg("hf_pyramid_slope_inv", 0.05),
        ]
    return [
        TerrainKindCfg("pyramid_stairs", 0.20),
        TerrainKindCfg("pyramid_stairs_inv", 0.20),
        TerrainKindCfg("boxes", 0.20),
        TerrainKindCfg("random_rough", 0.20),
        TerrainKindCfg("hf_pyramid_slope", 0.10),
        TerrainKindCfg("hf_pyramid_slope_inv", 0.10),
    ]


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build an Isaac-like tiled MuJoCo terrain scene.")
    parser.add_argument("--output-xml", type=Path, required=True)
    parser.add_argument(
        "--go2-xml",
        type=Path,
        default=Path("/data2/sdam/unitree_mujoco/unitree_robots/go2/go2.xml"),
    )
    parser.add_argument("--preset", choices=["rough", "stairs-heavy"], default="stairs-heavy")
    parser.add_argument(
        "--layout-mode",
        choices=["play", "train"],
        default="play",
        help="Match Isaac play overrides (`5x5`, no curriculum) or training-style tiled curriculum layout.",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--num-rows", type=int, default=None)
    parser.add_argument("--num-cols", type=int, default=None)
    parser.add_argument("--terrain-size-x", type=float, default=8.0)
    parser.add_argument("--terrain-size-y", type=float, default=8.0)
    parser.add_argument("--difficulty-low", type=float, default=0.0)
    parser.add_argument("--difficulty-high", type=float, default=1.0)

    parser.add_argument("--stairs-border-width", type=float, default=1.0)
    parser.add_argument("--stairs-platform-width", type=float, default=3.0)
    parser.add_argument("--stairs-step-width", type=float, default=0.30)
    parser.add_argument("--stairs-step-height-min", type=float, default=0.05)
    parser.add_argument("--stairs-step-height-max", type=float, default=0.23)

    parser.add_argument("--boxes-grid-width", type=float, default=0.45)
    parser.add_argument("--boxes-grid-height-min", type=float, default=0.05)
    parser.add_argument("--boxes-grid-height-max", type=float, default=0.20)
    parser.add_argument("--boxes-platform-width", type=float, default=2.0)

    parser.add_argument("--hf-horizontal-scale", type=float, default=0.1)
    parser.add_argument("--hf-vertical-scale", type=float, default=0.005)
    parser.add_argument("--rough-noise-min", type=float, default=0.02)
    parser.add_argument("--rough-noise-max", type=float, default=0.10)
    parser.add_argument("--rough-noise-step", type=float, default=0.02)
    parser.add_argument("--slope-min", type=float, default=0.0)
    parser.add_argument("--slope-max", type=float, default=0.4)
    parser.add_argument("--slope-platform-width", type=float, default=2.0)
    parser.add_argument(
        "--with-ground-plane",
        action="store_true",
        help="Add MuJoCo's default infinite plane under the generated terrain grid.",
    )
    return parser


def build_scene(args: argparse.Namespace) -> Path:
    output_xml = args.output_xml.expanduser().resolve()
    go2_xml = args.go2_xml.expanduser().resolve()
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    resolved_go2 = _write_resolved_go2_xml(go2_xml, output_xml)
    asset_dir = output_xml.with_name(f"{output_xml.stem}_assets")

    rng = np.random.default_rng(args.seed)
    terrain_kinds = _preset_kinds(args.preset)
    num_rows = args.num_rows if args.num_rows is not None else (5 if args.layout_mode == "play" else 10)
    num_cols = args.num_cols if args.num_cols is not None else (5 if args.layout_mode == "play" else 20)
    if args.layout_mode == "play":
        # Mirror the local play.py behavior: shrink the terrain grid and disable curriculum.
        col_kinds = _random_column_assignment(rng, num_cols, terrain_kinds)
    else:
        col_kinds = _column_assignment(num_cols, terrain_kinds)

    geoms: list[str] = []
    assets: list[str] = []
    max_origin_z = 0.0

    for col in range(num_cols):
        kind = col_kinds[col]
        for row in range(num_rows):
            if args.layout_mode == "play":
                difficulty = float(rng.uniform(args.difficulty_low, args.difficulty_high))
            else:
                difficulty = _difficulty_for_row(rng, row, num_rows, args.difficulty_low, args.difficulty_high)
            cell_cx, cell_cy = _cell_center(row, col, num_rows, num_cols, args.terrain_size_x, args.terrain_size_y)
            prefix = f"r{row:02d}_c{col:02d}_{kind}"

            if kind == "pyramid_stairs":
                step_height = _stair_height(difficulty, args.stairs_step_height_min, args.stairs_step_height_max)
                cell_geoms, origin_z = _pyramid_stairs_cell(
                    prefix, cell_cx, cell_cy, args.terrain_size_x, args.terrain_size_y,
                    args.stairs_border_width, args.stairs_platform_width, args.stairs_step_width,
                    step_height, False,
                )
                geoms.extend(cell_geoms)
            elif kind == "pyramid_stairs_inv":
                step_height = _stair_height(difficulty, args.stairs_step_height_min, args.stairs_step_height_max)
                cell_geoms, origin_z = _pyramid_stairs_cell(
                    prefix, cell_cx, cell_cy, args.terrain_size_x, args.terrain_size_y,
                    args.stairs_border_width, args.stairs_platform_width, args.stairs_step_width,
                    step_height, True,
                )
                geoms.extend(cell_geoms)
            elif kind == "boxes":
                cell_geoms, origin_z = _boxes_cell(
                    rng, prefix, cell_cx, cell_cy, min(args.terrain_size_x, args.terrain_size_y),
                    difficulty, args.boxes_grid_width, args.boxes_grid_height_min, args.boxes_grid_height_max,
                    args.boxes_platform_width,
                )
                geoms.extend(cell_geoms)
            elif kind == "random_rough":
                heights = _hf_random_uniform(
                    args.terrain_size_x, args.terrain_size_y, args.hf_horizontal_scale, args.hf_vertical_scale,
                    args.rough_noise_min, args.rough_noise_max, args.rough_noise_step, rng,
                )
                asset_xml, geom_xml, origin_z = _hfield_cell(asset_dir, prefix, cell_cx, cell_cy, args.terrain_size_x, args.terrain_size_y, heights, output_xml.parent)
                assets.append(asset_xml)
                geoms.append(geom_xml)
            elif kind == "hf_pyramid_slope":
                heights = _hf_pyramid_slope(
                    args.terrain_size_x, args.terrain_size_y, args.hf_horizontal_scale, args.hf_vertical_scale,
                    args.slope_min, args.slope_max, args.slope_platform_width, difficulty, False,
                )
                asset_xml, geom_xml, origin_z = _hfield_cell(asset_dir, prefix, cell_cx, cell_cy, args.terrain_size_x, args.terrain_size_y, heights, output_xml.parent)
                assets.append(asset_xml)
                geoms.append(geom_xml)
            elif kind == "hf_pyramid_slope_inv":
                heights = _hf_pyramid_slope(
                    args.terrain_size_x, args.terrain_size_y, args.hf_horizontal_scale, args.hf_vertical_scale,
                    args.slope_min, args.slope_max, args.slope_platform_width, difficulty, True,
                )
                asset_xml, geom_xml, origin_z = _hfield_cell(asset_dir, prefix, cell_cx, cell_cy, args.terrain_size_x, args.terrain_size_y, heights, output_xml.parent)
                assets.append(asset_xml)
                geoms.append(geom_xml)
            else:
                raise ValueError(f"Unsupported terrain kind: {kind}")

            max_origin_z = max(max_origin_z, origin_z)

    total_x = num_rows * args.terrain_size_x
    total_y = num_cols * args.terrain_size_y
    stat_center_x = 0.0
    stat_center_y = 0.0
    stat_center_z = max_origin_z * 0.5
    stat_extent = max(total_x, total_y, max_origin_z + 4.0) * 0.6

    scene = SCENE_TEMPLATE.format(
        go2_xml=resolved_go2.as_posix(),
        stat_center_x=f"{stat_center_x:.6f}",
        stat_center_y=f"{stat_center_y:.6f}",
        stat_center_z=f"{stat_center_z:.6f}",
        stat_extent=f"{stat_extent:.6f}",
        floor_geom='    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>\n'
        if args.with_ground_plane
        else "",
        assets="".join(assets),
        geoms="".join(geoms),
    )
    output_xml.write_text(scene, encoding="utf-8")
    return output_xml


def main() -> None:
    args = build_argparser().parse_args()
    out = build_scene(args)
    print(out)


if __name__ == "__main__":
    main()
