#!/usr/bin/env python3
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Build MuJoCo stair scenes for Go2 sim2sim evaluation.

Supports:
- `isaac-pyramid`: mirrors Isaac Lab's trimesh `pyramid_stairs_terrain` /
  `inverted_pyramid_stairs_terrain` geometry logic as closely as practical
  using MuJoCo-native box geoms.
- `linear`: legacy one-direction stair flight for quick manual tests.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


SCENE_TEMPLATE = """<mujoco model="go2 stair scene">
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
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <material name="stairs" rgba="0.45 0.47 0.50 1"/>
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
{floor_geom}
{geoms}
  </worldbody>
</mujoco>
"""


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build MuJoCo stair scenes for sim2sim.")
    parser.add_argument("--output-xml", type=Path, required=True, help="Output scene XML path.")
    parser.add_argument(
        "--go2-xml",
        type=Path,
        default=Path("/data2/sdam/unitree_mujoco/unitree_robots/go2/go2.xml"),
        help="Path to Unitree Go2 base XML to include.",
    )
    parser.add_argument(
        "--scene-type",
        type=str,
        default="isaac-pyramid",
        choices=["isaac-pyramid", "linear"],
        help="Scene family to generate.",
    )

    # Isaac-compatible stair params
    parser.add_argument("--difficulty", type=float, default=0.6, help="Difficulty in [0, 1].")
    parser.add_argument("--terrain-size-x", type=float, default=8.0, help="Isaac terrain cell size along x.")
    parser.add_argument("--terrain-size-y", type=float, default=8.0, help="Isaac terrain cell size along y.")
    parser.add_argument("--border-width", type=float, default=1.0, help="Isaac stair border width.")
    parser.add_argument("--platform-width", type=float, default=3.0, help="Central platform width.")
    parser.add_argument(
        "--step-height-min",
        type=float,
        default=0.05,
        help="Minimum stair height in Isaac step_height_range.",
    )
    parser.add_argument(
        "--step-height-max",
        type=float,
        default=0.23,
        help="Maximum stair height in Isaac step_height_range.",
    )
    parser.add_argument("--step-width", type=float, default=0.30, help="Step tread width.")
    parser.add_argument("--holes", action="store_true", help="Mirror Isaac holes=True variant.")
    parser.add_argument("--inverted", action="store_true", help="Build inverted pyramid stairs.")

    # Legacy linear stair params
    parser.add_argument("--num-steps", type=int, default=6, help="Number of steps for linear scene.")
    parser.add_argument("--stair-width", type=float, default=2.0, help="Linear stair width along y.")
    parser.add_argument("--start-x", type=float, default=1.8, help="Linear stair start x.")
    parser.add_argument("--descending", action="store_true", help="Linear descending stairs.")
    parser.add_argument("--platform-length", type=float, default=0.0, help="Optional platform length.")
    parser.add_argument("--platform-height", type=float, default=None, help="Optional platform height override.")
    parser.add_argument(
        "--with-ground-plane",
        action="store_true",
        help="Add MuJoCo's default infinite plane under the generated terrain.",
    )
    return parser


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


def _box_xml(name: str, center_x: float, center_y: float, half_x: float, half_y: float, half_z: float) -> str:
    return (
        f'    <geom name="{name}" pos="{center_x:.6f} {center_y:.6f} {half_z:.6f}" '
        f'type="box" size="{half_x:.6f} {half_y:.6f} {half_z:.6f}" '
        f'quat="1 0 0 0" material="stairs"/>\n'
    )


def _stair_height(difficulty: float, low: float, high: float) -> float:
    difficulty = min(max(difficulty, 0.0), 1.0)
    return low + difficulty * (high - low)


def _build_isaac_pyramid_geoms(
    size_x: float,
    size_y: float,
    border_width: float,
    platform_width: float,
    step_width: float,
    step_height: float,
    holes: bool,
    inverted: bool,
) -> tuple[list[str], tuple[float, float, float, float]]:
    terrain_center_x = 0.5 * size_x
    terrain_center_y = 0.5 * size_y
    terrain_size_x = size_x - 2.0 * border_width
    terrain_size_y = size_y - 2.0 * border_width
    if terrain_size_x <= 0 or terrain_size_y <= 0:
        raise ValueError("border_width too large for terrain size.")

    num_steps_x = math.floor((size_x - 2.0 * border_width - platform_width) / (2.0 * step_width)) + 1
    num_steps_y = math.floor((size_y - 2.0 * border_width - platform_width) / (2.0 * step_width)) + 1
    num_steps = int(min(num_steps_x, num_steps_y))
    if num_steps <= 0:
        raise ValueError("No valid pyramid steps for the given size/border/platform/step_width.")

    geoms: list[str] = []

    if border_width > 0.0 and not holes:
        border_half_z = step_height * 0.5
        inner_x = size_x - 2.0 * border_width
        inner_y = size_y - 2.0 * border_width
        top_strip_half_x = size_x * 0.5
        top_strip_half_y = border_width * 0.5
        side_strip_half_x = border_width * 0.5
        side_strip_half_y = inner_y * 0.5
        geoms.extend(
            [
                _box_xml("border_top", terrain_center_x, size_y - border_width * 0.5, top_strip_half_x, top_strip_half_y, border_half_z),
                _box_xml("border_bottom", terrain_center_x, border_width * 0.5, top_strip_half_x, top_strip_half_y, border_half_z),
                _box_xml("border_right", size_x - border_width * 0.5, terrain_center_y, side_strip_half_x, side_strip_half_y, border_half_z),
                _box_xml("border_left", border_width * 0.5, terrain_center_y, side_strip_half_x, side_strip_half_y, border_half_z),
            ]
        )

    total_height = (num_steps + 1) * step_height

    for k in range(num_steps):
        if holes:
            box_size_x = platform_width
            box_size_y = platform_width
        else:
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

        # top and bottom strips
        top_bottom_half_x = box_size_x * 0.5
        top_bottom_half_y = step_width * 0.5
        geoms.append(
            _box_xml(
                f"step_{k:02d}_top",
                terrain_center_x,
                terrain_center_y + terrain_size_y * 0.5 - box_offset,
                top_bottom_half_x,
                top_bottom_half_y,
                half_z,
            )
        )
        geoms.append(
            _box_xml(
                f"step_{k:02d}_bottom",
                terrain_center_x,
                terrain_center_y - terrain_size_y * 0.5 + box_offset,
                top_bottom_half_x,
                top_bottom_half_y,
                half_z,
            )
        )

        side_half_x = step_width * 0.5
        side_span_y = box_size_y if holes else (box_size_y - 2.0 * step_width)
        side_half_y = max(side_span_y * 0.5, 1e-4)
        geoms.append(
            _box_xml(
                f"step_{k:02d}_right",
                terrain_center_x + terrain_size_x * 0.5 - box_offset,
                terrain_center_y,
                side_half_x,
                side_half_y,
                half_z,
            )
        )
        geoms.append(
            _box_xml(
                f"step_{k:02d}_left",
                terrain_center_x - terrain_size_x * 0.5 + box_offset,
                terrain_center_y,
                side_half_x,
                side_half_y,
                half_z,
            )
        )

    middle_size_x = terrain_size_x - 2.0 * num_steps * step_width
    middle_size_y = terrain_size_y - 2.0 * num_steps * step_width
    if middle_size_x <= 0 or middle_size_y <= 0:
        raise ValueError("Center platform collapsed; decrease step_width or platform_width.")

    if inverted:
        middle_height = step_height
        middle_center_z = -total_height - step_height * 0.5
    else:
        middle_height = (num_steps + 2.0) * step_height
        middle_center_z = num_steps * step_height * 0.5

    geoms.append(
        _box_xml(
            "center_platform",
            terrain_center_x,
            terrain_center_y,
            middle_size_x * 0.5,
            middle_size_y * 0.5,
            middle_height * 0.5,
        ).replace(
            f'pos="{terrain_center_x:.6f} {terrain_center_y:.6f} {middle_height * 0.5:.6f}"',
            f'pos="{terrain_center_x:.6f} {terrain_center_y:.6f} {middle_center_z:.6f}"'
        )
    )

    max_top = total_height if not inverted else step_height * 0.5
    min_z = -total_height - step_height if inverted else 0.0
    center_z = 0.5 * (max_top + min_z)
    extent = max(size_x, size_y, total_height + 2.0)
    return geoms, (terrain_center_x, terrain_center_y, center_z, extent)


def _build_linear_geoms(
    step_height: float,
    step_width: float,
    num_steps: int,
    stair_width: float,
    start_x: float,
    descending: bool,
    platform_length: float,
    platform_height: float | None,
) -> tuple[list[str], tuple[float, float, float, float]]:
    half_y = stair_width * 0.5
    geoms: list[str] = []

    for i in range(num_steps):
        level = num_steps - i if descending else i + 1
        top_height = level * step_height
        geoms.append(
            _box_xml(
                name=f"stair_{i:02d}",
                center_x=start_x + (i + 0.5) * step_width,
                center_y=0.0,
                half_x=step_width * 0.5,
                half_y=half_y,
                half_z=top_height * 0.5,
            )
        )

    if platform_length > 0.0:
        top_level = 1 if descending else num_steps
        top_height = platform_height if platform_height is not None else top_level * step_height
        geoms.append(
            _box_xml(
                name="top_platform",
                center_x=start_x + num_steps * step_width + platform_length * 0.5,
                center_y=0.0,
                half_x=platform_length * 0.5,
                half_y=half_y,
                half_z=top_height * 0.5,
            )
        )

    max_top = max((1 if descending else num_steps) * step_height, 0.1)
    end_x = start_x + num_steps * step_width + platform_length
    center_x = 0.5 * (start_x + end_x)
    center_z = max_top * 0.5
    extent = max(3.0, (end_x - start_x) * 0.8)
    return geoms, (center_x, 0.0, center_z, extent)


def build_scene_xml(args: argparse.Namespace) -> Path:
    output_xml = args.output_xml.expanduser().resolve()
    go2_xml = args.go2_xml.expanduser().resolve()
    if not go2_xml.exists():
        raise FileNotFoundError(f"Go2 XML not found: {go2_xml}")

    output_xml.parent.mkdir(parents=True, exist_ok=True)
    resolved_go2_xml = _write_resolved_go2_xml(go2_xml, output_xml)

    if args.scene_type == "isaac-pyramid":
        step_height = _stair_height(args.difficulty, args.step_height_min, args.step_height_max)
        geoms, (stat_center_x, stat_center_y, stat_center_z, stat_extent) = _build_isaac_pyramid_geoms(
            size_x=args.terrain_size_x,
            size_y=args.terrain_size_y,
            border_width=args.border_width,
            platform_width=args.platform_width,
            step_width=args.step_width,
            step_height=step_height,
            holes=args.holes,
            inverted=args.inverted,
        )
    else:
        geoms, (stat_center_x, stat_center_y, stat_center_z, stat_extent) = _build_linear_geoms(
            step_height=args.step_height_max if args.platform_height is None else args.platform_height,
            step_width=args.step_width,
            num_steps=args.num_steps,
            stair_width=args.stair_width,
            start_x=args.start_x,
            descending=args.descending,
            platform_length=args.platform_length,
            platform_height=args.platform_height,
        )

    scene = SCENE_TEMPLATE.format(
        go2_xml=resolved_go2_xml.as_posix(),
        stat_center_x=f"{stat_center_x:.6f}",
        stat_center_y=f"{stat_center_y:.6f}",
        stat_center_z=f"{stat_center_z:.6f}",
        stat_extent=f"{stat_extent:.6f}",
        floor_geom='    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>\n'
        if args.with_ground_plane
        else "",
        geoms="".join(geoms),
    )
    output_xml.write_text(scene, encoding="utf-8")
    return output_xml


def main() -> None:
    args = build_argparser().parse_args()
    out = build_scene_xml(args)
    print(out)


if __name__ == "__main__":
    main()
