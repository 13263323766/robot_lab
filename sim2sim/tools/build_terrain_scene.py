#!/usr/bin/env python3
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Build a MuJoCo scene that uses an exported Isaac terrain as an hfield with Unitree Go2."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import trimesh


SCENE_TEMPLATE = """<mujoco model="go2 exported terrain scene">
  <include file="{go2_xml}"/>

  <statistic center="{stat_center_x} {stat_center_y} {stat_center_z}" extent="{stat_extent}"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="terrain_tex" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="terrain_mat" texture="terrain_tex" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <hfield name="isaac_terrain_hfield" size="{hfield_size_x} {hfield_size_y} {hfield_size_z} {hfield_size_base}" file="{hfield_png}"/>
  </asset>

  <worldbody>
    <light pos="0 0 3.0" dir="0 0 -1" directional="true"/>
    <geom
      name="isaac_terrain"
      type="hfield"
      hfield="isaac_terrain_hfield"
      material="terrain_mat"
      friction="{friction0} {friction1} {friction2}"
      condim="{condim}"
      pos="{terrain_pos_x} {terrain_pos_y} {terrain_pos_z}"
      quat="1 0 0 0"/>
  </worldbody>
</mujoco>
"""


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a MuJoCo scene XML using an exported Isaac terrain mesh.")
    parser.add_argument("--terrain-mesh", type=Path, required=True, help="Path to exported terrain mesh (.obj/.stl).")
    parser.add_argument("--output-xml", type=Path, required=True, help="Path to output MuJoCo scene XML.")
    parser.add_argument(
        "--go2-xml",
        type=Path,
        default=Path("/data2/sdam/unitree_mujoco/unitree_robots/go2/go2.xml"),
        help="Path to Unitree Go2 base XML to include.",
    )
    parser.add_argument("--friction", type=float, nargs=3, default=(0.8, 0.02, 0.01), help="Terrain friction triplet.")
    parser.add_argument("--condim", type=int, default=6, help="Terrain contact dimensionality.")
    parser.add_argument(
        "--crop-to-inner-terrain",
        action="store_true",
        default=True,
        help="Crop the large exported Isaac terrain border and keep only the actual sub-terrain area when metadata is available.",
    )
    parser.add_argument(
        "--grid-resolution",
        type=float,
        default=None,
        help="Optional hfield rasterization resolution in meters. Defaults to exported horizontal_scale or 0.1.",
    )
    return parser


def _load_sidecar_metadata(terrain_mesh: Path) -> dict | None:
    metadata_path = terrain_mesh.with_suffix(".json")
    if not metadata_path.exists():
        return None
    try:
        return json.loads(metadata_path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _write_resolved_go2_xml(go2_xml: Path, output_xml: Path) -> Path:
    go2_text = go2_xml.read_text(encoding="utf-8")
    asset_dir = (go2_xml.parent / "assets").resolve()

    go2_text = re.sub(r'<compiler[^>]*meshdir="[^"]*"[^>]*/>', '<compiler angle="radian" autolimits="true" />', go2_text)
    go2_text = re.sub(
        r'file="([^"]+)"',
        lambda m: f'file="{(asset_dir / m.group(1)).resolve().as_posix()}"',
        go2_text,
    )

    resolved_go2_xml = output_xml.with_name(f"{output_xml.stem}_go2_resolved.xml")
    resolved_go2_xml.write_text(go2_text, encoding="utf-8")
    return resolved_go2_xml


def _crop_to_inner_terrain(terrain_mesh: Path, output_xml: Path) -> tuple[Path, np.ndarray, dict | None]:
    metadata = _load_sidecar_metadata(terrain_mesh)
    mesh = trimesh.load(terrain_mesh, force="mesh")
    if metadata is None:
        return terrain_mesh, mesh.bounds, None

    size = metadata.get("size")
    num_rows = metadata.get("num_rows")
    num_cols = metadata.get("num_cols")
    if size is None or num_rows is None or num_cols is None:
        return terrain_mesh, mesh.bounds, metadata

    inner_x = float(size[0]) * int(num_rows)
    inner_y = float(size[1]) * int(num_cols)
    x_half = inner_x * 0.5
    y_half = inner_y * 0.5

    vertices = mesh.vertices
    faces = mesh.faces
    face_centers = vertices[faces].mean(axis=1)
    face_mask = (
        (face_centers[:, 0] >= -x_half)
        & (face_centers[:, 0] <= x_half)
        & (face_centers[:, 1] >= -y_half)
        & (face_centers[:, 1] <= y_half)
    )
    if not np.any(face_mask):
        return terrain_mesh, mesh.bounds, metadata

    cropped = mesh.submesh([face_mask], append=True, repair=True)
    cropped_path = output_xml.with_name(f"{output_xml.stem}_terrain_cropped.obj")
    cropped.export(cropped_path)
    return cropped_path, cropped.bounds, metadata


def _fill_nan_nearest(height_grid: np.ndarray) -> np.ndarray:
    grid = height_grid.copy()
    nan_mask = np.isnan(grid)
    if not nan_mask.any():
        return grid

    fill_value = np.nanmedian(grid)
    if np.isnan(fill_value):
        fill_value = 0.0
    grid[nan_mask] = fill_value

    for _ in range(12):
        nan_mask = np.isnan(grid)
        if not nan_mask.any():
            break
        padded = np.pad(grid, 1, mode="edge")
        neighbors = []
        for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            neighbors.append(padded[1 + dx : 1 + dx + grid.shape[0], 1 + dy : 1 + dy + grid.shape[1]])
        neighbor_stack = np.stack(neighbors, axis=0)
        grid[nan_mask] = np.nanmean(neighbor_stack[:, nan_mask], axis=0)
    grid = np.nan_to_num(grid, nan=fill_value)
    return grid


def _suppress_boundary_walls(height_grid: np.ndarray, border_cells: int = 4) -> np.ndarray:
    """Remove artificial cut-wall spikes near the cropped terrain boundary."""
    grid = height_grid.copy()
    if min(grid.shape) <= border_cells * 2:
        return grid
    grid[:border_cells, :] = grid[border_cells : border_cells + 1, :]
    grid[-border_cells:, :] = grid[-border_cells - 1 : -border_cells, :]
    grid[:, :border_cells] = grid[:, border_cells : border_cells + 1]
    grid[:, -border_cells:] = grid[:, -border_cells - 1 : -border_cells]
    return grid


def _mesh_to_hfield(
    terrain_mesh: Path,
    output_xml: Path,
    metadata: dict | None,
    bounds: np.ndarray,
    grid_resolution: float | None,
) -> tuple[Path, dict[str, float]]:
    mesh = trimesh.load(terrain_mesh, force="mesh")
    vertices = mesh.vertices
    x_min, y_min, z_min = bounds[0]
    x_max, y_max, z_max = bounds[1]

    resolution = (
        float(grid_resolution)
        if grid_resolution is not None
        else float((metadata or {}).get("horizontal_scale", 0.1))
    )
    ncol = max(2, int(round((x_max - x_min) / resolution)) + 1)
    nrow = max(2, int(round((y_max - y_min) / resolution)) + 1)

    x_idx = np.clip(np.round((vertices[:, 0] - x_min) / resolution).astype(int), 0, ncol - 1)
    y_idx = np.clip(np.round((vertices[:, 1] - y_min) / resolution).astype(int), 0, nrow - 1)
    height_grid = np.full((nrow, ncol), -np.inf, dtype=np.float32)
    np.maximum.at(height_grid, (y_idx, x_idx), vertices[:, 2].astype(np.float32))
    height_grid[np.isneginf(height_grid)] = np.nan
    height_grid = _fill_nan_nearest(height_grid)
    height_grid = _suppress_boundary_walls(height_grid, border_cells=max(2, int(round(0.5 / resolution))))

    z_range = max(1e-4, float(z_max - z_min))
    normalized = np.clip((height_grid - z_min) / z_range, 0.0, 1.0)
    image = np.flipud((normalized * 65535.0).astype(np.uint16))
    hfield_png = output_xml.with_name(f"{output_xml.stem}_terrain_hfield.png")
    imageio.imwrite(hfield_png, image)

    hfield_info = {
        "size_x": float((x_max - x_min) * 0.5),
        "size_y": float((y_max - y_min) * 0.5),
        "size_z": float(z_range),
        "size_base": max(0.001, float(abs(z_min))),
        "pos_x": float((x_min + x_max) * 0.5),
        "pos_y": float((y_min + y_max) * 0.5),
        "pos_z": float(z_min),
    }
    return hfield_png, hfield_info


def build_scene_xml(
    terrain_mesh: Path,
    output_xml: Path,
    go2_xml: Path,
    friction: tuple[float, float, float],
    condim: int,
    crop_to_inner_terrain: bool,
    grid_resolution: float | None,
) -> Path:
    terrain_mesh = terrain_mesh.expanduser().resolve()
    output_xml = output_xml.expanduser().resolve()
    go2_xml = go2_xml.expanduser().resolve()

    if not terrain_mesh.exists():
        raise FileNotFoundError(f"Terrain mesh not found: {terrain_mesh}")
    if not go2_xml.exists():
        raise FileNotFoundError(f"Go2 XML not found: {go2_xml}")

    output_xml.parent.mkdir(parents=True, exist_ok=True)
    resolved_go2_xml = _write_resolved_go2_xml(go2_xml, output_xml)
    if crop_to_inner_terrain:
        terrain_mesh_for_scene, terrain_bounds, metadata = _crop_to_inner_terrain(terrain_mesh, output_xml)
    else:
        metadata = _load_sidecar_metadata(terrain_mesh)
        terrain_mesh_for_scene = terrain_mesh
        terrain_bounds = trimesh.load(terrain_mesh, force="mesh").bounds

    hfield_png, hfield_info = _mesh_to_hfield(
        terrain_mesh_for_scene,
        output_xml,
        metadata,
        terrain_bounds,
        grid_resolution,
    )

    bounds_center = (terrain_bounds[0] + terrain_bounds[1]) * 0.5
    bounds_extents = terrain_bounds[1] - terrain_bounds[0]
    stat_extent = max(bounds_extents[0], bounds_extents[1]) * 0.55
    scene_text = SCENE_TEMPLATE.format(
        go2_xml=resolved_go2_xml.as_posix(),
        hfield_png=hfield_png.as_posix(),
        friction0=friction[0],
        friction1=friction[1],
        friction2=friction[2],
        condim=condim,
        stat_center_x=bounds_center[0],
        stat_center_y=bounds_center[1],
        stat_center_z=max(0.1, bounds_center[2]),
        stat_extent=max(4.0, stat_extent),
        hfield_size_x=hfield_info["size_x"],
        hfield_size_y=hfield_info["size_y"],
        hfield_size_z=hfield_info["size_z"],
        hfield_size_base=hfield_info["size_base"],
        terrain_pos_x=hfield_info["pos_x"],
        terrain_pos_y=hfield_info["pos_y"],
        terrain_pos_z=hfield_info["pos_z"],
    )
    output_xml.write_text(scene_text, encoding="utf-8")
    return output_xml


def main() -> None:
    args = build_argparser().parse_args()
    output_xml = build_scene_xml(
        args.terrain_mesh,
        args.output_xml,
        args.go2_xml,
        tuple(args.friction),
        args.condim,
        args.crop_to_inner_terrain,
        args.grid_resolution,
    )
    print(output_xml)


if __name__ == "__main__":
    main()
