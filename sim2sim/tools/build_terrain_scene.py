#!/usr/bin/env python3
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Build a MuJoCo scene that uses an exported Isaac terrain mesh with Unitree Go2."""

from __future__ import annotations

import argparse
from pathlib import Path


SCENE_TEMPLATE = """<mujoco model="go2 exported terrain scene">
  <include file="{go2_xml}"/>

  <statistic center="0 0 0.1" extent="4.0"/>

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
    <mesh name="isaac_terrain_mesh" file="{terrain_mesh}"/>
  </asset>

  <worldbody>
    <light pos="0 0 3.0" dir="0 0 -1" directional="true"/>
    <geom
      name="isaac_terrain"
      type="mesh"
      mesh="isaac_terrain_mesh"
      material="terrain_mat"
      friction="{friction0} {friction1} {friction2}"
      condim="{condim}"
      pos="0 0 0"
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
    return parser


def build_scene_xml(terrain_mesh: Path, output_xml: Path, go2_xml: Path, friction: tuple[float, float, float], condim: int) -> Path:
    terrain_mesh = terrain_mesh.expanduser().resolve()
    output_xml = output_xml.expanduser().resolve()
    go2_xml = go2_xml.expanduser().resolve()

    if not terrain_mesh.exists():
        raise FileNotFoundError(f"Terrain mesh not found: {terrain_mesh}")
    if not go2_xml.exists():
        raise FileNotFoundError(f"Go2 XML not found: {go2_xml}")

    output_xml.parent.mkdir(parents=True, exist_ok=True)
    scene_text = SCENE_TEMPLATE.format(
        go2_xml=go2_xml.as_posix(),
        terrain_mesh=terrain_mesh.as_posix(),
        friction0=friction[0],
        friction1=friction[1],
        friction2=friction[2],
        condim=condim,
    )
    output_xml.write_text(scene_text, encoding="utf-8")
    return output_xml


def main() -> None:
    args = build_argparser().parse_args()
    output_xml = build_scene_xml(args.terrain_mesh, args.output_xml, args.go2_xml, tuple(args.friction), args.condim)
    print(output_xml)


if __name__ == "__main__":
    main()
