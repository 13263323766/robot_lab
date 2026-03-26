# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Prepare a training URDF so MuJoCo can parse it more reliably."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

from sim2sim.asset_zoo.robots import get_robot_asset_cfg


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Sanitize a URDF for MuJoCo loading.")
    parser.add_argument("--input-urdf", type=str, required=True, help="Path to source URDF.")
    parser.add_argument("--output-urdf", type=str, required=True, help="Path to sanitized URDF.")
    parser.add_argument(
        "--mesh-root",
        type=str,
        default=None,
        help="Directory that package://.../meshes/ paths should resolve against. Defaults to ../meshes next to URDF.",
    )
    return parser


def prepare_urdf(input_urdf: str, output_urdf: str, mesh_root: str | None = None) -> Path:
    input_urdf = Path(input_urdf).expanduser().resolve()
    output_urdf = Path(output_urdf).expanduser().resolve()
    mesh_root = (
        Path(mesh_root).expanduser().resolve()
        if mesh_root is not None
        else input_urdf.parent.parent / "meshes"
    )

    root = ET.parse(input_urdf).getroot()

    for visual in root.iter("visual"):
        for material in list(visual.findall("material")):
            visual.remove(material)

    for mesh in root.iter("mesh"):
        filename = mesh.attrib.get("filename", "")
        if "/meshes/" in filename and filename.startswith("package://"):
            suffix = filename.split("/meshes/", 1)[1]
            mesh.attrib["filename"] = str(mesh_root / suffix)

    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(output_urdf, encoding="utf-8", xml_declaration=True)
    return output_urdf


def prepare_urdf_for_robot(robot_name: str, input_urdf: str | None = None, output_urdf: str | None = None) -> Path:
    asset_cfg = get_robot_asset_cfg(robot_name)
    return prepare_urdf(
        input_urdf=input_urdf or asset_cfg.paths.source_urdf,
        output_urdf=output_urdf or asset_cfg.paths.sanitized_urdf,
        mesh_root=asset_cfg.paths.mesh_root,
    )


def main() -> None:
    args = build_argparser().parse_args()
    print(prepare_urdf(args.input_urdf, args.output_urdf, args.mesh_root))


__all__ = ["main", "prepare_urdf_for_robot"]


if __name__ == "__main__":
    main()
