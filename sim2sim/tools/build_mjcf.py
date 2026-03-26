# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Build a floating-base MuJoCo MJCF wrapper from a compiled URDF import."""

from __future__ import annotations

import argparse
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

from sim2sim.asset_zoo.robots import get_robot_asset_cfg


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Wrap a compiled URDF import as a floating-base MJCF.")
    parser.add_argument(
        "--compiled-xml",
        type=str,
        required=True,
        help="Path to the MuJoCo XML produced from a URDF import.",
    )
    parser.add_argument(
        "--source-urdf",
        type=str,
        required=True,
        help="Path to the source URDF used to recover root inertial and joint effort limits.",
    )
    parser.add_argument("--output-xml", type=str, required=True, help="Path to the generated floating-base MJCF.")
    parser.add_argument(
        "--root-link",
        type=str,
        default="base",
        help="Name of the URDF root link that should become the floating base body.",
    )
    parser.add_argument(
        "--base-pos",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.35),
        help="Initial floating-base position written into the MJCF.",
    )
    parser.add_argument(
        "--timestep",
        type=float,
        default=0.005,
        help="MuJoCo simulation timestep to bake into the MJCF.",
    )
    parser.add_argument(
        "--floor-friction",
        type=float,
        nargs=3,
        default=(1.0, 0.02, 0.01),
        help="Floor friction tuple: sliding torsional rolling.",
    )
    parser.add_argument(
        "--foot-friction",
        type=float,
        nargs=3,
        default=(1.0, 0.02, 0.01),
        help="Foot collision friction tuple: sliding torsional rolling.",
    )
    parser.add_argument(
        "--joint-damping",
        type=float,
        default=0.1,
        help="Joint damping applied to all articulated joints in the generated model.",
    )
    parser.add_argument(
        "--feet-only-collision",
        action="store_true",
        help="Disable all robot collisions except foot spheres, inspired by mjlab's feet-only collision mode.",
    )
    parser.add_argument(
        "--uniform-effort-limit",
        type=float,
        default=None,
        help="Optional actuator ctrlrange limit applied uniformly to all joints.",
    )
    parser.add_argument(
        "--actuator-type",
        type=str,
        choices=("motor", "position"),
        default="motor",
        help="Actuator type to create in the generated MJCF.",
    )
    parser.add_argument(
        "--actuator-stiffness",
        type=float,
        default=25.0,
        help="Stiffness for generated position actuators.",
    )
    parser.add_argument(
        "--actuator-damping",
        type=float,
        default=0.5,
        help="Damping for generated position actuators.",
    )
    parser.add_argument(
        "--unitree-go1-like-armature",
        action="store_true",
        help="Apply mjlab-inspired Unitree leg armature values to hip/thigh/calf joints.",
    )
    return parser


def _find_root_inertial(urdf_root: ET.Element, root_link_name: str) -> ET.Element:
    for link in urdf_root.findall("link"):
        if link.attrib.get("name") == root_link_name:
            inertial = link.find("inertial")
            if inertial is None:
                raise ValueError(f"Root link '{root_link_name}' does not define <inertial> in the source URDF.")
            return inertial
    raise ValueError(f"Unable to find root link '{root_link_name}' in source URDF.")


def _parse_urdf_origin(element: ET.Element) -> tuple[str, str, str]:
    origin = element.find("origin")
    if origin is None:
        return ("0", "0", "0")
    return tuple(origin.attrib.get("xyz", "0 0 0").split())


def _build_inertial_from_urdf(urdf_inertial: ET.Element) -> ET.Element:
    mass = urdf_inertial.find("mass")
    inertia = urdf_inertial.find("inertia")
    if mass is None or inertia is None:
        raise ValueError("URDF root inertial is missing <mass> or <inertia>.")

    xyz = _parse_urdf_origin(urdf_inertial)
    attrs = {
        "pos": " ".join(xyz),
        "mass": mass.attrib["value"],
        "fullinertia": " ".join(
            [
                inertia.attrib["ixx"],
                inertia.attrib["iyy"],
                inertia.attrib["izz"],
                inertia.attrib["ixy"],
                inertia.attrib["ixz"],
                inertia.attrib["iyz"],
            ]
        ),
    }
    return ET.Element("inertial", attrs)


def _parse_joint_effort_limits(urdf_root: ET.Element) -> dict[str, float]:
    efforts: dict[str, float] = {}
    for joint in urdf_root.findall("joint"):
        joint_name = joint.attrib.get("name")
        limit = joint.find("limit")
        if joint_name is None or limit is None or "effort" not in limit.attrib:
            continue
        efforts[joint_name] = float(limit.attrib["effort"])
    return efforts


def _ensure_option(root: ET.Element, timestep: float) -> None:
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.attrib["timestep"] = f"{timestep:.6f}"
    option.attrib.setdefault("gravity", "0 0 -9.81")


def _set_joint_damping(worldbody: ET.Element, damping: float) -> list[str]:
    joint_names: list[str] = []
    for joint in worldbody.iter("joint"):
        name = joint.attrib.get("name")
        if name is None:
            continue
        joint.attrib["damping"] = f"{damping:.6f}"
        joint_names.append(name)
    return joint_names


def _apply_unitree_like_armature(base_body: ET.Element) -> None:
    for joint in base_body.iter("joint"):
        name = joint.attrib.get("name", "")
        if name.endswith("_hip_joint") or name.endswith("_thigh_joint"):
            joint.attrib["armature"] = "0.004026312"
        elif name.endswith("_calf_joint"):
            joint.attrib["armature"] = "0.009059202"


def _append_floor(worldbody: ET.Element, friction: tuple[float, float, float]) -> None:
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "size": "0 0 1",
            "pos": "0 0 0",
            "rgba": "0.9 0.9 0.9 1",
            "friction": f"{friction[0]} {friction[1]} {friction[2]}",
        },
    )


def _assign_geom_names_and_filter_collisions(
    base_body: ET.Element, feet_only_collision: bool, foot_friction: tuple[float, float, float]
) -> None:
    for body in base_body.iter("body"):
        body_name = body.attrib.get("name", "body")
        geom_index = 0
        for geom in body.findall("geom"):
            geom_index += 1
            geom_name = geom.attrib.get("name")
            if not geom_name:
                geom_name = f"{body_name}_geom_{geom_index}"
                geom.attrib["name"] = geom_name

            is_foot = (
                body_name.endswith("_calf")
                and geom.attrib.get("size") == "0.022"
                and geom.attrib.get("pos") == "-0.002 0 -0.213"
            )
            if is_foot:
                leg_name = body_name.replace("_calf", "")
                geom.attrib["name"] = f"{leg_name}_foot_collision"
                geom.attrib["contype"] = "1"
                geom.attrib["conaffinity"] = "1"
                geom.attrib["condim"] = "3"
                geom.attrib["friction"] = f"{foot_friction[0]} {foot_friction[1]} {foot_friction[2]}"
                continue

            if feet_only_collision:
                geom.attrib["contype"] = "0"
                geom.attrib["conaffinity"] = "0"


def build_mjcf_from_args(args: argparse.Namespace) -> Path:
    compiled_xml = Path(args.compiled_xml).expanduser().resolve()
    source_urdf = Path(args.source_urdf).expanduser().resolve()
    output_xml = Path(args.output_xml).expanduser().resolve()

    compiled_root = ET.parse(compiled_xml).getroot()
    urdf_root = ET.parse(source_urdf).getroot()

    root_inertial = _find_root_inertial(urdf_root, args.root_link)
    effort_limits = _parse_joint_effort_limits(urdf_root)

    generated_root = ET.Element("mujoco", {"model": compiled_root.attrib.get("model", args.root_link)})

    for child in list(compiled_root):
        if child.tag == "worldbody":
            continue
        generated_root.append(copy.deepcopy(child))
    _ensure_option(generated_root, args.timestep)

    worldbody = ET.SubElement(generated_root, "worldbody")
    _append_floor(worldbody, tuple(args.floor_friction))

    base_body = ET.SubElement(
        worldbody,
        "body",
        {"name": args.root_link, "pos": " ".join(map(str, args.base_pos))},
    )
    ET.SubElement(base_body, "freejoint", {"name": f"{args.root_link}_freejoint"})
    base_body.append(_build_inertial_from_urdf(root_inertial))

    compiled_worldbody = compiled_root.find("worldbody")
    if compiled_worldbody is None:
        raise ValueError("Compiled MuJoCo XML does not contain <worldbody>.")

    for child in list(compiled_worldbody):
        base_body.append(copy.deepcopy(child))

    _assign_geom_names_and_filter_collisions(base_body, args.feet_only_collision, tuple(args.foot_friction))
    if args.unitree_go1_like_armature:
        _apply_unitree_like_armature(base_body)
    joint_names = _set_joint_damping(base_body, args.joint_damping)
    actuator = ET.SubElement(generated_root, "actuator")
    for joint_name in joint_names:
        effort_limit = args.uniform_effort_limit if args.uniform_effort_limit is not None else effort_limits.get(joint_name)
        if args.actuator_type == "motor":
            attrs = {
                "name": f"{joint_name}_motor",
                "joint": joint_name,
                "gear": "1",
            }
            if effort_limit is not None:
                attrs["ctrllimited"] = "true"
                attrs["ctrlrange"] = f"{-effort_limit:.6f} {effort_limit:.6f}"
            ET.SubElement(actuator, "motor", attrs)
        else:
            attrs = {
                "name": f"{joint_name}_motor",
                "joint": joint_name,
                "kp": f"{args.actuator_stiffness:.6f}",
                "kv": f"{args.actuator_damping:.6f}",
                "ctrllimited": "false",
            }
            if effort_limit is not None:
                attrs["forcelimited"] = "true"
                attrs["forcerange"] = f"{-effort_limit:.6f} {effort_limit:.6f}"
            ET.SubElement(actuator, "position", attrs)

    output_xml.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(generated_root)
    ET.ElementTree(generated_root).write(output_xml, encoding="utf-8", xml_declaration=True)
    return output_xml


def build_mjcf_for_robot(
    robot_name: str,
    compiled_xml: str,
    source_urdf: str,
    output_xml: str | None = None,
    *,
    timestep: float = 0.005,
    joint_damping: float = 0.1,
    floor_friction: tuple[float, float, float] = (1.0, 0.02, 0.01),
    foot_friction: tuple[float, float, float] | None = None,
    feet_only_collision: bool = False,
    actuator_type: str | None = None,
    unitree_go1_like_armature: bool = True,
) -> Path:
    asset_cfg = get_robot_asset_cfg(robot_name)
    actuator_type = actuator_type or asset_cfg.preferred_actuator_type
    feet_only_collision = feet_only_collision or asset_cfg.preferred_collision_profile == "feet_only_collision"
    if foot_friction is None:
        foot_friction = asset_cfg.collision_profiles[0].friction
        for profile in asset_cfg.collision_profiles:
            if profile.name == asset_cfg.preferred_collision_profile:
                foot_friction = profile.friction
                break
    if output_xml is None:
        if actuator_type == "position" and feet_only_collision and asset_cfg.paths.feet_only_position_armature_xml:
            output_xml = asset_cfg.paths.feet_only_position_armature_xml
        elif actuator_type == "position" and feet_only_collision and asset_cfg.paths.feet_only_position_xml:
            output_xml = asset_cfg.paths.feet_only_position_xml
        elif feet_only_collision and asset_cfg.paths.feet_only_xml:
            output_xml = asset_cfg.paths.feet_only_xml
        else:
            output_xml = asset_cfg.paths.floating_xml
    args = argparse.Namespace(
        compiled_xml=compiled_xml,
        source_urdf=source_urdf,
        output_xml=output_xml,
        root_link="base",
        base_pos=tuple(float(v) for v in asset_cfg.default_base_pos.tolist()),
        timestep=timestep,
        floor_friction=floor_friction,
        foot_friction=foot_friction,
        joint_damping=joint_damping,
        feet_only_collision=feet_only_collision,
        uniform_effort_limit=None,
        actuator_type=actuator_type,
        actuator_stiffness=asset_cfg.position_actuators[0].stiffness,
        actuator_damping=asset_cfg.position_actuators[0].damping,
        unitree_go1_like_armature=unitree_go1_like_armature,
    )
    return build_mjcf_from_args(args)


def main() -> None:
    args = build_argparser().parse_args()
    print(build_mjcf_from_args(args))


__all__ = ["build_mjcf_for_robot", "main"]


if __name__ == "__main__":
    main()
