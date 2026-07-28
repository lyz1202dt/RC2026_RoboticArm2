from __future__ import annotations

import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Mapping


INERTIAL_KEYS = ("m", "mx", "my", "mz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz")
INERTIA_KEYS = ("Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz")


def joint_child_link_map(urdf_path: str | Path) -> dict[str, str]:
    root = ET.parse(urdf_path).getroot()
    mapping: dict[str, str] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        child = joint.find("child")
        if name and child is not None and child.get("link"):
            mapping[name] = child.get("link", "")
    return mapping


def write_identified_urdf(
    input_urdf: str | Path,
    output_urdf: str | Path,
    parameter_dict: Mapping[str, float],
    joint_names: list[str],
) -> list[str]:
    tree = ET.parse(input_urdf)
    root = tree.getroot()
    link_by_name = {link.get("name"): link for link in root.findall("link")}
    child_by_joint = joint_child_link_map(input_urdf)

    updated_links: list[str] = []
    for joint_name in joint_names:
        if not all(f"{key}_{joint_name}" in parameter_dict for key in INERTIAL_KEYS):
            continue

        link_name = child_by_joint.get(joint_name)
        link = link_by_name.get(link_name)
        if link is None:
            continue

        p = {key: float(parameter_dict[f"{key}_{joint_name}"]) for key in INERTIAL_KEYS}
        mass = p["m"]
        if not math.isfinite(mass) or mass <= 0.0:
            continue

        xyz, inertia_at_com = dynamic_parameters_to_urdf_inertial(p)

        inertial = link.find("inertial")
        if inertial is None:
            inertial = ET.SubElement(link, "inertial")

        origin = inertial.find("origin")
        if origin is None:
            origin = ET.SubElement(inertial, "origin")
        origin.set("xyz", _fmt_vec(xyz))
        origin.set("rpy", origin.get("rpy", "0 0 0"))

        mass_elem = inertial.find("mass")
        if mass_elem is None:
            mass_elem = ET.SubElement(inertial, "mass")
        mass_elem.set("value", _fmt(mass))

        inertia = inertial.find("inertia")
        if inertia is None:
            inertia = ET.SubElement(inertial, "inertia")
        inertia.set("ixx", _fmt(inertia_at_com["Ixx"]))
        inertia.set("ixy", _fmt(inertia_at_com["Ixy"]))
        inertia.set("ixz", _fmt(inertia_at_com["Ixz"]))
        inertia.set("iyy", _fmt(inertia_at_com["Iyy"]))
        inertia.set("iyz", _fmt(inertia_at_com["Iyz"]))
        inertia.set("izz", _fmt(inertia_at_com["Izz"]))

        updated_links.append(link_name or joint_name)

    _indent(root)
    output_path = Path(output_urdf)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return updated_links


def dynamic_parameters_to_urdf_inertial(
    parameters: Mapping[str, float],
) -> tuple[tuple[float, float, float], dict[str, float]]:
    """Convert Pinocchio dynamic parameters to URDF inertial fields.

    Pinocchio dynamic parameters store the inertia about the link reference
    frame origin: I_O = I_C + m * S(c).T * S(c). URDF stores the inertia in
    the inertial frame, whose origin is the center of mass, so we subtract the
    parallel-axis term before writing the tensor.
    """
    mass = float(parameters["m"])
    if not math.isfinite(mass) or mass <= 0.0:
        raise ValueError(f"mass must be positive and finite, got {mass!r}")

    cx = float(parameters["mx"]) / mass
    cy = float(parameters["my"]) / mass
    cz = float(parameters["mz"]) / mass
    c2 = cx * cx + cy * cy + cz * cz

    inertia_at_origin = {key: float(parameters[key]) for key in INERTIA_KEYS}
    inertia_at_com = {
        "Ixx": inertia_at_origin["Ixx"] - mass * (c2 - cx * cx),
        "Ixy": inertia_at_origin["Ixy"] + mass * cx * cy,
        "Iyy": inertia_at_origin["Iyy"] - mass * (c2 - cy * cy),
        "Ixz": inertia_at_origin["Ixz"] + mass * cx * cz,
        "Iyz": inertia_at_origin["Iyz"] + mass * cy * cz,
        "Izz": inertia_at_origin["Izz"] - mass * (c2 - cz * cz),
    }
    return (cx, cy, cz), inertia_at_com


def _fmt(value: float) -> str:
    return f"{value:.9g}"


def _fmt_vec(values: tuple[float, float, float]) -> str:
    return " ".join(_fmt(value) for value in values)


def _indent(elem: ET.Element, level: int = 0) -> None:
    children = list(elem)
    if not children:
        return
    indent_text = "\n" + level * "  "
    child_indent = "\n" + (level + 1) * "  "
    if not elem.text or not elem.text.strip():
        elem.text = child_indent
    for child in children:
        _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = child_indent
    if not children[-1].tail or not children[-1].tail.strip():
        children[-1].tail = indent_text
