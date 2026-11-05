"""Read OpenLoong model parameters without importing Isaac or third-party packages.

All values are taken from the checked-out URDF/MJCF. MJCF ``size`` for a box
contains half extents; URDF ``size`` contains full extents. Joint armature is
already joint-side inertia and must NOT be multiplied by the motor gear again.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _numbers(text: str) -> list[float]:
    values = [float(value) for value in text.split()]
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"Nonfinite model value: {text}")
    return values


def _default_classes(root: ET.Element) -> dict[str, dict[str, dict[str, str]]]:
    """Resolve nested MJCF default classes, including inherited attributes."""
    result: dict[str, dict[str, dict[str, str]]] = {"main": {}}

    def visit(node, inherited, name):
        effective = {tag: dict(attrs) for tag, attrs in inherited.items()}
        for child in node:
            if child.tag != "default":
                effective.setdefault(child.tag, {}).update(child.attrib)
        result[name] = effective
        for child in node.findall("default"):
            visit(child, effective, child.get("class", name))

    for default in root.findall("default"):
        visit(default, result["main"], default.get("class", "main"))
    return result


def _mesh_path(filename: str, models: Path) -> Path:
    if filename.startswith("package://"):
        package, separator, relative = filename[len("package://"):].partition("/")
        if package not in ("AzureLoong", "AzureDragon") or not separator:
            raise ValueError(f"Unknown mesh package: {filename}")
        path = models / relative
    elif filename.startswith("file://"):
        path = Path(filename[len("file://"):])
    else:
        path = Path(filename)
        if not path.is_absolute():
            path = models / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Robot mesh does not exist: {path}")
    return path


def _orientation_rpy(attributes: dict[str, str]) -> list[float]:
    if "euler" in attributes:
        return _numbers(attributes["euler"])
    if "quat" in attributes:
        w, x, y, z = _numbers(attributes["quat"])
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        w, x, y, z = (v / norm for v in (w, x, y, z))
        return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
                math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]
    if "axisangle" in attributes or "xyaxes" in attributes or "zaxis" in attributes:
        raise ValueError("This model adapter requires Euler or quaternion geom orientation")
    return [0.0, 0.0, 0.0]


def read_model_metadata(repo_root: Path) -> dict:
    """Return SI-valued metadata keyed by exact URDF joint/link names."""
    repo_root = Path(repo_root).resolve()
    models = repo_root / "models"
    urdf_path, mjcf_path = models / "AzureLoong.urdf", models / "AzureLoong.xml"
    urdf, mjcf = ET.parse(urdf_path).getroot(), ET.parse(mjcf_path).getroot()
    compiler = mjcf.find("compiler")
    if compiler is not None and compiler.get("angle", "degree") != "radian":
        raise ValueError("The OpenLoong adapter expects a radian MJCF source")
    defaults = _default_classes(mjcf)
    joints: dict[str, dict] = {}
    feet: dict[str, dict] = {}
    mujoco_joints: dict[str, dict[str, str]] = {}

    def visit_body(body, inherited_class="main"):
        child_class = body.get("childclass", inherited_class)
        for joint in body.findall("joint"):
            attrs = dict(defaults[joint.get("class", child_class)].get("joint", {}))
            attrs.update(joint.attrib)
            mujoco_joints[attrs["name"]] = attrs
        if body.get("name") in ("Link_ankle_l_roll", "Link_ankle_r_roll"):
            collision_boxes = []
            for geom in body.findall("geom"):
                attrs = dict(defaults[geom.get("class", child_class)].get("geom", {}))
                attrs.update(geom.attrib)
                if attrs.get("type") == "box" and (int(attrs.get("contype", "1")) or int(attrs.get("conaffinity", "1"))):
                    collision_boxes.append(attrs)
            if len(collision_boxes) != 1:
                raise ValueError(f"Expected one MuJoCo foot collision box: {body.get('name')}")
            attrs = collision_boxes[0]
            half = _numbers(attrs["size"])
            feet[body.get("name")] = {
                "center": _numbers(attrs.get("pos", "0 0 0")),
                "rpy": _orientation_rpy(attrs), "half_extents": half,
                "size": [2*v for v in half],
                "friction": _numbers(attrs.get("friction", "1 0.005 0.0001")),
                "condim": int(attrs.get("condim", "3")),
            }
        for child in body.findall("body"):
            visit_body(child, child_class)

    for body in mjcf.findall("worldbody/body"):
        visit_body(body)
    actuators = {element.get("joint"): element for element in mjcf.findall("actuator/motor")}
    for joint in urdf.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        name = joint.attrib["name"]
        attrs = mujoco_joints[name]
        motor = actuators[name]
        gear = _numbers(motor.get("gear", "1"))
        if len(gear) != 1 or gear[0] == 0:
            raise ValueError(f"Only nonzero scalar motor gears are supported: {name}")
        control_range = _numbers(motor.attrib["ctrlrange"])
        limit = joint.find("limit")
        joints[name] = {
            "parent": joint.find("parent").attrib["link"],
            "child": joint.find("child").attrib["link"],
            "axis": _numbers(attrs.get("axis", "0 0 1")),
            "range": _numbers(attrs["range"]),
            "armature": float(attrs.get("armature", "0")),
            "damping": float(attrs.get("damping", "0")),
            "frictionloss": float(attrs.get("frictionloss", "0")),
            "gear": gear[0], "motor_control_range": control_range,
            "joint_effort_limit": max(abs(v * gear[0]) for v in control_range),
            "urdf_velocity_limit": float(limit.get("velocity")),
        }
    if len(joints) != 31 or set(joints) != set(mujoco_joints):
        raise ValueError("Expected exactly 31 matching URDF/MJCF actuated joints")
    if len(feet) != 2:
        raise ValueError("Expected both MJCF foot collision boxes")
    root_body = mjcf.find("worldbody/body")
    root_link = urdf.find(f"link[@name='{root_body.get('name')}']")
    option = mjcf.find("option")
    return {
        "schema_version": 1, "repo_root": str(repo_root),
        "source_urdf": str(urdf_path), "source_mjcf": str(mjcf_path),
        "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (urdf_path, mjcf_path)},
        "joint_names": list(joints), "joints": joints, "feet": feet,
        "base_link": root_body.attrib["name"],
        "base_position": _numbers(root_body.get("pos", "0 0 0")),
        "base_inertial_origin": _numbers(root_link.find("inertial/origin").get("xyz", "0 0 0")),
        "gravity": _numbers(option.get("gravity", "0 0 -9.81")),
        "physics_dt": float(option.get("timestep", "0.002")),
        "collision_excludes": [[e.attrib["body1"], e.attrib["body2"]] for e in mjcf.findall("contact/exclude")],
        "total_link_mass": sum(float(link.find("inertial/mass").get("value")) for link in urdf.findall("link")),
        "model_notes": [
            "Link masses and inertia tensors retain the controller URDF values.",
            "Joint ranges, armatures, damping, Coulomb friction and motor limits come from the simulation MJCF.",
            "PhysX and MuJoCo contact solvers differ; MuJoCo torsional/rolling friction has no direct material coefficient mapping.",
        ],
    }


def prepare_urdf(repo_root: Path, output_dir: Path) -> tuple[Path, dict]:
    """Write a generated URDF and model_metadata.json; never modify the source."""
    metadata = read_model_metadata(repo_root)
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    root = ET.parse(metadata["source_urdf"]).getroot()
    # A stable asset name and real mesh paths avoid either ROS package alias.
    root.set("name", "AzureLoong_Isaac")
    for element in list(root.findall("mujoco")):
        root.remove(element)
    for mesh in root.iter("mesh"):
        mesh.set("filename", str(_mesh_path(mesh.attrib["filename"], Path(repo_root) / "models")))
    for link in root.findall("link"):
        if link.attrib["name"] not in metadata["feet"]:
            continue
        foot = metadata["feet"][link.attrib["name"]]
        for collision in list(link.findall("collision")):
            link.remove(collision)
        collision = ET.SubElement(link, "collision", name=f"{link.attrib['name']}_sole")
        ET.SubElement(collision, "origin", xyz=" ".join(map(str, foot["center"])), rpy=" ".join(map(str, foot["rpy"])))
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "box", size=" ".join(map(str, foot["size"])))
    for joint in root.findall("joint"):
        if joint.attrib["name"] not in metadata["joints"]:
            continue
        values = metadata["joints"][joint.attrib["name"]]
        limit = joint.find("limit")
        limit.set("lower", str(values["range"][0]))
        limit.set("upper", str(values["range"][1]))
        limit.set("effort", str(values["joint_effort_limit"]))
        # Physical damping/friction are authored later through PhysxJointAxisAPI.
        # URDF importer drive damping must remain zero for external torque control.
        for dynamics in list(joint.findall("dynamics")):
            joint.remove(dynamics)
    output = output_dir / "AzureLoong_Isaac.urdf"
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    (output_dir / "model_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return output, metadata


def initial_base_height(repo_root: Path, joint_positions: dict[str, float],
                        clearance: float = 0.002) -> float:
    """Place the lowest sole-box corner ``clearance`` metres above the floor.

    The returned height is the world Z of the URDF ``base_link`` frame with an
    identity base orientation, not its centre of mass. Joint angles are radians
    and keyed by exact URDF joint names. Forward kinematics includes every joint
    origin, its RPY rotation, its axis, and its joint displacement. No simulator
    or NumPy dependency is required.
    """
    metadata = read_model_metadata(repo_root)
    if not math.isfinite(clearance) or clearance < 0:
        raise ValueError("Foot clearance must be finite and nonnegative")
    expected = set(metadata["joint_names"])
    if set(joint_positions) != expected:
        raise ValueError(f"Joint-position names differ from URDF: {set(joint_positions) ^ expected}")
    if not all(math.isfinite(float(q)) for q in joint_positions.values()):
        raise ValueError("Initial joint positions must be finite")

    def matmul(left, right):
        return [[sum(left[i][k] * right[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    def matvec(matrix, vector):
        return [sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)]

    def rpy_matrix(rpy):
        roll, pitch, yaw = rpy
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        sy, cy = math.sin(yaw), math.cos(yaw)
        # URDF fixed-axis roll/pitch/yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
        return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                [-sp, cp*sr, cp*cr]]

    def axis_rotation(axis, angle):
        norm = math.sqrt(sum(value*value for value in axis))
        if norm <= 0:
            raise ValueError("URDF joint axis has zero length")
        x, y, z = [value/norm for value in axis]
        c, s, v = math.cos(angle), math.sin(angle), 1-math.cos(angle)
        return [[x*x*v+c, x*y*v-z*s, x*z*v+y*s],
                [y*x*v+z*s, y*y*v+c, y*z*v-x*s],
                [z*x*v-y*s, z*y*v+x*s, z*z*v+c]]

    root = ET.parse(metadata["source_urdf"]).getroot()
    children: dict[str, list[ET.Element]] = {}
    for joint in root.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(joint)
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    transforms = {metadata["base_link"]: (identity, [0.0, 0.0, 0.0])}
    queue = [metadata["base_link"]]
    for parent in queue:
        parent_rotation, parent_position = transforms[parent]
        for joint in children.get(parent, []):
            origin = joint.find("origin")
            xyz = _numbers(origin.get("xyz", "0 0 0")) if origin is not None else [0.0]*3
            rpy = _numbers(origin.get("rpy", "0 0 0")) if origin is not None else [0.0]*3
            offset = matvec(parent_rotation, xyz)
            position = [parent_position[i] + offset[i] for i in range(3)]
            rotation = matmul(parent_rotation, rpy_matrix(rpy))
            joint_type = joint.get("type")
            if joint_type != "fixed":
                axis_element = joint.find("axis")
                axis = _numbers(axis_element.get("xyz", "1 0 0")) if axis_element is not None else [1.0, 0.0, 0.0]
                q = float(joint_positions[joint.get("name")])
                if joint_type in ("revolute", "continuous"):
                    rotation = matmul(rotation, axis_rotation(axis, q))
                elif joint_type == "prismatic":
                    displacement = matvec(rotation, [value*q for value in axis])
                    position = [position[i] + displacement[i] for i in range(3)]
                else:
                    raise ValueError(f"Unsupported FK joint type: {joint_type}")
            child = joint.find("child").get("link")
            if child in transforms:
                raise ValueError(f"URDF is not a tree at link {child}")
            transforms[child] = (rotation, position)
            queue.append(child)
    minimum = math.inf
    for name, foot in metadata["feet"].items():
        rotation, position = transforms[name]
        box_rotation = rpy_matrix(foot["rpy"])
        for signs in itertools.product((-1.0, 1.0), repeat=3):
            corner = matvec(box_rotation, [signs[i] * foot["half_extents"][i] for i in range(3)])
            corner = [corner[i] + foot["center"][i] for i in range(3)]
            corner = matvec(rotation, corner)
            minimum = min(minimum, position[2] + corner[2])
    return float(clearance - minimum)
