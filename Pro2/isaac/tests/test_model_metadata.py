"""Model migration invariants runnable without the Isaac runtime."""

from pathlib import Path
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET

from Pro2.isaac.model_metadata import initial_base_height, prepare_urdf, read_model_metadata


REPO = Path(__file__).resolve().parents[2] / "OpenLoong-Dyn-Control"


class ModelMigrationTest(unittest.TestCase):
    def test_initial_height_uses_lowest_rotated_sole_corner(self):
        metadata = read_model_metadata(REPO)
        positions = {name: 0.0 for name in metadata["joint_names"]}
        # Both ankle frames are at z=-1.0695 m in the straight-legged URDF.
        # The left box extends lower, so it determines zero-pose clearance.
        self.assertAlmostEqual(initial_base_height(REPO, positions), 1.1429239876674935)
        positions["J_ankle_l_roll"] = 0.5
        foot = metadata["feet"]["Link_ankle_l_roll"]
        c, s = math.cos(0.5), math.sin(0.5)
        left_bottom = -1.0695 + s*foot["center"][1] + c*foot["center"][2] - s*foot["half_extents"][1] - c*foot["half_extents"][2]
        right = metadata["feet"]["Link_ankle_r_roll"]
        right_bottom = -1.0695 + right["center"][2] - right["half_extents"][2]
        self.assertAlmostEqual(initial_base_height(REPO, positions), 0.002-min(left_bottom, right_bottom))
        self.assertAlmostEqual(initial_base_height(REPO, positions, 0.005), 0.005-min(left_bottom, right_bottom))
        with self.assertRaises(ValueError):
            initial_base_height(REPO, {}, 0.002)
        with self.assertRaises(ValueError):
            initial_base_height(REPO, positions, -0.001)

    def test_actual_motor_gears_and_inherited_passive_parameters(self):
        metadata = read_model_metadata(REPO)
        self.assertEqual(len(metadata["joint_names"]), 31)
        waist = metadata["joints"]["J_waist_pitch"]
        self.assertEqual(waist["gear"], 81.0)
        self.assertAlmostEqual(waist["joint_effort_limit"], 315.0009)
        # Armature is already joint-side inertia, independent of actuator gear.
        self.assertEqual(waist["armature"], 0.00008)
        arm = metadata["joints"]["J_arm_l_01"]
        self.assertEqual(arm["armature"], 0.01)
        self.assertEqual(arm["damping"], 0.1)
        self.assertEqual(arm["frictionloss"], 2.0)
        self.assertAlmostEqual(metadata["total_link_mass"], 77.35258437854651)

    def test_mesh_resolution_foot_geometry_and_source_immutability(self):
        source = REPO / "models/AzureLoong.urdf"
        source_bytes = source.read_bytes()
        original = ET.fromstring(source_bytes)
        with tempfile.TemporaryDirectory() as directory:
            output, metadata = prepare_urdf(REPO, Path(directory))
            generated = ET.parse(output).getroot()
            original_joints = [j.get("name") for j in original.findall("joint")]
            self.assertEqual([j.get("name") for j in generated.findall("joint")], original_joints)
            self.assertEqual(len(generated.findall("joint[@type='fixed']")), 0)
            for mesh in generated.iter("mesh"):
                self.assertTrue(Path(mesh.get("filename")).is_file())
                self.assertTrue(Path(mesh.get("filename")).is_absolute())
            for link in generated.findall("link"):
                original_link = original.find(f"link[@name='{link.get('name')}']")
                # Conversion must preserve the full inertia tensor and CoM.
                for child in ("origin", "mass", "inertia"):
                    self.assertEqual(link.find(f"inertial/{child}").attrib,
                                     original_link.find(f"inertial/{child}").attrib)
            right = generated.find("link[@name='Link_ankle_r_roll']")
            self.assertEqual(len(right.findall("collision")), 1)
            self.assertEqual([float(x) for x in right.find("collision/geometry/box").get("size").split()],
                             [0.245, 0.08, 0.087])
            left = generated.find("link[@name='Link_ankle_l_roll']")
            self.assertEqual([float(x) for x in left.find("collision/geometry/box").get("size").split()],
                             [0.245, 0.0804, 0.0884])
            # Source URDF ankle limits differ; use the actual simulation limits.
            ankle = generated.find("joint[@name='J_ankle_l_pitch']/limit")
            self.assertEqual(float(ankle.get("lower")), -0.47)
            self.assertEqual(float(ankle.get("upper")), 0.87)
            self.assertTrue((Path(directory) / "model_metadata.json").is_file())
        self.assertEqual(source.read_bytes(), source_bytes)


if __name__ == "__main__":
    unittest.main()
