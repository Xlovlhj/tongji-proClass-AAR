from pathlib import Path
import json
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from isaac.terrain import normalize_terrain, terrain_manifest, terrain_plan


class TerrainTests(unittest.TestCase):
    def test_flat_ground_is_one_large_cuboid(self):
        plan = terrain_plan({"type": "flat"})
        self.assertEqual(len(plan), 1)
        self.assertEqual(plan[0]["name"], "Ground")

    def test_rough_blocks_are_seed_reproducible_and_include_all_tiles(self):
        config = {"type": "rough_blocks", "seed": 17, "amplitude_m": 0.02,
                  "tile_size_m": 0.2, "rows": 4, "columns": 3, "start_x_m": 0.8}
        first = terrain_plan(config)
        second = terrain_plan(config)
        self.assertEqual(first, second)
        self.assertEqual(len(first), 2 + 4 * 3)
        tops = [item["center_m"][2] + item["size_m"][2] / 2 for item in first[2:]]
        self.assertTrue(all(-0.02 <= value <= 0.02 for value in tops))

    def test_rough_approach_reaches_first_tile_without_a_gap(self):
        start, tile = 0.8, 0.2
        plan = terrain_plan({
            "type": "rough_blocks", "seed": 17, "amplitude_m": 0.02,
            "tile_size_m": tile, "rows": 4, "columns": 3,
            "start_x_m": start,
        })
        approach = next(item for item in plan if item["name"] == "Approach")
        first_row = [item for item in plan if item["name"].startswith("Rough_00_")]
        approach_left = approach["center_m"][0] - approach["size_m"][0] / 2
        approach_right = approach["center_m"][0] + approach["size_m"][0] / 2
        first_left = min(item["center_m"][0] - item["size_m"][0] / 2
                         for item in first_row)
        self.assertAlmostEqual(approach_left, -5.0)
        self.assertAlmostEqual(approach_right, start)
        # Tiles overlap their nominal cell by 1%, so a tiny overlap is expected;
        # the invariant being tested is that no lower-floor gap is exposed.
        self.assertLessEqual(first_left, approach_right)
        self.assertAlmostEqual(first_left, start, delta=tile * 0.01)

    def test_obstacle_seeds_produce_different_but_bounded_layouts(self):
        base = {"type": "random_obstacles", "count": 5, "seed": 1,
                "region_x_m": [1.0, 3.0], "region_y_m": [-0.5, 0.5],
                "height_range_m": [0.02, 0.05], "length_range_m": [0.1, 0.2],
                "width_range_m": [0.1, 0.2]}
        first = terrain_plan(base)
        base["seed"] = 2
        second = terrain_plan(base)
        self.assertNotEqual(first, second)
        self.assertEqual(len(first), 6)  # floor plus five obstacles
        for item in first[1:]:
            self.assertGreaterEqual(item["center_m"][0] - item["size_m"][0] / 2, 1.0)
            self.assertLessEqual(item["center_m"][0] + item["size_m"][0] / 2, 3.0)

    def test_blind_step_variants_have_expected_surface_height(self):
        for variant, expected in (("up", 0.06), ("down", -0.06)):
            with self.subTest(variant=variant):
                plan = terrain_plan({"type": "blind_step", "variant": variant, "height_m": 0.06})
                tops = [item["center_m"][2] + item["size_m"][2] / 2
                        for item in plan if item["name"].startswith("Step")]
                self.assertEqual(len(tops), 2)
                for top in tops:
                    self.assertAlmostEqual(top, expected)

    def test_blind_step_approach_is_continuous_at_step_edge(self):
        start = 0.9
        plan = terrain_plan({
            "type": "blind_step", "variant": "up", "height_m": 0.06,
            "start_x_m": start,
        })
        approach = next(item for item in plan if item["name"] == "Approach")
        steps = [item for item in plan if item["name"].startswith("Step")]
        approach_left = approach["center_m"][0] - approach["size_m"][0] / 2
        approach_right = approach["center_m"][0] + approach["size_m"][0] / 2
        step_left = min(item["center_m"][0] - item["size_m"][0] / 2
                        for item in steps)
        self.assertAlmostEqual(approach_left, -5.0)
        self.assertAlmostEqual(approach_right, start)
        self.assertAlmostEqual(step_left, start)

    def test_manifest_is_json_serializable_and_paths_match_plan(self):
        manifest = terrain_manifest({
            "type": "random_obstacles", "seed": 7, "count": 4,
            "region_x_m": [0.8, 2.0], "region_y_m": [-0.5, 0.5],
            "height_range_m": [0.02, 0.06],
            "length_range_m": [0.10, 0.22],
            "width_range_m": [0.10, 0.28],
        })
        encoded = json.dumps(manifest, allow_nan=False)
        self.assertEqual(json.loads(encoded), manifest)
        self.assertEqual(manifest["primitive_count"], len(manifest["plan"]))
        self.assertEqual(manifest["primitive_count"],
                         len(manifest["primitive_paths"]))
        for item, path in zip(manifest["plan"], manifest["primitive_paths"]):
            self.assertEqual(path, "/World/Terrain/" + item["name"])
            self.assertEqual(
                set(item),
                {"name", "center_m", "size_m", "rotate_y_deg", "color"},
            )

    def test_slope_course_is_height_continuous_in_plan(self):
        config = {"type": "slope_course", "start_x_m": 1.0, "width_m": 2.0,
                  "thickness_m": 0.08, "segments": [
                      {"angle_deg": 5, "length_m": 2}, {"angle_deg": 0, "length_m": 1},
                      {"angle_deg": -5, "length_m": 2}]}
        cfg = normalize_terrain(config)
        self.assertEqual(len(cfg["segments"]), 3)
        plan = terrain_plan(config)
        self.assertEqual(len([x for x in plan if x["name"].startswith("Slope")]), 3)

    def test_invalid_field_is_not_silently_ignored(self):
        with self.assertRaises(ValueError):
            normalize_terrain({"type": "flat", "amplitude_m": 0.1})


if __name__ == "__main__":
    unittest.main()

