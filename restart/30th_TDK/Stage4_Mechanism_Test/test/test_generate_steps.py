"""Schema rejection and regeneration checks for the configuration users edit."""
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from generate_steps import ConfigError, StrictLoader, compile_document, generate, wire_document, yaml


class YamlTests(unittest.TestCase):
    def parse(self, source):
        return compile_document(yaml.load(source, Loader=StrictLoader))

    def profile(self, stage):
        source = Path(__file__).resolve().parents[1] / f"stage{stage}_steps.yaml"
        return yaml.load(source.read_text(encoding="utf-8"), Loader=StrictLoader)

    def test_stage3_disconnected_standby_only(self):
        document=self.profile(3)
        code=compile_document(document)
        self.assertIn("namespace Stage3",code)
        self.assertIn("REPEAT_FROM = 1",code)
        self.assertEqual(wire_document(document),([[2,0,0,0,0,0,0,1,1,0]],[[0,1]]))
        for action in ({"servo":{"pin":4,"pulse_us":1500}},{"stepper":{"node":6,"position":0,"rpm":320}},{"delay":{"ms":1}}):
            document["steps"][0]["actions"]=[action]
            with self.assertRaisesRegex(ConfigError,"disconnected"):
                wire_document(document)
    def test_stage4_launch_auto_drop_and_ramp(self):
        document = self.profile(4)
        code = compile_document(document)
        self.assertEqual(code.count("  Stage4::step(YAML_STEP_"), 6)
        self.assertIn("constexpr uint8_t REPEAT_FROM = 1;", code)
        actions, steps = wire_document(document)
        motors = [(a[9], a[1], a[2], a[3]) for a in actions if a[0] == 0]
        self.assertEqual(motors, [(6, -600, 320, 0), (6, -8100, 320, 5),
                                 (6, -600, 1500, 0), (6, -2600, 320, 0), (6, 0, 320, 0)])
        start, count = steps[2]
        ramp = actions[start:start + count]
        self.assertEqual(ramp[1][4:8], [5, 1550, 2000, 0])
        self.assertEqual(ramp[-1][0], 3)
        start, count = steps[3]
        launch = actions[start:start + count]
        self.assertEqual([a[0] for a in launch], [0, 1, 3, 0, 1, 3])
        self.assertEqual([a[8] for a in launch], [1, 1, 1, 2, 2, 2])
        self.assertEqual([a[5] for a in launch if a[0] == 1], [1400, 1800])
        start, count = steps[5]
        self.assertEqual([(a[4], a[5]) for a in actions[start:start + count] if a[0] == 1],
                         [(2, 1150), (5, 2100)])

    def test_legacy_default_node_and_stage(self):
        document = yaml.load("steps: [{actions: [{stepper: {position: -100, rpm: 320}}]}]", Loader=StrictLoader)
        code = compile_document(document)
        self.assertIn("namespace Stage4", code)
        self.assertIn("STAGE_ID = 4", code)
        self.assertIn("REPEAT_FROM = 1", code)
        self.assertIn("stepper(-100, 320, 0, true)", code)
        actions, steps = wire_document(document)
        self.assertEqual(actions, [[0, -100, 320, 0, 0, 0, 0, 1, 1, 6]])
        self.assertEqual(steps, [[0, 1]])

    def test_seven_distinct_parallel_actuators_motor_first(self):
        document = {"stage": 4, "steps": [{"actions": [{"parallel": [
            {"servo": {"pin": pin, "pulse_us": 1500}} for pin in (2, 5, 4, 6, 9)
        ] + [{"stepper": {"node": node, "position": 0, "rpm": 320}} for node in (6,7)]}]}]}
        code = compile_document(document)
        self.assertLess(code.index("stepperFor(6"), code.index("servo(2"))
        actions, steps = wire_document(document)
        self.assertEqual([a[0] for a in actions], [0, 0, 1, 1, 1, 1, 1, 3])
        self.assertEqual([a[9] for a in actions], [6, 7, 0, 0, 0, 0, 0, 0])
        self.assertTrue(all(len(a) == 10 for a in actions))
        self.assertEqual(steps, [[0, 8]])
        document["steps"][0]["actions"][0]["parallel"].append({"servo":{"pin":2,"pulse_us":1500}})
        with self.assertRaisesRegex(ConfigError,"1..7 distinct"):
            wire_document(document)

    def test_order_and_numbering(self):
        code = self.parse("""steps:
  - actions:
      - parallel:
          - servo: {pin: 5, pulse_us: 1700, duration_ms: 1000}
          - stepper: {position: -100, rpm: 320}
      - delay: {ms: 0}
  - actions:
      - servo: {pin: 2, pulse_us: 1200}
""")
        self.assertLess(code.index("stepper(-100"), code.index("servo(5"))
        self.assertIn("Stage4::branch(Stage4::waitAll(), 1)", code)
        self.assertIn("Stage4::branch(Stage4::delay(0), 2)", code)
        self.assertIn("step(YAML_STEP_2)", code)

    def test_invalid_configs(self):
        invalid = [
            "steps: []", "steps: [{actions: []}]",
            "stage: 0\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "stage: 5\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "stage: true\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "repeat_from: 0\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "repeat_from: 2\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "repeat_from: true\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "steps: [{name: old, actions: [{delay: {ms: 1}}]}]",
            "steps: [{actions: [{stepper: {node: 3, position: 0, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {node: 4, position: 0, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {node: 5, position: 0, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {node: 8, position: 0, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {node: true, position: 0, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {position: 8388608, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {position: 0, rpm: 0}}]}]",
            "steps: [{actions: [{stepper: {position: 0, rpm: 320, acc: 256}}]}]",
            "steps: [{actions: [{servo: {pin: 5, pulse_us: 499}}]}]",
            "steps: [{actions: [{servo: {pin: 3, pulse_us: 1400}}]}]",
            "steps: [{actions: [{servo: {pin: 5, pulse_us: 1400, duration_ms: 60001}}]}]",
            "steps: [{actions: [{delay: {ms: true}}]}]",
            "steps: [{actions: [{delay: {ms: -1}}]}]",
            "steps: [{actions: [{servo: {pin: 5, pulse_us: 1400, typo: 1}}]}]",
            "steps: [{actions: [{parallel: [{delay: {ms: 1}}]}]}]",
            "steps: [{actions: [{parallel: [{servo: {pin: 5, pulse_us: 1400}}, {servo: {pin: 5, pulse_us: 1500}}]}]}]",
            "steps: [{actions: [{parallel: [{stepper: {position: 0, rpm: 320}}, {stepper: {node: 6, position: 10, rpm: 320}}]}]}]",
            "steps: [{actions: [{parallel: [{stepper: {node: 4, position: 0, rpm: 320}}, {stepper: {node: 4, position: 10, rpm: 320}}]}]}]",
            "steps: [{actions: [{parallel: [{stepper: {node: 7, position: 0, rpm: 320}}, {stepper: {node: 7, position: 10, rpm: 320}}]}]}]",
            "steps: [{actions: [{parallel: [{parallel: []}]}]}]",
            "steps: [{actions: [{delay: {ms: 1, ms: 2}}]}]",
            "steps: [{actions: [{text: message}]}]",
        ]
        for source in invalid:
            with self.subTest(source=source), self.assertRaises(ConfigError):
                self.parse(source)

    def test_edit_regenerates_header_and_failure_preserves_previous(self):
        with tempfile.TemporaryDirectory() as directory:
            source, target = Path(directory) / "steps.yaml", Path(directory) / "generated.h"
            source.write_text("steps: [{actions: [{delay: {ms: 100}}]}]", encoding="utf-8")
            generate(source, target)
            first = target.read_text(encoding="utf-8")
            stamp = target.stat().st_mtime_ns
            generate(source, target)
            self.assertEqual(stamp, target.stat().st_mtime_ns)
            source.write_text("steps: [{actions: [{delay: {ms: 200}}]}]", encoding="utf-8")
            generate(source, target)
            self.assertNotEqual(first, target.read_text(encoding="utf-8"))
            previous = target.read_text(encoding="utf-8")
            source.write_text("steps: []", encoding="utf-8")
            with self.assertRaises(ConfigError):
                generate(source, target)
            self.assertEqual(previous, target.read_text(encoding="utf-8"))

    def test_stage2_four_steps_id7_repeat_and_parallel_wire(self):
        document=self.profile(2)
        self.assertIn("namespace Stage2",compile_document(document))
        self.assertEqual(document["repeat_from"],2)
        actions,steps=wire_document(document)
        self.assertEqual(steps,[[0,4],[4,3],[7,3],[10,5]])
        self.assertEqual(len(actions),15)
        expected=[[(7,-8000),(9,1800),(4,1300)],[(9,1100),(4,2000)],[(7,-3000),(6,2200)],[(7,-8000),(6,1600),(9,1800),(4,1300)]]
        for (start,count),targets in zip(steps,expected):
            group=actions[start:start+count]
            self.assertEqual([(a[9],a[1]) if a[0]==0 else (a[4],a[5]) for a in group[:-1]],targets)
            self.assertEqual(group[-1][0],3)
            self.assertTrue(all(a[7]==0 for a in group[:-1]))
            self.assertEqual(group[-1][7],1)
        self.assertEqual([(a[2],a[3]) for a in actions if a[0]==0],[(320,2)]*3)
    def test_runtime_capacity_matches_build_generation(self):
        document={"stage":4,"steps":[{"actions":[{"delay":{"ms":0}}]*256}]*8}
        actions,steps=wire_document(document)
        self.assertEqual(len(actions),2048)
        document["steps"].append({"actions":[{"delay":{"ms":0}}]})
        with self.assertRaisesRegex(ConfigError,"2048"):
            compile_document(document)

    def test_build_profile_stage_mismatch_preserves_header(self):
        with tempfile.TemporaryDirectory() as directory:
            source,target=Path(directory)/"stage2_steps.yaml",Path(directory)/"stage2_steps.generated.h"
            source.write_text("stage: 2\nsteps: [{actions: [{stepper: {node: 6, position: -8388607, rpm: 320}}]}]",encoding="utf-8")
            generate(source,target,expected_stage=2)
            self.assertIn("-8388607",target.read_text(encoding="utf-8"))
            previous=target.read_text(encoding="utf-8")
            source.write_text("stage: 4\nsteps: [{actions: [{delay: {ms: 0}}]}]",encoding="utf-8")
            with self.assertRaisesRegex(ConfigError,"expected 2"):
                generate(source,target,expected_stage=2)
            self.assertEqual(target.read_text(encoding="utf-8"),previous)


if __name__ == "__main__":
    unittest.main()
