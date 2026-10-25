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

    def test_stage3_paired_positions_claw_and_repeat(self):
        document=self.profile(3)
        code=compile_document(document)
        self.assertIn("namespace Stage3",code)
        self.assertIn("REPEAT_FROM = 2",code)
        actions,steps=wire_document(document)
        self.assertEqual(steps,[[0,4],[4,4],[8,4],[12,4]])
        for (start,count),entry in zip(steps,document["steps"]):
            group=actions[start:start+count]
            children=entry["actions"][0]["parallel"]
            motors={child["stepper"]["node"]: child["stepper"] for child in children if "stepper" in child}
            claw=next(child["servo"] for child in children if "servo" in child)
            self.assertEqual(set(motors),{4,5})
            self.assertEqual([(a[9],a[1],a[2],a[3]) for a in group if a[0]==0],
                             [(node,motors[node]["position"],motors[node]["rpm"],motors[node].get("acc",0)) for node in (4,5)])
            self.assertEqual(group[2][4:8],[3,claw["pulse_us"],claw.get("duration_ms",0),0])
            self.assertEqual(group[3][0],3)
            self.assertTrue(all(a[7]==0 for a in group[:3]))
    def test_stage4_launch_auto_drop_and_ramp(self):
        document = self.profile(4)
        code = compile_document(document)
        self.assertEqual(code.count("  MechanismConfig::step(YAML_STEP_"), 6)
        self.assertIn("constexpr uint8_t REPEAT_FROM = 1;", code)
        actions, steps = wire_document(document)
        motors = [(a[9], a[1], a[2], a[3]) for a in actions if a[0] == 0]
        expected=[]
        for entry in document["steps"]:
            for item in entry["actions"]:
                for child in item.get("parallel",[item]):
                    if "stepper" in child:
                        p=child["stepper"]
                        expected.append((p.get("node",6),p["position"],p["rpm"],p.get("acc",0)))
        self.assertEqual(motors,expected)
        self.assertEqual(len(motors),5)
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

    def test_ten_distinct_parallel_actuators_motor_first(self):
        document = {"stage": 4, "steps": [{"actions": [{"parallel": [
            {"servo": {"pin": pin, "pulse_us": 1500}} for pin in (2, 3, 4, 5, 6, 9)
        ] + [{"stepper": {"node": node, "position": 0, "rpm": 320}} for node in (4,5,6)] + [{"cylinder":{"action":"extend","duration_ms":6000}}]}]}]}
        code = compile_document(document)
        self.assertLess(code.index("stepperFor(6"), code.index("servo(2"))
        actions, steps = wire_document(document)
        self.assertEqual([a[0] for a in actions], [0, 0, 0, 4, 1, 1, 1, 1, 1, 1, 3])
        self.assertEqual([a[9] for a in actions], [4, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0])
        self.assertTrue(all(len(a) == (11 if a[0]==4 else 10) for a in actions))
        self.assertEqual(actions[3], [4,0,0,0,0,0,6000,0,1,0,1])
        self.assertEqual(steps, [[0, 11]])
        document["steps"][0]["actions"][0]["parallel"].append({"servo":{"pin":2,"pulse_us":1500}})
        with self.assertRaisesRegex(ConfigError,"appears twice"):
            wire_document(document)

    def test_new_debug_servo_pins_boundaries_and_duration(self):
        for pin in (10,11,12):
            for pulse in (500,1500,2500):
                document={"stage":5,"steps":[{"actions":[{"servo":{"pin":pin,"pulse_us":pulse,"duration_ms":1234}}]}]}
                actions,steps=wire_document(document)
                self.assertEqual(actions,[[1,0,0,0,pin,pulse,1234,1,1,0]])
                self.assertEqual(steps,[[0,1]])
                self.assertIn(f"MechanismConfig::servo({pin}, {pulse}, 1234, true)",compile_document(document))
            for bad in (499,2501):
                with self.assertRaises(ConfigError):
                    compile_document({"steps":[{"actions":[{"servo":{"pin":pin,"pulse_us":bad}}]}]})

    def test_thirteen_parallel_actuators_keep_unique_bits_and_order(self):
        pins=(2,3,4,5,6,9,10,11,12)
        group=[{"servo":{"pin":pin,"pulse_us":1500}} for pin in pins]
        group += [{"stepper":{"node":node,"position":0,"rpm":320}} for node in (4,5,6)]
        group += [{"cylinder":{"action":"extend","duration_ms":6000}}]
        document={"stage":5,"steps":[{"actions":[{"parallel":group}]}]}
        actions,steps=wire_document(document)
        self.assertEqual([a[0] for a in actions],[0,0,0,4]+[1]*9+[3])
        self.assertEqual([a[4] for a in actions if a[0]==1],list(pins))
        self.assertEqual(steps,[[0,14]])
        group.append({"servo":{"pin":12,"pulse_us":1600}})
        with self.assertRaisesRegex(ConfigError,"1..13 distinct"):
            compile_document(document)

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
        self.assertIn("MechanismConfig::branch(MechanismConfig::waitAll(), 1)", code)
        self.assertIn("MechanismConfig::branch(MechanismConfig::delay(0), 2)", code)
        self.assertIn("step(YAML_STEP_2)", code)

    def test_invalid_configs(self):
        invalid = [
            "steps: []", "steps: [{actions: []}]",
            "stage: 0\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "stage: 6\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "stage: 2\nprop: 1\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "stage: 5\nprop: 0\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "stage: 1\nprop: 0\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "stage: 1\nprop: 6\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "stage: 1\nprop: true\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "stage: true\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "repeat_from: 0\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "repeat_from: 2\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "repeat_from: true\nsteps: [{actions: [{delay: {ms: 0}}]}]",
            "steps: [{name: old, actions: [{delay: {ms: 1}}]}]",
            "steps: [{actions: [{stepper: {node: 3, position: 0, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {node: 8, position: 0, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {node: true, position: 0, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {position: 8388608, rpm: 320}}]}]",
            "steps: [{actions: [{stepper: {position: 0, rpm: 0}}]}]",
            "steps: [{actions: [{stepper: {position: 0, rpm: 320, acc: 256}}]}]",
            "steps: [{actions: [{servo: {pin: 5, pulse_us: 499}}]}]",
            "steps: [{actions: [{servo: {pin: 7, pulse_us: 1400}}]}]",
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

    def test_stage1_preserves_vase_source_values(self):
        document=self.profile(1)
        code=compile_document(document)
        self.assertIn("namespace Stage1 {",code)
        self.assertIn("constexpr uint8_t PROP_ID = 2;",code)
        self.assertIn("constexpr uint8_t STEP_COUNT = 3;",code)
        actions,steps=wire_document(document)
        self.assertEqual([(a[9],a[1],a[2],a[3]) for a in actions if a[0]==0],
                         [(6,8000,1000,230),(6,5000,1000,245)])
        self.assertEqual([(a[4],a[5]) for a in actions if a[0]==1],
                         [(5,2056),(2,1833),(2,2111),(2,1389)])
        self.assertEqual(steps,[[0,4],[4,1],[5,3]])
        self.assertTrue(all(a[7]==0 for a in actions[0:3]))
        self.assertEqual(actions[3][0],3)

    def test_five_prop_profiles_exact_moves_servos_and_namespaces(self):
        root=Path(__file__).resolve().parents[1]
        specs=(
            (1,"basket",[(6,8000,1000,230),(6,3000,500,150)],[(5,2056),(2,1650),(2,2100),(2,1300)],[[0,4],[4,2],[6,1]]),
            (2,"vase",[(6,8000,1000,230),(6,2000,1000,245),(6,-1000,320,10)],[(5,2056),(2,1833),(2,2167),(2,1389),(2,1500)],[[0,4],[4,1],[5,3],[8,3]]),
            (3,"tube",[(6,8000,1000,230),(6,0,1000,255)],[(5,2056),(2,1667)],[[0,4],[4,1]]),
            (4,"chicken",[(6,9200,1000,230),(6,-1000,1500,240)],[(5,1722),(2,1611),(2,889),(5,1422),(2,1611)],[[0,4],[4,1],[5,2],[7,1]]),
            (5,"bird",[(6,9200,1000,230),(6,-1000,1500,240)],[(5,1722),(2,1611),(2,889),(5,1422),(2,1611)],[[0,4],[4,1],[5,2],[7,1]]))
        for prop,name,motors,servos,expected_steps in specs:
            with self.subTest(prop=prop):
                document=yaml.load((root/f"stage1_{name}.yaml").read_text(encoding="utf-8"),Loader=StrictLoader)
                self.assertEqual(document["stage"],1)
                self.assertEqual(document["prop"],prop)
                code=compile_document(document)
                self.assertIn(f"namespace Stage1{name.title()} {{",code)
                self.assertIn(f"constexpr uint8_t PROP_ID = {prop};",code)
                actions,steps=wire_document(document)
                self.assertEqual(steps,expected_steps)
                self.assertEqual([(a[9],a[1],a[2],a[3]) for a in actions if a[0]==0],motors)
                self.assertEqual([(a[4],a[5]) for a in actions if a[0]==1],servos)
                self.assertEqual([a[0] for a in actions[:4]],[0,1,1,3])
                self.assertEqual([a[7] for a in actions[:4]],[0,0,0,1])
                self.assertTrue(all(a[9]==6 for a in actions if a[0]==0))
                # Preserve the current on-disk tuning, including sequential servo ramps.
                if prop == 1:
                    self.assertEqual(actions[4], [1,0,0,0,2,2100,1000,1,1,0])
                    self.assertEqual(actions[5], [0,3000,500,150,0,0,0,1,2,6])
                    self.assertEqual(actions[6], [1,0,0,0,2,1300,1000,1,1,0])
                elif prop == 3:
                    self.assertTrue(all(a[6]==0 for a in actions))
                if prop in (4,5):
                    self.assertEqual(actions[5], [0,*motors[-1][1:],0,0,0,1,1,6])
                    self.assertEqual(actions[6], [1,0,0,0,5,1422,1000,1,2,0])
                    self.assertEqual(actions[7], [1,0,0,0,2,1611,0,1,1,0])
    def test_prop_metadata_default_alias_and_root_schema(self):
        entry={"actions":[{"delay":{"ms":0}}]}
        legacy={"stage":1,"steps":[entry]}
        self.assertIn("namespace Stage1 {",compile_document(legacy))
        self.assertIn("PROP_ID = 2",compile_document(legacy))
        self.assertIn("namespace Stage1Vase {",compile_document(dict(legacy,prop=2)))
        for stage in (2,3,4,5):
            with self.subTest(stage=stage):
                self.assertIn(f"namespace Stage{stage} {{",compile_document({"stage":stage,"steps":[entry]}))
                self.assertIn("PROP_ID = 0",compile_document({"stage":stage,"steps":[entry]}))
                with self.assertRaisesRegex(ConfigError,"only stage 1"):
                    compile_document({"stage":stage,"prop":2,"steps":[entry]})

    def test_stage5_is_no_actuator_placeholder(self):
        document=self.profile(5)
        self.assertEqual(document,{"stage":5,"repeat_from":1,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        code=compile_document(document)
        self.assertIn("namespace Stage5 {",code)
        self.assertIn("PROP_ID = 0",code)
        self.assertEqual(wire_document(document),([[2,0,0,0,0,0,0,1,1,0]],[[0,1]]))
        self.assertNotIn("stepperFor(",code)
        self.assertNotIn("servo(",code)

    def test_expected_prop_rejects_mismatch_or_legacy_alias_without_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            source,target=Path(directory)/"stage1_vase.yaml",Path(directory)/"generated.h"
            source.write_text("stage: 1\nprop: 2\nsteps: [{actions: [{delay: {ms: 0}}]}]",encoding="utf-8")
            generate(source,target,expected_stage=1,expected_prop=2)
            previous=target.read_text(encoding="utf-8")
            self.assertIn("namespace Stage1Vase {",previous)
            for metadata in ("stage: 1\nprop: 3", "stage: 1", "stage: 5"):
                with self.subTest(metadata=metadata):
                    source.write_text(metadata+"\nsteps: [{actions: [{delay: {ms: 0}}]}]",encoding="utf-8")
                    with self.assertRaises(ConfigError):
                        generate(source,target,expected_stage=1,expected_prop=2)
                    self.assertEqual(target.read_text(encoding="utf-8"),previous)

    def test_stage2_exact_targets_and_generated_wire(self):
        document=self.profile(2)
        expected={"stage":2,"repeat_from":2,"steps":[
            {"actions":[{"parallel":[
                {"servo":{"pin":9,"pulse_us":1800}},
                {"servo":{"pin":4,"pulse_us":1300}}]}]},
            {"actions":[{"parallel":[
                {"servo":{"pin":9,"pulse_us":1000}},
                {"servo":{"pin":4,"pulse_us":2100}}]}]},
            {"actions":[{"cylinder":{"action":"extend","duration_ms":3000}}]},
            {"actions":[{"servo":{"pin":6,"pulse_us":883,"duration_ms":1000}},
                        {"delay":{"ms":1000}}]},
            {"actions":[{"cylinder":{"action":"retract","duration_ms":3000}}]},
            {"actions":[{"parallel":[
                {"servo":{"pin":4,"pulse_us":1300,"duration_ms":1000}},
                {"servo":{"pin":9,"pulse_us":1800,"duration_ms":1000}}]}]}]}
        self.assertEqual(document,expected)
        actions,steps=wire_document(document)
        self.assertEqual(steps,[[0,3],[3,3],[6,1],[7,2],[9,1],[10,3]])
        self.assertEqual(actions,[
            [1,0,0,0,9,1800,0,0,1,0],
            [1,0,0,0,4,1300,0,0,1,0],
            [3,0,0,0,0,0,0,1,1,0],
            [1,0,0,0,9,1000,0,0,1,0],
            [1,0,0,0,4,2100,0,0,1,0],
            [3,0,0,0,0,0,0,1,1,0],
            [4,0,0,0,0,0,3000,1,1,0,1],
            [1,0,0,0,6,883,1000,1,1,0],
            [2,0,0,0,0,0,1000,1,2,0],
            [4,0,0,0,0,0,3000,1,1,0,2],
            [1,0,0,0,4,1300,1000,0,1,0],
            [1,0,0,0,9,1800,1000,0,1,0],
            [3,0,0,0,0,0,0,1,1,0]])
        code=compile_document(document)
        for expression in (
                "MechanismConfig::servo(9, 1800, 0, false)",
                "MechanismConfig::servo(4, 1300, 0, false)",
                "MechanismConfig::servo(9, 1000, 0, false)",
                "MechanismConfig::servo(4, 2100, 0, false)",
                "MechanismConfig::cylinder(MechanismConfig::CylinderAction::Retract, 3000, true)",
                "MechanismConfig::servo(6, 883, 1000, true)",
                "MechanismConfig::delay(1000)",
                "MechanismConfig::cylinder(MechanismConfig::CylinderAction::Extend, 3000, true)",
                "MechanismConfig::servo(4, 1300, 1000, false)",
                "MechanismConfig::servo(9, 1800, 1000, false)"):
            self.assertIn(expression,code)
        self.assertIn("constexpr uint8_t REPEAT_FROM = 2;",code)
        self.assertIn("constexpr uint8_t STEP_COUNT = 6;",code)

    def test_stage2_preserves_current_user_tuning_in_wire(self):
        document=self.profile(2)
        self.assertIn("namespace Stage2",compile_document(document))
        self.assertEqual(document["repeat_from"],2)
        actions,steps=wire_document(document)
        self.assertEqual(len(steps),len(document["steps"]))
        offset=0
        for entry,(start,count) in zip(document["steps"],steps):
            self.assertEqual(start,offset)
            offset+=count
            group=actions[start:start+count]
            cursor=0
            for branch,item in enumerate(entry["actions"],1):
                parallel="parallel" in item
                children=sorted(item["parallel"],key=lambda child:{"stepper":0,"cylinder":1,"servo":2}[next(iter(child))]) if parallel else [item]
                for child in children:
                    action=group[cursor]
                    cursor+=1
                    self.assertEqual(action[7:9],[0 if parallel else 1,branch])
                    if "stepper" in child:
                        p=child["stepper"]
                        self.assertEqual(action[0:4],[0,p["position"],p["rpm"],p.get("acc",0)])
                        self.assertEqual(action[9],p.get("node",6))
                    elif "servo" in child:
                        p=child["servo"]
                        self.assertEqual(action[0],1)
                        self.assertEqual(action[4:7],[p["pin"],p["pulse_us"],p.get("duration_ms",0)])
                    elif "cylinder" in child:
                        p=child["cylinder"]
                        self.assertEqual(action[0],4)
                        self.assertEqual(action[6],p["duration_ms"])
                        self.assertEqual(action[9],0)
                        self.assertEqual(action[10],{"stop":0,"extend":1,"retract":2}[p["action"]])
                    else:
                        self.assertEqual(action[0],2)
                        self.assertEqual(action[6],child["delay"]["ms"])
                if parallel:
                    self.assertEqual([group[cursor][i] for i in (0,7,8)],[3,1,branch])
                    cursor+=1
            self.assertEqual(cursor,count)
        self.assertEqual(offset,len(actions))
    def test_cylinder_defaults_stop_hold_and_wire_enum(self):
        document={"stage":2,"steps":[{"actions":[
            {"cylinder":{"action":"extend"}},
            {"cylinder":{"action":"retract"}},
            {"cylinder":{"action":"stop"}},
            {"cylinder":{"action":"stop","duration_ms":1500}}]}]}
        code=compile_document(document)
        actions,steps=wire_document(document)
        self.assertEqual(steps,[[0,4]])
        self.assertEqual([a[6] for a in actions],[6000,6000,0,1500])
        self.assertEqual([a[10] for a in actions],[1,2,0,0])
        self.assertTrue(all(a[0]==4 and a[9]==0 and len(a)==11 for a in actions))
        self.assertIn("CylinderAction::Stop, 1500, true",code)

    def test_cylinder_schema_rejects_unsafe_or_ambiguous_commands(self):
        invalid=[{}, {"action":"Extend"}, {"action":"left"}, {"action":True},
                 {"action":1}, {"action":"extend","duration_ms":0},
                 {"action":"retract","duration_ms":-1},
                 {"action":"stop","duration_ms":-1},
                 {"action":"stop","duration_ms":60001},
                 {"action":"extend","duration_ms":True},
                 {"action":"extend","duration_ms":1.5},
                 {"action":"extend","duration_ms":"6000"},
                 {"action":"retract","duration_ms":60001},
                 {"action":"stop","pin":14}, {"action":"extend","wait":False}]
        for params in invalid:
            document={"stage":2,"steps":[{"actions":[{"cylinder":params}]}]}
            with self.subTest(params=params),self.assertRaises(ConfigError):
                wire_document(document)
        duplicate={"stage":2,"steps":[{"actions":[{"parallel":[
            {"cylinder":{"action":"extend"}}, {"cylinder":{"action":"stop"}}]}]}]}
        with self.assertRaisesRegex(ConfigError,"appears twice"):
            wire_document(duplicate)
        removed={"stage":2,"steps":[{"actions":[{"stepper":{"node":7,"position":0,"rpm":320}}]}]}
        with self.assertRaises(ConfigError):
            wire_document(removed)

    def test_cylinder_edit_regenerates_and_rejection_preserves_header(self):
        with tempfile.TemporaryDirectory() as directory:
            source,target=Path(directory)/"stage2_steps.yaml",Path(directory)/"generated.h"
            source.write_text("stage: 2\nsteps: [{actions: [{cylinder: {action: extend, duration_ms: 6000}}]}]",encoding="utf-8")
            generate(source,target,expected_stage=2)
            source.write_text("stage: 2\nsteps: [{actions: [{cylinder: {action: retract, duration_ms: 5000}}]}]",encoding="utf-8")
            generate(source,target,expected_stage=2)
            previous=target.read_text(encoding="utf-8")
            self.assertIn("CylinderAction::Retract, 5000, true",previous)
            source.write_text("stage: 2\nsteps: [{actions: [{cylinder: {action: retract, duration_ms: 0}}]}]",encoding="utf-8")
            with self.assertRaises(ConfigError):
                generate(source,target,expected_stage=2)
            self.assertEqual(target.read_text(encoding="utf-8"),previous)

    def test_runtime_capacity_matches_build_generation(self):
        document={"stage":1,"steps":[{"actions":[{"delay":{"ms":0}}]*256}]*2}
        actions,steps=wire_document(document)
        self.assertEqual(len(actions),512)
        document["steps"].append({"actions":[{"delay":{"ms":0}}]})
        with self.assertRaisesRegex(ConfigError,"512"):
            compile_document(document)

    def test_build_profile_stage_mismatch_preserves_header(self):
        with tempfile.TemporaryDirectory() as directory:
            source,target=Path(directory)/"stage1_steps.yaml",Path(directory)/"stage1_steps.generated.h"
            source.write_text("stage: 1\nsteps: [{actions: [{stepper: {node: 6, position: -8388608, rpm: 320}}]}]",encoding="utf-8")
            generate(source,target,expected_stage=1)
            self.assertIn("-8388608",target.read_text(encoding="utf-8"))
            previous=target.read_text(encoding="utf-8")
            source.write_text("stage: 4\nsteps: [{actions: [{delay: {ms: 0}}]}]",encoding="utf-8")
            with self.assertRaisesRegex(ConfigError,"expected 1"):
                generate(source,target,expected_stage=1)
            self.assertEqual(target.read_text(encoding="utf-8"),previous)


if __name__ == "__main__":
    unittest.main()
