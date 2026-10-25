"""PlatformIO pre-script: load YAML before the compiler scans includes."""
import subprocess
import sys
from pathlib import Path

Import("env")

project = Path(env.subst("$PROJECT_DIR"))
dependencies = project / ".pio" / "python-deps"
if not (dependencies / "yaml" / "__init__.py").exists():
    subprocess.check_call([
        env.subst("$PYTHONEXE"), "-m", "pip", "install", "--disable-pip-version-check",
        "--target", str(dependencies), "-r", str(project / "tools" / "requirements-yaml.txt")
    ])
sys.path.insert(0, str(project / "tools"))
from generate_steps import generate

generated = Path(env.subst("$BUILD_DIR")) / "generated"
try:
    for stage in (1, 2, 3, 4, 5, 6):
        generate(project / f"stage{stage}_steps.yaml", generated / f"stage{stage}_steps.generated.h",expected_stage=stage)
    for prop,name in enumerate(("basket", "vase", "tube", "chicken", "bird"),1):
        generate(project / f"stage1_{name}.yaml", generated / f"stage1_{name}.generated.h",
                 expected_stage=1,expected_prop=prop)
        # These requested, reviewable snapshots live in include/. Regenerate them
        # on every Build too, so default include lookup cannot use a stale table.
        if name in ("basket", "vase", "chicken", "bird"):
            generate(project / f"stage1_{name}.yaml", project / "include" / f"stage1_{name}.generated.h",
                     expected_stage=1,expected_prop=prop)
except Exception as exc:
    print(f"YAML configuration error: {exc}")
    env.Exit(1)
env.Append(CPPPATH=[str(generated)])
