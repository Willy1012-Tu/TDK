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
    for stage in (2, 3, 4):
        generate(project / f"stage{stage}_steps.yaml", generated / f"stage{stage}_steps.generated.h", expected_stage=stage)
except Exception as exc:
    print(f"YAML configuration error: {exc}")
    env.Exit(1)
env.Append(CPPPATH=[str(generated)])
