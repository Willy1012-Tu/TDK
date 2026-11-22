"""Browser QA only: serve a copied YAML and a fake USB device on loopback."""
import shutil
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from web_gui import Application,Bridge,PROJECT,make_server,PROFILES
from test_web_gui import FakePort

with tempfile.TemporaryDirectory(prefix="stage4-web-preview-") as directory:
    root=Path(directory)
    for filename in set(PROFILES.values()):
        shutil.copy2(PROJECT/filename,root/filename)
    shutil.copytree(PROJECT/"web",root/"web")
    app=Application(root,Bridge(FakePort)); app.simulation=True
    app.ports=lambda:[SimpleNamespace(device="COM99",description="SIMULATION · no real hardware")]
    server=make_server("127.0.0.1",8766,app)
    print("SIMULATION PREVIEW: http://127.0.0.1:8766 (no real USB, copied YAML)",flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:app.bridge.disconnect(); server.server_close()
