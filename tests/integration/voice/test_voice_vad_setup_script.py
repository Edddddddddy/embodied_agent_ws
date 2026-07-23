import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_silero_setup_dry_run_pins_lightweight_onnx_model_and_checksum():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "VOICE_VAD_SETUP_DRY_RUN": "true",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "setup_voice_vad_runtime.sh"), "silero"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "silero-vad/v6.2.1/src/silero_vad/data/silero_vad.onnx" in result.stdout
    assert "1a153a22f4509e292a94e67d6f9b85e8deb25b4988682b7e174c65279d8788e3" in result.stdout
    assert "models/silero_vad/silero_vad.onnx" in result.stdout
    assert "verify sha256=" in result.stdout
