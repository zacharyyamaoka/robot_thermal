"""The runtime core is standard-library only: a rig's control loop imports it with nothing installed."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"

CORE = ["robot_thermal.burst", "robot_thermal.drives.arm"]


def test_core_imports_with_no_site_packages_and_pulls_no_numpy():
    code = (
        "import sys; sys.path.insert(0, %r)\n"
        "import builtins\n"
        "real = builtins.__import__\n"
        "def guard(name, *a, **k):\n"
        "    if name.split('.')[0] in ('numpy', 'scipy', 'robot_msgs'):\n"
        "        raise ImportError('runtime core must not import ' + name)\n"
        "    return real(name, *a, **k)\n"
        "builtins.__import__ = guard\n"
        "%s\n"
        "print('ok')\n"
    ) % (str(SRC), "\n".join(f"import {m}" for m in CORE + EXTRA_CORE))
    out = subprocess.run([sys.executable, "-I", "-S", "-c", code], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == "ok"


# modules added to the core later (the thermal model and observer) append themselves here
EXTRA_CORE: list[str] = ["robot_thermal.thermal", "robot_thermal.observer", "robot_thermal.fit", "robot_thermal.sim",
                          "robot_thermal.drives.jmc_60asm400_ad60_thermal", "robot_thermal.drives.gim4305_10_thermal"]
