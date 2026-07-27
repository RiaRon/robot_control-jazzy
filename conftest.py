"""Make a bare checkout testable.

The README installs the package into a venv, but pre-push hooks and CI run
pytest against the checkout with whatever interpreter is on PATH. Without this
every test fails on `import robot_control`, which reads as a broken branch
rather than a missing install.
"""

import sys
from pathlib import Path

SRC = Path(__file__).parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
