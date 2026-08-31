import subprocess
import sys

import pytest


@pytest.mark.parametrize("sub", ["open", "close", "goto", "home", "status", "drive"])
def test_verbs_parse(sub):
    argv = [sys.executable, "-m", "fastgripper_dm.cli", sub, "-h"]
    out = subprocess.run(argv, capture_output=True, text=True)
    assert out.returncode == 0, out.stderr


def test_goto_requires_a_percentage():
    out = subprocess.run([sys.executable, "-m", "fastgripper_dm.cli", "goto"],
                        capture_output=True, text=True)
    assert out.returncode != 0
