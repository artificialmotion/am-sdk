# SPDX-License-Identifier: Apache-2.0
"""Release hygiene: every version string agrees, and the Python distribution carries the licence."""
import re
import unittest
from pathlib import Path

import am_sdk
from am_sdk import am_types

PY = Path(__file__).resolve().parents[1]
ROOT = PY.parent


def _read(p: Path) -> str:
    return p.read_text(encoding="utf-8")


class TestReleaseMetadata(unittest.TestCase):
    def test_python_versions_agree(self):
        m = re.search(r'^version = "([^"]+)"', _read(PY / "pyproject.toml"), re.M)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), am_sdk.__version__)
        base = re.match(r"(\d+)\.(\d+)\.(\d+)", am_sdk.__version__)
        self.assertEqual(tuple(int(x) for x in base.groups()), am_types.API_VERSION)

    def test_python_licence_files_match_top_level(self):
        for name in ("LICENSE", "NOTICE"):
            if not (ROOT / name).exists():
                self.skipTest("not a repository checkout")
            self.assertEqual(_read(PY / name), _read(ROOT / name), name)

    def test_c_and_ros2_versions_agree(self):
        header = ROOT / "c" / "include" / "am_api.h"
        if not header.exists():
            self.skipTest("not a repository checkout")
        h = _read(header)
        c_ver = tuple(int(re.search(r"#define AM_API_VERSION_%s (\d+)" % k, h).group(1))
                      for k in ("MAJOR", "MINOR", "PATCH"))
        self.assertEqual(c_ver, am_types.API_VERSION)
        dotted = "%d.%d.%d" % c_ver
        for name in ("am_api.h", "am_est.h", "am_sim.h"):
            self.assertIn("C API %s" % dotted, _read(ROOT / "c" / "include" / name), name)
        self.assertIn("project(am_sdk VERSION %s" % dotted, _read(ROOT / "c" / "CMakeLists.txt"))
        self.assertIn("version %s" % dotted, _read(ROOT / "docs" / "API.md").splitlines()[0])
        for pkg in sorted((ROOT / "ros2").glob("*/package.xml")):
            self.assertIn("<version>%s</version>" % dotted, _read(pkg), str(pkg))

    def test_c_and_python_constants_agree(self):
        header = ROOT / "c" / "include" / "am_api.h"
        if not header.exists():
            self.skipTest("not a repository checkout")
        h = _read(header)
        for m in re.finditer(r"(AM_ERR_[A-Z_]+|AM_OK)\s*=\s*(-?\d+)", h):
            self.assertEqual(int(am_types.AmStatus[m.group(1)]), int(m.group(2)), m.group(1))
        bits = re.findall(r"#define (AM_(?:CAP|SYNC|SIO|WARN|SW)_[A-Z0-9_]+)\s+\(1u << (\d+)\)", h)
        self.assertTrue(bits)
        for name, bit in bits:
            self.assertEqual(getattr(am_types, name, None), 1 << int(bit), name)
        src = _read(ROOT / "c" / "src" / "am_session.c")
        c_faults = {int(c, 16): n for c, n in re.findall(r'\{(0x[0-9A-Fa-f]{4}), "([A-Z0-9_]+)"\}', src)}
        self.assertEqual(c_faults, am_types.FAULT_NAMES)


if __name__ == "__main__":
    unittest.main()
