# Release steps for 0.2.0 (the public flip)

Version strings are already 0.2.0 (`c/CMakeLists.txt`, headers, `python/pyproject.toml`, `am_sdk.__version__`, ROS 2 packages). `python/tests/test_release_metadata.py` checks that they agree. Do not tag until the steps in "Before the flip" are done.

## Before the flip
1. Merge the release-prep pull request after CI is green.
2. Regenerate the unit data at the six-unit lock (`python tools/gen_unit_data.py`, `python tools/gen_actuator_specs.py`) if the locked values differ. Commit.
3. Confirm the remote has only `main` (D8, done: merged branches deleted).
4. Check the founder items below.

## Founder actions
- **PyPI name (D3):** reserve `am-sdk` on PyPI. This needs the founder's PyPI account. Until then install from git as the site shows.
- **ETG (D5):** join the EtherCAT Technology Group after incorporation to get the vendor ID. The ESI file ships with the evaluation units, not in this repository; ESI XSD validation also needs the membership.
- **Legal name (D1):** NOTICE says "Artificial Motion" until incorporation. Replace it with the legal entity name after incorporation.

## The flip
1. Make the repository public.
2. Tag and release: `git tag -a v0.2.0 -m "am-sdk 0.2.0" && git push origin v0.2.0`. Create the GitHub release with the 0.2.0 section of CHANGELOG.md.
3. When the PyPI name is reserved: `python -m build python && twine upload python/dist/*` (founder account).
4. In repository settings, close outside pull requests per CONTRIBUTING.md (for example, a pull request template notice or disabling PRs from non-collaborators).
