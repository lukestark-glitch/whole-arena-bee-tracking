# Changelog

One entry per stage or notable change, newest first. Record measured results here when a
stage asks for it.

## 2026-11-13: Stage 0, foundation

- The project installs with `pip install -e ".[dev]"`.
- `config/default.yaml`: arena, 16 cameras on 8 nodes, camera model, noise, ports, seed.
- `shared/config.py` loads and validates the configuration.
- `shared/protocol.py` encodes and decodes `detections`, `target`, `robot_pose` and `truth`,
  with rounding and splitting below 1400 bytes.
- `docs/protocol.md`: splitting rule and rounding table made complete, `truth.robot` is
  optional (decisions D14 to D18).

## 2026-11-13: Readable concept paper

- Added the concept paper as Markdown (`docs/concept-paper.md`) with its figures as SVG
  files, so it can be read directly on GitHub. Removed the HTML source.

## 2026-11-13: Project foundation

- Added the concept paper, the implementation plan, the rules for Claude Code and the
  documentation skeleton.
- No code yet.
