"""Locations for persistent project data when running from a source release."""
from __future__ import annotations

from pathlib import Path


def project_data_root() -> Path:
    """Return the repository/release source root, never its ``src`` directory.

    Deployment copies the complete repository into an isolated release source
    directory and runs the package from ``src/field_control`` beneath it.
    Operator profiles and diagnostic reports must remain beside that ``src``
    directory, rather than being written into importable package files.
    """
    package_dir = Path(__file__).resolve().parent
    source_dir = package_dir.parent
    project_root = source_dir.parent
    if source_dir.name != "src" or not (project_root / "pyproject.toml").is_file():
        raise RuntimeError(
            "field_control kräver en källa/release med src/ och pyproject.toml "
            "för beständig projektdata"
        )
    return project_root
