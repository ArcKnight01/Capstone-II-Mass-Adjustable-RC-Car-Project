"""
ConfigLoader.py - YAML configuration loader for the RC car project.

This module provides helpers to load YAML configuration files from the
`config/` directory, to read nested settings safely, and to unwrap config
entries that use a ``value`` wrapper.

Usage:
    from ConfigLoader import load_config_bundle, read_nested
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml


CONFIG_DIR = Path(__file__).resolve().parent / "config"


def load_yaml_file(path: str | Path) -> dict[str, Any]:
    """Load a YAML file and return a dictionary."""
    yaml_path = Path(path)
    with yaml_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping at top level of {yaml_path}, got {type(data).__name__}.")
    return data


def get_section(config_data: Mapping[str, Any] | None, section_name: str) -> dict[str, Any]:
    """
    Return a named top-level section when present, otherwise treat the supplied
    mapping itself as the section.
    """
    if not isinstance(config_data, Mapping):
        return {}

    section = config_data.get(section_name, config_data)
    return dict(section) if isinstance(section, Mapping) else {}


def read_config_value(value: Any, default: Any = None) -> Any:
    """
    Unwrap config entries that store metadata alongside a ``value`` field.
    """
    if isinstance(value, Mapping):
        if "value" in value:
            return value.get("value", default)
        return default if value == {} else value
    return default if value is None else value


def read_nested(
    config_data: Mapping[str, Any] | None,
    *keys: str,
    default: Any = None,
    unwrap_value: bool = False,
) -> Any:
    """
    Traverse nested mappings and optionally unwrap a final ``value`` field.
    """
    current: Any = config_data
    for key in keys:
        if not isinstance(current, Mapping) or key not in current:
            return default
        current = current[key]

    if unwrap_value:
        return read_config_value(current, default=default)
    return current


def load_config_bundle(config_dir: str | Path = CONFIG_DIR) -> dict[str, dict[str, Any]]:
    """
    Load every top-level YAML file in the config directory.

    The returned dictionary is keyed by filename stem, for example:
    - bundle['vehicle']
    - bundle['ekf']
    - bundle['sensors']
    """
    directory = Path(config_dir)
    if not directory.exists():
        raise FileNotFoundError(f"Config directory not found: {directory}")

    bundle: dict[str, dict[str, Any]] = {}
    for yaml_path in sorted(directory.glob("*.yaml")):
        bundle[yaml_path.stem] = load_yaml_file(yaml_path)
    return bundle


if __name__ == "__main__":
    loaded = load_config_bundle()
    print("Loaded config files:")
    for name in loaded:
        print(f"  - {name}")
