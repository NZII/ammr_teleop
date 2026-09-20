"""Versioned, reviewable Piper profiles shared by simulation and hardware."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


PROFILE_DIRECTORY = Path(__file__).with_name("profiles")


def available_profiles() -> tuple[str, ...]:
    """Return profile names in deterministic CLI-help order."""
    if not PROFILE_DIRECTORY.is_dir():
        return ()
    return tuple(sorted(path.stem for path in PROFILE_DIRECTORY.glob("*.json")))


def load_profile(name: str) -> dict[str, Any]:
    """Load and minimally validate a named JSON control profile."""
    if name not in available_profiles():
        raise ValueError(f"unknown control profile: {name}")
    path = PROFILE_DIRECTORY / f"{name}.json"
    with path.open("r", encoding="utf-8") as file:
        document = json.load(file)
    if not isinstance(document, dict):
        raise ValueError(f"control profile {name} must contain a JSON object")
    if document.get("name") != name:
        raise ValueError(f"control profile name mismatch in {path}")
    parameters = document.get("parameters")
    if not isinstance(parameters, dict) or not parameters:
        raise ValueError(f"control profile {name} has no parameters")
    return document
