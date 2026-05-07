"""CARLA import compatibility.

This repo is typically run inside a conda environment that provides the `carla`
Python module. However, many scripts (e.g. compileall / static helpers) should
remain importable even when CARLA is not installed.
"""


from __future__ import annotations


try:
    import carla as _carla  # type: ignore
except ModuleNotFoundError:  # pragma: no cover
    _carla = None


def require_carla():
    """Return the imported `carla` module or raise a helpful error."""

    if _carla is None:
        raise RuntimeError(
            "The 'carla' Python module is not available. "
            "Activate your conda environment (e.g. `conda activate carla`) "
            "and ensure the CARLA PythonAPI is installed and on PYTHONPATH."
        )
    return _carla
