"""Shared twin-wing airframe description, procedural geometry and model generators.

``twin_wing.yaml`` is the single source of truth; see ``docs/TWIN_WING_AIRFRAME.md``.
"""

from .spec import DEFAULT_SPEC_PATH, TwinWingSpec, load_twin_wing_spec

__all__ = ["DEFAULT_SPEC_PATH", "TwinWingSpec", "load_twin_wing_spec"]
