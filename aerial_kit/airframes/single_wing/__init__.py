"""Single-motor flying-wing airframe description."""

from .spec import (
    DEFAULT_SPEC_PATH,
    SingleWingSpec,
    load_single_wing_spec,
)

__all__ = ["DEFAULT_SPEC_PATH", "SingleWingSpec", "load_single_wing_spec"]
