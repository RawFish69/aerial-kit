from __future__ import annotations

import os
import sys
from pathlib import Path


def ensure_repo_root_on_path() -> Path:
    """Add repo root to ``sys.path`` so planner wrappers can import ``sim_py``.

    Works for both source-tree runs and installed ROS2 package layouts.
    """

    def _is_repo_root(p: Path) -> bool:
        return (p / "sim_py").is_dir() and (p / "ros2_ws").is_dir()

    candidates: list[Path] = []

    env_root = os.environ.get("AERIAL_KIT_REPO_ROOT")
    if env_root:
        candidates.append(Path(env_root).expanduser())

    here = Path(__file__).resolve()
    candidates.extend(here.parents)

    cwd = Path.cwd().resolve()
    candidates.append(cwd)
    candidates.extend(cwd.parents)

    for cand in candidates:
        if _is_repo_root(cand):
            repo_root = cand
            if str(repo_root) not in sys.path:
                sys.path.insert(0, str(repo_root))
            return repo_root

    raise RuntimeError(
        "Could not locate aerial-kit repo root for importing sim_py. "
        "Set AERIAL_KIT_REPO_ROOT or run from within the repo."
    )
