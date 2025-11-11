"""
MuJoCo AR Viewer - AR visualization for MuJoCo physics simulations.

This package provides tools to visualize MuJoCo simulations in Augmented Reality
using Apple Vision Pro and other AR devices.

Primary entry:
    - mujoco_arviewer: Main class for AR visualization of MuJoCo simulations

Convenience aliases:
    - Viewer: Class alias for readability (mujoco_arviewer.Viewer)
    - create: Factory function returning a mujoco_arviewer instance

Additionally, this package makes the module callable so you can do:
    import mujoco_arviewer
    viewer = mujoco_arviewer(...)
"""

from __future__ import annotations

import sys
import types

__version__ = "0.4.1"
__author__ = "Improbable AI"

# Import the main viewer class from the implementation module
from .viewer import mujoco_arviewer as mujoco_arviewer  # re-export with the same name

# Friendly aliases
Viewer = mujoco_arviewer

def create(*args, **kwargs) -> mujoco_arviewer:
    """Factory function: create and return a mujoco_arviewer instance."""
    return mujoco_arviewer(*args, **kwargs)

__all__ = [
    "mujoco_arviewer",
    "Viewer",
    "create",
]

# Optional: make the module itself callable so `mujoco_arviewer(...)` works after `import mujoco_arviewer`.
# This keeps normal module attributes while enabling direct-call syntax.
class _CallableModule(types.ModuleType):
    def __call__(self, *args, **kwargs):
        return mujoco_arviewer(*args, **kwargs)

# Replace the current module in sys.modules with a callable module, preserving attributes
_self = sys.modules.get(__name__)
_callable = _CallableModule(__name__)
_callable.__dict__.update(_self.__dict__)  # type: ignore[arg-type]
sys.modules[__name__] = _callable