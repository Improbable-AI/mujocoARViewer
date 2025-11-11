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

__version__ = "0.4.3"
__author__ = "Improbable AI"

from .viewer import mujocoARViewer  # re-export with the same name