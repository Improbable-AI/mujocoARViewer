"""
MuJoCo AR Viewer - AR visualization for MuJoCo physics simulations.

This package provides tools to visualize MuJoCo simulations in Augmented Reality
using Apple Vision Pro and other AR devices.

Classes:
    MJARViewer: Main class for AR visualization of MuJoCo simulations
    MJARView: Legacy class for backward compatibility
"""

__version__ = "0.1.0"
__author__ = "Improbable AI"

from .viewer import MJARViewer

__all__ = ["MJARViewer"]