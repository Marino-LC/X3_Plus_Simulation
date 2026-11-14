"""Permite ejecutar los scripts sin compilar el workspace (sin colcon)."""
import os
import sys

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..")))
