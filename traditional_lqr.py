"""Compatibility shim for the historical ``traditional_lqr`` import path.

The monolith moved into the :mod:`balance_lab` package (constants in
``balance_lab.control.tuning``, the controller split into
``controller``/``jump``/``guards``/``recovery`` modules -- see
docs/ARCHITECTURE.md).  Every public name of the old module is re-exported
here, so existing scripts keep working unchanged; new code should import
from ``balance_lab`` directly.
"""

from balance_lab import *  # noqa: F401,F403
from balance_lab import __all__

__all__ = list(__all__)
