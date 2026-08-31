"""fastgripper-dm: FastGripper worm-gear gripper on a Damiao DM-J4310 over CAN.

v0.0.x is the "harvest" release: the hardware-validated bench tools under one
package and one CLI. The controller/port refactor (see the design spec) lands
in v0.1.
"""

from .facade import FastGripper, HomingError
from .profile import GripperProfile, PRESETS, TMAX_CAP

__version__ = "0.1.0"   # unchanged; the single bump to 0.1.0 happens in Task 12
__all__ = ["FastGripper", "HomingError", "GripperProfile", "PRESETS", "TMAX_CAP", "__version__"]
