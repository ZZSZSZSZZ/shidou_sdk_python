"""Python binding for the shidou_sdk robot control protocol.

Importing this package loads libshidou_py.so, which the build stages next to
these modules (SHIDOU_LIB overrides the path); a missing or mismatched shared
object fails here, at import, rather than mid-call. The public surface is the
names below; see python/README.md for usage and contracts.
"""

from .robot import (CSPWaypoint, Config, ControlMode, JointFeedback,
                    MITWaypoint, Robot, RobotState, ShidouError,
                    init_logging, shutdown)

__all__ = [
    "CSPWaypoint",
    "Config",
    "ControlMode",
    "JointFeedback",
    "MITWaypoint",
    "Robot",
    "RobotState",
    "ShidouError",
    "init_logging",
    "shutdown",
]
