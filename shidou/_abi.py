"""ctypes declarations for libshidou_py.so.

The C ABI lives in python/src/shidou_py.cc; this module mirrors its struct
layouts and entry-point signatures one for one, and loads the shared object.
The object is looked up next to these modules (a source-tree build staged by
cmake) and then under _lib/<slug>/ (a prebuilt payload, see _library_path);
SHIDOU_LIB overrides both.
Keep the two in step: the ABI version check below catches a stale library,
and every struct carries the struct_size the C side validates, so a layout
change that was not mirrored fails loudly instead of reading garbage.

Comments and names follow the C side; see python/README.md for the Python
API built on top of this module.
"""

import ctypes
import os
import platform
import sys

# Mirrors SHIDOU_PY_ABI_VERSION in python/src/shidou_py.cc.
ABI_VERSION = 1

LIBRARY_NAME = "libshidou_py.so"

# Prebuilt payload layout: shidou/_lib/<slug>/ holds the shared object next to
# the libzenohc.so it links (the object carries RUNPATH=$ORIGIN). The slugs are
# the ones ci/build_dist_python.py writes and match the C++ payload's
# lib/<platform>/ names.
PAYLOAD_SLUGS = {
    ("linux", "x86_64"): "linux",
    ("linux", "amd64"): "linux",
    ("linux", "aarch64"): "linux-arm64",
    ("linux", "arm64"): "linux-arm64",
}


def _payload_slug():
    """Payload directory for this machine, or None when none is shipped.

    Exact matches only: falling back to "the only directory present" would try
    to load a Linux ELF on a host that cannot, and report that instead of the
    real reason.
    """
    return PAYLOAD_SLUGS.get((sys.platform.split("-")[0], platform.machine().lower()))


def _library_path():
    override = os.environ.get("SHIDOU_LIB")
    if override:
        return override
    here = os.path.dirname(os.path.abspath(__file__))
    # Development build (cmake -DSHIDOU_BUILD_PYTHON=ON): the stage target
    # copies the shared object next to these modules.
    staged = os.path.join(here, LIBRARY_NAME)
    if os.path.exists(staged):
        return staged
    slug = _payload_slug()
    if slug is None:
        return staged
    return os.path.join(here, "_lib", slug, LIBRARY_NAME)


def _shipped_payloads():
    """Payload slugs present in this package, for error messages."""
    libdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_lib")
    try:
        return sorted(name for name in os.listdir(libdir)
                      if os.path.isdir(os.path.join(libdir, name)))
    except OSError:
        return []


def _load_library():
    path = _library_path()
    if not os.path.exists(path):
        shipped = _shipped_payloads()
        detail = ""
        if shipped:
            detail = (" This package ships prebuilt payloads for {}; this machine "
                      "reports {}/{}, which has none.".format(
                          ", ".join(shipped), sys.platform, platform.machine()))
        raise ImportError(
            "{} not found at {}: build it with "
            "cmake -DSHIDOU_BUILD_PYTHON=ON -S <source tree> -B <build dir>, then run "
            "python with PYTHONPATH=<build dir>/python (or point SHIDOU_LIB at the "
            "shared object).{}".format(LIBRARY_NAME, path, detail))
    return ctypes.CDLL(path)


lib = _load_library()

_version = lib.shidou_abi_version()
if _version != ABI_VERSION:
    raise ImportError(
        "libshidou_py.so reports ABI version {} but this package expects {}: rebuild the "
        "binding from the matching source tree".format(_version, ABI_VERSION))


# ---------------------------------------------------------------------------
# Structs (python/src/shidou_py.cc).
# ---------------------------------------------------------------------------

class ShidouRobot(ctypes.Structure):
    """Opaque handle; the C side owns the layout."""


class ShidouConfig(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("robot_address", ctypes.c_char_p),
        ("mode", ctypes.c_char_p),
        ("enable_scouting", ctypes.c_int),
        ("ns", ctypes.c_char_p),
        ("log_level", ctypes.c_char_p),
    ]


class ShidouMitTarget(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("motor_ids", ctypes.POINTER(ctypes.c_uint32)),
        ("motor_ids_len", ctypes.c_size_t),
        ("positions", ctypes.POINTER(ctypes.c_double)),
        ("positions_len", ctypes.c_size_t),
        ("velocities", ctypes.POINTER(ctypes.c_double)),
        ("velocities_len", ctypes.c_size_t),
        ("torques", ctypes.POINTER(ctypes.c_double)),
        ("torques_len", ctypes.c_size_t),
        ("kps", ctypes.POINTER(ctypes.c_double)),
        ("kps_len", ctypes.c_size_t),
        ("kds", ctypes.POINTER(ctypes.c_double)),
        ("kds_len", ctypes.c_size_t),
    ]


class ShidouCspTarget(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("motor_ids", ctypes.POINTER(ctypes.c_uint32)),
        ("motor_ids_len", ctypes.c_size_t),
        ("positions", ctypes.POINTER(ctypes.c_double)),
        ("positions_len", ctypes.c_size_t),
        ("velocities", ctypes.POINTER(ctypes.c_double)),
        ("velocities_len", ctypes.c_size_t),
        ("torques", ctypes.POINTER(ctypes.c_double)),
        ("torques_len", ctypes.c_size_t),
    ]


class ShidouPositionTarget(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("motor_ids", ctypes.POINTER(ctypes.c_uint32)),
        ("motor_ids_len", ctypes.c_size_t),
        ("positions", ctypes.POINTER(ctypes.c_double)),
        ("positions_len", ctypes.c_size_t),
        ("velocities", ctypes.POINTER(ctypes.c_double)),
        ("velocities_len", ctypes.c_size_t),
        ("torques", ctypes.POINTER(ctypes.c_double)),
        ("torques_len", ctypes.c_size_t),
        ("accelerations", ctypes.POINTER(ctypes.c_double)),
        ("accelerations_len", ctypes.c_size_t),
    ]


class ShidouGripperTarget(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("motor_ids", ctypes.POINTER(ctypes.c_uint32)),
        ("motor_ids_len", ctypes.c_size_t),
        ("open", ctypes.POINTER(ctypes.c_double)),
        ("open_len", ctypes.c_size_t),
        ("kps", ctypes.POINTER(ctypes.c_double)),
        ("kps_len", ctypes.c_size_t),
        ("kds", ctypes.POINTER(ctypes.c_double)),
        ("kds_len", ctypes.c_size_t),
    ]


class ShidouBodyTarget(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("wheel_ids", ctypes.POINTER(ctypes.c_uint32)),
        ("wheel_ids_len", ctypes.c_size_t),
        ("velocities", ctypes.POINTER(ctypes.c_double)),
        ("velocities_len", ctypes.c_size_t),
        ("max_currents", ctypes.POINTER(ctypes.c_double)),
        ("max_currents_len", ctypes.c_size_t),
        ("pushrod_id", ctypes.c_uint32),
        ("position", ctypes.c_double),
        ("velocity", ctypes.c_double),
        ("acceleration", ctypes.c_double),
    ]


class ShidouMitWaypoint(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("positions", ctypes.POINTER(ctypes.c_double)),
        ("positions_len", ctypes.c_size_t),
        ("velocities", ctypes.POINTER(ctypes.c_double)),
        ("velocities_len", ctypes.c_size_t),
        ("torques", ctypes.POINTER(ctypes.c_double)),
        ("torques_len", ctypes.c_size_t),
        ("stop_point", ctypes.c_int),
    ]


class ShidouCspWaypoint(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("positions", ctypes.POINTER(ctypes.c_double)),
        ("positions_len", ctypes.c_size_t),
        ("max_velocity", ctypes.POINTER(ctypes.c_double)),
        ("max_velocity_len", ctypes.c_size_t),
        ("max_acceleration", ctypes.POINTER(ctypes.c_double)),
        ("max_acceleration_len", ctypes.c_size_t),
        ("torques", ctypes.POINTER(ctypes.c_double)),
        ("torques_len", ctypes.c_size_t),
    ]


class ShidouTrajectory(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("stamp_sec", ctypes.c_int32),
        ("stamp_nanosec", ctypes.c_uint32),
        ("frame_id", ctypes.c_char_p),
        ("joint_id", ctypes.POINTER(ctypes.c_uint32)),
        ("joint_id_len", ctypes.c_size_t),
        ("mit_points", ctypes.POINTER(ShidouMitWaypoint)),
        ("mit_points_len", ctypes.c_size_t),
        ("csp_points", ctypes.POINTER(ShidouCspWaypoint)),
        ("csp_points_len", ctypes.c_size_t),
    ]


class ShidouState(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("fsm_state", ctypes.c_char_p),
        ("arm_info", ctypes.c_char_p),
        ("gripper_info", ctypes.c_char_p),
        ("motor_ids", ctypes.POINTER(ctypes.c_uint32)),
        ("motor_ids_len", ctypes.c_size_t),
        ("positions", ctypes.POINTER(ctypes.c_double)),
        ("positions_len", ctypes.c_size_t),
        ("velocities", ctypes.POINTER(ctypes.c_double)),
        ("velocities_len", ctypes.c_size_t),
        ("torques", ctypes.POINTER(ctypes.c_double)),
        ("torques_len", ctypes.c_size_t),
        ("library_status", ctypes.c_uint32),
        ("control_freq_hz", ctypes.c_double),
        ("control_cycle_avg_ms", ctypes.c_double),
        ("control_cycle_max_ms", ctypes.c_double),
        ("cycle_overruns", ctypes.c_uint32),
        ("has_feedback", ctypes.c_int),
        ("feedback_seq", ctypes.c_uint64),
        ("feedback_age_ms", ctypes.c_double),
    ]


class ShidouFeedback(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("motor_ids", ctypes.POINTER(ctypes.c_uint32)),
        ("motor_ids_len", ctypes.c_size_t),
        ("position", ctypes.POINTER(ctypes.c_double)),
        ("position_len", ctypes.c_size_t),
        ("velocity", ctypes.POINTER(ctypes.c_double)),
        ("velocity_len", ctypes.c_size_t),
        ("effort", ctypes.POINTER(ctypes.c_double)),
        ("effort_len", ctypes.c_size_t),
        ("enabled", ctypes.POINTER(ctypes.c_uint8)),
        ("enabled_len", ctypes.c_size_t),
        ("online", ctypes.POINTER(ctypes.c_uint8)),
        ("online_len", ctypes.c_size_t),
        ("fault_code", ctypes.POINTER(ctypes.c_uint32)),
        ("fault_code_len", ctypes.c_size_t),
        ("temperature", ctypes.POINTER(ctypes.c_float)),
        ("temperature_len", ctypes.c_size_t),
    ]


FEEDBACK_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.POINTER(ShidouFeedback), ctypes.c_void_p)
FSM_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_void_p)
STALE_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_double, ctypes.c_void_p)

HANDLE = ctypes.POINTER(ShidouRobot)

# ---------------------------------------------------------------------------
# Entry points.
# ---------------------------------------------------------------------------

lib.shidou_abi_version.argtypes = []
lib.shidou_abi_version.restype = ctypes.c_int

lib.shidou_init_logging.argtypes = [ctypes.c_char_p]
lib.shidou_init_logging.restype = None

lib.shidou_create.argtypes = [ctypes.POINTER(ShidouConfig)]
lib.shidou_create.restype = HANDLE

lib.shidou_destroy.argtypes = [HANDLE]
lib.shidou_destroy.restype = None

lib.shidou_shutdown.argtypes = []
lib.shidou_shutdown.restype = None

lib.shidou_ready.argtypes = [HANDLE]
lib.shidou_ready.restype = ctypes.c_int

lib.shidou_set_namespace.argtypes = [HANDLE, ctypes.c_char_p]
lib.shidou_set_namespace.restype = ctypes.c_int

lib.shidou_enable.argtypes = [HANDLE, ctypes.c_longlong]
lib.shidou_enable.restype = ctypes.c_int

lib.shidou_stop.argtypes = [HANDLE, ctypes.c_longlong]
lib.shidou_stop.restype = ctypes.c_int

lib.shidou_set_mode.argtypes = [HANDLE, ctypes.c_int, ctypes.c_longlong]
lib.shidou_set_mode.restype = ctypes.c_int

lib.shidou_upload_trajectory.argtypes = [HANDLE, ctypes.POINTER(ShidouTrajectory),
                                         ctypes.c_longlong]
lib.shidou_upload_trajectory.restype = ctypes.c_int

lib.shidou_get_state.argtypes = [HANDLE, ctypes.POINTER(ShidouState), ctypes.c_longlong]
lib.shidou_get_state.restype = ctypes.c_int

lib.shidou_send_mit.argtypes = [HANDLE, ctypes.POINTER(ShidouMitTarget)]
lib.shidou_send_mit.restype = ctypes.c_int

lib.shidou_send_csp.argtypes = [HANDLE, ctypes.POINTER(ShidouCspTarget)]
lib.shidou_send_csp.restype = ctypes.c_int

lib.shidou_send_position.argtypes = [HANDLE, ctypes.POINTER(ShidouPositionTarget)]
lib.shidou_send_position.restype = ctypes.c_int

lib.shidou_send_gripper.argtypes = [HANDLE, ctypes.POINTER(ShidouGripperTarget)]
lib.shidou_send_gripper.restype = ctypes.c_int

lib.shidou_send_body.argtypes = [HANDLE, ctypes.POINTER(ShidouBodyTarget)]
lib.shidou_send_body.restype = ctypes.c_int

lib.shidou_last_feedback.argtypes = [HANDLE, ctypes.POINTER(ShidouFeedback)]
lib.shidou_last_feedback.restype = ctypes.c_int

lib.shidou_feedback_age_ms.argtypes = [HANDLE]
lib.shidou_feedback_age_ms.restype = ctypes.c_double

lib.shidou_feedback_seq.argtypes = [HANDLE]
lib.shidou_feedback_seq.restype = ctypes.c_uint64

lib.shidou_fsm_state.argtypes = [HANDLE, ctypes.c_char_p, ctypes.c_size_t]
lib.shidou_fsm_state.restype = ctypes.c_longlong

lib.shidou_set_feedback_callback.argtypes = [HANDLE, FEEDBACK_CALLBACK, ctypes.c_void_p]
lib.shidou_set_feedback_callback.restype = ctypes.c_int

lib.shidou_set_fsm_callback.argtypes = [HANDLE, FSM_CALLBACK, ctypes.c_void_p]
lib.shidou_set_fsm_callback.restype = ctypes.c_int

lib.shidou_set_stale_callback.argtypes = [HANDLE, ctypes.c_longlong, STALE_CALLBACK,
                                          ctypes.c_void_p]
lib.shidou_set_stale_callback.restype = ctypes.c_int

lib.shidou_last_error.argtypes = [HANDLE, ctypes.c_char_p, ctypes.c_size_t]
lib.shidou_last_error.restype = ctypes.c_longlong


# ---------------------------------------------------------------------------
# Marshalling helpers.
# ---------------------------------------------------------------------------

def new(struct_class):
    """Allocates a struct with struct_size filled in (the C side validates it)."""
    obj = struct_class()
    obj.struct_size = ctypes.sizeof(struct_class)
    return obj


def array(values, ctype):
    """Builds a ctypes array for a (pointer, length) pair; empty -> (None, 0).

    The caller must keep the returned array alive for as long as the C side
    may read it: pass it straight into the call that consumes it.
    """
    values = list(values or ())
    if not values:
        return None, 0
    return (ctype * len(values))(*values), len(values)


def read_string(pointer):
    """Decodes a borrowed const char* (NULL -> "")."""
    if not pointer:
        return ""
    return pointer.decode("utf-8", "replace")


def read_array(pointer, length):
    """Copies a borrowed array into a list; NULL/0 -> []."""
    if not pointer or not length:
        return []
    return [pointer[i] for i in range(length)]
