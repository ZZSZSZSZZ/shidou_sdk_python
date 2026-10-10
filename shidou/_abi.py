"""ctypes declarations for the shidou_py shared library.

The C ABI lives in python/src/shidou_py.cc; this module mirrors its struct
layouts and entry-point signatures one for one, and loads the shared library
(libshidou_py.so on ELF, shidou_py.dll on Windows). The object is looked up
next to these modules (a source-tree build staged by cmake) and then under
_lib/<slug>/ (a prebuilt payload, see _library_path); SHIDOU_LIB overrides
both.
Keep the two in step: the ABI version check below catches a stale library,
the layout check at the end compares every field against the table the
library exports, and every struct carries the struct_size the C side
validates -- so a layout change that was not mirrored fails at import, by
field name, instead of reading garbage.

Comments and names follow the C side; see python/README.md for the Python
API built on top of this module.
"""

import ctypes
import os
import platform
import sys

# Mirrors SHIDOU_PY_ABI_VERSION in python/src/shidou_py.cc. 3: a failed
# shidou_create leaves the reason in the handle's error text.
ABI_VERSION = 3

# The name the build stages next to these modules: MSVC keeps its default
# output name for a shared library (no lib prefix), the same convention as the
# zenohc.dll it links; the ELF build stages libshidou_py.so.
LIBRARY_NAME = "shidou_py.dll" if sys.platform == "win32" else "libshidou_py.so"

# Prebuilt payload layout: shidou/_lib/<slug>/ holds the shared library next to
# the runtime it links (libzenohc.so resolved through RUNPATH=$ORIGIN on ELF;
# zenohc.dll resolved through the library directory added to the DLL search
# path on Windows). The slugs are the ones ci/build_dist_python.py writes and
# match the C++ payload's lib/<platform>/ names.
PAYLOAD_SLUGS = {
    ("linux", "x86_64"): "linux",
    ("linux", "amd64"): "linux",
    ("linux", "aarch64"): "linux-arm64",
    ("linux", "arm64"): "linux-arm64",
    ("win32", "amd64"): "win",
    ("win32", "arm64"): "win-arm64",
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


# os.add_dll_directory handles stay referenced for the process lifetime:
# closing one (or letting it be collected) removes the directory again, and the
# loaded binding resolves its dependencies from it for as long as it is loaded.
_DLL_DIRECTORIES = []


def _add_dll_search_directory(path):
    """Windows: let the loader find the library's dependencies beside it.

    The counterpart of the ELF $ORIGIN RUNPATH the installed object carries:
    the Windows loader does not search the directory of the library being
    loaded, so zenohc.dll sitting next to the binding is resolved only after
    that directory is added to the DLL search path.
    """
    if sys.platform == "win32":
        _DLL_DIRECTORIES.append(
            os.add_dll_directory(os.path.dirname(os.path.abspath(path))))


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
            "shared library).{}".format(LIBRARY_NAME, path, detail))
    _add_dll_search_directory(path)
    try:
        return ctypes.CDLL(path)
    except OSError as error:
        # The Windows loader names no module when a dependency is missing
        # ("The specified module could not be found"), which reads as if the
        # binding itself were absent; translate it into the diagnostic the
        # other error paths give. ELF names the dependency in its own message,
        # so there the system error is raised unchanged.
        if sys.platform != "win32":
            raise
        raise ImportError(
            "{} was found at {} but could not be loaded: {}. It links the runtime "
            "the build stages beside it (libzenohc.so on Linux, zenohc.dll on "
            "Windows); this machine reports {}/{}. Point SHIDOU_LIB at the staged "
            "copy in the build tree, or place that runtime library next to the one "
            "you point at.".format(
                LIBRARY_NAME, path, error, sys.platform, platform.machine())) from None


lib = _load_library()

_version = lib.shidou_abi_version()
if _version != ABI_VERSION:
    raise ImportError(
        "{} reports ABI version {} but this package expects {}: rebuild the "
        "binding from the matching source tree".format(
            LIBRARY_NAME, _version, ABI_VERSION))


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


class ShidouFieldDesc(ctypes.Structure):
    """One row of the library's layout table (shidou_layout)."""

    _fields_ = [
        ("struct_name", ctypes.c_char_p),
        ("field_name", ctypes.c_char_p),
        ("offset", ctypes.c_size_t),
        ("size", ctypes.c_size_t),
        ("kind", ctypes.c_int),
        ("elem", ctypes.c_char_p),
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
# Layout check.
#
# The ctypes structs above mirror python/src/shidou_py.cc by hand, and nothing
# else compares the two: the struct_size guards catch only a mirror that became
# smaller, while a reordered field or a same-size substitution (int32 for
# uint32, a pointer for size_t) reads garbage with every guard green. The
# library exports its own field table, built with offsetof/sizeof so it cannot
# drift from the C definitions; the comparison below runs at import and names
# every field the two descriptions disagree on.
# ---------------------------------------------------------------------------

KIND_U8 = 0
KIND_U32 = 1
KIND_I32 = 2
KIND_U64 = 3
KIND_F32 = 4
KIND_F64 = 5
KIND_STR = 6
KIND_PTR = 7

_KIND_NAMES = ("u8", "u32", "i32", "u64", "f32", "f64", "str", "ptr")

# ctypes type -> kind. These numbers must match enum shidou_field_kind in
# shidou_py.cc; a mismatch shows up here as a wrong kind, never as silence.
# size_t and uint64_t are one kind because ctypes makes them the same class on
# the platforms the binding ships for.
_MIRROR_KINDS = {
    ctypes.c_uint8: KIND_U8,
    ctypes.c_uint32: KIND_U32,
    ctypes.c_int32: KIND_I32,
    ctypes.c_int: KIND_I32,
    ctypes.c_size_t: KIND_U64,
    ctypes.c_uint64: KIND_U64,
    ctypes.c_float: KIND_F32,
    ctypes.c_double: KIND_F64,
    ctypes.c_char_p: KIND_STR,
}

# Pointee names as spelled by SHIDOU_PTR_FIELD in the table.
_POINTEE_NAMES = {
    ctypes.c_uint32: "uint32_t",
    ctypes.c_uint8: "uint8_t",
    ctypes.c_double: "double",
    ctypes.c_float: "float",
    ShidouMitWaypoint: "shidou_mit_waypoint_t",
    ShidouCspWaypoint: "shidou_csp_waypoint_t",
}

_STRUCTS = (
    ("shidou_config_t", ShidouConfig),
    ("shidou_mit_target_t", ShidouMitTarget),
    ("shidou_csp_target_t", ShidouCspTarget),
    ("shidou_position_target_t", ShidouPositionTarget),
    ("shidou_gripper_target_t", ShidouGripperTarget),
    ("shidou_body_target_t", ShidouBodyTarget),
    ("shidou_mit_waypoint_t", ShidouMitWaypoint),
    ("shidou_csp_waypoint_t", ShidouCspWaypoint),
    ("shidou_trajectory_t", ShidouTrajectory),
    ("shidou_state_t", ShidouState),
    ("shidou_feedback_t", ShidouFeedback),
)


def _mirror_description(field_type):
    """(kind, elem) for one ctypes field type; kind is None when unknown.

    POINTER(T) carries T in _type_, while the scalar types inherit _type_ as
    the character code ctypes uses internally ('i', 'P', ...) -- only a class
    there means a pointer.
    """
    pointee = getattr(field_type, "_type_", None)
    if isinstance(pointee, type):
        return KIND_PTR, _POINTEE_NAMES.get(pointee, "?")
    return _MIRROR_KINDS.get(field_type), ""


def _mirror_fields(struct_class):
    """field name -> (offset, size, kind, elem) for one ctypes struct."""
    fields = {}
    for name, field_type in struct_class._fields_:
        descriptor = getattr(struct_class, name)
        kind, elem = _mirror_description(field_type)
        fields[name] = (descriptor.offset, descriptor.size, kind, elem)
    return fields


def _format_description(description):
    offset, size, kind, elem = description
    kind_name = _KIND_NAMES[kind] if kind is not None and 0 <= kind < len(_KIND_NAMES) else "?"
    return "offset {} size {} {}{}".format(offset, size, kind_name,
                                           " " + elem if elem else "")


def _library_fields():
    """The library's layout table as struct name -> field name -> tuple."""
    layout = lib.shidou_layout
    layout.argtypes = [ctypes.POINTER(ctypes.POINTER(ShidouFieldDesc))]
    layout.restype = ctypes.c_size_t
    first = ctypes.POINTER(ShidouFieldDesc)()
    count = layout(ctypes.byref(first))
    fields = {}
    for index in range(count):
        row = first[index]
        fields.setdefault(row.struct_name.decode(), {})[row.field_name.decode()] = (
            row.offset, row.size, row.kind, row.elem.decode() if row.elem else "")
    return fields


def _check_layout():
    """Compares the ctypes mirror against the library's table; raises on drift.

    Runs at import: a mismatch means this package and this shared object do not
    describe the same structs, and every call would read or write the wrong
    bytes. Naming the fields turns that into a one-line fix.
    """
    try:
        library = _library_fields()
    except AttributeError:
        raise ImportError(
            "{} does not export shidou_layout: it is older than this package and "
            "cannot describe its struct layouts; rebuild the binding from the "
            "matching source tree".format(LIBRARY_NAME)) from None

    problems = []
    for struct_name, struct_class in _STRUCTS:
        table = library.pop(struct_name, None)
        if table is None:
            problems.append("{}: in the mirror, missing from the library".format(struct_name))
            continue
        mirror = _mirror_fields(struct_class)
        for field_name in sorted(set(table) | set(mirror)):
            if field_name not in table:
                problems.append("{}.{}: in the mirror, missing from the library".format(
                    struct_name, field_name))
            elif field_name not in mirror:
                problems.append("{}.{}: in the library, missing from the mirror".format(
                    struct_name, field_name))
            elif table[field_name] != mirror[field_name]:
                problems.append("{}.{}: the library has {}, the mirror has {}".format(
                    struct_name, field_name, _format_description(table[field_name]),
                    _format_description(mirror[field_name])))
    for struct_name in sorted(library):
        problems.append("{}: in the library, missing from the mirror".format(struct_name))

    if problems:
        shown = problems[:5]
        remainder = len(problems) - len(shown)
        raise ImportError(
            "the ctypes mirror in python/shidou/_abi.py does not match the structs "
            "in {}:\n  {}{}".format(
                LIBRARY_NAME,
                "\n  ".join(shown),
                "\n  ... and {} more".format(remainder) if remainder else ""))


_check_layout()


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
