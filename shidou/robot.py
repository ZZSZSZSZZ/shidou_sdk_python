"""Python API for the shidou_sdk robot control protocol.

Thin wrapper over the C ABI in _abi.py: it converts Python lists into the
(pointer, length) pairs the ABI expects, copies borrowed views into plain
Python objects before they expire, and keeps the callback trampolines alive
for as long as the C side may call them.

Conventions:

- Blocking handshakes (enable, stop, set_mode, set_namespace,
  upload_trajectory, get_state) either succeed or raise ShidouError carrying
  the SDK's own message; the C++ SDK reports the same failures as `false`.
- send_* are best-effort target streams: they return True when the frame was
  published and False otherwise, so a streaming loop does not pay for
  exceptions on every frame.
- Callbacks run on zenoh session threads (feedback, fsm_state) or on the
  telemetry watchdog thread (stale): they must not block, and must not call
  the blocking methods above.
"""

import ctypes
import enum
import threading

from . import _abi
from ._abi import lib

# Trampolines handed to the C side are never released: clearing or replacing a
# callback does not wait for an invocation that already started (the shim
# snapshots the slot and calls it outside its lock), so freeing the Python
# object could leave C calling a freed thunk. They are tiny and few - a
# program would have to re-register callbacks in a hot loop to notice.
_RETAINED = []


class ShidouError(RuntimeError):
    """An SDK operation failed; the message is the SDK's own error text."""


class ControlMode(enum.IntEnum):
    """Modes selectable after the ENABLED handshake."""

    CSP = 0
    POSITION = 1
    TRAJECTORY = 2


class Config:
    """Session configuration.

    None leaves the SDK default in place (robot_address 192.168.168.168:7447,
    mode "client", scouting off, no namespace prefix, log level "info"). The
    namespace must match the robot-side bridge configuration exactly; an
    empty string means "no prefix", which is different from the default.
    """

    def __init__(self, robot_address=None, mode=None, enable_scouting=False,
                 ns=None, log_level=None):
        self.robot_address = robot_address
        self.mode = mode
        self.enable_scouting = enable_scouting
        self.ns = ns
        self.log_level = log_level


class RobotState:
    """Snapshot from Robot.get_state(): the service response plus telemetry."""

    def __repr__(self):
        return ("RobotState(fsm_state={!r}, motor_ids={!r}, positions={!r}, "
                "has_feedback={!r}, feedback_age_ms={!r})").format(
                    self.fsm_state, self.motor_ids, self.positions,
                    self.has_feedback, self.feedback_age_ms)


class JointFeedback:
    """One joint_states sample; all arrays are index-aligned with motor_ids."""

    def __repr__(self):
        return ("JointFeedback(motor_ids={!r}, position={!r}, enabled={!r}, "
                "online={!r})").format(self.motor_ids, self.position,
                                       self.enabled, self.online)


class MITWaypoint:
    """Trajectory waypoint for MIT drives; omitted arrays mean zero."""

    def __init__(self, positions, velocities=None, torques=None, stop_point=False):
        self.positions = list(positions)
        self.velocities = list(velocities) if velocities else []
        self.torques = list(torques) if torques else []
        self.stop_point = bool(stop_point)


class CSPWaypoint:
    """Trajectory waypoint for CSP drives (Ruckig limits per joint)."""

    def __init__(self, positions, max_velocity=None, max_acceleration=None, torques=None):
        self.positions = list(positions)
        self.max_velocity = list(max_velocity) if max_velocity else []
        self.max_acceleration = list(max_acceleration) if max_acceleration else []
        self.torques = list(torques) if torques else []


def _encode(text):
    """str -> bytes for a const char* field; None stays NULL (SDK default)."""
    return None if text is None else text.encode("utf-8")


def _read_string_call(function, handle):
    """Two-call convention: ask for the size, then read into a buffer."""
    need = function(handle, None, 0)
    if need <= 0:
        return ""
    buf = ctypes.create_string_buffer(need)
    function(handle, buf, need)
    return buf.value.decode("utf-8", "replace")


def _state_from(view):
    state = RobotState()
    state.fsm_state = _abi.read_string(view.fsm_state)
    state.arm_info = _abi.read_string(view.arm_info)
    state.gripper_info = _abi.read_string(view.gripper_info)
    state.motor_ids = _abi.read_array(view.motor_ids, view.motor_ids_len)
    state.positions = _abi.read_array(view.positions, view.positions_len)
    state.velocities = _abi.read_array(view.velocities, view.velocities_len)
    state.torques = _abi.read_array(view.torques, view.torques_len)
    state.library_status = view.library_status
    state.control_freq_hz = view.control_freq_hz
    state.control_cycle_avg_ms = view.control_cycle_avg_ms
    state.control_cycle_max_ms = view.control_cycle_max_ms
    state.cycle_overruns = view.cycle_overruns
    state.has_feedback = view.has_feedback == 1
    state.feedback_seq = view.feedback_seq
    state.feedback_age_ms = view.feedback_age_ms
    return state


def _feedback_from(view):
    feedback = JointFeedback()
    feedback.motor_ids = _abi.read_array(view.motor_ids, view.motor_ids_len)
    feedback.position = _abi.read_array(view.position, view.position_len)
    feedback.velocity = _abi.read_array(view.velocity, view.velocity_len)
    feedback.effort = _abi.read_array(view.effort, view.effort_len)
    feedback.enabled = _abi.read_array(view.enabled, view.enabled_len)
    feedback.online = _abi.read_array(view.online, view.online_len)
    feedback.fault_code = _abi.read_array(view.fault_code, view.fault_code_len)
    feedback.temperature = _abi.read_array(view.temperature, view.temperature_len)
    return feedback


def init_logging(level):
    """Applies a spdlog level ("trace".."off") to the SDK loggers."""
    lib.shidou_init_logging(level.encode("utf-8"))


def shutdown():
    """Closes the process-wide session; call once, after all robots closed."""
    lib.shidou_shutdown()


class Robot:
    """A robot client.

    Creating one initializes the process-wide zenoh session, so the first
    Robot in a process fixes the session configuration; a later Robot with a
    different configuration is not ready (check `ready`). Close every handle
    before calling shidou.shutdown() at process end.
    """

    def __init__(self, config=None):
        self._handle = None
        self._callbacks = {}
        # Out-structs borrow per-handle storage that the next call of the same
        # kind refills, so the call and the copy out of it must not interleave
        # with another thread doing the same. get_state and last_feedback use
        # separate C-side storage, hence separate locks: a slow get_state does
        # not stall a telemetry read.
        self._state_lock = threading.Lock()
        self._feedback_lock = threading.Lock()
        raw = _abi.new(_abi.ShidouConfig)
        if config is not None:
            raw.robot_address = _encode(config.robot_address)
            raw.mode = _encode(config.mode)
            raw.enable_scouting = 1 if config.enable_scouting else 0
            raw.ns = _encode(config.ns)
            raw.log_level = _encode(config.log_level)
        handle = lib.shidou_create(ctypes.byref(raw))
        if not handle:
            raise ShidouError(_read_string_call(lib.shidou_last_error, None))
        self._handle = handle

    # -- lifecycle ---------------------------------------------------------

    def _h(self):
        if self._handle is None:
            raise ShidouError("the robot handle is closed")
        return self._handle

    @property
    def ready(self):
        """True when the comm objects were created; see last_error()."""
        if self._handle is None:
            return False
        return lib.shidou_ready(self._handle) == 1

    def last_error(self):
        """Last failure message; sticky, exactly as the C++ SDK reports it."""
        return _read_string_call(lib.shidou_last_error, self._handle)

    def close(self):
        """Destroys the handle. Idempotent; leaves the session open."""
        if self._handle is not None:
            handle = self._handle
            self._handle = None
            lib.shidou_destroy(handle)
            # Retired, not dropped: an invocation that started before the
            # destroy is not waited for (see _RETAINED).
            _RETAINED.extend(self._callbacks.values())
            self._callbacks.clear()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False

    # -- FSM handshakes ----------------------------------------------------

    def set_namespace(self, ns):
        """Rebuilds the comm objects under a new keyexpr prefix.

        Call only with no other operation on this robot in flight; the
        telemetry cache is cleared on success.
        """
        rc = lib.shidou_set_namespace(self._h(), _encode(ns) or b"")
        if rc != 0:
            raise ShidouError(self.last_error())

    def enable(self, timeout_ms=-1):
        """Handshake to ENABLED; timeout_ms < 0 uses the SDK default (5 s)."""
        rc = lib.shidou_enable(self._h(), timeout_ms)
        if rc != 0:
            raise ShidouError(self.last_error())

    def stop(self, timeout_ms=-1):
        """Handshake to STOP; timeout_ms < 0 uses the SDK default (5 s)."""
        rc = lib.shidou_stop(self._h(), timeout_ms)
        if rc != 0:
            raise ShidouError(self.last_error())

    def set_mode(self, mode, timeout_ms=-1):
        """Switches the control mode; requires ENABLED on the robot side."""
        rc = lib.shidou_set_mode(self._h(), int(mode), timeout_ms)
        if rc != 0:
            raise ShidouError(self.last_error())

    def upload_trajectory(self, joint_id, mit_points=None, csp_points=None,
                          frame_id="", stamp_sec=0, stamp_nanosec=0, timeout_ms=-1):
        """Uploads a trajectory; raises unless the robot accepted it.

        Waypoint arrays are index-aligned with joint_id. Fill the list that
        matches the drives (MIT or CSP); the other one stays empty, and both
        take the message's field order (mit_points first). Pass the waypoints
        by keyword: the two lists mean different things and are the same
        shape, so a positional call is easy to get wrong.
        """
        trajectory = _abi.new(_abi.ShidouTrajectory)
        trajectory.frame_id = _encode(frame_id) or b""
        trajectory.stamp_sec = stamp_sec
        trajectory.stamp_nanosec = stamp_nanosec
        # The C side reads these arrays during the call; keeping them in
        # `alive` holds the buffers until it returns.
        alive = []
        ids, trajectory.joint_id_len = _abi.array(joint_id, ctypes.c_uint32)
        trajectory.joint_id = ids
        alive.append(ids)

        if mit_points:
            structs = (_abi.ShidouMitWaypoint * len(mit_points))()
            for index, point in enumerate(mit_points):
                item = _abi.new(_abi.ShidouMitWaypoint)
                item.positions, item.positions_len = _abi.array(point.positions, ctypes.c_double)
                item.velocities, item.velocities_len = _abi.array(point.velocities, ctypes.c_double)
                item.torques, item.torques_len = _abi.array(point.torques, ctypes.c_double)
                item.stop_point = 1 if point.stop_point else 0
                structs[index] = item
                alive.extend([item.positions, item.velocities, item.torques])
            trajectory.mit_points = structs
            trajectory.mit_points_len = len(mit_points)
            alive.append(structs)

        if csp_points:
            structs = (_abi.ShidouCspWaypoint * len(csp_points))()
            for index, point in enumerate(csp_points):
                item = _abi.new(_abi.ShidouCspWaypoint)
                item.positions, item.positions_len = _abi.array(point.positions, ctypes.c_double)
                item.max_velocity, item.max_velocity_len = _abi.array(point.max_velocity,
                                                                      ctypes.c_double)
                item.max_acceleration, item.max_acceleration_len = _abi.array(
                    point.max_acceleration, ctypes.c_double)
                item.torques, item.torques_len = _abi.array(point.torques, ctypes.c_double)
                structs[index] = item
                alive.extend([item.positions, item.max_velocity, item.max_acceleration,
                              item.torques])
            trajectory.csp_points = structs
            trajectory.csp_points_len = len(csp_points)
            alive.append(structs)

        rc = lib.shidou_upload_trajectory(self._h(), ctypes.byref(trajectory), timeout_ms)
        if rc != 0:
            raise ShidouError(self.last_error())

    # -- target streams ----------------------------------------------------

    def send_mit(self, motor_ids, positions, velocities=None, torques=None,
                 kps=None, kds=None):
        """Streams one MIT (impedance) target; True when it was published.

        positions is required; the other arrays are optional and, when given,
        must have the same length as motor_ids.
        """
        target = _abi.new(_abi.ShidouMitTarget)
        ids, target.motor_ids_len = _abi.array(motor_ids, ctypes.c_uint32)
        target.motor_ids = ids
        values, target.positions_len = _abi.array(positions, ctypes.c_double)
        target.positions = values
        extra, target.velocities_len = _abi.array(velocities, ctypes.c_double)
        target.velocities = extra
        extra, target.torques_len = _abi.array(torques, ctypes.c_double)
        target.torques = extra
        extra, target.kps_len = _abi.array(kps, ctypes.c_double)
        target.kps = extra
        extra, target.kds_len = _abi.array(kds, ctypes.c_double)
        target.kds = extra
        return lib.shidou_send_mit(self._h(), ctypes.byref(target)) == 0

    def send_csp(self, motor_ids, positions, velocities=None, torques=None):
        """Streams one CSP target; True when it was published."""
        target = _abi.new(_abi.ShidouCspTarget)
        ids, target.motor_ids_len = _abi.array(motor_ids, ctypes.c_uint32)
        target.motor_ids = ids
        values, target.positions_len = _abi.array(positions, ctypes.c_double)
        target.positions = values
        extra, target.velocities_len = _abi.array(velocities, ctypes.c_double)
        target.velocities = extra
        extra, target.torques_len = _abi.array(torques, ctypes.c_double)
        target.torques = extra
        return lib.shidou_send_csp(self._h(), ctypes.byref(target)) == 0

    def send_position(self, motor_ids, positions, velocities=None, torques=None,
                      accelerations=None):
        """Streams one Profile Position target; True when it was published."""
        target = _abi.new(_abi.ShidouPositionTarget)
        ids, target.motor_ids_len = _abi.array(motor_ids, ctypes.c_uint32)
        target.motor_ids = ids
        values, target.positions_len = _abi.array(positions, ctypes.c_double)
        target.positions = values
        extra, target.velocities_len = _abi.array(velocities, ctypes.c_double)
        target.velocities = extra
        extra, target.torques_len = _abi.array(torques, ctypes.c_double)
        target.torques = extra
        extra, target.accelerations_len = _abi.array(accelerations, ctypes.c_double)
        target.accelerations = extra
        return lib.shidou_send_position(self._h(), ctypes.byref(target)) == 0

    def send_gripper(self, motor_ids, open, kps=None, kds=None):
        """Streams one gripper target (open is normalized [0, 1])."""
        target = _abi.new(_abi.ShidouGripperTarget)
        ids, target.motor_ids_len = _abi.array(motor_ids, ctypes.c_uint32)
        target.motor_ids = ids
        values, target.open_len = _abi.array(open, ctypes.c_double)
        target.open = values
        extra, target.kps_len = _abi.array(kps, ctypes.c_double)
        target.kps = extra
        extra, target.kds_len = _abi.array(kds, ctypes.c_double)
        target.kds = extra
        return lib.shidou_send_gripper(self._h(), ctypes.byref(target)) == 0

    def send_body(self, wheel_ids=None, velocities=None, max_currents=None,
                  pushrod_id=0, position=0.0, velocity=0.0, acceleration=0.0):
        """Streams one body target (hub wheels and/or the pushrod).

        Wheel arrays are optional: a frame may carry only a pushrod target
        (pushrod_id != 0). pushrod_id 0 means "no pushrod target in this
        frame". max_currents may be shorter than wheel_ids - the robot
        applies its device default to the rest.
        """
        target = _abi.new(_abi.ShidouBodyTarget)
        ids, target.wheel_ids_len = _abi.array(wheel_ids, ctypes.c_uint32)
        target.wheel_ids = ids
        values, target.velocities_len = _abi.array(velocities, ctypes.c_double)
        target.velocities = values
        extra, target.max_currents_len = _abi.array(max_currents, ctypes.c_double)
        target.max_currents = extra
        target.pushrod_id = pushrod_id
        target.position = position
        target.velocity = velocity
        target.acceleration = acceleration
        return lib.shidou_send_body(self._h(), ctypes.byref(target)) == 0

    # -- telemetry ---------------------------------------------------------

    def get_state(self, timeout_ms=-1):
        """Returns a RobotState; raises ShidouError on failure.

        Blocks up to timeout_ms (< 0 uses the SDK default, 5 s). Safe to call
        from several threads; a concurrent call waits for this one.
        """
        view = _abi.new(_abi.ShidouState)
        with self._state_lock:
            rc = lib.shidou_get_state(self._h(), ctypes.byref(view), timeout_ms)
            if rc != 0:
                raise ShidouError(self.last_error())
            return _state_from(view)

    def last_feedback(self):
        """Latest joint_states sample, or None when none has arrived yet.

        Safe to call from several threads; a concurrent call waits for this
        one (it does not wait for get_state).
        """
        view = _abi.new(_abi.ShidouFeedback)
        with self._feedback_lock:
            rc = lib.shidou_last_feedback(self._h(), ctypes.byref(view))
            if rc < 0:
                raise ShidouError(self.last_error())
            if rc == 0:
                return None
            return _feedback_from(view)

    @property
    def feedback_age_ms(self):
        """Milliseconds since the latest sample; -1.0 when none has arrived."""
        return lib.shidou_feedback_age_ms(self._h())

    @property
    def feedback_seq(self):
        """Number of samples received since the last namespace switch."""
        return lib.shidou_feedback_seq(self._h())

    @property
    def fsm_state(self):
        """Current fsm_state as published by the robot ("" before the first)."""
        return _read_string_call(lib.shidou_fsm_state, self._h())

    # -- callbacks ---------------------------------------------------------

    def _retire(self, name):
        """Drops a trampoline from the live set without freeing it."""
        previous = self._callbacks.pop(name, None)
        if previous is not None:
            _RETAINED.append(previous)

    def set_feedback_callback(self, callback):
        """Registers callback(feedback) for joint_states; None clears it.

        The callback runs on a zenoh session thread: do not block in it and
        do not call the blocking methods from it. The JointFeedback it
        receives is a plain copy, safe to keep. A callback that has just been
        cleared or replaced may still run once.
        """
        if callback is None:
            lib.shidou_set_feedback_callback(self._h(), _abi.FEEDBACK_CALLBACK(), None)
            self._retire("feedback")
            return

        def bridge(view, user_data):
            callback(_feedback_from(view.contents))

        trampoline = _abi.FEEDBACK_CALLBACK(bridge)
        if lib.shidou_set_feedback_callback(self._h(), trampoline, None) != 0:
            raise ShidouError(self.last_error())
        self._retire("feedback")
        self._callbacks["feedback"] = trampoline

    def set_fsm_callback(self, callback):
        """Registers callback(state) for fsm_state; None clears it.

        Runs on a zenoh session thread, same rules as set_feedback_callback.
        """
        if callback is None:
            lib.shidou_set_fsm_callback(self._h(), _abi.FSM_CALLBACK(), None)
            self._retire("fsm")
            return

        def bridge(state, user_data):
            callback(state.decode("utf-8", "replace") if state else "")

        trampoline = _abi.FSM_CALLBACK(bridge)
        if lib.shidou_set_fsm_callback(self._h(), trampoline, None) != 0:
            raise ShidouError(self.last_error())
        self._retire("fsm")
        self._callbacks["fsm"] = trampoline

    def set_stale_callback(self, callback, threshold_ms=1000):
        """Registers callback(age_ms) for the staleness watchdog; None clears.

        Fires once per stale transition when no sample arrived for
        threshold_ms, on the telemetry watchdog thread.
        """
        if callback is None:
            lib.shidou_set_stale_callback(self._h(), 0, _abi.STALE_CALLBACK(), None)
            self._retire("stale")
            return

        def bridge(age_ms, user_data):
            callback(age_ms)

        trampoline = _abi.STALE_CALLBACK(bridge)
        if lib.shidou_set_stale_callback(self._h(), int(threshold_ms), trampoline, None) != 0:
            raise ShidouError(self.last_error())
        self._retire("stale")
        self._callbacks["stale"] = trampoline
