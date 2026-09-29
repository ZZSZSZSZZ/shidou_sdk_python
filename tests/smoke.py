#!/usr/bin/env python3
"""Runtime smoke for the Python binding: no robot and no bridge required.

Peer mode opens a standalone session, which exercises the full stack (handle,
session, comm objects, telemetry watchdog) and the failure paths that do not
need a robot standing behind the services. Run it with PYTHONPATH pointing at
<build>/python, or let ctest run it as test_python_smoke.
"""

import sys
import threading

import shidou
from shidou.robot import _RETAINED

FAILURES = []


def check(condition, message):
    print("[{}] {}".format("PASS" if condition else "FAIL", message))
    if not condition:
        FAILURES.append(message)


def main():
    shidou.init_logging("warn")
    config = shidou.Config(robot_address="", mode="peer", enable_scouting=False)

    with shidou.Robot(config) as robot:
        check(robot.ready, "session and comm objects come up (peer mode)")
        check(robot.last_error() == "", "no error reported for a healthy handle")
        check(robot.fsm_state == "", "fsm_state is empty before the first sample")
        check(robot.last_feedback() is None, "no joint_states sample has arrived")
        check(robot.feedback_age_ms == -1.0, "age is the -1 sentinel without a sample")
        check(robot.feedback_seq == 0, "sequence starts at zero")

        # No robot answers the get_state service: the call must time out,
        # raise, and leave the SDK's own message behind.
        try:
            robot.get_state(timeout_ms=500)
        except shidou.ShidouError as error:
            check("get_state" in str(error), "get_state failure carries the SDK message: {}".format(error))
        else:
            check(False, "get_state must fail without a robot")

        # Length mismatches are shim-side errors: they must be refused before
        # anything reaches the encoder, and report why.
        check(not robot.send_mit(motor_ids=[1, 2], positions=[0.1]),
              "send_mit rejects arrays that are not index-aligned")
        check("positions" in robot.last_error(), "the mismatch is explained: {}".format(robot.last_error()))
        check(not robot.send_body(), "send_body rejects a frame with no target at all")
        # max_currents is the one array the message allows to be shorter than
        # the array it aligns with (device default for the rest).
        check(robot.send_body(wheel_ids=[1, 2], velocities=[0.0, 0.0], max_currents=[1.0]),
              "send_body accepts a short max_currents")

        # A well-formed frame is accepted (best-effort: nothing subscribes in
        # peer mode, so this only checks that the call path works).
        check(robot.send_mit(motor_ids=[1, 2], positions=[0.0, 0.0], kps=[0.0, 0.0]),
              "send_mit publishes a well-formed frame")

        # Trajectory marshalling (nested waypoints and their arrays) up to the
        # service call, which cannot be answered here.
        try:
            robot.upload_trajectory(
                joint_id=[1, 2],
                mit_points=[shidou.MITWaypoint(positions=[0.1, 0.1], stop_point=True)],
                timeout_ms=200)
        except shidou.ShidouError as error:
            check("trajectory" in str(error),
                  "upload_trajectory marshals and reports the unanswered call: {}".format(error))
        else:
            check(False, "upload_trajectory must fail without a robot")

        # get_state and last_feedback hand out views into per-handle storage
        # the next call of the same kind refills, so the binding serializes
        # them. Two threads on each path must neither crash nor deadlock.
        failures = []

        def reader():
            try:
                robot.last_feedback()
            except Exception as error:  # noqa: BLE001 - reported below
                failures.append(repr(error))

        def querier():
            try:
                robot.get_state(timeout_ms=200)
            except shidou.ShidouError:
                pass
            except Exception as error:  # noqa: BLE001 - reported below
                failures.append(repr(error))

        threads = [threading.Thread(target=reader) for _ in range(2)]
        threads += [threading.Thread(target=querier) for _ in range(2)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=30)
        check(not failures and not any(t.is_alive() for t in threads),
              "concurrent telemetry reads are serialized: {}".format(failures))

        # The error text is sticky, matching the C++ SDK.
        check(robot.last_error() != "", "last_error stays until the next failure")

        # Callbacks register and clear without traffic.
        robot.set_feedback_callback(lambda feedback: None)
        robot.set_fsm_callback(lambda state: None)
        robot.set_stale_callback(lambda age_ms: None, threshold_ms=100)
        robot.set_stale_callback(None)
        robot.set_fsm_callback(None)
        robot.set_feedback_callback(None)
        check(True, "callbacks register and clear")

        # A trampoline handed to C is never freed: clearing or replacing a
        # callback does not wait for an invocation that already started, so
        # releasing the Python object could leave C calling a freed thunk.
        # Traffic cannot be produced here, so this checks the bookkeeping
        # that guarantees it rather than the resulting crash.
        robot.set_feedback_callback(lambda feedback: None)
        replaced = robot._callbacks["feedback"]
        robot.set_feedback_callback(lambda feedback: None)
        check(replaced in _RETAINED, "a replaced trampoline is retained, not freed")
        cleared = robot._callbacks["feedback"]
        robot.set_feedback_callback(None)
        check(cleared in _RETAINED and "feedback" not in robot._callbacks,
              "a cleared trampoline is retained, not freed")

        handle_closed = robot
    try:
        handle_closed.get_state(timeout_ms=0)
    except shidou.ShidouError as error:
        check("closed" in str(error), "using a closed robot reports it: {}".format(error))
    else:
        check(False, "a closed robot must not be usable")

    shidou.shutdown()
    if FAILURES:
        print("PYTHON SMOKE FAILED ({} checks)".format(len(FAILURES)))
        return 1
    print("PYTHON SMOKE PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
