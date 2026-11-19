"""RACCOON_NO_HEARTBEATS switches the heartbeat daemon off in a process that only borrows the library."""

import json
import os
import subprocess
import sys

import pytest

hal = pytest.importorskip("raccoon.hal")
if not hasattr(hal, "heartbeat_diagnostics"):  # pragma: no cover - an extension built without the daemon
    pytest.skip("raccoon.hal has no heartbeat_diagnostics", allow_module_level=True)


@pytest.mark.parametrize("value", ["1", "true", "TRUE", "yes", "on", "anything", " 1"])
def test_these_values_switch_the_heartbeat_off(value):
    assert hal._heartbeat_env_disables(value) is True


@pytest.mark.parametrize("value", [None, "", "0", "false", "False", "no", "No", "off", "OFF"])
def test_unset_and_values_that_mean_no_leave_the_heartbeat_on(value):
    assert hal._heartbeat_env_disables(value) is False


def test_the_diagnostics_say_whether_the_variable_switched_it_off():
    """The daemon reads the variable when the library is loaded, which is before this test runs."""
    diagnostics = hal.heartbeat_diagnostics()
    assert diagnostics["disabled"] is hal._heartbeat_env_disables(os.environ.get("RACCOON_NO_HEARTBEATS"))
    if diagnostics["disabled"]:
        assert diagnostics["daemon_started"] is False and diagnostics["is_owner"] is False


def _can_start_a_python_process():
    # Under qemu-user (the cross build's test run) a child interpreter cannot be executed.
    try:
        return subprocess.run([sys.executable, "-c", "pass"], capture_output=True, timeout=60).returncode == 0
    except OSError:
        return False


@pytest.mark.skipif(not _can_start_a_python_process(), reason="cannot start a child Python process here")
@pytest.mark.parametrize("value, disabled", [("1", True), (None, False), ("0", False)])
def test_a_new_process_starts_or_skips_the_daemon_as_told(value, disabled):
    env = dict(os.environ)
    env.pop("RACCOON_NO_HEARTBEATS", None)
    if value is not None:
        env["RACCOON_NO_HEARTBEATS"] = value
    code = ("import json, time\nimport raccoon.hal as hal\ntime.sleep(1.5)\n"
            "print(json.dumps(hal.heartbeat_diagnostics()))\n")
    result = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stderr
    diagnostics = json.loads(result.stdout.strip().splitlines()[-1])
    assert diagnostics["disabled"] is disabled
    if disabled:
        assert diagnostics["daemon_started"] is False and diagnostics["is_owner"] is False
        assert diagnostics["sent"] == 0
