import importlib.util
from pathlib import Path

import pytest


path = Path(__file__).resolve().parents[1] / "scripts/launch.py"
spec = importlib.util.spec_from_file_location("ammr_launcher", path)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


def test_real_is_default_and_arguments_are_forwarded_without_shell_expansion():
    args, extra = launcher.parse(
        ["teleop", "--task", "name; $(echo nope)", "--hardware-components", "left_arm"]
    )
    command = launcher.commands(args, extra)[0]
    assert "run_xr_teleop_ammr_real_ros2.sh" in command[1]
    assert "name; $(echo nope)" in command
    assert "--record" not in command


def test_sim_selects_sim_control_but_does_not_start_isaac():
    args, extra = launcher.parse(["teleop", "--sim", "--profile", "piper_absolute_smooth_v1"])
    command = launcher.commands(args, extra)[0]
    assert len(launcher.commands(args, extra)) == 1
    assert "run_xr_teleop_ammr_sim_ros2.sh" in command[1]
    assert "ammr_sim/scripts/run_sim.sh" not in " ".join(command)


@pytest.mark.parametrize("mode", ([], ["--sim"], ["--real"]))
def test_record_is_forwarded_to_both_teleop_modes(mode):
    args, extra = launcher.parse(["teleop", *mode, "--record"])
    command = launcher.commands(args, extra)[0]
    assert command.count("--record") == 1


def test_hardware_output_is_rejected_in_simulation():
    with pytest.raises(SystemExit):
        launcher.parse(["teleop", "--sim", "--enable-hardware-output"])


def test_record_is_rejected_for_replay():
    with pytest.raises(SystemExit):
        launcher.parse(["replay", "--record"])


def test_replay_routes_by_mode():
    sim_args, sim_extra = launcher.parse(["replay", "--sim", "--validate-only"])
    real_args, real_extra = launcher.parse(["replay", "--real", "--validate-only"])
    assert "run_replay_ammr_sim_ros2.sh" in launcher.commands(sim_args, sim_extra)[0][1]
    assert "run_replay_ammr_real_ros2.sh" in launcher.commands(real_args, real_extra)[0][1]


def test_sim_replay_backend_marks_shared_replayer_as_simulation():
    backend = path.parent / "backends/run_replay_ammr_sim_ros2.sh"
    contents = backend.read_text(encoding="utf-8")
    assert "--target-mode sim" in contents
    assert "--enable-hardware-output" in contents


@pytest.mark.parametrize("flag", ("--smoothing", "--no-smoothing"))
def test_replay_forwards_smoothing_choice(flag):
    args, extra = launcher.parse(["replay", "--sim", flag, "--validate-only"])
    command = launcher.commands(args, extra)[0]
    assert flag in command


def test_dataset_inspector_uses_dedicated_backend():
    args, extra = launcher.parse(
        ["inspect-dataset", "--dataset-path", "/tmp/example dataset"]
    )
    command = launcher.commands(args, extra)[0]
    assert "run_dataset_inspector.sh" in command[1]
    assert command[-2:] == ["--dataset-path", "/tmp/example dataset"]


def test_plan_does_not_start_processes(capsys):
    assert launcher.main(["teleop", "--sim", "--record", "--plan"]) == 0
    output = capsys.readouterr().out
    assert '"mode": "sim"' in output
    assert '"record": true' in output
