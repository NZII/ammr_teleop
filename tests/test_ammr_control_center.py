from types import SimpleNamespace

from ammr_teleop.real.control_center_web import WEB_PAGE, WebControlCenter, parse_args


def _command(left_enable: bool, right_enable: bool) -> str:
    center = WebControlCenter.__new__(WebControlCenter)
    center.args = SimpleNamespace(
        domain_id=101,
        display_mode="pass-through",
        task="test",
    )
    center.left_enable = left_enable
    center.right_enable = right_enable
    return center._server_command()


def test_web_control_center_defaults_to_dual_arm_control():
    args = parse_args([])
    assert args.left_enable is True
    assert args.right_enable is True
    assert 'id="leftEnable"' in WEB_PAGE
    assert 'id="rightEnable"' in WEB_PAGE


def test_web_server_command_keeps_both_hardware_arms_for_left_only():
    command = _command(True, False)
    assert "--hardware-components left_arm right_arm lift chassis" in command
    assert "--left-enable --no-right-enable" in command


def test_web_server_command_keeps_both_hardware_arms_for_right_only():
    command = _command(False, True)
    assert "--hardware-components left_arm right_arm lift chassis" in command
    assert "--no-left-enable --right-enable" in command
