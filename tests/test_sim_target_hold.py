import numpy as np

from ammr_teleop.common.sim_target_hold import SimTargetHold


def test_holds_each_wrist_independently_and_copies_targets():
    policy = SimTargetHold()
    left, right = np.eye(4), np.eye(4)
    assert policy.held(left, right) == (False, False)
    policy.accept(left, right)
    assert policy.held(left, right) == (True, True)
    left[0, 3] = 0.01
    assert policy.held(left, right) == (False, True)
    policy.accept(left, right)
    right[1, 3] = 0.01
    assert policy.held(left, right) == (True, False)


def test_rejected_target_remains_retryable_and_reset_clears_history():
    policy = SimTargetHold()
    left, right = np.eye(4), np.eye(4)
    policy.accept(left, right)
    left[0, 3] = 0.02
    assert policy.held(left, right) == (False, True)
    assert policy.held(left, right) == (False, True)
    policy.clear()
    assert policy.held(left, right) == (False, False)
