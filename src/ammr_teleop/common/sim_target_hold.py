"""Legacy simulation wrist-target hold policy, without Isaac dependencies."""

import numpy as np


class SimTargetHold:
    def __init__(self):
        self.clear()

    def clear(self):
        self.targets = None

    def held(self, left, right):
        return tuple(
            self.targets is not None and np.array_equal(target, self.targets[index])
            for index, target in enumerate((left, right))
        )

    def accept(self, left, right):
        # Only latch successful solves, so rejected targets can be retried.
        self.targets = (left.copy(), right.copy())
