"""Small target-independent adapter around TeleVuerWrapper."""

from __future__ import annotations

import sys

import numpy as np


def _load_televuer_wrapper():
    """Import TeleVuer without letting its app framework consume our CLI."""
    argv = sys.argv
    try:
        sys.argv = [argv[0]]
        from televuer import TeleVuerWrapper
    finally:
        sys.argv = argv
    return TeleVuerWrapper


class XRInputFrontend:
    def __init__(
        self,
        input_mode: str,
        display_mode: str,
        image_shape: tuple[int, int],
        display_fps: float,
    ):
        if input_mode not in ("controller", "hand"):
            raise ValueError(f"unsupported XR input mode: {input_mode}")
        self.input_mode = input_mode
        self.display_mode = display_mode
        televuer_wrapper = _load_televuer_wrapper()
        self.wrapper = televuer_wrapper(
            use_hand_tracking=input_mode == "hand",
            binocular=False,
            img_shape=image_shape,
            display_fps=display_fps,
            display_mode=display_mode,
            zmq=display_mode != "pass-through",
            webrtc=False,
        )

    def sample(self):
        """Return the latest shared XR state, including the previous state on dropout."""
        return self.wrapper.get_tele_data()

    def render(self, image_bgr: np.ndarray) -> None:
        if self.display_mode != "pass-through":
            self.wrapper.render_to_xr(image_bgr)

    def close(self) -> None:
        self.wrapper.close()


def controller_open_fraction(trigger_value: float) -> float:
    """TeleVuer exposes controller trigger as 10=open through 0=pressed."""
    return float(np.clip(float(trigger_value) / 10.0, 0.0, 1.0))


def hand_open_fraction(pinch_value: float) -> float:
    """Map TeleVuer's thumb/index distance to closed=0 through open=1."""
    closed_distance = 5.0
    open_distance = 7.0
    fraction = (float(pinch_value) - closed_distance) / (
        open_distance - closed_distance
    )
    return float(np.clip(fraction, 0.0, 1.0))
