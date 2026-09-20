import logging
from pathlib import Path
import tempfile
import unittest

from ammr_teleop.common.event_logging import AmmrEventLogger


class AmmrEventLoggingTest(unittest.TestCase):
    def test_writes_structured_event_to_session_file(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = AmmrEventLogger("real", directory)
            path = logger.path
            logger.info("session_start", "teleoperation starting", domain=101)
            logger.warning("camera_stale", left_ms=620.0)
            logger.close()

            text = Path(path).read_text(encoding="utf-8")
            self.assertIn("[AMMR][REAL][SESSION_START]", text)
            self.assertIn("domain=101", text)
            self.assertIn("[AMMR][REAL][CAMERA_STALE]", text)
            self.assertIn("left_ms=620.000", text)

    def test_close_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = AmmrEventLogger("sim", directory)
            logger.close()
            logger.close()
            logger.log(logging.INFO, "ignored")


if __name__ == "__main__":
    unittest.main()
