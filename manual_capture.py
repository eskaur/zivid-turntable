"""Module for manual capture (for testing)"""

import zivid
from zivid_turntable.capture import manual_capture_loop


def _main() -> None:

    # Connect to camera
    app = zivid.Application()
    cam = app.connect_camera()

    # Capture
    manual_capture_loop("manual_test", cam, 1.2)


if __name__ == "__main__":
    _main()
