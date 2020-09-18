"""Script for capture data from turntable

Run this if the USB access is denied:
> sudo chmod 666 /dev/ttyACM2

"""
import argparse

import zivid
from zivid_turntable.capture import auto_capture


def _main() -> None:

    # Get args
    parser = argparse.ArgumentParser(description="Capture turntable data")
    parser.add_argument("images", type=int, help="number of images")
    parser.add_argument("--label", type=str, help="label for dataset")
    args = parser.parse_args()
    print(args)

    # Connect to camera
    app = zivid.Application()
    cam = app.connect_camera()

    # Start capture loop
    auto_capture(
        label=args.label, camera=cam, n_images=args.images, capture_budget_seconds=1.2
    )


if __name__ == "__main__":
    _main()
