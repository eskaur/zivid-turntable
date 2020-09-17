"""Module for capturing point clouds"""

import time
from datetime import timedelta
import itertools
from pathlib import Path

import zivid

from .io import create_casedir
from .arduino_com import ArduinoCom


def get_settings(camera: zivid.Camera, capture_budget_seconds: float) -> zivid.Settings:
    """Get optimal settings based on current scene

    Arguments:
        camera:                 A Zivid camera
        capture_budget_seconds: Capture budget in seconds
    Returns:
        A zivid.Settings instance
    """

    # Run capture assistant
    ca_params = zivid.capture_assistant.SuggestSettingsParameters(
        max_capture_time=timedelta(seconds=capture_budget_seconds),
        ambient_light_frequency="hz50",
    )
    settings = zivid.capture_assistant.suggest_settings(camera, ca_params)

    # Adjust filter settings
    settings.processing.filters.smoothing.gaussian.enabled = False
    settings.processing.filters.noise.removal.threshold = 15
    settings.processing.filters.outlier.removal.threshold = 2.0
    settings.processing.filters.experimental.contrast_distortion.correction.enabled = (
        True
    )
    settings.processing.filters.experimental.contrast_distortion.removal.enabled = True
    settings.processing.filters.experimental.contrast_distortion.removal.threshold = 0.2

    return settings


def manual_capture_loop(
    label: str,
    camera: zivid.Camera,
    capture_budget_seconds: float = 1.2,
    workdir: Path = Path("."),
) -> None:
    """Capture frames by manual operation

    Arguments:
        label:                  Label for the case
        camera:                 A Zivid camera
        capture_budget_seconds: Capture budget in seconds
        workdir                 Path to root directory to create case in
    """

    # Create case directory
    dirpath = create_casedir(label, workdir)

    # Get settings
    settings = get_settings(camera, capture_budget_seconds)

    # Loop until manually aborted
    for i in itertools.count(start=0):

        try:
            input("Press any key to capture (ctrl-C to abort)")
        except KeyboardInterrupt:
            print("\n\nManually aborted")
            break

        filename = f"frame_{i:02d}.zdf"
        print(f"Capturing frame: {filename}")
        with camera.capture(settings) as frame:
            frame.save(dirpath / filename)


def auto_capture(
    label: str,
    camera: zivid.Camera,
    n_images: int,
    capture_budget_seconds: float,
    workdir: Path = Path("."),
) -> None:
    """Auto-capture with turntable

    Arguments:
        label:                  Label to store data with (set None for dry-run)
        camera:                 Zivid camera to capture with
        n_images:               Number of images to capture
        capture_budget_seconds: Time-budget per capture
        workdir                 Path to root directory to create case in
    """

    # Create case directory
    if label is not None:
        dirpath = create_casedir(label, workdir)

    # Get settings
    settings = get_settings(camera, capture_budget_seconds)

    # Connect to motor
    motor_controller = ArduinoCom()
    steps_per_rev = motor_controller.get_steps_per_rev()
    steps_per_move = int(steps_per_rev / n_images)
    time.sleep(1)

    for i in range(n_images):

        filename = f"frame_{i:02d}.zdf"
        print(f"Capturing frame: {filename}")
        with camera.capture(settings) as frame:
            if label is not None:
                frame.save(dirpath / filename)

        print(f"Sending move signal: {steps_per_move}")
        motor_controller.move_steps(steps_per_move)
        print("Move done")
