"""Modules that make shit work"""

import serial.tools.list_ports


class ArduinoCom:
    """Class to handle communications and info on motor control
    -Sets serial port communications
    -Sets relative number of steps to move to from current position
    -Returns: steps per a revolution, and degrees per step
    """

    degrees = 1.8
    steps_per_rev = 200

    def __init__(self) -> None:
        """Checks for connection of Arduino and returns port connected
        and sets baud rate
        """
        use_port = ""
        port_list = list(serial.tools.list_ports.comports())
        for port in port_list:
            if (
                "VID:PID=2341:0043" in port[0]
                or "VID:PID=2341:0043" in port[1]
                or "VID:PID=2341:0043" in port[2]
            ):
                use_port = port[0]
        print(use_port)
        serial_uno = serial.Serial(use_port, 9600)
        self.serial_connect = serial_uno

    def get_degrees_per_step(self) -> float:
        """Returns degrees per a step of the motor"""
        return self.degrees

    def get_steps_per_rev(self) -> int:
        """Returns steps per a full revolution of motor"""
        return self.steps_per_rev

    def move_steps(self, steps: int) -> None:
        """Send number of steps to move to the arduino"""
        steps_b = str(steps).encode()
        self.serial_connect.write(steps_b)
        self.serial_connect.read()
