"""
PS4 controller test script.

Maps controller buttons to system actions:
- Circle: run temperature & humidity script
- Triangle: run image capture
- Other buttons: print debug messages to terminal

Uses pyPS4Controller to read input events.
"""

import subprocess
import sys
from pyPS4Controller.controller import Controller

# Paths to external scripts triggered by controller buttons
TEMP_HUM_SCRIPT = "/home/rapi6/Biomimicry_group6/System/hum_temp.py"
IMAGE_CAPTURE_SCRIPT = "/home/rapi6/Biomimicry_group6/System/pipeline_image_capture.py"


class TestController(Controller):
    """Custom controller class mapping buttons to actions."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # Run temperature & humidity measurement script
    def on_circle_press(self):
        try:
            subprocess.run([sys.executable, TEMP_HUM_SCRIPT], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running temp/humidity script: {e}")

    def on_square_press(self):
        print("Square pressed")

    # Run image capture / grain analysis pipeline
    def on_triangle_press(self):
        try:
            subprocess.run([sys.executable, IMAGE_CAPTURE_SCRIPT], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running image capture script: {e}")

    def on_L1_press(self):
        print("L1 pressed")

    def on_R1_press(self):
        print("R1 pressed")

    # D-pad directions (debug output only)
    def on_up_press(self):
        print("D-pad Up pressed")

    def on_down_press(self):
        print("D-pad Down pressed")

    def on_left_press(self):
        print("D-pad Left pressed")

    def on_right_press(self):
        print("D-pad Right pressed")

    # Left joystick movement reporting
    def on_L3_up(self, value):
        print(f"L3 up: {value}")

    def on_L3_down(self, value):
        print(f"L3 down: {value}")

    def on_L3_left(self, value):
        print(f"L3 left: {value}")

    def on_L3_right(self, value):
        print(f"L3 right: {value}")

    def on_options_press(self):
        print("Options pressed")

    def on_share_press(self):
        print("Share pressed")

    def on_ps_press(self):
        print("PS button pressed")


if __name__ == "__main__":
    # Initialize controller listener
    controller = TestController(
        interface="/dev/input/js0",
        connecting_using_ds4drv=False
    )

    print("Controller ready:")
    print("  Triangle: Image capture & grain analysis")
    print("  O: Temperature & humidity\n")

    controller.listen(timeout=999999)
