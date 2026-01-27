import subprocess
import sys
from pyPS4Controller.controller import Controller

# ==========================
# PATHS TO YOUR SCRIPTS
# ==========================

TEMP_HUM_SCRIPT = "/home/rapi6/Biomimicry_group6/System/hum_temp.py"
IMAGE_CAPTURE_SCRIPT = "/home/rapi6/Biomimicry_group6/System/pipeline_image_capture.py"
# ↑ change filename if your pipeline script has a different name


class TestController(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # ==========================
    # FACE BUTTONS
    # ==========================

    def on_circle_press(self):
        print("O button pressed → Reading temperature & humidity...")
        try:
            subprocess.run([sys.executable, TEMP_HUM_SCRIPT], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running temp/humidity script: {e}")

    def on_square_press(self):
        print("Square pressed")

    def on_triangle_press(self):
        print("Triangle pressed")
        try:
            subprocess.run([sys.executable, IMAGE_CAPTURE_SCRIPT], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running image capture script: {e}")

    # ==========================
    # L1 / R1
    # ==========================

    def on_L1_press(self):
        print("L1 pressed")

    def on_R1_press(self):
        print("R1 pressed")

    # ==========================
    # D-PAD
    # ==========================

    def on_up_press(self):
        print("D-pad Up pressed")

    def on_down_press(self):
        print("D-pad Down pressed")

    def on_left_press(self):
        print("D-pad Left pressed")

    def on_right_press(self):
        print("D-pad Right pressed")

    # ==========================
    # JOYSTICKS
    # ==========================

    def on_L3_up(self, value):
        print(f"L3 up: {value}")

    def on_L3_down(self, value):
        print(f"L3 down: {value}")

    def on_L3_left(self, value):
        print(f"L3 left: {value}")

    def on_L3_right(self, value):
        print(f"L3 right: {value}")

    # ==========================
    # SYSTEM BUTTONS
    # ==========================

    def on_options_press(self):
        print("Options pressed")

    def on_share_press(self):
        print("Share pressed")

    def on_ps_press(self):
        print("PS button pressed")


# ==========================
# MAIN
# ==========================

if __name__ == "__main__":
    controller = TestController(
        interface="/dev/input/js0",
        connecting_using_ds4drv=False
    )

    print("Controller ready:")
    print("  X  → Image capture & grain analysis")
    print("  O  → Temperature & humidity\n")

    controller.listen(timeout=999999)
