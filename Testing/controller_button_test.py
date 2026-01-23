from pyPS4Controller.controller import Controller
from imu_logger import IMULogger
import threading
import time

# Create IMU logger instance
logger = IMULogger()

# Background thread to keep logging if started
def imu_background():
    while True:
        logger.update()
        time.sleep(0.05)

threading.Thread(target=imu_background, daemon=True).start()


class TestController(Controller):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # Triangle → start IMU logging
    def on_triangle_press(self):
        print("🔺 Triangle pressed → START IMU LOG")
        logger.start_logging()

    # Square → stop IMU logging
    def on_square_press(self):
        print("⬛ Square pressed → STOP IMU LOG")
        logger.stop_logging()

    # Options → just print emergency stop
    def on_options_press(self):
        print("⚙️ OPTIONS pressed → EMERGENCY STOP (no motors in this test)")

    # Circle → print message
    def on_circle_press(self):
        print("⭕ Circle pressed → temp/humidity script placeholder")

    # X → print message
    def on_x_press(self):
        print("❌ X pressed → camera script placeholder")

    # L3 up/down/left/right just print
    def on_L3_up(self, value):
        print(f"L3 UP: {value}")

    def on_L3_down(self, value):
        print(f"L3 DOWN: {value}")

    def on_L3_left(self, value):
        print(f"L3 LEFT: {value}")

    def on_L3_right(self, value):
        print(f"L3 RIGHT: {value}")

    def on_L3_x_at_rest(self):
        print("L3 X released")

    def on_L3_y_at_rest(self):
        print("L3 Y released")


# -------- MAIN --------
try:
    controller = TestController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    print("Controller test running. Press buttons to see output.")
    controller.listen(timeout=999999)

except KeyboardInterrupt:
    print("Test stopped by user.")

finally:
    # Stop logging if still running
    logger.stop_logging()
    print("Exiting controller button test.")
