import sys
import os
import time

sys.path.append("/home/rapi6/Biomimicry_group6/DFRobot_RaspberryPi_A02YYUW")

from DFRobot_RaspberryPi_A02YYUW import DFRobot_A02_Distance as Board

board = Board()

MIN_RANGE = 0
MAX_RANGE = 4500

STOP_DISTANCE = 500     
SLOW_DISTANCE = 1200    

board.set_dis_range(MIN_RANGE, MAX_RANGE)
print("Obstacle avoidance running")

try:
    while True:
        distance = board.getDistance()

        if board.last_operate_status == board.STA_OK:
            print(f"Distance: {distance} mm")

            if distance <= STOP_DISTANCE:
                print("STOP")

            elif distance <= SLOW_DISTANCE:
                print("SLOW / TURN")

            else:
                print("MOVE FORWARD")

        elif board.last_operate_status == board.STA_ERR_DATA:
            print("No data received")

        elif board.last_operate_status == board.STA_ERR_SERIAL:
            print("Serial error")

        time.sleep(0.3) 

except KeyboardInterrupt:
    print("\nStopped by user")
