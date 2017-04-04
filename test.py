import time

from dobot import DobotMagician

arm = DobotMagician(port='/dev/cu.wchusbserial1410')

time.sleep(0.5)
arm.move(250.0, 0.0, -25.0)
time.sleep(2)
arm.move(250.0, 0.0, 25.0)
time.sleep(2)
arm.close()
