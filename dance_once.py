#!/usr/bin/env python3
import time
from pymycobot.mycobot import MyCobot

try:
    mc = MyCobot('/dev/ttyTHS1', 1000000)
    mc.power_on()
except Exception as e:
    exit(1)

mc.send_angles([0, 0, 0, 0, 0, 0], 50)
time.sleep(2)
mc.send_angles([30, -10, -20, 0, 0, 0], 60)
time.sleep(1)
mc.send_angles([-30, -10, -20, 0, 0, 0], 60)
time.sleep(1)
mc.send_angles([0, -20, -30, -10, 0, 0], 60)
time.sleep(1)
mc.send_angles([0, -5, -15, -20, 0, 0], 80)
time.sleep(0.5)
mc.send_angles([0, -25, -35, 10, 0, 0], 80)
time.sleep(0.5)
mc.send_angles([0, -5, -15, -20, 0, 0], 80)
time.sleep(0.5)
mc.send_angles([0, 0, 0, 0, 0, 0], 50)
time.sleep(2)
