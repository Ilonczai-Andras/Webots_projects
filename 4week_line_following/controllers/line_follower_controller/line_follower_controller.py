"""line_follower_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
import math

# create the Robot instance.
robot = Robot()


MAX_SPEED = 6.28 / 2
WHEEL_RADIUS = 0.0201
AXLE_LENGTH = 0.052

timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(3.14)
rightMotor.setVelocity(3.14)

ir_sensors = []
sensor_names = ['gs0', 'gs1', 'gs2']
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ir_sensors.append(sensor)
    
distance_traveled = 0.0  # m
orientation = 0.0  # rad

while robot.step(timestep) != -1:
    g = [s.getValue() for s in ir_sensors]
    print(g)

    phildot = 0.0
    phirdot = 0.0

    if g[0] > 500 and g[1] < 350 and g[2] > 500:
        phildot = MAX_SPEED
        phirdot = MAX_SPEED
    elif g[2] < 550:
        phildot = 0.25 * MAX_SPEED
        phirdot = -0.1 * MAX_SPEED
    elif g[0] < 550:
        phildot = -0.1 * MAX_SPEED
        phirdot = 0.25 * MAX_SPEED
    else:
        phildot = 0.2 * MAX_SPEED
        phirdot = 0.2 * MAX_SPEED

    TIME_STEP_SEC = timestep / 1000 #s

    delta_phi_l = phildot * TIME_STEP_SEC
    delta_phi_r = phirdot * TIME_STEP_SEC

    delta_x = (WHEEL_RADIUS * (delta_phi_l + delta_phi_r) / 2.0)
    delta_theta = (WHEEL_RADIUS * (delta_phi_r - delta_phi_l) / AXLE_LENGTH)

    distance_traveled += delta_x
    orientation += delta_theta

    print(f"Megtett távolság: {distance_traveled:.3f} m | Orientáció: {math.degrees(orientation):.2f}°")

    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)
