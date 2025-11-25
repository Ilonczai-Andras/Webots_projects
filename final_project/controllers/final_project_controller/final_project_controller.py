"""line_follower_controller controller."""

from scipy import signal
from controller import Supervisor
import numpy as np
import math
from matplotlib import pyplot as plt
import matplotlib
matplotlib.use('TkAgg')


def world2map(xw, yw):

    xmin = - 0.2
    xmax = 0.83
    px = (xw - xmin)*(300/(xmax - xmin))
    
    ymin = - 0.24
    ymax = 0.81
    py = (ymax - yw)*(300/(ymax - ymin))

    return[px,py]

# --- Alap konstansok ---
robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28                 # kerék maximális szögsebessége [rad/s]
FORWARD_SPEED = MAX_SPEED / 5    # alap előremeneti sebesség (MAX_SPEED ötöde)
Kp = 0.0090                      # arányos szabályozó erősítés (hangolható)
WHEEL_RADIUS = 0.0201            # kerék sugara [m]
AXLE_LENGTH = 0.05725            # tengelyhossz [m]

p1 = 1.0    # irányhiba (alpha) erősítése
p2 = 1.0    # távolsághiba (rho) erősítése

goal_counter = 0
GOAL_THRESHOLD = 29

map = np.zeros((300,300))

# --- Motorok beállítása ---
# --- Motors ---
leftMotor = robot.getDevice("wheel_left_joint")
rightMotor = robot.getDevice("wheel_right_joint")

# velocity control mode:
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# --- Sensors (if needed) ---
leftSensor = robot.getDevice("wheel_left_joint_sensor")
rightSensor = robot.getDevice("wheel_right_joint_sensor")
leftSensor.enable(timestep)
rightSensor.enable(timestep)

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

display = robot.getDevice('display')
display.setColor(0xFF0000)

lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

marker_node = robot.getFromDef('MARKER')
translation_field = marker_node.getField('translation')

print("Robot elindult, arányos vonalkövetés és odometria aktív...\n")

WP = [(0.64, 0.0), (0.85, -1.3), (-0.52, -3.76), (-1.81, -3.16), (-1.63, -1.27), (-1.35, 0.11), (-1.35, 0.55), (-1.79, -1.42), (-1.79, -3.23), (0.63, -3.23), (0.63, -1.49), (0.31, -0.15)]
index = 0

while robot.step(timestep) != -1:
    # Szenzorértékek beolvasása
    
    xw = gps.getValues()[0]        # x pozíció [m]
    yw = gps.getValues()[1]        # y pozíció [m] (startvonal előtt)
    omegaz = np.arctan2(compass.getValues()[0], compass.getValues()[1]) # orientáció (yaw) [rad]
    
    px, py = world2map(xw, yw)
    display.setColor(0xFF0000)
    display.drawPixel(px, py)
    
    ranges = np.array(lidar.getRangeImage())
    angles = np.linspace(np.pi, -np.pi, len(ranges))
    
    valid = np.isfinite(ranges) & (ranges > 0) & (ranges < 5.0)
    ranges = ranges[valid]
    angles = angles[valid]
    
    x_i = np.array([
        ranges * np.cos(angles),
        ranges * np.sin(angles),
        np.ones(len(angles))])
    
    w_T_r = np.array([
        [np.cos(omegaz), -np.sin(omegaz), xw],
        [np.sin(omegaz),  np.cos(omegaz), yw],
        [0, 0, 1]])
        
    D = w_T_r @ x_i
    
    
    # Valószínűségi térképezés
    for i in range(D.shape[1]):
        px, py = world2map(D[0, i], D[1, i])
        px, py = int(px), int(py)
    
        if 0 <= px < 300 and 0 <= py < 300:

            map[px, py] = min(1.0, map[px, py] + 0.01)
            
            v = int(map[px, py] * 255)
            
            color = (v * 256**2 + v * 256 + v)
            display.setColor(color)
            display.drawPixel(px, py)

    # Távolság a célponttól (rho)
    rho = np.sqrt((WP[index][0] - xw)**2 + (WP[index][1] - yw)**2)
    # Irányhiba (alpha), ami a célpont iránya és a robot irányának különbsége
    alpha = np.arctan2(WP[index][1] - yw, WP[index][0] - xw) - omegaz
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
    #print(f"Rho: {rho}, Alpha: {math.degrees(alpha)} {math.degrees(omegaz)}")
    
    if rho < 0.1:
        if not (len(WP) - 1 == index):
            index += 1
            translation_field.setSFVec3f([WP[index][0], WP[index][1], 0.0])
    
    # --- ÚJ: ρ–α alapú arányos szabályozó ---
    leftSpeed_cmd  = -alpha * p1 + rho * p2
    rightSpeed_cmd =  alpha * p1 + rho * p2
    
    # --- ÚJ: kerék sebesség limit ---
    leftSpeed  = max(min(leftSpeed_cmd,  MAX_SPEED), -MAX_SPEED)
    rightSpeed = max(min(rightSpeed_cmd, MAX_SPEED), -MAX_SPEED)
    
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)


""" # Konfigurációs tér
kernel = np.ones((20, 20))
cmap = signal.convolve2d(map, kernel, mode='same')
cspace = cmap > 0.9

plt.imshow(cspace, cmap='cividis')
plt.title("Konfigurációs tér")
plt.show()
"""