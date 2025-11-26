"""line_follower_controller controller."""

from scipy import signal
from controller import Supervisor
import numpy as np
import math
from matplotlib import pyplot as plt
import matplotlib
matplotlib.use('TkAgg')


def world2map(xw, yw):
    xmin = -2.47
    xmax = 2.34
    
    ymin = -3.89
    ymax = 1.67
    
    map_width = 200.0   # X tengely terjedelme
    map_height = 300.0  # Y tengely terjedelme
    px = (xw - xmin) * (map_width / (xmax - xmin))
    
    py = (yw - ymin) * (map_height / (ymax - ymin))

    return [px, py]

# --- Alap konstansok ---
robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28                 # kerék maximális szögsebessége [rad/s]
FORWARD_SPEED = MAX_SPEED / 6    # alap előremeneti sebesség (MAX_SPEED ötöde)
Kp = 0.0090                      # arányos szabályozó erősítés (hangolható)
WHEEL_RADIUS = 0.0201            # kerék sugara [m]
AXLE_LENGTH = 0.05725            # tengelyhossz [m]

p1 = 1.0    # irányhiba (alpha) erősítése
p2 = 1.0    # távolsághiba (rho) erősítése

goal_counter = 0
GOAL_THRESHOLD = 29

map = np.zeros((200, 300))

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
#lidar.enablePointCloud()

marker_node = robot.getFromDef('MARKER')
translation_field = marker_node.getField('translation')

print("Robot elindult, arányos vonalkövetés és odometria aktív...\n")

WP = [(0.64, 0.0), (0.85, -1.3), (0.57, -3.18), (-0.52, -3.25), (-1.52, -3.18), (-1.70, -1), (-1.28, 0.07), (-0.47, 0.02)]
index = 0
step_direction = 1

translation_field.setSFVec3f([WP[index][0], WP[index][1], 0.0])

while robot.step(timestep) != -1:
    # Szenzorértékek beolvasása
    
    xw = gps.getValues()[0]        # x pozíció [m]
    yw = gps.getValues()[1]        # y pozíció [m] (startvonal előtt)
    omegaz = np.arctan2(compass.getValues()[0], compass.getValues()[1]) # orientáció (yaw) [rad]
    
    px, py = world2map(xw, yw)
    px, py = int(px), int(py)
    display.setColor(0xFF0000)
    display.drawPixel(px, 299 - py)
    
    full_ranges = np.array(lidar.getRangeImage())
    
    fov = lidar.getFov()
    full_angles = np.linspace(fov / 2, -fov / 2, len(full_ranges))
    
    ranges = full_ranges[80:-80]
    angles = full_angles[80:-80]
    
    ranges[np.isinf(ranges)] = 100.0
    valid = ranges > 0

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
    
    r_T_l = np.array([
        [1, 0, 0.202],
        [0, 1, 0],
        [0, 0, 1]])
    
    w_T_l = w_T_r @ r_T_l
        
    D = w_T_l @ x_i
        
    for i in range(D.shape[1]):
        px, py = world2map(D[0, i], D[1, i])
        px, py = int(px), int(py)
    
        if 0 <= px < 200 and 0 <= py < 300:

            map[px, py] = min(1.0, map[px, py] + 0.01)
            
            v = int(map[px, py] * 255)
            if map[px, py] > 0.2:
                 color = (v * 256**2 + v * 256 + v)
                 display.setColor(color)
                 display_y_flipped = 299 - py
                 display.drawPixel(px, display_y_flipped)

    # Távolság a célponttól (rho)
    rho = np.sqrt((WP[index][0] - xw)**2 + (WP[index][1] - yw)**2)
    
    # Irányhiba (alpha), ami a célpont iránya és a robot irányának különbsége
    alpha = np.arctan2(WP[index][1] - yw, WP[index][0] - xw) - omegaz
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
    #print(f"Rho: {rho}, Alpha: {math.degrees(alpha)} {math.degrees(omegaz)}")
    translation_field.setSFVec3f([WP[index][0], WP[index][1], 0.0])
    
    if rho < 0.3:
        if index == len(WP) - 1:
            step_direction = -1
            print("Utolsó pont elérve. Fordulás visszafelé.")
            index += step_direction
            translation_field.setSFVec3f([WP[index][0], WP[index][1], 0.0])

        elif index == 0 and step_direction == -1:
            print("Visszaértünk a startra. Program vége.")
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            break

        else:
            index += step_direction
            translation_field.setSFVec3f([WP[index][0], WP[index][1], 0.0])
    
    # --- ÚJ: ρ–α alapú arányos szabályozó ---
    leftSpeed_cmd  = -alpha * p1 + rho * p2
    rightSpeed_cmd =  alpha * p1 + rho * p2
    
    # --- ÚJ: kerék sebesség limit ---
    leftSpeed  = max(min(leftSpeed_cmd,  MAX_SPEED), -MAX_SPEED)
    rightSpeed = max(min(rightSpeed_cmd, MAX_SPEED), -MAX_SPEED)
    
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

# Konfigurációs tér
binary_map = map > 0.3 
kernel_size = 26 
kernel = np.ones((kernel_size, kernel_size))
cmap = signal.convolve2d(binary_map.astype(float), kernel, mode='same')
cspace = cmap > 0.9
# Megjelenítés
plt.figure(figsize=(10, 10))
plt.imshow(cspace, cmap='cividis')
plt.title("Konfigurációs tér")
plt.show()