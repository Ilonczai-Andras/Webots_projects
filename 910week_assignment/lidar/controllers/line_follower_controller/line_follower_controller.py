"""line_follower_controller controller."""

from scipy import signal
from controller import Robot
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
robot = Robot()
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28                 # kerék maximális szögsebessége [rad/s]
FORWARD_SPEED = MAX_SPEED / 5    # alap előremeneti sebesség (MAX_SPEED ötöde)
Kp = 0.0090                      # arányos szabályozó erősítés (hangolható)
WHEEL_RADIUS = 0.0201            # kerék sugara [m]
AXLE_LENGTH = 0.05725            # tengelyhossz [m]

goal_counter = 0
GOAL_THRESHOLD = 29

map = np.zeros((300,300))

# --- Motorok beállítása ---
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# --- Szenzorok beolvasása ---
ir_sensors = []
sensor_names = ['gs0', 'gs1', 'gs2']
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ir_sensors.append(sensor)

lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

display = robot.getDevice('display')
display.setColor(0xFF0000)

print("Robot elindult, arányos vonalkövetés és odometria aktív...\n")

# --- Fő ciklus ---
while robot.step(timestep) != -1:
    # Szenzorértékek beolvasása
    
    xw = gps.getValues()[0]        # x pozíció [m]
    yw = gps.getValues()[1]        # y pozíció [m] (startvonal előtt)
    omegaz = np.arctan2(compass.getValues()[0], compass.getValues()[1]) # orientáció (yaw) [rad]
    
    px, py = world2map(xw, yw)
    display.setColor(0xFF0000)
    display.drawPixel(px, py)
    
    g = [s.getValue() for s in ir_sensors]
    
    ranges = np.array(lidar.getRangeImage())
    angles = np.linspace(np.pi, -np.pi, len(ranges))
    
    valid = np.isfinite(ranges) & (ranges > 0) & (ranges < 5.0)
    ranges = ranges[valid]
    angles = angles[valid]
    
    #fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    #ax.plot(angles,ranges,'.')
    #ax.set_theta_offset(np.pi / 2)
    #plt.show()
    
    x_i = np.array([
        ranges * np.cos(angles),
        ranges * np.sin(angles),
        np.ones(len(angles))])
    
    w_T_r = np.array([
        [np.cos(omegaz), -np.sin(omegaz), xw],
        [np.sin(omegaz),  np.cos(omegaz), yw],
        [0, 0, 1]])
        
    D = w_T_r @ x_i
    
    for i in range(D.shape[1]):
        px, py = world2map(D[0, i], D[1, i])
        px, py = int(px), int(py)
    
        if 0 <= px < 300 and 0 <= py < 300:

            map[px, py] = min(1.0, map[px, py] + 0.01)
            
            v = int(map[px, py] * 255)
            
            color = (v * 256**2 + v * 256 + v)
            display.setColor(color)
            display.drawPixel(px, py)
    
    #plt.ion()
    #plt.plot(D[0, :], D[1, :], '.')
    #plt.axis('equal')
    #plt.pause(0.01)
    #plt.show()
    #plt.clf()
            
    # --- Arányos szabályozás a két szélső szenzor különbségére ---
    error = g[0] - g[2]   # bal - jobb szenzor
    correction = Kp * error

    # A korrekció ellentétesen hat a két kerékre
    phildot = FORWARD_SPEED + correction    # bal kerék
    phirdot = FORWARD_SPEED - correction    # jobb kerék

    # --- Odometria számítás ---
    TIME_STEP_SEC = timestep / 1000.0
    delta_phi_l = phildot * TIME_STEP_SEC
    delta_phi_r = phirdot * TIME_STEP_SEC

    delta_x_local = (WHEEL_RADIUS * (delta_phi_l + delta_phi_r) / 2.0)
    delta_theta = (WHEEL_RADIUS * (delta_phi_r - delta_phi_l) / AXLE_LENGTH)

    # Világkoordinátás pozíció frissítése
    omegaz += delta_theta
    xw += delta_x_local * math.cos(omegaz)
    yw += delta_x_local * math.sin(omegaz)

    # --- Pozícióhiba számítása az origóhoz ---
    position_error = np.sqrt(xw**2 + yw**2)

    # --- Kiírás ---
    #print(f"xw={xw:.3f} m | yw={yw:.3f} m | omegaz={math.degrees(omegaz):.2f}° | "
    #      f"Hiba az origótól: {position_error:.3f} m")
          
    if all(295 <= val <= 305 for val in g):
        goal_counter += 1
        #print(f"Célvonal detektálás számláló: {goal_counter}")
    else:
        goal_counter = 0  # visszaállítjuk, ha kiesett a sávból

    # Ha már 6 egymás utáni alkalommal a célvonalon van → megáll
    if goal_counter >= GOAL_THRESHOLD:
        print(f"\nCÉLVONAL ÉRZÉKELVE ({goal_counter} egymást követő alkalommal) → Robot megáll!\n")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break

    # --- Sebességek beállítása ---
    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)

kernel = np.ones((40, 40))
cmap = signal.convolve2d(map, kernel, mode='same')
cspace = cmap > 0.9

plt.imshow(cspace, cmap='cividis')
plt.title("Konfigurációs tér")
plt.show()