"""line_follower_controller controller."""

from controller import Robot
import numpy as np
import math

# --- Alap konstansok ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28                 # kerék maximális szögsebessége [rad/s]
FORWARD_SPEED = MAX_SPEED / 5  # alap előremeneti sebesség (MAX_SPEED ötöde)
Kp = 0.0090                      # arányos szabályozó erősítés (hangolható)
WHEEL_RADIUS = 0.0201            # kerék sugara [m]
AXLE_LENGTH = 0.052              # tengelyhossz [m]

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

# --- Kezdeti pozíció a világban ---
xw = 0.0       # x pozíció [m]
yw = 0.028     # y pozíció [m] (startvonal előtt)
omegaz = 0.0   # orientáció (yaw) [rad]

print("Robot elindult, arányos vonalkövetés és odometria aktív...\n")

# --- Fő ciklus ---
while robot.step(timestep) != -1:
    # Szenzorértékek beolvasása
    g = [s.getValue() for s in ir_sensors]
    print(g)
    
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

    delta_x_local = WHEEL_RADIUS * (delta_phi_l + delta_phi_r) / 2.0
    delta_theta = WHEEL_RADIUS * (delta_phi_r - delta_phi_l) / AXLE_LENGTH

    # Világkoordinátás pozíció frissítése
    omegaz += delta_theta
    xw += delta_x_local * math.cos(omegaz)
    yw += delta_x_local * math.sin(omegaz)

    # --- Pozícióhiba számítása az origóhoz ---
    position_error = np.sqrt(xw**2 + yw**2)

    # --- Kiírás ---
    #print(f"xw={xw:.3f} m | yw={yw:.3f} m | omegaz={math.degrees(omegaz):.2f}° | "
    #      f"Hiba az origótól: {position_error:.3f} m")

    # --- Sebességek beállítása ---
    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)
