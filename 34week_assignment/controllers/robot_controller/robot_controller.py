from controller import Robot
import math

class State:
    FORWARD_TO_FIRST = "FORWARD_TO_FIRST"
    TURN_180 = "TURN_180"
    FORWARD_TO_SECOND = "FORWARD_TO_SECOND" 
    TURN_RIGHT = "TURN_RIGHT"
    WALL_FOLLOWING = "WALL_FOLLOWING"
    STOPPED = "STOPPED"

class EPuckFSMController:
    def __init__(self):
        # Robot inicializálása
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        if self.robot.getBasicTimeStep() != 64:
            print(f"Warning: Using overridden TIME_STEP={64}ms.")
            self.timestep = 64
        
        # Motorok inicializálása
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Inertial Unit (IMU) inicializálása
        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(self.timestep)
        
        # Távolságérzékelők inicializálása
        self.distance_sensors = []
        for i in range(8):
            sensor = self.robot.getDevice(f'ps{i}')
            sensor.enable(self.timestep)
            self.distance_sensors.append(sensor)
        
        # Paraméterek
        self.MAX_SPEED = 6.28  # rad/s
        self.DISTANCE_THRESHOLD = 80  # Szenzorérték ~0.05m távolságnak megfelelő
        self.TURN_SPEED = 2.5  # Optimális sebesség a hátsó szenzoros forduláshoz
        self.FORWARD_SPEED = 4.0
        self.TARGET_ANGLE = math.pi # 180 fok
        
        # FSM állapot
        self.current_state = State.FORWARD_TO_FIRST
        self.turn_counter = 0
        self.max_turn_duration = 120  # Biztonsági korlát
        
        # Hátsó szenzor alapú fordulás követése
        self.initial_rear_sensor_diff = None
        self.turn_stabilization_counter = 0
        self.stabilization_required = 5  # Hány lépésig kell stabilan jelezniük
        
        print(f"Kezdő állapot: {self.current_state}")
        
    def get_yaw_angle(self, imu):
        """
        Retrieves the current yaw angle (rotation around the vertical/y-axis) 
        from the Inertial Unit.
        
        The IMU returns the Roll, Pitch, and Yaw in that order (x, y, z axis rotation).
        We are interested in the Yaw (index 2).
        """
        orientation = imu.getRollPitchYaw() 
        return orientation[2] 

    def normalize_angle(self, angle):
        """
        Normalizes an angle to the range [-pi, pi].
        This is crucial for robust angle comparison when crossing the +/- pi boundary.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_sensor_values(self):
        """Szenzorértékek beolvasása"""
        return [sensor.getValue() for sensor in self.distance_sensors]
    
    def set_motor_speeds(self, left_speed, right_speed):
        """Motor sebességek beállítása"""
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
    
    def stop_motors(self):
        """Motorok megállítása"""
        self.set_motor_speeds(0.0, 0.0)
        
    def move_forward(self, speed=None):
        """Előremenés"""
        if speed is None:
            speed = self.FORWARD_SPEED
        self.set_motor_speeds(speed, speed)

    def state_forward_to_first(self, sensors):
        """Első akadály felé haladás"""
        if self.detect_obstacle_front(sensors):
            print("Első akadály elérve, 180° fordulás kezdése")
            self.current_state = State.TURN_180
        else:
            self.move_forward()
            
    def state_turn_180(self):
        self.stop_motors()
        self.turn_180_degrees()
            
    def turn_180_degrees(self):
        """
        Performs a precise 180-degree turn in place using the Inertial Unit.
        
        It rotates until the change in angle equals TARGET_ANGLE (pi radians).
        """
        # 1. Get initial yaw angle
        initial_yaw = self.get_yaw_angle(self.imu)
        print(initial_yaw)
        
        # 2. Determine the target yaw angle
        # We want to rotate 'TARGET_ANGLE' (pi) away from the initial yaw.
        target_yaw = initial_yaw + self.TARGET_ANGLE
        print(target_yaw)
        
        # Normalize the target to be within the [-pi, pi] range for comparison
        target_yaw = self.normalize_angle(target_yaw)
        print(target_yaw)
    
        # 3. Start the in-place rotation
        # Set wheel velocities to be equal and opposite for a pivot turn
        # *** CHANGE: Now using MAX_SPEED / 16 for maximum precision. ***
        # Left motor forward, Right motor backward (Rotates Left/Counter-Clockwise)
        self.left_motor.setVelocity(-self.MAX_SPEED / 8) # Use a very slow speed for minimal overshoot
        self.right_motor.setVelocity(self.MAX_SPEED / 8)
    
        print(f"Initial Yaw: {initial_yaw:.3f} rad ({math.degrees(initial_yaw):.2f}°)")
        print(f"Target Yaw (normalized): {target_yaw:.3f} rad ({math.degrees(target_yaw):.2f}°)")
        print("Starting 180-degree rotation (turning left)...")
    
        # 4. Loop until the turn is complete
        rotation_complete = False
        
        while self.robot.step(self.timestep) != -1 and not rotation_complete:
            current_yaw = self.get_yaw_angle(self.imu)
            
            # Calculate the angular difference (error)
            # Using math.atan2 is robust for finding the shortest angular distance
            angle_diff = self.normalize_angle(current_yaw - initial_yaw)
            
            # Check if the desired rotation has been achieved
            # We check if the angle change is close to TARGET_ANGLE
            # Use a small tolerance (e.g., 0.05 radians ≈ 3 degrees)
            tolerance = 0.05
            
            # We need to check if the ABSOLUTE rotation is close to pi.
            # Note: If the speed is very low, you might be able to lower the tolerance too!
            if abs(abs(angle_diff) - self.TARGET_ANGLE) < tolerance:
                rotation_complete = True
                
            # Optional: Print progress for debugging
            # print(f"Current Yaw: {current_yaw:.3f} rad | Angle Change: {angle_diff:.3f} rad")
    
        # 5. Stop the motors
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        final_yaw = self.get_yaw_angle(self.imu)
        print("--- Rotation Finished ---")
        print(f"Final Yaw: {final_yaw:.3f} rad ({math.degrees(final_yaw):.2f}°)")
        
        # Wait for one more step to ensure motors are stopped before program exit
        self.robot.step(self.timestep)
            
    def detect_obstacle_front(self, sensors):
        """Elülső akadály észlelése (ps0, ps7)"""
        front_left = sensors[0]  # ps0
        front_right = sensors[7]  # ps7
        return front_left > self.DISTANCE_THRESHOLD or front_right > self.DISTANCE_THRESHOLD

    def detect_obstacle_left(self, sensors):
        """Bal oldali akadály észlelése (ps5)"""
        left_side = sensors[5]  # ps5
        return left_side > self.DISTANCE_THRESHOLD
        
    def run_fsm_step(self):
        """Egy FSM lépés végrehajtása"""
        # Szenzorértékek beolvasása
        sensors = self.get_sensor_values()
        
        # Állapot-specifikus logika végrehajtása
        if self.current_state == State.FORWARD_TO_FIRST:
            self.state_forward_to_first(sensors)
        elif self.current_state == State.TURN_180:
            self.state_turn_180()

    def run(self):
        """Fő futási ciklus"""
        print("Robot indítása...")
        
        while self.robot.step(self.timestep) != -1:
            self.run_fsm_step()

# Fő program
if __name__ == "__main__":
    controller = EPuckFSMController()
    controller.run()
