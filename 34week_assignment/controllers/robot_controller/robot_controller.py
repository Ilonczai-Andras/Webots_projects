from controller import Robot
import math

class State:
    FORWARD_TO_FIRST = "FORWARD_TO_FIRST"
    TURN_180 = "TURN_180"
    FORWARD_TO_SECOND = "FORWARD_TO_SECOND" 
    TURN_RIGHT = "TURN_RIGHT"
    TURN_RIGHT_UNTIL_CLEAR = "TURN_RIGHT_UNTIL_CLEAR"
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
        self.FRONT_DISTANCE_THRESHOLD = 80  # Szenzorérték ~0.05m távolságnak megfelelő
        self.LEFT_DISTANCE_THRESHOLD = 80
        self.TURN_SPEED = 2.5  # Optimális sebesség a hátsó szenzoros forduláshoz
        self.FORWARD_SPEED = 4.0
        self.TARGET_ANGLE = math.pi # 180 fok
        
        # FSM állapot
        self.current_state = State.FORWARD_TO_FIRST
        
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
    
    def turn_right(self, speed=None):
        """Jobbra fordulás"""
        if speed is None:
            speed = self.TURN_SPEED
        self.set_motor_speeds(speed, -speed)
            
    def detect_obstacle_front(self, sensors):
        """Elülső akadály észlelése (ps0, ps7)"""
        front_left = sensors[7]  # ps7
        front_right = sensors[0]  # ps0
        return front_left > self.FRONT_DISTANCE_THRESHOLD or front_right > self.FRONT_DISTANCE_THRESHOLD

    def detect_obstacle_left(self, sensors):
        """Bal oldali akadály észlelése (ps5)"""
        left_side = sensors[5]  # ps5
        return left_side > self.LEFT_DISTANCE_THRESHOLD

    def state_forward_to_first(self, sensors):
        """Első akadály felé haladás"""
        if self.detect_obstacle_front(sensors):
            print("Első akadály elérve, 180° fordulás kezdése")
            self.current_state = State.TURN_180
        else:
            self.move_forward()
    
    def state_turn_180(self):
        print(f"Állapot: {self.current_state}")
        if not hasattr(self, "turn_initialized") or not self.turn_initialized:
            # Step 1: Initialize turn (only once when entering state)
            self.initial_yaw = self.get_yaw_angle(self.imu)
            self.target_yaw = self.normalize_angle(self.initial_yaw + self.TARGET_ANGLE)
            self.turn_initialized = True
    
            # Start rotation (left pivot)
            self.set_motor_speeds(-self.MAX_SPEED / 8, self.MAX_SPEED / 8)
    
            print(f"Turning 180°: Initial={math.degrees(self.initial_yaw):.2f}°, Target={math.degrees(self.target_yaw):.2f}°")
    
        # Step 2: Check progress
        current_yaw = self.get_yaw_angle(self.imu)
        angle_diff = self.normalize_angle(current_yaw - self.initial_yaw)
    
        # Use tolerance for stopping
        tolerance = 0.01
        if abs(abs(angle_diff) - self.TARGET_ANGLE) < tolerance:
            # Turn complete
            self.stop_motors()
            print(f"Turn complete. Final Yaw={math.degrees(current_yaw):.2f}°")
    
            # Reset flags
            self.turn_initialized = False
    
            # Next state
            self.current_state = State.FORWARD_TO_SECOND
           
    def state_forward_to_second(self, sensors):
        """Második akadály felé haladás"""
        print(f"Állapot: {self.current_state}")
        if self.detect_obstacle_front(sensors):
            print("Második akadály elérve, jobbra fordulás kezdése")
            self.current_state = State.TURN_RIGHT_UNTIL_CLEAR
            self.turn_right()
        else:
            self.move_forward()
            
    def state_turn_right(self, sensors):
        """Jobbra fordulás, amíg bal oldali szenzor akadályt nem érzékel"""
        if self.detect_obstacle_left(sensors):
            print("Bal oldali fal észlelve -> fal követés")
            self.current_state = State.WALL_FOLLOWING
            self.move_forward()
        else:
            self.turn_right()

    def state_turn_right_until_clear(self, sensors):
        """Jobbra fordulás addig, amíg az első szenzorok szabadok nem lesznek"""
        if not self.detect_obstacle_front(sensors):
            print("Előre szabad út, fal követés engedélyezve")
            self.current_state = State.TURN_RIGHT
        else:
            self.turn_right()

    def state_wall_following(self, sensors):
        print(f"Állapot: {self.current_state}")
        """Fal követése a bal oldalon"""
        if not self.detect_obstacle_left(sensors):
            print("Fal vége elérve, megállás")
            self.current_state = State.STOPPED
            self.stop_motors()
        else:
            self.move_forward()
            
    def state_stopped(self, sensors):
        print(f"Állapot: {self.current_state}")
        """Megállási állapot"""
        self.stop_motors()
        
    def run_fsm_step(self):
        """Egy FSM lépés végrehajtása"""
        # Szenzorértékek beolvasása
        sensors = self.get_sensor_values()
        
        # Állapot-specifikus logika végrehajtása
        if self.current_state == State.FORWARD_TO_FIRST:
            self.state_forward_to_first(sensors)
        elif self.current_state == State.TURN_180:
            self.state_turn_180()
        elif self.current_state == State.FORWARD_TO_SECOND:
            self.state_forward_to_second(sensors)
        elif self.current_state == State.TURN_RIGHT:
            self.state_turn_right(sensors)
        elif self.current_state == State.TURN_RIGHT_UNTIL_CLEAR:
            self.state_turn_right_until_clear(sensors)
        elif self.current_state == State.WALL_FOLLOWING:
            self.state_wall_following(sensors)
        elif self.current_state == State.STOPPED:
            self.state_stopped(sensors)

    def run(self):
        """Fő futási ciklus"""
        print("Robot indítása...")
        
        while self.robot.step(self.timestep) != -1:
            self.run_fsm_step()

# Fő program
if __name__ == "__main__":
    controller = EPuckFSMController()
    controller.run()
