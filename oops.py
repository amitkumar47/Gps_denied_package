import rospy
import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from geometry_msgs.msg import PoseStamped


class ControlTheDroneClass:     
    def __init__(self):
        print("In init of class")
        self.current_x =0.0
        self.current_y = 0.0
        self.goal_x = 5.0
        self.goal_y =0.0

        self.PID_P = 0.135
        self.PID_I = 0.135
        self.PID_D = 0.0036

        self.prev_error_pitch = 0.0
        self.prev_error_roll = 0.0
        self.integral_pitch = 0.0
        self.integral_roll = 0.0
        self.yaw_angle_v = 0.0

        connection_string = "/dev/ttyACM0"
        baud_rate = 115200
        self.vehicle = connect(connection_string, wait_ready=True)      


    #code for arming and taking off
    def arm_and_takeoff_nogps(self, aTargetAltitude):
        
        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6

        # print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check,
        # just comment it with your own responsibility.
        # while not vehicle.is_armable:
        #     print(" Waiting for vehicle to initialise...")
        #     time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

        print("Taking off!")

        thrust = DEFAULT_TAKEOFF_THRUST
        while True:

            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(f"current_altitude : {current_altitude}, Desired_Altitude : {aTargetAltitude}")
            # print(" Altitude: %f  Desired: %f" %
            #     (current_altitude, aTargetAltitude))

            if current_altitude<=aTargetAltitude*0.6 :
                pass
            elif current_altitude<=aTargetAltitude*0.95:
                thrust = SMOOTH_TAKEOFF_THRUST
            else :
                print("Reached target altitude")
                break            
            # if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            #     print("Reached target altitude")
            #     break
            # elif current_altitude >= aTargetAltitude*0.6:
            #     thrust = SMOOTH_TAKEOFF_THRUST
            self.set_attitude(thrust = thrust, takeoff=1)
            # time.sleep(0.2)
        # while True:
        #     vehicle.mode = VehicleMode("LAND")
        time.sleep(0.5)
        



    def send_attitude_target(self, roll_angle = 0.0, pitch_angle = 0.0,
                            yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                            thrust = 0.5):
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                    When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = self.vehicle.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0,
                    yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                    thrust = 0.5, duration = 0.2, takeoff=0.0):
        threshold = 15
        prev_error_pitch = 0.0
        integral_pitch = 0.0
        prev_error_roll = 0.0
        integral_roll = 0.0
        start = time.time()
        while time.time() - start < duration:

            

            if takeoff==1:
                self.send_attitude_target(0, 0,
                                yaw_angle, yaw_rate, True,
                                thrust)

            else :
                error_pitch = self.goal_x - self.current_x
                error_roll = self.goal_y - self.current_y

                integral_pitch += error_pitch
                integral_roll += error_roll

                derivative_pitch = error_pitch - prev_error_pitch
                derivative_roll = error_roll - prev_error_roll

                output_pitch = self.PID_P * error_pitch + self.PID_I * integral_pitch + self.PID_D * derivative_pitch
                output_roll = self.PID_P * error_roll + self.PID_I * integral_roll + self.PID_D * derivative_roll

                prev_error_pitch = error_pitch
                prev_error_roll = error_roll 

                #Thresholding in pitch
                if output_pitch<-1*threshold:
                    output_pitch =-1* threshold
                elif output_pitch > threshold:
                    output_pitch = threshold

                #Thresholding in roll
                if output_roll<-1*threshold:
                    output_roll = -1*threshold
                elif output_roll > threshold:
                    output_roll = threshold

                self.send_attitude_target(-1*output_roll, -1*output_pitch,
                                    yaw_angle, yaw_rate, True,
                                    thrust)
            time.sleep(0.05)

        # Reset attitude, or it will persist for 1s more due to the timeout
        # self.send_attitude_target(0, 0,
        #                     yaw_angle, yaw_rate, True,
        #                     thrust)

    def to_quaternion(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
        return [w, x, y, z]
    
    def get_distance(self, current_x, current_y, goal_x, goal_y):
        distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        return distance
    
    def pose_callback(self,data):
        #  print("In callback")
        position = data.pose.position
        orientation = data.pose.orientation

        self.current_x = position.x
        self.current_y = position.y  

    # def tf_read(self):        
    #     rospy.Subscriber('/qvio/pose', PoseStamped, self.pose_callback)

    
    def ControllerFunction(self):
        # self.set_attitude() 
        
        start = time.time()
        #distance = 100000000
        while True:

            rospy.Subscriber('/qvio/pose', PoseStamped, self.pose_callback)
            print(f"current_x : {self.current_x},current_y : {self.current_y}")
            
            distance = self.get_distance(self.current_x, self.current_y, self.goal_x, self.goal_y)
            print(f"Distance from Target : {distance}")
            # self.tf_read()
           
            self.set_attitude()
            if distance <= 1:
                print("Breaking loop, Target reached.")
                break 

        start = time.time()
        while (time.time() - start) <5 :
             self.send_attitude_target(0, 0, thrust = 0.5)

        self.vehicle.mode = VehicleMode("LAND")
        print("Drone Landing, Program Exit")