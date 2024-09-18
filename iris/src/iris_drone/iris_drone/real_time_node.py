import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import argparse
from pymavlink import mavutil
import time
import math
import threading
from rclpy.executors import MultiThreadedExecutor

class AutonomousMission(Node):
    def _init_(self):
        super()._init_('Autonomous_Mission')
        
        self.x = None
        self.y = None
        self.target_lat = None
        self.target_lon = None
        
        # Subscriber לקבלת קואורדינטות XY
        self.subscription = self.create_subscription(
            Point,
            '/xy_coordinates',
            self.coordinates_callback,
            10)
        
        # Subscriber לקבלת קואורדינטות יעד בזמן אמת
        self.destination_subscription = self.create_subscription(
            Point,
            '/destination_coordinates',
            self.destination_callback,
            10)

        self.h_x = 640
        self.h_y = 360
        self.groundspeed = 0.25
        self.mission_complete = False
        self.start_location = None  # משתנה לשמירת נקודת ההתחלה של הרחפן

    # פונקציית callback לקבלת קואורדינטות יעד
    def destination_callback(self, msg):
        self.target_lat = msg.x  # Latitude
        self.target_lon = msg.y  # Longitude
        self.get_logger().info(f"Received destination coordinates: Lat: {self.target_lat}, Lon: {self.target_lon}")

    def ConnectToVehicle(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default='127.0.0.1:14550')
        args = parser.parse_args()

        print('Connecting to vehicle on: %s' % args.connect)
        vehicle = connect(args.connect, baud=57600, wait_ready=True, timeout=60)
        return vehicle
    
    def coordinates_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
    
    def ArmAndTakeoff(self, aTargetAltitude):
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def get_distance_metres(self, aLocation1, aLocation2):
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def GotoLocation(self, to_lat, to_lon, alt):
        print("Increasing altitude to avoid obstacles")
        self.send_ned_velocity(0, 0, 1)
        time.sleep(3)

        target_Location = LocationGlobalRelative(to_lat, to_lon, alt)
        self.vehicle.simple_goto(target_Location, groundspeed=1.5)

        while self.vehicle.mode.name == "GUIDED":
            remainingDistance = self.get_distance_metres(self.vehicle.location.global_frame, target_Location)
            print("Distance to waypoint: ", remainingDistance)
            if remainingDistance <= 1:
                print("Reached target")
                break
            time.sleep(2)

    def TriggerServo(self, num, PWM):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            num,
            PWM,
            0, 0, 0, 0, 0)
        print("dropping payload...")
        self.vehicle.send_mavlink(msg)
        print("payload dropped...")

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b10111000111,
            0, 0, 0,
            velocity_y, velocity_x, velocity_z,
            0, 0, 0,
            0, 0)

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def landt(self):
        print("Raising altitude before landing")
        self.send_ned_velocity(0, 0, 1)
        time.sleep(3)

        groundspeed = 0.25
        h_x = 640
        h_y = 360
        x_pre = 0
        y_pre = 0
        while(1):
            x = self.x
            y = self.y
            time.sleep(0.1)
            
            print(x, y)
            if (x == 0 or x is None) and (y == 0 or y is None):
                if x_pre and y_pre:
                    print("going direct")
                    x = x_pre
                    y = y_pre
                else:
                    break
            else:
                x_pre = x
                y_pre = y

            if x != h_x and y != h_y:
                if x > h_x and y > h_y:
                    self.send_ned_velocity(groundspeed, -groundspeed, groundspeed)
                    time.sleep(0.1)
                elif x < h_x and y < h_y:
                    self.send_ned_velocity(-groundspeed, groundspeed, groundspeed)
                    time.sleep(0.1)
                elif x < h_x and y > h_y:
                    self.send_ned_velocity(-groundspeed, -groundspeed, groundspeed)
                    time.sleep(0.1)
                elif x > h_x and y < h_y:
                    self.send_ned_velocity(groundspeed, groundspeed, groundspeed)
                    time.sleep(0.1)
            elif x == h_x and y != h_y:
                if y > h_y:
                    self.send_ned_velocity(0, -groundspeed, groundspeed)
                    time.sleep(0.1)
                elif y < h_y:
                    self.send_ned_velocity(0, groundspeed, groundspeed)
                    time.sleep(0.1)
            elif y == h_y and x != h_x:
                if x > h_x:
                    self.send_ned_velocity(groundspeed, 0, groundspeed)
                    time.sleep(0.1)
                elif x < h_x:
                    self.send_ned_velocity(-groundspeed, 0, groundspeed)
                    time.sleep(0.1)
            else:
                self.send_ned_velocity(0, 0, 0)
                break

            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt <= 2.5:
                print(f"Reached target : {self.vehicle.location.global_frame.lat}, {self.vehicle.location.global_frame.lon}")
                return

        print("Returning to start location")
        self.GotoLocation(self.start_location.lat, self.start_location.lon, 10)
        self.vehicle.mode = VehicleMode("RTL")
        self.vehicle.close()
        self.mission_complete = True

    def mission(self):
        self.vehicle = self.ConnectToVehicle()

        self.start_location = LocationGlobalRelative(
            self.vehicle.location.global_frame.lat,
            self.vehicle.location.global_frame.lon,
            10
        )

        alt = 10
        self.ArmAndTakeoff(alt)

        # המתנה לקבלת קואורדינטות יעד
        while self.target_lat is None or self.target_lon is None:
            self.get_logger().info("Waiting for destination coordinates...")
            time.sleep(1)

        self.get_logger().info(f"Flying to: Lat: {self.target_lat}, Lon: {self.target_lon}")

        # טיסה ליעד שקיבלנו
        self.GotoLocation(self.target_lat, self.target_lon, alt)
        time.sleep(4)

        self.landt()
        print("Mission complete")
        self.vehicle.mode = VehicleMode("RTL")
        self.vehicle.close()
        self.mission_complete = True

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMission()

    # Start the mission in a separate thread
    mission_thread = threading.Thread(target=node.mission)
    mission_thread.start()

    # Spin the node to keep receiving the messages
    while rclpy.ok() and not node.mission_complete:
        rclpy.spin_once(node, timeout_sec=1)

    # Mission completed, clean up
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
