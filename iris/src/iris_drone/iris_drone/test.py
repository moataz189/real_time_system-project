import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import time
import threading


class AutonomousMission(Node):
    def __init__(self):
        super().__init__('Autonomous_Mission')
        
        self.x = None
        self.y = None
        self.subscription = self.create_subscription(
            Point,
            '/xy_coordinates',
            self.coordinates_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.h_x = 640
        self.h_y = 360
        self.groundspeed = 0.25
        self.mission_complete = False

    def ConnectToVehicle(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default='127.0.0.1:14550')
        args = parser.parse_args()

        # Connect to the Vehicle
        print ('Connecting to vehicle on: %s' % args.connect)
        vehicle = connect(args.connect, baud=57600, wait_ready=True, timeout=60)
        return vehicle
    
    def coordinates_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
    
    def ArmAndTakeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

    def GotoLocation(self, to_lat, to_lon, alt):
        target_Location = LocationGlobalRelative(to_lat, to_lon, alt)
        self.vehicle.simple_goto(target_Location, groundspeed=1.5)

        while self.vehicle.mode.name == "GUIDED":
            remainingDistance = self.get_distance_metres(self.vehicle.location.global_frame, target_Location)
            print("Distance to wavepoint: ", remainingDistance)
            if remainingDistance <= 1:  # Just below target, in case of undershoot.
                print("Reached target")
                break
            time.sleep(2)

    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def mission(self):
        self.vehicle = self.ConnectToVehicle()
        alt = 10
        print(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)
        print(self.vehicle.heading)
        self.ArmAndTakeoff(alt)
        time.sleep(4)
        self.GotoLocation(-35.3630969, 149.1651725, alt)
        print("Arrived at target location")
        # Do not land, just hover
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

if __name__ == '__main__':
    main()

