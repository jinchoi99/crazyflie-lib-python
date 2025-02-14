"""
Simple jerk
"""
import time
import math

from threading import Timer
import cflib
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from set_point_thread import SetPointThread


class BasicSlant:
    # Distance (m)
    DIST_UP = 0.5
    DIST_LAND = 0.2
    # Velocity (m/s)
    VELOCITY_UP = 0.2
    VELOCITY_LAND = 0.07
    # Time (sec)
    TIME_HOVER_UP = 2

    def __init__(self, link_uri):
        """ Initialize and run the motion with the specified link_uri """
        self._cf = Crazyflie(rw_cache="./cache")
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.open_link(link_uri)
        print("Connecting to %s" % link_uri)
        self.is_connected = True
        self._is_flying = False
        self._thread = None

    def _connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected to %s" % link_uri)
        # self._basic_up_motors(self.DIST_UP, self.VELOCITY_UP)
        self._flip_motors()
        # self.stop()
        # self.land(self.VELOCITY_LAND)
        print("Start disconnect!")
        self._cf.close_link()

    def _basic_land_motors(self):
        print("Start _basic_land_motors!")
        roll = 0
        pitch = 0
        yawrate = 0
        thrust = 30000
        while(thrust > 1000):
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            thrust -= 1000
            time.sleep(0.1)
        self._cf.commander.send_setpoint(0, 0, 0, 0)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))

    def _basic_up_motors(self, dist_up, vel_up):
        print("Start _basic_up_motors!")
        self.take_off(dist_up, vel_up)
        print("remain up")
        time.sleep(self.TIME_HOVER_UP)

    def take_off(self, dist_up, vel_up):
        print("Start take_off!")
        if self._is_flying:
            raise Exception('Already flying')
        if not self._cf.is_connected():
            raise Exception('Crazyflie is not connected')
        self._is_flying = True
        self._reset_position_estimator()
        self._thread = SetPointThread(self._cf)
        # thread.start() runs thread's run()
        self._thread.start()
        self.up(dist_up, vel_up)

    def up(self, dist_up, vel_up):
        print("up!")
        self.move_distance(0.0, 0.0, dist_up, vel_up)

    def land(self, velocity_land):
        print("Start land!")
        if self._is_flying:
            # self.down(self._thread.get_height(), velocity_land)
            self.down(self.DIST_LAND, velocity_land)
            self._thread.stop()
            self._thread = None
            self._cf.commander.send_stop_setpoint()
            self._is_flying = False

    def down(self, dist_down, vel_down):
        self.move_distance(0.0, 0.0, -1*dist_down, vel_down)

    def move_distance(self, distance_x_m, distance_y_m, distance_z_m,
                      velocity_m):
        """
        Move in a straight line.
        positive X is forward
        positive Y is left
        positive Z is up
        """
        distance = math.sqrt(distance_x_m * distance_x_m +
                             distance_y_m * distance_y_m +
                             distance_z_m * distance_z_m)
        flight_time = distance / velocity_m
        velocity_x = velocity_m * distance_x_m / distance
        velocity_y = velocity_m * distance_y_m / distance
        velocity_z = velocity_m * distance_z_m / distance
        self.start_linear_motion(velocity_x, velocity_y, velocity_z)
        time.sleep(flight_time)
        self.stop()

    def start_linear_motion(self, velocity_x_m, velocity_y_m, velocity_z_m, rate_yaw=0.0):
        """
        Start a linear motion with an optional yaw rate input. This function returns immediately.
        positive X is forward
        positive Y is left
        positive Z is up
        """
        self._set_vel_setpoint(
            velocity_x_m, velocity_y_m, velocity_z_m, rate_yaw)

    def stop(self):
        # Stop any motion and hover.
        self._set_vel_setpoint(0.0, 0.0, 0.0, 0.0)

    def _set_vel_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        if not self._is_flying:
            raise Exception('Can not move on the ground. Take off first!')
        self._thread.set_vel_setpoint(
            velocity_x, velocity_y, velocity_z, rate_yaw)

    def _reset_position_estimator(self):
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

    def _flip_motors(self):
        # have to totally end hover set point? smt messes up the setpoint
        print("Start _flip_motors!")
        # 0 - 65535
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # self._cf.commander.send_setpoint(0, 0, 0, 65535)
        # time.sleep(1)
        # self._cf.commander.send_setpoint(0, 0, 0, 0)
        # time.sleep(0.1)
        # self._cf.commander.send_setpoint(0, 0, 0, 30000)
        # time.sleep(1)
        # self._cf.commander.send_setpoint(0, 0, 0, 0)
        # time.sleep(0.1)
        self._cf.commander.send_setpoint(0, 0, 0, 1)
        time.sleep(1)
        # self._cf.commander.send_setpoint(0, 0, 0, 0)
        # time.sleep(0.1)
        # self._cf.commander.send_setpoint(0, 0, 0, 65535)
        # time.sleep(0.05)
        # self._cf.commander.send_setpoint(0, 0, 0, 65535)
        # time.sleep(0.05)

        # self._cf.commander.send_setpoint(500, 0, 0, 65535)
        # time.sleep(0.05)
        # self._cf.commander.send_setpoint(500, 0, 0, 10000)
        # time.sleep(0.05)
        # self._cf.commander.send_setpoint(500, 0, 0, 10000)
        # time.sleep(0.05)
        # self._cf.commander.send_setpoint(0, 0, 0, 0)


if __name__ == "__main__":
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print("Scanning interfaces for Crazyflies...")
    available = cflib.crtp.scan_interfaces()
    print("Crazyflies found:")
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = BasicSlant(available[0][0])
    else:
        print("No Crazyflies found, cannot run example")
