"""
Simple example that connects to the first Crazyflie found,
moves it to the right by thrust + yaw + pitch + roll
"""
import time
import math
from threading import Thread
from threading import Timer
import cflib
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from queue import Empty
from queue import Queue

class BasicRightExample:
    HEIGHT = 0.1
    VELOCITY = 0.2
    RATE = 70
    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """
        self._cf = Crazyflie(rw_cache="./cache")
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.open_link(link_uri)
        print("Connecting to %s" % link_uri)
        self.is_connected = True
        # change the parameters,
        self._param_check_list = []
        self._param_groups = []

        self.default_height = self.HEIGHT
        self._is_flying = False
        self._thread = None

    def _connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected to %s" % link_uri)
        self._basic_right_motors()
        # Start a timer to disconnect in 3s
        t = Timer(2, self._cf.close_link)
        t.start()

    def _a_propTest_callback(self, name, value):
        """Callback for pid_attitude.pitch_kd"""
        print("Readback: {0}={1}".format(name, value))

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print("Error when logging %s: %s" % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        print("[%d][%s]: %s" % (timestamp, logconf.name, data), flush=True)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)

    def _basic_right_motors(self):
        self.take_off(self.default_height)
        self.left(1)
        print("start ramp motors!")
        time.sleep(1)
        print("end ramp motors!")
        # self._cf.close_link()

    def take_off(self, height=None, velocity=VELOCITY):
        """
        Takes off, that is starts the motors, goes straight up and hovers.
        Do not call this function if you use the with keyword. Take off is
        done automatically when the context is created.

        :param height: The height (meters) to hover at. None uses the default
                       height set when constructed.
        :param velocity: The velocity (meters/second) when taking off
        :return:
        """
        if self._is_flying:
            raise Exception('Already flying')
        if not self._cf.is_connected():
            raise Exception('Crazyflie is not connected')

        self._is_flying = True
        self._reset_position_estimator()

        self._thread = _SetPointThread(self._cf)
        self._thread.start()

        if height is None:
            height = self.default_height

        self.up(height, velocity)

    def land(self, velocity=VELOCITY):
        """
        Go straight down and turn off the motors.

        Do not call this function if you use the with keyword. Landing is
        done automatically when the context goes out of scope.

        :param velocity: The velocity (meters/second) when going down
        :return:
        """
        if self._is_flying:
            self.down(self._thread.get_height(), velocity)

            self._thread.stop()
            self._thread = None

            self._cf.commander.send_stop_setpoint()
            self._is_flying = False

    def __enter__(self):
        self.take_off()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.land()

    def left(self, distance_m, velocity=VELOCITY):
        """
        Go left

        :param distance_m: The distance to travel (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(0.0, distance_m, 0.0, velocity)

    def right(self, distance_m, velocity=VELOCITY):
        """
        Go right

        :param distance_m: The distance to travel (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(0.0, -distance_m, 0.0, velocity)

    def forward(self, distance_m, velocity=VELOCITY):
        """
        Go forward

        :param distance_m: The distance to travel (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(distance_m, 0.0, 0.0, velocity)

    def back(self, distance_m, velocity=VELOCITY):
        """
        Go backwards

        :param distance_m: The distance to travel (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(-distance_m, 0.0, 0.0, velocity)

    def up(self, distance_m, velocity=VELOCITY):
        """
        Go up

        :param distance_m: The distance to travel (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(0.0, 0.0, distance_m, velocity)

    def down(self, distance_m, velocity=VELOCITY):
        """
        Go down

        :param distance_m: The distance to travel (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(0.0, 0.0, -distance_m, velocity)

    def turn_left(self, angle_degrees, rate=RATE):
        """
        Turn to the left, staying on the spot

        :param angle_degrees: How far to turn (degrees)
        :param rate: The turning speed (degrees/second)
        :return:
        """
        flight_time = angle_degrees / rate

        self.start_turn_left(rate)
        time.sleep(flight_time)
        self.stop()

    def turn_right(self, angle_degrees, rate=RATE):
        """
        Turn to the right, staying on the spot

        :param angle_degrees: How far to turn (degrees)
        :param rate: The turning speed (degrees/second)
        :return:
        """
        flight_time = angle_degrees / rate

        self.start_turn_right(rate)
        time.sleep(flight_time)
        self.stop()

    def circle_left(self, radius_m, velocity=VELOCITY, angle_degrees=360.0):
        """
        Go in circle, counter clock wise

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity along the circle (meters/second)
        :param angle_degrees: How far to go in the circle (degrees)
        :return:
        """
        distance = 2 * radius_m * math.pi * angle_degrees / 360.0
        flight_time = distance / velocity

        self.start_circle_left(radius_m, velocity)
        time.sleep(flight_time)
        self.stop()

    def circle_right(self, radius_m, velocity=VELOCITY, angle_degrees=360.0):
        """
        Go in circle, clock wise

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity along the circle (meters/second)
        :param angle_degrees: How far to go in the circle (degrees)
        :return:
        """
        distance = 2 * radius_m * math.pi * angle_degrees / 360.0
        flight_time = distance / velocity

        self.start_circle_right(radius_m, velocity)
        time.sleep(flight_time)
        self.stop()

    def move_distance(self, distance_x_m, distance_y_m, distance_z_m,
                      velocity=VELOCITY):
        """
        Move in a straight line.
        positive X is forward
        positive Y is left
        positive Z is up

        :param distance_x_m: The distance to travel along the X-axis (meters)
        :param distance_y_m: The distance to travel along the Y-axis (meters)
        :param distance_z_m: The distance to travel along the Z-axis (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        distance = math.sqrt(distance_x_m * distance_x_m +
                             distance_y_m * distance_y_m +
                             distance_z_m * distance_z_m)
        flight_time = distance / velocity

        velocity_x = velocity * distance_x_m / distance
        velocity_y = velocity * distance_y_m / distance
        velocity_z = velocity * distance_z_m / distance

        self.start_linear_motion(velocity_x, velocity_y, velocity_z)
        time.sleep(flight_time)
        self.stop()

    # Velocity based primitives

    def start_left(self, velocity=VELOCITY):
        """
        Start moving left. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(0.0, velocity, 0.0)

    def start_right(self, velocity=VELOCITY):
        """
        Start moving right. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(0.0, -velocity, 0.0)

    def start_forward(self, velocity=VELOCITY):
        """
        Start moving forward. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(velocity, 0.0, 0.0)

    def start_back(self, velocity=VELOCITY):
        """
        Start moving backwards. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(-velocity, 0.0, 0.0)

    def start_up(self, velocity=VELOCITY):
        """
        Start moving up. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(0.0, 0.0, velocity)

    def start_down(self, velocity=VELOCITY):
        """
        Start moving down. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(0.0, 0.0, -velocity)

    def stop(self):
        """
        Stop any motion and hover.

        :return:
        """
        self._set_vel_setpoint(0.0, 0.0, 0.0, 0.0)

    def start_turn_left(self, rate=RATE):
        """
        Start turning left. This function returns immediately.

        :param rate: The angular rate (degrees/second)
        :return:
        """
        self._set_vel_setpoint(0.0, 0.0, 0.0, -rate)

    def start_turn_right(self, rate=RATE):
        """
        Start turning right. This function returns immediately.

        :param rate: The angular rate (degrees/second)
        :return:
        """
        self._set_vel_setpoint(0.0, 0.0, 0.0, rate)

    def start_circle_left(self, radius_m, velocity=VELOCITY):
        """
        Start a circular motion to the left. This function returns immediately.

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        circumference = 2 * radius_m * math.pi
        rate = 360.0 * velocity / circumference

        self._set_vel_setpoint(velocity, 0.0, 0.0, -rate)

    def start_circle_right(self, radius_m, velocity=VELOCITY):
        """
        Start a circular motion to the right. This function returns immediately

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        circumference = 2 * radius_m * math.pi
        rate = 360.0 * velocity / circumference

        self._set_vel_setpoint(velocity, 0.0, 0.0, rate)

    def start_linear_motion(self, velocity_x_m, velocity_y_m, velocity_z_m, rate_yaw=0.0):
        """
        Start a linear motion with an optional yaw rate input. This function returns immediately.

        positive X is forward
        positive Y is left
        positive Z is up

        :param velocity_x_m: The velocity along the X-axis (meters/second)
        :param velocity_y_m: The velocity along the Y-axis (meters/second)
        :param velocity_z_m: The velocity along the Z-axis (meters/second)
        :param rate: The angular rate (degrees/second)
        :return:
        """
        self._set_vel_setpoint(
            velocity_x_m, velocity_y_m, velocity_z_m, rate_yaw)

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


class _SetPointThread(Thread):
    TERMINATE_EVENT = 'terminate'
    UPDATE_PERIOD = 0.2
    ABS_Z_INDEX = 3

    def __init__(self, cf, update_period=UPDATE_PERIOD):
        Thread.__init__(self)
        self.update_period = update_period

        self._queue = Queue()
        self._cf = cf

        self._hover_setpoint = [0.0, 0.0, 0.0, 0.0]

        self._z_base = 0.0
        self._z_velocity = 0.0
        self._z_base_time = 0.0

    def stop(self):
        """
        Stop the thread and wait for it to terminate

        :return:
        """
        self._queue.put(self.TERMINATE_EVENT)
        self.join()

    def set_vel_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        """Set the velocity setpoint to use for the future motion"""
        self._queue.put((velocity_x, velocity_y, velocity_z, rate_yaw))

    def get_height(self):
        """
        Get the current height of the Crazyflie.

        :return: The height (meters)
        """
        return self._hover_setpoint[self.ABS_Z_INDEX]

    def run(self):
        while True:
            try:
                event = self._queue.get(block=True, timeout=self.update_period)
                if event == self.TERMINATE_EVENT:
                    return

                self._new_setpoint(*event)
            except Empty:
                pass

            self._update_z_in_setpoint()
            self._cf.commander.send_hover_setpoint(*self._hover_setpoint)

    def _new_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        self._z_base = self._current_z()
        self._z_velocity = velocity_z
        self._z_base_time = time.time()

        self._hover_setpoint = [velocity_x, velocity_y, rate_yaw, self._z_base]

    def _update_z_in_setpoint(self):
        self._hover_setpoint[self.ABS_Z_INDEX] = self._current_z()

    def _current_z(self):
        now = time.time()
        return self._z_base + self._z_velocity * (now - self._z_base_time)

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
        le = BasicRightExample(available[0][0])
    else:
        print("No Crazyflies found, cannot run example")
