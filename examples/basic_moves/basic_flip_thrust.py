"""
Simple example that connects to the first Crazyflie found,
moves it up, flip, down
"""
import time
import math

from threading import Timer
import cflib
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from set_point_thread import SetPointThread


class BasicFlipThrust:

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
        self._basic_flip_motors()
        self._cf.close_link()

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

    def _basic_flip_motors(self):
        # thrust: 0-65535
        thrust = 42000
        pitch = 0
        roll = 0
        yawrate = 0
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        print("going up!")
        for x in range(5):
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
        print("hovering!")
        for x in range(10):
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            if(thrust >= 37600):
                thrust -= 200
            time.sleep(0.1)
        # print("pitching!")
        # self._basic_pitch_motors(thrust)
        print("rolling!")
        self._basic_roll_motors()

    def _basic_pitch_motors(self):
        # self._cf.commander.send_setpoint(0, 180, 0, 8000)
        # time.sleep(0.01)
        # self._cf.commander.send_setpoint(0, 180, 0, 1000)
        # time.sleep(0.01)
        roll = 0
        pitch = 500
        yawrate = 0
        thrust = 1
        for x in range(1):
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.01)
        print("landing!")
        self._basic_land_motors(thrust, yawrate)

    def _basic_roll_motors(self):
        # roll to 500, pitch to 0 and raw thrust input to something very low (I used 0.17).
        # Now just hit the throttle to get upwards speed, hold the flip button shortly and watch it spin.
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        self._cf.commander.send_setpoint(360, 0, 0, 10000)
        time.sleep(0.2)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        print("landing!")
        self._basic_land_motors()

    def _basic_land_motors(self):
        roll = 0
        pitch = 0
        yawrate = 0
        thrust = 30000
        while(thrust > 20000):
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            thrust -= 500
            time.sleep(0.1)
        self._cf.commander.send_setpoint(0, 0, 0, 0)


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
        le = BasicFlipThrust(available[0][0])
    else:
        print("No Crazyflies found, cannot run example")
