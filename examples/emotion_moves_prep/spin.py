import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)


class Spin:
    def __init__(self, link_uri):
        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        # Thread(target=self._ramp_motors).start()
        self._ramp_motors()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _ramp_motors(self):
        # thrust: 0-65535
        # Unlock startup thrust protection
        # vx, vy, yawrate, zdistance
        # self._cf.commander.send_hover_setpoint(0, 0, 0, 0)
        zdistance = 0
        yawrate = 360*2
        thrust = 40000
        # self._cf.commander.send_hover_setpoint(0, 0, 0, zdistance)
        time.sleep(5)
        while(zdistance < 0.5):
            self._cf.commander.send_hover_setpoint(0, 0, 0, zdistance)
            time.sleep(0.1)
            zdistance += 0.05
        self._cf.commander.send_hover_setpoint(0, 0, 0, 0)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        while(thrust > 29000):
            self._cf.commander.send_setpoint(0, 0, yawrate, thrust)
            thrust -= 500
            time.sleep(0.1)
        self._cf.commander.send_stop_setpoint()
        self._cf.close_link()

# roll/pitch are in degree
# Yaw is in degree per seconds
# thrust is from 0 to 65535 with 0 meaning 0% and 65535 meaning 100%


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = Spin(uri)
