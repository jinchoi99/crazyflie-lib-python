import time
from threading import Thread
from queue import Empty
from queue import Queue


class SetPointThread(Thread):
    TERMINATE_EVENT = 'terminate'
    UPDATE_PERIOD = 0.2
    ABS_Z_INDEX = 3

    def __init__(self, cf, update_period=UPDATE_PERIOD):
        Thread.__init__(self)
        self.update_period = update_period

        self._queue_hover_setpoint = Queue()
        self._queue_setpoint = Queue()
        self._cf = cf

        # vx, vy, yawrate, zdistance
        self._hover_setpoint = [0.0, 0.0, 0.0, 0.0]
        # roll, pitch, yaw, thrust
        self._setpoint = [0.0, 0.0, 0, 0]

        self._z_base = 0.0
        self._z_velocity = 0.0
        self._z_base_time = 0.0

    def stop(self):
        """
        Stop the thread and wait for it to terminate
        :return:
        """
        self._queue_hover_setpoint.put(self.TERMINATE_EVENT)
        self._queue_setpoint.put(self.TERMINATE_EVENT)
        self.join()

    def set_setpoint(self, roll, pitch, yaw, thrust):
        """Set the velocity setpoint to use for the future motion"""
        self._queue_setpoint.put((roll, pitch, yaw, thrust))

    def set_hover_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        """Set the velocity setpoint to use for the future motion"""
        self._queue_hover_setpoint.put((velocity_x, velocity_y, velocity_z, rate_yaw))

    def get_height(self):
        """
        Get the current height of the Crazyflie.

        :return: The height (meters)
        """
        return self._hover_setpoint[self.ABS_Z_INDEX]

    def run(self):
        while True:
            try:
                event_hover_setpoint = self._queue_hover_setpoint.get(block=True, timeout=self.update_period)
                event_setpoint = self._queue_setpoint.get(block=True, timeout=self.update_period)
                if event_hover_setpoint == self.TERMINATE_EVENT:
                    return

                self._new_hover_setpoint(*event_hover_setpoint)
                self._new_setpoint(*event_setpoint)
            except Empty:
                pass

            self._update_z_in_setpoint()
            self._cf.commander.send_hover_setpoint(*self._hover_setpoint)
            self._cf.commander.send_setpoint(*self._setpoint)

    def _new_hover_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        self._z_base = self._current_z()
        self._z_velocity = velocity_z
        self._z_base_time = time.time()

        self._hover_setpoint = [velocity_x, velocity_y, rate_yaw, self._z_base]

    def _new_setpoint(self, roll, pitch, yaw, thrust):
        self._setpoint = [roll, pitch, yaw, thrust]

    def _update_z_in_setpoint(self):
        self._hover_setpoint[self.ABS_Z_INDEX] = self._current_z()

    def _current_z(self):
        now = time.time()
        return self._z_base + self._z_velocity * (now - self._z_base_time)
