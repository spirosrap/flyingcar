"""
This script contains a keyboard controller using the MotionCommander.

Info on API element used:
https://github.com/bitcraze/crazyflie-lib-python/blob/master/cflib/positioning/motion_commander.py
"""
import logging
from pynput import keyboard

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'  # ENSURE THIS MATCHES YOUR CRAZYFLIE CONFIGURATION

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class KeyboardDrone:

    def __init__(self, mc):
        self.mc = mc

        self.velocity = 0.75
        self.ang_velocity = 120

        self.sleeptime = 0.5
        # self.max_hight = 0.8
        # self.hight = 0.0
        print('Press u for taking off!')

    def on_press(self, key):

        if key.char == 'w':
            self.mc.start_forward(self.velocity)

        if key.char == 'u':
            self.mc.take_off(0.3)

        if key.char == 's':
            self.mc.start_back(self.velocity)

        if key.char == 'a':
            self.mc.start_left(self.velocity)

        if key.char == 'd':
            self.mc.start_right(self.velocity)

        if key.char == 'c':
            self.mc.start_down(self.velocity)

        if key == keyboard.Key.space:
            self.mc.start_up(self.velocity)

        if key.char == 'l':
            print('Kill engines')
            return False

        if key.char == 'q':
            self.mc.start_turn_left(self.ang_velocity)

        if key.char == 'e':
            self.mc.start_turn_right(self.ang_velocity)

    def on_release(self, key):
        self.mc.stop()


if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        # We take off when the commander is created
        mc = MotionCommander(scf)

        drone = KeyboardDrone(mc)

        with keyboard.Listener(on_press=drone.on_press, on_release=drone.on_release) as listener:
            listener.join()
