import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID
import visdom

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):

        super().__init__(connection)

        # default opens up to http://localhost:8097
        self.v = visdom.Visdom()
        assert self.v.check_connection()

        # Plot NE
        ne = np.array((self.local_position[0], self.local_position[1])).reshape(-1, 2)

        self.n_plot = self.v.scatter(ne, opts=dict(
            title="Local position (north, east)",
            xlabel='North',
            ylabel='East'
        ))

        # Plot D
        d = np.array([self.local_position[2]])

        self.t = np.array([1])
        self.d_plot = self.v.line(d, X=np.array(self.t), opts=dict(
            title="Altitude (meters)",
            xlabel='Timestep',
            ylabel='Down'
        ))


        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = self.calculate_box()
        self.waypoint = 4
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
        self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)

    def update_ne_plot(self):
        ne = np.array((self.local_position[0], self.local_position[1])).reshape(-1, 2)
        self.v.scatter(ne, win=self.n_plot, update='append')

    def update_d_plot(self):
        d = np.array([self.local_position[2]])
        # update timestep
        self.t += np.array([1])
        self.v.line(d, X=self.t, win=self.d_plot, update='append')

    def local_position_callback(self):
        altitude = -1.0 * self.local_position[2]
        if self.flight_state == States.TAKEOFF:

            # coordinate conversion

            # check if altitude is within 95% of target
            print(self.local_velocity)
            print(self.target_position)
            print(self.local_position)
            #cmd_position(north, east, altitude, heading)
            if altitude > 0.95 * self.target_position[2]: #and self.target_position[0] > 0.95* self.local_position[0]and self.target_position[1] > 0.95* self.local_position[1]
                self.waypoint_transition()
        elif self.flight_state == States.LANDING:
            self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1)):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()
        elif self.flight_state == States.WAYPOINT:
            self.waypoint_transition()

    def calculate_box(self):
        """TODO: Fill out this method

        1. Return waypoints to fly a box
        """
        square = [(10,0,3,0),(10,10,3,0),(0,10,3,0),(0,0,3,0)]
        return square

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        # # begin ADDED code
        # if self.global_position[0] == 0.0 and self.global_position[1] == 0.0:
        #     print("no global position data, wait")
        #     return
        # #end ADDED code
        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.target_position[0] = 0
        self.target_position[1] = 0
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method

        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        self.flight_state = States.WAYPOINT
        altitude = -1.0 * self.local_position[2]

        #cmd_position(north, east, altitude, heading)
        print(self.local_position)
        print(self.target_position)
        if altitude > 0.95 * self.target_position[2] and abs(self.local_position[0] - self.target_position[0]) <0.5 and abs(self.local_position[1] - self.target_position[1]) <0.3:
            self.waypoint -= 1
            print("waypoint",self.waypoint)
            if self.waypoint < -1:
                self.flight_state = States.LANDING
                return
            else:
                w = self.all_waypoints[self.waypoint]
                self.target_position[0] = w[0]
                self.target_position[1] = w[1]

            self.cmd_position(self.target_position[0],self.target_position[1],self.target_position[2],0)


        # time.sleep(4)



    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided

        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided

        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--visdom', type=bool, default=True, help='visdom server')

    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
