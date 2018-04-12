"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM
from udacidrone.frame_utils import global_to_local
from udacidrone.frame_utils import local_to_global
DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0

class NonlinearController(object):
    """Initialize the controller object and control gains"""
    def __init__(self,
                z_k_p=1.0,
                z_k_d=1.0,
                x_k_p=1.0,
                x_k_d=1.0,
                y_k_p=1.0,
                y_k_d=1.0,
                k_p_roll=1.0,
                k_p_pitch=1.0,
                k_p_yaw=1.0,
                k_p_p=1.0,
                k_p_q=1.0,
                k_p_r=1.0):


        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.x_k_p = x_k_p
        self.x_k_d = x_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.k_p_roll = k_p_roll
        self.k_p_pitch = k_p_pitch
        self.k_p_yaw = k_p_yaw
        self.k_p_p = k_p_p
        self.k_p_q = k_p_q
        self.k_p_r = k_p_r

        return

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory

        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds

        Returns: tuple (commanded position, commanded velocity, commanded yaw)

        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]


        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]

            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]

        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]

                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]

        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)


        return (position_cmd, velocity_cmd, yaw_cmd)

    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command

        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """

        x_dot_dot_command = self.x_k_p*(local_position_cmd[0] - local_position[0]) + self.x_k_d*(local_velocity_cmd[0] - local_velocity[0]) + acceleration_ff[0]
        y_dot_dot_command = self.y_k_p*(local_position_cmd[1] - local_position[1]) + self.y_k_d*(local_velocity_cmd[1] - local_velocity[1]) + acceleration_ff[1]

        return np.array([np.clip(-x_dot_dot_command,-10,10),np.clip(-y_dot_dot_command,-10,10)])

    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)

        Returns: thrust command for the vehicle (+up)
        """
        e = euler2RM(attitude[0],attitude[1],attitude[2])
        b_z = e[2][2]

        u_bar_1 = self.z_k_p * (altitude_cmd - altitude) + self.z_k_d*(vertical_velocity_cmd - vertical_velocity) + acceleration_ff
        c = (u_bar_1 + GRAVITY)/b_z

        return np.clip(c*DRONE_MASS_KG,-MAX_THRUST,MAX_THRUST)



    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame

        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thrust command in Newton

        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """

        e = euler2RM(attitude[0],attitude[1],attitude[2])
        b_x_a = e[0][2]
        b_y_a = e[1][2]
        R33 = e[2][2]
        R21 = e[1][0]
        R22 = e[1][1]
        R12 = e[0][1]
        R11 = e[0][0]

        b_x_c_target = acceleration_cmd[0]*DRONE_MASS_KG/thrust_cmd
        b_y_c_target = acceleration_cmd[1]*DRONE_MASS_KG/thrust_cmd

        b_dot_x_c = self.k_p_roll*(b_x_c_target - b_x_a)
        b_dot_y_c = self.k_p_pitch*(b_y_c_target - b_y_a)

        [[p_c],[q_c]] = (1.0/R33)*np.matmul(np.array([[R21,-R11],[R22,-R12]]),np.array([[b_dot_x_c],[b_dot_y_c]]))

        return np.array([p_c, q_c])        

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        [p_c,q_c,r_c] = body_rate_cmd
        [p_actual,q_actual,r_actual] = body_rate

        p_error = p_c - p_actual
        q_error = q_c - q_actual
        r_error = r_c - r_actual

        u_bar_p = self.k_p_p*p_error
        u_bar_q = self.k_p_q*q_error
        u_bar_r = self.k_p_r*r_error

        return np.array([np.clip(u_bar_p*MOI[0],-MAX_TORQUE,MAX_TORQUE), np.clip(u_bar_q*MOI[1],-MAX_TORQUE,MAX_TORQUE), np.clip(u_bar_r*MOI[2],-MAX_TORQUE,MAX_TORQUE)])

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yawrate in radians/sec
        """
        r_c = self.k_p_yaw*(yaw_cmd - yaw)

        return r_c
