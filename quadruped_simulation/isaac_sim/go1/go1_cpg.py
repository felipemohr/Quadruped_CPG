import numpy as np
import carb
from time import time

class GO1_CPG:
    def __init__(self, gait_type="trot"):
        self.coupling_weight = 1.0
        self.convergence_factor_a = 50.0
        self.swing_frequency = 1.5 # Hz
        self.stance_frequency = 0.5 # Hz
        self.amplitude_mu = 1.0 * np.ones(4)
        self.frequency_omega = self.stance_frequency * np.ones(4)
        self.set_gait_type(gait_type)

        self.dstep_x = 0.05*np.ones(4)
        self.dstep_y = 0.0*np.ones(4)
        self.ground_clearance = 0.05
        self.ground_penetration = 0.02

        self._amplitude_r = np.random.rand(4)
        self._amplitude_dr = np.zeros(4)
        self._amplitude_d2r = np.zeros(4)

        self._phase_theta = np.random.rand(4)
        self._phase_dtheta = np.zeros(4)

        self._ground_multiplier = np.zeros(4)

        self._last_time = time()
    
    def step(self):
        dt = time() - self._last_time

        self._amplitude_d2r = self.convergence_factor_a * (self.convergence_factor_a/4.0 * (self.amplitude_mu - self._amplitude_r) - self._amplitude_dr)
        self._amplitude_dr += self._amplitude_d2r * dt

        for i in range(4):
            self.frequency_omega[i] = 2*np.pi * self.swing_frequency if self._phase_theta[i] < np.pi else 2*np.pi * self.stance_frequency
            self._phase_dtheta[i] = self.frequency_omega[i]
            for j in range(4):
                self._phase_dtheta[i] += self._amplitude_r[j] * self.coupling_weight * np.sin(self._phase_theta[j] - self._phase_theta[i] - self.coupling_matrix[i][j])

        self._amplitude_r += self._amplitude_dr * dt
        self._phase_theta += self._phase_dtheta * dt

        self._phase_theta %= 2*np.pi
        self._ground_multiplier = np.where(np.sin(self._phase_theta) > 0, self.ground_clearance, self.ground_penetration)

        self._last_time = time()

        foot_x = -self.dstep_x * self._amplitude_r * np.cos(self._phase_theta)
        foot_y = -self.dstep_y * self._amplitude_r * np.cos(self._phase_theta)
        foot_z = self._ground_multiplier * np.sin(self._phase_theta)

        return np.array([foot_x, foot_y, foot_z]).T

    def set_gait_type(self, gait_type):
        if gait_type == "trot":
            self.coupling_matrix = np.array([[0, np.pi, np.pi, 0],
                                             [-np.pi, 0, 0, -np.pi],
                                             [-np.pi, 0, 0, -np.pi],
                                             [0, np.pi, np.pi, 0]])
        elif gait_type == "walk":
            self.coupling_matrix = np.array([[ 0, np.pi, np.pi/2, 3*np.pi/2],
                                             [-np.pi, 0, -np.pi/2, -3*np.pi/2],
                                             [-np.pi/2, np.pi/2, 0, -np.pi],
                                             [-3*np.pi/2, 3*np.pi/2, np.pi, 0]])
        elif gait_type == "pace":
            self.coupling_matrix = np.array([[ 0, np.pi, np.pi, np.pi],
                                             [-np.pi, 0, -np.pi, 0],
                                             [ 0, np.pi, 0, np.pi],
                                             [-np.pi, 0, -np.pi, 0]])
        elif gait_type == "gallop":
            self.coupling_matrix = np.array([[0, 0, -np.pi, -np.pi],
                                             [0, 0, -np.pi, -np.pi],
                                             [np.pi, np.pi, 0, 0],
                                             [np.pi, np.pi, 0, 0]])
        else:
            carb.log_error(f"Impossible to set {gait_type} gait type. Ignoring...")
