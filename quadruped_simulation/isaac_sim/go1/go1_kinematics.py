from math import sqrt, atan2, pi
from scipy.spatial.transform import Rotation
from enum import Enum
import numpy as np
import carb

class Legs(Enum):
    FL = 0
    FR = 1
    RL = 2
    RR = 3


class GO1_Kinematics():
    def __init__(self):
        self.leg_dimensions = {"L1": 0.080,
                               "L2": 0.213,
                               "L3": 0.213}
        self.body_dimensions = {"L": 0.376,
                                "W": 0.094,
                                "H": 0.300}

    def get_translation_matrix(self, translation: np.ndarray):
        T = np.eye(4, dtype=np.float64)
        T[:3, 3] = translation
        return T

    def get_transformation_matrix(self, translation: np.ndarray, rotation: np.ndarray):
        roll_rotation = Rotation.from_euler('x', rotation[0])
        pitch_rotation = Rotation.from_euler('y', rotation[1])
        yaw_rotation = Rotation.from_euler('z', rotation[2])

        R = (roll_rotation * pitch_rotation * yaw_rotation).as_matrix()

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = translation
        return T

    def get_body_leg_ik(self, leg: Legs, body_position: np.ndarray, body_rotation: np.ndarray):
        Tm = self.get_transformation_matrix(body_position, body_rotation)
        if (leg == Legs.FL):
            return Tm @ self.get_translation_matrix(np.array([self.body_dimensions["L"]/2.0, self.body_dimensions["W"]/2.0, 0.0]) )
        elif (leg == Legs.FR):
            return Tm @ self.get_translation_matrix(np.array([self.body_dimensions["L"]/2.0, -self.body_dimensions["W"]/2.0, 0.0]) )
        elif (leg == Legs.RL):
            return Tm @ self.get_translation_matrix(np.array([-self.body_dimensions["L"]/2.0, self.body_dimensions["W"]/2.0, 0.0]) )
        elif (leg == Legs.RR):
            return Tm @ self.get_translation_matrix(np.array([-self.body_dimensions["L"]/2.0, -self.body_dimensions["W"]/2.0, 0.0]) )
        else:
            carb.log_error(f"impossible to compute leg IK for leg {leg}")

    def get_leg_joints(self, point: np.ndarray, left=False):
        reflect = 1.0 if left else -1.0
        x, y, z = point[0], point[1], point[2]

        a = sqrt(y**2 + z**2 - self.leg_dimensions["L1"]**2)
        A = (a**2 + x**2 + self.leg_dimensions["L2"]**2 - self.leg_dimensions["L3"]**2) / (2*self.leg_dimensions["L2"] * sqrt(a**2 + x**2))
        B = (a**2 + x**2 - self.leg_dimensions["L2"]**2 - self.leg_dimensions["L3"]**2) / (2*self.leg_dimensions["L2"] * self.leg_dimensions["L3"])

        theta1 = atan2(y, -z) - atan2(reflect*self.leg_dimensions["L1"], reflect*a)
        theta2 = pi/2 - atan2(a, x) - atan2(sqrt(1.0 - A**2), A)
        theta3 = atan2(sqrt(1.0 - B**2), B)

        return np.array([theta1, -theta2, -theta3])
        
    def compute_leg_ik(self, leg: Legs, foot_point: np.ndarray, body_translation: np.ndarray, body_rotation: np.ndarray, use_foot_transform=True):
        left = leg in [Legs.FL, Legs.RL]
        if use_foot_transform:
            if (leg == Legs.FL):
                foot_point = ((self.get_translation_matrix(np.array([self.body_dimensions["L"]/2, (self.body_dimensions["W"]/2+self.leg_dimensions["L1"]), -self.body_dimensions["H"]])) @ self.get_translation_matrix(foot_point)))[:3,3]
            elif (leg == Legs.FR):
                foot_point = ((self.get_translation_matrix(np.array([self.body_dimensions["L"]/2, -(self.body_dimensions["W"]/2+self.leg_dimensions["L1"]), -self.body_dimensions["H"]])) @ self.get_translation_matrix(foot_point)))[:3,3]
            elif (leg == Legs.RL):
                foot_point = ((self.get_translation_matrix(np.array([-self.body_dimensions["L"]/2, (self.body_dimensions["W"]/2+self.leg_dimensions["L1"]), -self.body_dimensions["H"]])) @ self.get_translation_matrix(foot_point)))[:3,3]
            elif (leg == Legs.RR):
                foot_point = ((self.get_translation_matrix(np.array([-self.body_dimensions["L"]/2, -(self.body_dimensions["W"]/2+self.leg_dimensions["L1"]), -self.body_dimensions["H"]])) @ self.get_translation_matrix(foot_point)))[:3,3]
            else:
                carb.log_error(f"impossible to compute leg IK for leg {leg}")
        
        foot_body_ik = self.get_body_leg_ik(leg, body_translation, body_rotation)
        foot_point_ik = (np.linalg.inv(foot_body_ik) @ self.get_translation_matrix(foot_point))[:3,3]
        return self.get_leg_joints(foot_point_ik, left)

    def compute_quadruped_ik(self, body_translation: np.ndarray, body_rotation: np.ndarray, 
                             fl_foot_point: np.ndarray, fr_foot_point: np.ndarray, rl_foot_point: np.ndarray, rr_foot_point: np.ndarray,
                             use_foot_transform=True):
        fl_joints = self.compute_leg_ik(Legs.FL, fl_foot_point, body_translation, body_rotation, use_foot_transform)
        fr_joints = self.compute_leg_ik(Legs.FR, fr_foot_point, body_translation, body_rotation, use_foot_transform)
        rr_joints = self.compute_leg_ik(Legs.RR, rr_foot_point, body_translation, body_rotation, use_foot_transform)
        rl_joints = self.compute_leg_ik(Legs.RL, rl_foot_point, body_translation, body_rotation, use_foot_transform)
        return np.array([fl_joints, fr_joints, rr_joints, rl_joints])
