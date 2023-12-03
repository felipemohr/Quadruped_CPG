import omni

from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats, quats_to_euler_angles
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

from omni.isaac.sensor import IMUSensor, ContactSensor, Camera

from typing import Optional, Sequence

import numpy as np

class GO1_Robot(Robot):
    def __init__(
        self,
        usd_path: str,
        prim_path: str = "/World/Go1",
        name: str = "go1",
        use_camera: bool = True,
        physics_dt: Optional[float] = 1 / 400.0,
        position: Optional[Sequence[float]] = np.array([0.0, 0.0, 0.5], dtype=np.float32),
        orientation: Optional[Sequence[float]] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    ) -> None:

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        Robot.__init__(self, 
                       prim_path=prim_path, 
                       name=name, 
                       position=position, 
                       orientation=orientation)

        self._prim_path = prim_path
        self._physics_dt = physics_dt
        self._use_camera = use_camera

        self._init_pos = position
        self._init_orientation = orientation

        self._body_rotation = np.zeros(3)
        self._body_lin_acc = np.zeros(3)
        self._body_ang_vel = np.zeros(3)

        self._feet_order = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]

        self.initializeSensors()

    def initializeRobot(self, joint_kps=1000.0, joint_kds=20.0):
        self.set_world_pose(position=self._init_pos, orientation=self._init_orientation)
        self.set_linear_velocity(np.zeros(3))
        self.set_angular_velocity(np.zeros(3))
        self.initializeJoints(joint_kps, joint_kds)

    def initializeJoints(self, kps, kds):
        joint_pos = np.array([0.0, 0.0, 0.0, 0.0, np.pi/4, np.pi/4, np.pi/4, np.pi/4, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2], dtype=np.float32)
        self.set_joint_positions(positions=joint_pos)
        self.set_joint_velocities(velocities=np.zeros(12))
        self.set_joint_efforts(np.zeros(12))
        self.get_articulation_controller().set_gains(kps=kps, kds=kds)

    def initializeSensors(self):
        self._imu_path = self._prim_path + "/imu_link/imu_sensor"
        
        self._imu_rotation_std = 0.01
        self._imu_lin_acc_std = 0.01
        self._imu_ang_vel_std = 0.01

        self._imu_sensor = IMUSensor(
            prim_path=self._imu_path,
            name="imu",
            dt=self._physics_dt,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
        )

        self._sensors.append(self._imu_sensor)

        self._feet_contact = dict()
        for foot in self._feet_order:
            self._feet_contact[foot] = True

        self._contact_sensors = dict()
        for foot in self._feet_order:
            self._contact_sensors[foot] = ContactSensor(
                prim_path=self._prim_path + "/" + foot + "/sensor",
                min_threshold=0,
                max_threshold=1000000,
                radius=0.03,
                dt=self._physics_dt,
            )
            self._sensors.append(self._contact_sensors[foot])

        if self._use_camera:
            self._camera_face_path = self._prim_path + "/camera_optical_face/camera"
            self._camera_face_sensor = Camera(
                prim_path=self._camera_face_path,
                frequency=20,
                resolution=(640, 420),
                orientation=euler_angles_to_quats(np.array([-90, -90, 0]), degrees=True),
            )
            self._camera_face_sensor.set_horizontal_aperture(21)
            self._camera_face_sensor.set_vertical_aperture(16)
            self._camera_face_sensor.set_projection_mode("perspective")
            self._camera_face_sensor.set_focal_length(24)
            self._camera_face_sensor.set_focus_distance(400)
            self._camera_face_sensor.set_clipping_range(0.01, 1000)

            self._sensors.append(self._camera_face_sensor)

    def updateImuData(self):
        frame = self._imu_sensor.get_current_frame()

        self._body_rotation = quats_to_euler_angles(frame["orientation"]) + np.random.normal(0, self._imu_rotation_std, 3)
        self._body_lin_acc = frame["lin_acc"] + np.random.normal(0, self._imu_lin_acc_std, 3)
        self._body_ang_vel = frame["ang_vel"] + np.random.normal(0, self._imu_ang_vel_std, 3)
        
        return {"rotation": self._body_rotation,
                "lin_acc": self._body_lin_acc,
                "ang_vel": self._body_ang_vel,
                "rotation_cov": np.diag([self._imu_rotation_std**2, self._imu_rotation_std**2, self._imu_rotation_std**2]),
                "lin_acc_cov": np.diag([self._imu_lin_acc_std**2, self._imu_lin_acc_std**2, self._imu_lin_acc_std**2]),
                "ang_vel_cov": np.diag([self._imu_ang_vel_std**2, self._imu_ang_vel_std**2, self._imu_ang_vel_std**2])}
        
    def updateContactSensorsData(self):
        for foot in self._feet_order:
            frame = self._contact_sensors[foot].get_current_frame()
            self._feet_contact[foot] = frame["in_contact"]
        return self._feet_contact
    
    def sendJointsCommand(self, joint_pos):
        joint_pos = np.asarray(np.array(joint_pos.reshape([4, 3]).T.flat), dtype=np.float32)
        self.get_articulation_controller().apply_action(ArticulationAction(joint_positions=joint_pos))
