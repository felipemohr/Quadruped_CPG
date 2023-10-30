from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni

from omni.isaac.core import World
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats, quats_to_euler_angles
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

from omni.isaac.sensor import IMUSensor, Camera

import omni.graph.core as og

from typing import Optional, Sequence

import numpy as np
import carb
import sys
import os

enable_extension("omni.isaac.ros2_bridge-humble")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class GO1_Robot(Robot):
    def __init__(
        self,
        prim_path: str,
        name: str = "go1",
        usd_path: str = "go1.usd",
        use_camera: bool = True,
        physics_dt: Optional[float] = 1 / 400.0,
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
    ) -> None:

        if not os.path.exists(usd_path):
            carb.log_error("Could not find go1 USD file")
            simulation_app.close()
            sys.exit()

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        Robot.__init__(self, 
                       prim_path=prim_path, 
                       name=name, 
                       position=position, 
                       translation=translation, 
                       orientation=orientation)

        self._prim_path = prim_path
        self._physics_dt = physics_dt
        self._use_camera = use_camera

        self._body_rotation = np.zeros(3)
        self._body_lin_acc = np.zeros(3)
        self._body_ang_vel = np.zeros(3)

        self.createSensors()

    def createSensors(self):
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
        

class GO1_Node(Node):
    def __init__(self, go1_robot: GO1_Robot):
        self._go1 = go1_robot

        self.createROS2Graphs()

        Node.__init__(self, "go1_node")

        self._imu_publisher = self.create_publisher(Imu, 'go1_imu', 10)
        self._imu_msg = Imu()
        self._imu_timer = self.create_timer(0.02, self.imuCallback)

    def imuCallback(self):
        data = self._go1.updateImuData()
        quat = euler_angles_to_quats(data["rotation"])

        self._imu_msg.header.stamp = self.get_clock().now().to_msg()
        self._imu_msg.header.frame_id = "imu_link"
        self._imu_msg.orientation.w = quat[0]
        self._imu_msg.orientation.x = quat[1]
        self._imu_msg.orientation.y = quat[2]
        self._imu_msg.orientation.z = quat[3]
        self._imu_msg.linear_acceleration.x = data["lin_acc"][0]
        self._imu_msg.linear_acceleration.y = data["lin_acc"][1]
        self._imu_msg.linear_acceleration.z = data["lin_acc"][2]
        self._imu_msg.angular_velocity.x = data["ang_vel"][0]
        self._imu_msg.angular_velocity.y = data["ang_vel"][1]
        self._imu_msg.angular_velocity.z = data["ang_vel"][2]
        self._imu_msg.orientation_covariance = data["rotation_cov"].flatten()
        self._imu_msg.linear_acceleration_covariance = data["lin_acc_cov"].flatten()
        self._imu_msg.angular_velocity_covariance = data["ang_vel_cov"].flatten()

        self._imu_publisher.publish(self._imu_msg)

    def createROS2Graphs(self):
        try:
            self._go1_joints_graph = og.Controller.edit(
                {"graph_path": "/Go1_Joints", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                        ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                        ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                        ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                        ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnImpulseEvent.outputs:execOut", "PublishTF.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                        ("Context.outputs:context", "PublishTF.inputs:context"),
                        ("Context.outputs:context", "PublishClock.inputs:context"),
                        ("Context.outputs:context", "PublishJointState.inputs:context"),
                        ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                        ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                        ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ArticulationController.inputs:usePath", True),
                        ("ArticulationController.inputs:robotPath", self._go1._prim_path),
                    ],
                },
            )
            if self._go1._use_camera:
                self._go1_camera = og.Controller.edit(
                    {"graph_path": "/Go1_Cameras", "evaluator_name": "execution"},
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                            ("CreateViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                            ("GetViewportRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                            ("SetCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                            ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ],
                        og.Controller.Keys.CONNECT: [
                            ("OnImpulseEvent.outputs:execOut", "CreateViewport.inputs:execIn"),
                            ("CreateViewport.outputs:execOut", "GetViewportRenderProduct.inputs:execIn"),
                            ("CreateViewport.outputs:viewport", "GetViewportRenderProduct.inputs:viewport"),

                            ("GetViewportRenderProduct.outputs:execOut", "SetCamera.inputs:execIn"),
                            ("GetViewportRenderProduct.outputs:renderProductPath", "SetCamera.inputs:renderProductPath"),
                            ("SetCamera.outputs:execOut", "CameraHelper.inputs:execIn"),
                            ("GetViewportRenderProduct.outputs:renderProductPath", "CameraHelper.inputs:renderProductPath"),

                            ("Context.outputs:context", "CameraHelper.inputs:context"),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            ("CreateViewport.inputs:name", "Camera Viewport"),
                            ("CameraHelper.inputs:frameId", "camera"),
                            ("CameraHelper.inputs:topicName", "/camera/image"),
                            ("CameraHelper.inputs:type", "rgb"),
                        ],
                    },
                )
        except Exception as e:
            print(e)
        
        set_target_prims(
            primPath="/Go1_Joints/PublishJointState", targetPrimPaths=[self._go1._prim_path], inputName="inputs:targetPrim"
        )
        set_target_prims(
            primPath="/Go1_Joints/PublishTF", targetPrimPaths=[self._go1._prim_path], inputName="inputs:targetPrims"
        )
        if self._go1._use_camera:
            set_target_prims(
                primPath="/Go1_Cameras/SetCamera", targetPrimPaths=[self._go1._camera_face_path], inputName="inputs:cameraPrim"
            )

if __name__ == "__main__":

    rclpy.init()

    GO1_PRIM_PATH = "/World/Go1"

    world = World()
    world.scene.add_default_ground_plane()

    go1_robot = GO1_Robot(
        prim_path=GO1_PRIM_PATH,
        position=np.array([0.0, 0.0, 0.5]),
        name="go1",
        usd_path="go1.usd",
        use_camera=False
    )
    go1_node = GO1_Node(go1_robot)
    go1 = world.scene.add(go1_robot)

    simulation_app.update()
    world.reset()

    while simulation_app.is_running():
        world.step(render=True)
        
        og.Controller.set(og.Controller.attribute("/Go1_Joints/OnImpulseEvent.state:enableImpulse"), True)

        rclpy.spin_once(go1_node)

    simulation_app.close()
