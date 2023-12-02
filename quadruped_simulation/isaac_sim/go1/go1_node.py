import omni

from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.utils.extensions import enable_extension

import omni.graph.core as og

enable_extension("omni.isaac.ros2_bridge")
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu

class GO1_Node(Node):
    def __init__(self, go1_robot):
        self._go1 = go1_robot

        self._imu_data = self._go1.updateImuData()
        self._feet_contact_data = self._go1.updateContactSensorsData()

        self.createROS2Graphs()

        Node.__init__(self, "go1_node")

        self._imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self._imu_msg = Imu()
        self._imu_timer = self.create_timer(0.02, self.imuCallback)

        self._foot_publisher = dict()
        self._foot_msg = dict()
        for foot in self._go1._feet_order:
            self._foot_publisher[foot] = self.create_publisher(Bool, foot+'_contact', 10)
            self._foot_msg[foot] = Bool()
        self._feet_timer = self.create_timer(0.05, self.contactSensorsCallback)

    def update(self):
        self._imu_data = self._go1.updateImuData()
        self._feet_contact_data = self._go1.updateContactSensorsData()
        og.Controller.set(og.Controller.attribute("/Go1_Joints/OnImpulseEvent.state:enableImpulse"), True)
  
    def imuCallback(self):
        quat = euler_angles_to_quats(self._imu_data["rotation"])

        self._imu_msg.header.stamp = self.get_clock().now().to_msg()
        self._imu_msg.header.frame_id = "imu_link"
        self._imu_msg.orientation.w = quat[0]
        self._imu_msg.orientation.x = quat[1]
        self._imu_msg.orientation.y = quat[2]
        self._imu_msg.orientation.z = quat[3]
        self._imu_msg.linear_acceleration.x = self._imu_data["lin_acc"][0]
        self._imu_msg.linear_acceleration.y = self._imu_data["lin_acc"][1]
        self._imu_msg.linear_acceleration.z = self._imu_data["lin_acc"][2]
        self._imu_msg.angular_velocity.x = self._imu_data["ang_vel"][0]
        self._imu_msg.angular_velocity.y = self._imu_data["ang_vel"][1]
        self._imu_msg.angular_velocity.z = self._imu_data["ang_vel"][2]
        self._imu_msg.orientation_covariance = self._imu_data["rotation_cov"].flatten()
        self._imu_msg.linear_acceleration_covariance = self._imu_data["lin_acc_cov"].flatten()
        self._imu_msg.angular_velocity_covariance = self._imu_data["ang_vel_cov"].flatten()

        self._imu_publisher.publish(self._imu_msg)

    def contactSensorsCallback(self):
        for foot in self._go1._feet_order:
            self._foot_msg[foot].data = self._feet_contact_data[foot]
            self._foot_publisher[foot].publish(self._foot_msg[foot])

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
