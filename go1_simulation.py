from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni

from omni.isaac.core import World
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

from omni.isaac.sensor import IMUSensor, Camera

from omni.kit.viewport.utility import get_active_viewport



from pxr import UsdGeom, Gf

import omni.graph.core as og

import numpy as np
import carb
import sys
import os

GO1_PRIM_PATH = "/World/Go1"

enable_extension("omni.isaac.ros2_bridge-humble")

world = World()
world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
go1_asset_path = "go1.usd"

if not os.path.exists(go1_asset_path):
    carb.log_error("Could not find go1 USD file")
    simulation_app.close()
    sys.exit()

add_reference_to_stage(usd_path=go1_asset_path, prim_path=GO1_PRIM_PATH)

go1 = world.scene.add(
    Robot(
        prim_path=GO1_PRIM_PATH,
        position=np.array([0.0, 0.0, 0.5]),
        name="go1",
    )
)

imu_path = GO1_PRIM_PATH + "/imu_link/imu_sensor"
imu_sensor = IMUSensor(
    prim_path=imu_path,
    name="imu",
    dt=1/60,
    translation=np.array([0, 0, 0]),
    orientation=np.array([1, 0, 0, 0]),
)

camera_face_path = GO1_PRIM_PATH + "/camera_optical_face/camera"
camera_face_sensor = Camera(
    prim_path=camera_face_path,
    frequency=20,
    resolution=(640, 420),
    orientation=euler_angles_to_quats(np.array([-90, -90, 0]), degrees=True),
)
camera_face_sensor.set_horizontal_aperture(21)
camera_face_sensor.set_vertical_aperture(16)
camera_face_sensor.set_projection_mode("perspective")
camera_face_sensor.set_focal_length(24)
camera_face_sensor.set_focus_distance(400)
camera_face_sensor.set_clipping_range(0.01, 1000)

try:
    go1_joints_graph = og.Controller.edit(
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
                ("ArticulationController.inputs:robotPath", GO1_PRIM_PATH),
            ],
        },
    )
    go1_imu_graph = og.Controller.edit(
        {"graph_path": "/Go1_IMU", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),

                ("ReadIMU", "omni.isaac.sensor.IsaacReadIMU"),
                ("PublishIMU", "omni.isaac.ros2_bridge.ROS2PublishImu"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "ReadIMU.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                ("Context.outputs:context", "PublishIMU.inputs:context"),

                ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishIMU.inputs:publishAngularVelocity", True),
                ("PublishIMU.inputs:publishLinearAcceleration", True),
                ("PublishIMU.inputs:publishOrientation", True),
                ("PublishIMU.inputs:frameId", "imu"),
            ],
        },
    )
    go1_camera = og.Controller.edit(
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

simulation_app.update()

set_target_prims(
    primPath="/Go1_Joints/PublishJointState", targetPrimPaths=[GO1_PRIM_PATH], inputName="inputs:targetPrim"
)
set_target_prims(
    primPath="/Go1_Joints/PublishTF", targetPrimPaths=[GO1_PRIM_PATH], inputName="inputs:targetPrims"
)
set_target_prims(
    primPath="/Go1_IMU/ReadIMU", targetPrimPaths=[imu_path], inputName="inputs:imuPrim"
)
set_target_prims(
    primPath="/Go1_Cameras/SetCamera", targetPrimPaths=[camera_face_path], inputName="inputs:cameraPrim"
)


world.reset()

while simulation_app.is_running():
    world.step(render=True)

    og.Controller.set(og.Controller.attribute("/Go1_Joints/OnImpulseEvent.state:enableImpulse"), True)
    og.Controller.set(og.Controller.attribute("/Go1_IMU/OnImpulseEvent.state:enableImpulse"), True)
    og.Controller.set(og.Controller.attribute("/Go1_Cameras/OnImpulseEvent.state:enableImpulse"), True)

simulation_app.close()
