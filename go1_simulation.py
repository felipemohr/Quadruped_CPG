from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

import omni.graph.core as og

import numpy as np
import carb
import sys
import os

GO1_STAGE_PATH = "/World/Go1"

enable_extension("omni.isaac.ros2_bridge-humble")

world = World()
world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
go1_asset_path = "go1.usd"

if not os.path.exists(go1_asset_path):
    carb.log_error("Could not find go1 USD file")
    simulation_app.close()
    sys.exit()

add_reference_to_stage(usd_path=go1_asset_path, prim_path=GO1_STAGE_PATH)

go1 = world.scene.add(
    Robot(
        prim_path=GO1_STAGE_PATH,
        position=np.array([0.0, 0.0, 0.5]),
        name="go1",
    )
)

simulation_app.update()

try:
    og.Controller.edit(
        {"graph_path": "/Go1Joints", "evaluator_name": "execution"},
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
                ("ArticulationController.inputs:robotPath", GO1_STAGE_PATH),
            ],
        },
    )
except Exception as e:
    print(e)


set_target_prims(
    primPath="/Go1Joints/PublishJointState", targetPrimPaths=[GO1_STAGE_PATH], inputName="inputs:targetPrim"
)
set_target_prims(
    primPath="/Go1Joints/PublishTF", targetPrimPaths=[GO1_STAGE_PATH], inputName="inputs:targetPrims"
)


world.reset()

while simulation_app.is_running():
    world.step(render=True)

    og.Controller.set(og.Controller.attribute("/Go1Joints/OnImpulseEvent.state:enableImpulse"), True)


simulation_app.close()
