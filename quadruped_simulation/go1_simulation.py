from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path

from isaac_sim import go1

import rclpy
import numpy as np

import carb
carb.settings.get_settings().set("persistent/app/omniverse/gamepadCameraControl", False)

def on_physics_step(step_size):
    global go1_node
    go1_node.update()
    go1_node.publishImu()
    go1_node.publishContactSensors()

if __name__ == "__main__":
    rclpy.init()

    world = World(physics_dt=1/400.0, rendering_dt=10/400.0, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    assets_root_path = get_assets_root_path()
    go1_asset_path = assets_root_path + "/Isaac/Robots/Unitree/go1.usd"

    go1_robot = go1.GO1_Robot(
        usd_path=go1_asset_path,
        prim_path="/World/Go1",
        position=np.array([0.0, 0.0, 0.50]),
        physics_dt=1/400.0,
        name="go1",
        use_camera=False,
    )
    world.scene.add(go1_robot)

    simulation_app.update()
    world.reset()

    go1_robot.initializeRobot()
    go1_node = go1.GO1_Node(go1_robot)

    world.add_physics_callback("physics_callback", callback_fn=on_physics_step)

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()
