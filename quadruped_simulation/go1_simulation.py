from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
import omni.graph.core as og

from isaac_sim import go1

import rclpy
import numpy as np

if __name__ == "__main__":
    rclpy.init()

    world = World()
    world.scene.add_default_ground_plane()

    go1_robot = go1.GO1_Robot(
        prim_path="/World",
        position=np.array([0.0, 0.0, 0.5]),
        name="go1",
        usd_path="isaac_sim/go1/go1.usd",
        use_camera=False
    )
    go1_node = go1.GO1_Node(go1_robot)
    world.scene.add(go1_robot)

    simulation_app.update()
    world.reset()

    while simulation_app.is_running():
        world.step(render=True)
        
        og.Controller.set(og.Controller.attribute("/Go1_Joints/OnImpulseEvent.state:enableImpulse"), True)

        rclpy.spin_once(go1_node)

    simulation_app.close()
