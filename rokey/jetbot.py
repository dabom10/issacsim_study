from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
import numpy as np


class Move_Jetbot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        assets_root_path = get_assets_root_path()
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        world.scene.add(
            WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
            )
        )
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._jetbot = self._world.scene.get_object("fancy_robot")
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        self._my_controller = WheelBasePoseController(name="cool_controller",
                                                        open_loop_wheel_controller=
                                                            DifferentialController(name="simple_control",
                                                                                    wheel_radius=0.03, wheel_base=0.1125),
                                                    is_holonomic=False)
        
        self.task_phase = 1

        await self._world.play_async()
        return

    def send_robot_actions(self, step_size):
        if self.task_phase == 1:
            position, orientation = self._jetbot.get_world_pose()
            self._jetbot.apply_action(self._my_controller.forward(start_position=position,
                                                                start_orientation=orientation,
                                                                goal_position=np.array([1.0, 1.0])))
            if np.mean(np.abs(position[:2] - np.array([0.8, 0.8]))) < 0.04:
                self._my_controller.reset()
                self.task_phase = 2

        elif self.task_phase == 2:
            position, orientation = self._jetbot.get_world_pose()
            self._jetbot.apply_action(self._my_controller.forward(start_position=position,
                                                                start_orientation=orientation,
                                                                goal_position=np.array([-1.0, 0.0])))
            if np.mean(np.abs(position[:2] - np.array([0.0, 0.0]))) < 0.04:
                self._my_controller.reset()
                self.task_phase = 3

        elif self.task_phase == 3:
            position, orientation = self._jetbot.get_world_pose()
            self._jetbot.apply_action(self._my_controller.forward(start_position=position,
                                                                start_orientation=orientation,
                                                                goal_position=np.array([-1.0, 1.0])))
            if np.mean(np.abs(position[:2] - np.array([0.0, 0.0]))) < 0.04:
                self._my_controller.reset()
                self.task_phase = 4

        elif self.task_phase == 4:
            position, orientation = self._jetbot.get_world_pose()
            self._jetbot.apply_action(self._my_controller.forward(start_position=position,
                                                                start_orientation=orientation,
                                                                goal_position=np.array([0.0, 1.0])))
            if np.mean(np.abs(position[:2] - np.array([0.0, 0.0]))) < 0.04:
                self._my_controller.reset()
                self.task_phase = 5

        elif self.task_phase == 5:
            position, orientation = self._jetbot.get_world_pose()
            self._jetbot.apply_action(self._my_controller.forward(start_position=position,
                                                                start_orientation=orientation,
                                                                goal_position=position))
            self._my_controller.reset()
            self.task_phase = 6
        return


    
