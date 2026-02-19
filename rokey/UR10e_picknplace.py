import numpy as np
import sys
import carb

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.types import ArticulationAction

from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController


class Tutorial_UR10(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        self._placing_position = np.array([0.7, 0.7, 0.0515 / 2.0])
        self._end_effector_offset = np.array([0, 0, 0.02])
        self.task_phase = 1
        self._task_done = False
        
        self.my_controller = None
        self.articulation_controller = None
        self.cube = None
        self.robots = None
        return

    
    def setup_scene(self):
        """ì”¬ì„ ì„¤ì •í•©ë‹ˆë‹¤ (ë¡œë´‡, íë¸Œ, í™˜ê²½ ë“±)."""
        world = self.get_world()
        world.scene.add_default_ground_plane()

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            sys.exit()

        asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
        robot.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
        gripper = SurfaceGripper(
            end_effector_prim_path="/World/UR10/ee_link", surface_gripper_path="/World/UR10/ee_link/SurfaceGripper"
        )
        ur10 = world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper
            )
        )
        # ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        world.scene.add(
            DynamicCuboid(
                name="cube1",
                position=np.array([0.3, 0.3, 0.3]), 
                prim_path="/World/Cube1",
                scale=np.array([0.0515, 0.0515, 0.0515]),
                size=1.0,
                color=np.array([0, 0, 1]),
            )
        )
        world.scene.add(
            DynamicCuboid(
                name="cube2",
                position=np.array([0.4, 0.3, 0.3]), 
                prim_path="/World/Cube2",
                scale=np.array([0.0515, 0.0515, 0.0515]),
                size=1.0,
                color=np.array([0, 0, 1]),
            )
        )

        return

    async def setup_post_load(self):
        self._world = self.get_world()

        self.robots = self._world.scene.get_object("my_ur10")
        self.cube1 = self._world.scene.get_object("cube1")
        self.cube2 = self._world.scene.get_object("cube2")
        self.my_controller = PickPlaceController(
            name="pick_place_controller", 
            gripper=self.robots.gripper, 
            robot_articulation=self.robots
        )
        self.articulation_controller = self.robots.get_articulation_controller()

        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)

        await self._world.play_async()
        return
    

    def move_joint_direct(self, target_joint_positions: np.ndarray) -> bool:
        action = ArticulationAction(joint_positions=target_joint_positions)
        self.robots.apply_action(action)

        current_joint_positions = self.robots.get_joint_positions()

        is_reached = np.all(
            np.abs(current_joint_positions[:6] - target_joint_positions[:6]) < 0.01
        )

        return is_reached

    def physics_step(self, step_size):
        # 임의 초기자세
        if self.task_phase == 1:
            arm_target_deg_1 = np.array([0.0, -90.0, 0.0, 0.0, 90.0, 0.0])
            self._target_joint_positions_1 = np.deg2rad(arm_target_deg_1)
            if self.move_joint_direct(self._target_joint_positions_1):
                self.task_phase = 2

        # pick&place 초기 자세
        elif self.task_phase == 2:
            arm_target_deg_2 = np.array([0.0, -90.0, 0.0, -90.0, 0.0, 0.0])
            self._target_joint_positions_2 = np.deg2rad(arm_target_deg_2)
            if self.move_joint_direct(self._target_joint_positions_2):
                self.my_controller.reset()
                self.task_phase = 3


        # 첫번째 큐브 이동
        elif self.task_phase == 3:
            picking_position = self.cube1.get_world_pose()[0]

            actions = self.my_controller.forward(
                picking_position=picking_position,
                placing_position=self._placing_position,
                current_joint_positions=self.robots.get_joint_positions(),
                end_effector_offset=self._end_effector_offset,
            )

            self.articulation_controller.apply_action(actions)
            if self.my_controller.is_done():
                self.my_controller.reset()
                self.task_phase = 4

        # 두번째 큐브 이동
        elif self.task_phase == 4:
            picking_position = self.cube2.get_world_pose()[0]

            actions = self.my_controller.forward(
                picking_position=picking_position,
                placing_position=self._placing_position,
                current_joint_positions=self.robots.get_joint_positions(),
                end_effector_offset=self._end_effector_offset,
            )

            self.articulation_controller.apply_action(actions)
            if self.my_controller.is_done():
                self.my_controller.reset()
                self.task_phase = 5
        return

    async def setup_pre_reset(self):
        if self.my_controller is not None:
            self.my_controller.reset()
        self._task_done = False
        return