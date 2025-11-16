import sys
import numpy as np
import genesis as gs
from typing import Any, Dict, Tuple
from planning import PlannerInterface
import robot_adapter
import builder

class MotionPrimitives:
    def __init__(self, robot, blocks_state, scene):
        self.robot =  robot
        self.blocks_state = blocks_state
        self.scene = scene
        self.planInterface = PlannerInterface(robot, scene)



        self.Z_DISTANCE_GAP = 0.03
        self.GRIPPER_OPEN = 0.05
        self.GRIPPER_CLOSE = 0.00
        self.LINK_NAME = "ee"

    def get_cube_pos(self, cube: str):
        return builder.get_block_pos(cube, self.blocks_state)
    

    def waypoint_plan(self, path):
         for waypoint in path:
            waypoint = np.asarray(waypoint, dtype=np.float32)
            self.robot.control_dofs_position(waypoint)
            self.scene.step()
        
    def pick_up(self, cube):

        #pre-grasp - z-distance above cube
        cube_pos = self.get_cube_pos(cube)
        x_cube, y_cube, z_cube = float(cube_pos[0]), float(cube_pos[1]), float(cube_pos[2])

        target_pos1 = np.array([x_cube, y_cube, z_cube + self.Z_DISTANCE_GAP])

        target_quat1 = np.array([0.0, 1.0, 0.0, 0.0])

        ee_link = self.robot.get_link(self.LINK_NAME)
        goal1 = self.robot.inverse_kinematics(link= ee_link, pos=target_pos1, quat=target_quat1)

        goal1 = np.array([goal1])
        goal1[-2:] = self.GRIPPER_OPEN

        path1 = PlannerInterface.plan_path(
            qpos_goal=goal1,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=200,  # 5s duration
            attached_object=None,
            planner="RRT"
        )

        if not self._exec_path(path1):
            print(f"[pick_up] planning failed for pre-grasp over {cube}")
            return False
        
        print(f"[pick_up] Reached pre-grasp over block {cube}")

        #Grasp
        target_pos2 = np.array([x_cube, y_cube, z_cube + 0.02])
        target_quat2 = np.array([0.0, 1.0, 0.0, 0.0])

        goal2 = self.robot.inverse_kinematics(link= ee_link, pos=target_pos2, quat=target_quat2)

        path2 = PlannerInterface.plan_path(
            qpos_goal=goal1,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=200,  # 5s duration
            attached_object=None,
            planner="RRT"
        )

        if not self._exec_path(path2):
            print(f"[pick_up] planning failed for grasp move over {cube}")
            return False
        
        goal2 = np.array([goal1])
        goal2[-2:] = self.GRIPPER_CLOSE

        #lift
        target_pos3  = np.array([x_cube, y_cube, z_cube + 1.0])
        target_quat3 = target_quat2

        goal3 = self.robot.inverse_kinematics(link=ee_link, pos=target_pos3, quat=target_quat3)
        goal3 = np.array(goal3, dtype=float)
        goal3[-2:] = self.GRIPPER_CLOSE

        path3 = self.planInterface.plan_path(
            qpos_goal=goal3,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=100,
            attached_object=None, 
            planner="RRT"
        )
        if not self._exec_path(path3):
            print(f"[pick_up] planning failed for lift move over {cube}")
            return False









    


