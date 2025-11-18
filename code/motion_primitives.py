import sys
import numpy as np
import genesis as gs
from typing import Any, Dict, Tuple
from planning import PlannerInterface
import robot_adapter
import builder

class MotionPrimitives:
    def __init__(self, robot, blocks_state, scene) -> None:
        self.robot =  robot
        self.blocks_state = blocks_state
        self.scene = scene
        self.planInterface = PlannerInterface(robot, scene)



        self.Z_DISTANCE_GAP = 0.03
        self.GRIPPER_OPEN = 0.05
        self.GRIPPER_CLOSE = 0.00
        self.LINK_NAME = "hand"

    def get_cube_pos(self, cube: str) -> np.array:
        return builder.get_block_pos(cube, self.blocks_state)
    

    def waypoint_plan(self, path):
         for waypoint in path:
            waypoint = np.asarray(waypoint, dtype=np.float32)
            self.robot.control_dofs_position(waypoint)
            self.scene.step()
        
    def pick_up(self, cube) -> bool:

        #pre-grasp - z-distance above cube
        cube_pos = self.get_cube_pos(cube)
        x_cube, y_cube, z_cube = float(cube_pos[0]), float(cube_pos[1]), float(cube_pos[2])

        target_pos1 = np.array([x_cube, y_cube, z_cube + self.Z_DISTANCE_GAP])

        target_quat1 = np.array([0.0, 1.0, 0.0, 0.0])

        ee_link = self.robot.get_link(self.LINK_NAME)
        goal1 = self.robot.inverse_kinematics(link= ee_link, pos=target_pos1, quat=target_quat1)

        goal1 = np.array(goal1, dtype=float)
        goal1[-2:] = self.GRIPPER_OPEN

        path1 = self.planInterface.plan_path(
            qpos_goal=goal1,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=200,  # 5s duration
            attached_object=None,
            planner="RRT"
        )

        if not self.waypoint_plan(path1):
            print(f"[pick_up] planning failed for pre-grasp over {cube}")
            return False
        
        print(f"[pick_up] Reached pre-grasp over block {cube}")

        #Grasp
        target_pos2 = np.array([x_cube, y_cube, z_cube + 0.02])
        target_quat2 = np.array([0.0, 1.0, 0.0, 0.0])

        goal2 = self.robot.inverse_kinematics(link= ee_link, pos=target_pos2, quat=target_quat2)

        path2 = self.planInterface.plan_path(
            qpos_goal=goal1,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=200,  # 5s duration
            attached_object=None,
            planner="RRT"
        )

        if not self.waypoint_plan(path2):
            print(f"[pick_up] planning failed for grasp move over {cube}")
            return False
        
        goal2 = np.array(goal2, dtype=float)
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
        if not self.waypoint_plan(path3):
            print(f"[pick_up] planning failed for lift move over {cube}")
            return False
        
        return True


    def stack(self, cube1: str, cube2: str) -> bool:
        
        # Finding the target cube to stack on and saving coordinated and preplace position
        that_cube = self.blocks_state[cube2]
        cube2_pos = self.get_cube_pos(cube2)
        target_x, target_y, target_z = np.array[float(cube2_pos[0]), float(cube2_pos[1]), float(cube2_pos[2])]
        target_cube_pos =  np.array[target_x, target_y, target_z + self.Z_DISTANCE_GAP]
        target_quat1 = np.array([0.0, 1.0, 0.0, 0.0])

        ee_link = self.robot.get_link(self.LINK_NAME)
        # Gripper remains in the same state as previously called assuming 'stack'
        #   is always called after 'pick-up' (gripper is closed while holding another cube)
        stack_goal = self.robot.inverse_kinematics(link= ee_link, pos=target_cube_pos, quat=target_quat1)
        stack_goal = np.array(stack_goal, dtype=float)

        stack_path = self.planInterface.plan_path(
            qpos_goal=stack_goal,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=300,  # 3s duration
            attached_object=that_cube,
            planner="RRT"
        )

        if not self.waypoint_plan(stack_path):
            print(f"[Stack]: planning failed for motion involving initial cube grasped {cube1}")
            return False

        print(f"[Stacking]: Reached pre-positioning spot on top of block {cube2}")

        # Let it gooooo
        letgo_goal = stack_goal
        letgo_goal[-2:] = self.GRIPPER_OPEN

        letgo_path = self.planInterface.plan_path(
            qpos_goal=letgo_goal,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=100,  # 1s duration
            attached_object=None,   # No more object
            planner="RRT"
        )

        if not self.waypoint_plan(letgo_path):
            print(f"[Letting Gooo]: planning failed for motion involving releasing grasped cube {cube1}")
            return False

        print(f"[Letting Gooo]: Successfully placed block on top of block {cube2}")

        # Return to a more natural state for better path planning
        #   we move away from the original cube we had grasped (now placed on top of another)
        holding_cube1_pos = self.get_cube_pos(cube1)
        move_x, move_y, move_z = np.array[float(holding_cube1_pos[0]), float(holding_cube1_pos[1]), float(holding_cube1_pos[2])]
        move_away_target = np.array[move_x, move_y, move_z + self.Z_DISTANCE_GAP + 0.2]

        move_away_goal = self.robot.inverse_kinematics(link= ee_link, pos=move_away_target, quat=target_quat1)
        move_away_goal = np.array(stack_goal, dtype=float)

        # Gripper stays open
        move_away_path = self.planInterface.plan_path(
            qpos_goal=move_away_goal,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=300,  # 3s duration
            attached_object=that_cube,
            planner="RRT"
        )

        if not self.waypoint_plan(move_away_path):
            print(f"[Moving Away]: planning failed for motion involving moving away from {cube1}")
            return False

        print(f"[Moving Away]: Successfully distanced ourselves from block {cube2}")

        return True


    def parse_symbolic_plan(self, plan):
        """
        Parses a symbolic plan into a sequence of motion primitives.

        Args:
            plan: A list of symbolic actions.
        Returns:
            A list of motion primitives and their arguments.
        """
        string2action = {
            'pick-up': self.pick_up,
            'stack': self.stack,
            # 'unstack': self.unstack
            }

        cleaned_plan = plan.replace('(', '').replace(')', '')
        steps = cleaned_plan.split('\n')
        actions = []
        for step in steps:
            step_cleaned = step.split()
            primative = string2action[step_cleaned[0]]
            args = [color[0] for color in step_cleaned[2:]]
            actions.append((primative, args))
        return actions

    def execute_symbolic_plan(self, input_file, blockstate):
        with open(input_file, 'r') as file:
            plan = file.read()

        actions = self.parse_symbolic_plan(plan)
        for i in range(len(actions)):
            action, args = actions[i]
            cubes = [blockstate[arg] for arg in args]
            action(*cubes)

if __name__ == "__main__":
    test_plan = """(pick-up r magenta)
    (stack r magenta cyan)
    (pick-up r yellow)
    (stack r yellow magenta)
    (pick-up r green)
    (stack r green blue)
    (pick-up r red)
    (stack r red green)"""
    
    # actions = parse_symbolic_plan(test_plan)
    # for action in actions:
    #     print(action)












    
