import sys
import numpy as np
import genesis as gs
from typing import Any, Dict, Tuple
from planning import PlannerInterface

class MotionPrimitives:
    def __init__(self, robot, blocks_state, scene) -> None:
        self.robot =  robot
        self.blocks_state = blocks_state
        self.scene = scene
        self.planInterface = PlannerInterface(robot, scene)

        self.complete_waypoint_path = []

        self.Z_DISTANCE_GAP = 0.15
        self.GRIPPER_OPEN = 0.035
        self.GRIPPER_CLOSE = 0.020
        self.LINK_NAME = "hand"

    def get_block_pos(sef, block_color: str, BlocksState: Dict[str, Any]) -> np.array:
        
        that_block = BlocksState[block_color]
        block_position = that_block.get_pos()
        '''
        if (block_color == "r"):
            print("Red block position (x, y, z):", block_position)
        if (block_color == "g"):
            print("Green block position (x, y, z):", block_position)
        if (block_color == "b"):
            print("Blue block position (x, y, z):", block_position)
        if (block_color == "y"):
            print("Yellow block position (x, y, z):", block_position)
        if (block_color == "m"):
            print("Magenta block position (x, y, z):", block_position)
        if (block_color == "c"):
            print("Cyan block position (x, y, z):", block_position)
        '''
        return block_position
    
    def get_cube_pos(self, cube: str) -> np.array:
        return self.get_block_pos(cube, self.blocks_state)
    

    def waypoint_plan(self, path, cube: str = None):
        """Execute a list of waypoints."""
        if not path:
            print("There are no waypoints to plot or animate.")
            return False
        for waypoint in path:
            if cube:
                offset = np.array([0.0, 0.019, -0.047])
                finger_pose = self.robot.get_link('left_finger').get_pos()
                cube_pose = finger_pose
                cube_pose[:3] += offset

                #new_cube_pose = finger_pose
                #new_cube_pose[:3] += offset

                self.blocks_state.get(cube).set_pos(cube_pose)
            waypoint = np.asarray(waypoint, dtype=np.float32)
            self.robot.control_dofs_position(waypoint)
            self.scene.step()
        return True
        
    def pick_up(self, cube: str) -> bool:
      
        cube_pos = self.get_cube_pos(cube)
        x_cube, y_cube, z_cube = float(cube_pos[0]), float(cube_pos[1]), float(cube_pos[2])

        target_pos1 = np.array([x_cube, y_cube, z_cube + self.Z_DISTANCE_GAP], dtype=float)
        target_quat1 = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)

        ee_link = self.robot.get_link(self.LINK_NAME)
        goal1 = self.robot.inverse_kinematics(link=ee_link, pos=target_pos1, quat=target_quat1)

        gs.logger.warning(f"The IK joint values for this goal position are:{goal1}")

        if goal1 is None:
            print(f"[pick_up]: IK failed for pre-grasp over {cube}")
            return False

        goal1 = np.array(goal1, dtype=float)
        goal1[-2:] = self.GRIPPER_OPEN

        path1 = self.planInterface.plan_path(
            qpos_goal=goal1,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=300,  # 5s duration
            attached_object=None,
            planner="RRTstar",
        )

        if not self.waypoint_plan(path1):
            print(f"[pick_up]: planning failed for pre-grasp over {cube}")
            return False

        print(f"[pick_up]: Reached pre-grasp over block {cube}")

        
        target_pos2 = np.array([x_cube, y_cube, z_cube + 0.12], dtype=float)
        target_quat2 = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)

        goal2 = self.robot.inverse_kinematics(link=ee_link, pos=target_pos2, quat=target_quat2)
        if goal2 is None:
            print(f"[pick_up]: IK failed for grasp pose over {cube}")
            return False

        goal2 = np.array(goal2, dtype=float)
        
        #goal2[-2:] = self.GRIPPER_OPEN

        path2 = self.planInterface.plan_path(
            qpos_goal=goal2,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=300,
            attached_object=self.blocks_state.get(cube),
            planner="RRTstar",
        )

        if not self.waypoint_plan(path2):
            print(f"[pick_up] planning failed for grasp move over {cube}")
            return False

        # close gripper at bottom
        goal2_closed = goal2.copy()
        goal2_closed[-2:] = self.GRIPPER_CLOSE
        self.robot.control_dofs_position(goal2_closed)
        for _ in range(20):
            self.scene.step()

       
        target_pos3 = np.array([x_cube, y_cube, z_cube + 0.22], dtype=float)
        target_quat3 = target_quat2

        goal3 = self.robot.inverse_kinematics(link=ee_link, pos=target_pos3, quat=target_quat3)
        if goal3 is None:
            print(f"[pick_up] IK failed for lift pose over {cube}")
            return False

        goal3 = np.array(goal3, dtype=float)
        goal3[-2:] = self.GRIPPER_CLOSE

        path3 = self.planInterface.plan_path(
            qpos_goal=goal3,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=300,
            attached_object=self.blocks_state.get(cube),  #pass attached cube entity here
            planner="RRTstar",
        )
        
        if not self.waypoint_plan(path3, cube):
            print(f"[pick_up] planning failed for lift move over {cube}")
            return False

        print(f"[pick_up] Finished pick-up of {cube}")
        return True
    

    def stack(self, cube1: str, cube2: str) -> bool:
    
        #that_cube = self.blocks_state[cube2]
        cube2_pos = self.get_cube_pos(cube2)

        target_x = float(cube2_pos[0])
        target_y = float(cube2_pos[1])
        target_z = float(cube2_pos[2])
        target_cube_pos = np.array(
            [target_x, target_y + 0.01, target_z + self.Z_DISTANCE_GAP + 0.02], dtype=float
        )
        target_quat1 = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)

        ee_link = self.robot.get_link(self.LINK_NAME)

        stack_goal = self.robot.inverse_kinematics(
            link=ee_link, pos=target_cube_pos, quat=target_quat1
        )
        if stack_goal is None:
            print(f"[stack] IK failed for stacking {cube1} on {cube2}")
            return False

        stack_goal = np.array(stack_goal, dtype=float)

        stack_path = self.planInterface.plan_path(
            qpos_goal=stack_goal,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=300,  # 3s duration
            attached_object=self.blocks_state.get(cube1),  
            planner="RRTstar",
        )

        if not self.waypoint_plan(stack_path, cube1):
            print(f"[Stack]: planning failed for motion involving initial cube grasped {cube1}")
            return False

        print(f"[Stacking]: Reached pre-positioning spot on top of block {cube2}")

      
        letgo_goal = stack_goal.copy()
        letgo_goal[-2:] = self.GRIPPER_OPEN

        letgo_path = self.planInterface.plan_path(
            qpos_goal=letgo_goal,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=300,  # 1s duration
            attached_object=self.blocks_state.get(cube1),
            planner="RRTstar",
        )

        if not self.waypoint_plan(letgo_path):
            print(f"[Letting Gooo]: planning failed for releasing grasped cube {cube1}")
            return False

        print(f"[Letting Gooo]: Successfully placed block on top of block {cube2}")

        
        holding_cube1_pos = self.get_cube_pos(cube1)
        move_x = float(holding_cube1_pos[0])
        move_y = float(holding_cube1_pos[1])
        move_z = float(holding_cube1_pos[2])

        move_away_target = np.array(
            [move_x, move_y, move_z + self.Z_DISTANCE_GAP + 0.22], dtype=float
        )

        move_away_goal = self.robot.inverse_kinematics(
            link=ee_link, pos=move_away_target, quat=target_quat1
        )
        if move_away_goal is None:
            print(f"[Moving Away]: IK failed when moving away from {cube1}")
            return False

        move_away_goal = np.array(move_away_goal, dtype=float)

        move_away_path = self.planInterface.plan_path(
            qpos_goal=move_away_goal,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=300,  # 3s duration
            attached_object=self.blocks_state.get(cube1),
            planner="RRTstar",
        )

        if not self.waypoint_plan(move_away_path):
            print(f"[Moving Away]: planning failed for motion involving moving away from {cube1}")
            return False

        print(f"[Moving Away]: Successfully distanced ourselves from block {cube2}")
        return True




    def parse_symbolic_plan(self, plan):
     
        string2action = {
            'pick-up': self.pick_up,
            'stack': self.stack,
            # 'unstack': self.unstack
            }

        cleaned_plan = plan.replace('(', '').replace(')', '')
        steps = cleaned_plan.split('\n')
        actions = []
        for step in steps:
            if not step.strip():
                continue
            
            step_cleaned = step.split()
            print(step_cleaned)
            primative = string2action[step_cleaned[0]]
            args = [color[0] for color in step_cleaned[2:]]
            actions.append((primative, args))
        return actions

    def execute_symbolic_plan(self, input_file, blockstate):
        with open(input_file, 'r') as file:
            plan = file.read()
        print(plan)
        actions = self.parse_symbolic_plan(plan)
        for i in range(len(actions)):
            action, args = actions[i]
            
            action(*args)



    if __name__ == "__main__":
        test_plan = """(pick-up r magenta)
        (stack r magenta cyan)
        (pick-up r yellow)
        (stack r yellow magenta)
        (pick-up r green)
        (stack r green blue)
        (pick-up r red)
        (stack r red green)"""
        
        actions = parse_symbolic_plan(test_plan)
        for action in actions:
            print(action)















    