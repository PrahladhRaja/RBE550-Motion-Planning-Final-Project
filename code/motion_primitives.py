import sys
import numpy as np
import genesis as gs
import random as rand
import math
from typing import Any, Dict, Tuple
from planning import PlannerInterface
from scipy.spatial.transform import Rotation as R
from symbolic_predicates import lift_scene_to_predicates
from symbolic_taskplan import run_symbolic_taskplan


class MotionPrimitives:
    def __init__(self, robot, blocks_state, scene) -> None:
        self.robot = robot
        self.blocks_state = blocks_state
        self.scene = scene
        self.planInterface = PlannerInterface(robot, scene)

        self.complete_waypoint_path = []
        self.transitional_state = []   
        self.held_cube = None         

        self.Z_DISTANCE_GAP = 0.15
        self.GRIPPER_OPEN = 0.035
        self.GRIPPER_CLOSE = 0.001  
        self.LINK_NAME = "hand"

        self.tower_base_pos = []
        self.stack_count = 0

  

    def get_block_pos(self, block_color: str, BlocksState: Dict[str, Any]) -> np.ndarray:
        name_colors = {"r": "Red", "g": "Green", "b": "Blue", "y": "Yellow","m": "Magenta", "c": "Cyan","r2": "Red2", "g2": "Green2", "b2": "Blue2",
                       "r3": "Red3", "r4": "Red4", "r5": "Red5","r6": "Red6", "r7": "Red7", "r8": "Red8","r9": "Red9", "r10": "Red10"}
        that_block = BlocksState[block_color]
        block_position = that_block.get_pos()
        print(f"{name_colors[block_color]} block position (x, y, z):", block_position)
        return block_position

    def get_cube_pos(self, cube: str) -> np.ndarray:
        return self.get_block_pos(cube, self.blocks_state)

    def waypoint_plan(self, path, cube: str = None) -> bool:
        """Execute a list of waypoints, updating the attached cube pose explicitly."""
        if not path:
            print("There are no waypoints to plot or animate.")
            return False

        for waypoint in path:
            if cube is not None:
                # keep cube rigidly attached to left_finger
                offset = np.array([0.0, 0.019, -0.047])
                finger_pose = self.robot.get_link("left_finger").get_pos()
                finger_pose_np = np.asarray(finger_pose, dtype=float)
                cube_pose = finger_pose_np.copy()
                cube_pose[:3] += offset
                self.blocks_state.get(cube).set_pos(cube_pose)

            waypoint = np.asarray(waypoint, dtype=np.float32)
            self.robot.control_dofs_position(waypoint)
            self.scene.step()
        return True
    
    def move_to_safe_pre_pick(self) -> bool:
   
        ee_link = self.robot.get_link(self.LINK_NAME)

        safe_pos = np.array([0.55, 0.0, 0.55], dtype=float)
        safe_quat = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)

        safe_q = self.robot.inverse_kinematics(
            link=ee_link, pos=safe_pos, quat=safe_quat
        )
        if safe_q is None:
            print("[safe_pre_pick] IK failed for safe pose, falling back to initial qpos.")
            safe_q = np.array(
                [0.0, -0.5, -0.2, -1.0, 0.0, 1.00, 0.5, 0.02, 0.02], dtype=float
            )

        safe_q = np.asarray(safe_q, dtype=float)
        safe_q[-2:] = self.GRIPPER_OPEN

    
        current_q = np.asarray(self.robot.get_qpos(), dtype=float)

        safe_path = self.planInterface.plan_path(
            qpos_goal=safe_q,
            qpos_start=current_q,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=150,
            attached_object=None,   
            planner="RRTConnect",
        )

        if not self.waypoint_plan(safe_path):
            print("[safe_pre_pick] Planning to safe pose failed.")
            return False

        self.transitional_state = safe_q
        return True

    def pick_up(self, cube: str) -> bool:

        if not self.move_to_safe_pre_pick():
            print(f"[pick_up] Could not move to safe pre-pick pose before picking {cube}")
            return False
        
        cube_pos = self.get_cube_pos(cube)
        x_cube, y_cube, z_cube = map(float, cube_pos[:3])

        target_pos1 = np.array([x_cube, y_cube, z_cube + self.Z_DISTANCE_GAP], dtype=float)
        communal_target_quat = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)

        ee_link = self.robot.get_link(self.LINK_NAME)
        goal1 = self.robot.inverse_kinematics(
            link=ee_link, pos=target_pos1, quat=communal_target_quat
        )

        gs.logger.warning(f"The IK joint values for this goal position are:{goal1}")

        if goal1 is None:
            print(f"[pick_up]: IK calculations failed for pre-grasp over {cube}")
            return False

        goal1 = np.array(goal1, dtype=float)
        goal1[-2:] = self.GRIPPER_OPEN

        path1 = self.planInterface.plan_path(
            qpos_goal=goal1,
            qpos_start=self.transitional_state,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=200,
            attached_object=None,
            planner="RRTConnect",
        )

        if not self.waypoint_plan(path1):
            print(f"[pick_up]: planning failed for pre-grasp over {cube}")
            return False
        else:
            print(f"[pick_up]: Reached pre-grasp over block {cube}")

        # Move down to grasp
        target_pos2 = np.array([x_cube, y_cube, z_cube + 0.11], dtype=float)
        goal2 = self.robot.inverse_kinematics(
            link=ee_link, pos=target_pos2, quat=communal_target_quat
        )
        if goal2 is None:
            print(f"[pick_up]: IK failed for grasp pose over {cube}")
            return False

        goal2 = np.array(goal2, dtype=float)
        self.robot.control_dofs_position(goal2)
        for _ in range(50):
            self.scene.step()

        # Close gripper
        goal2_closed = goal2.copy()
        goal2_closed[-2:] = self.GRIPPER_CLOSE
        self.robot.control_dofs_position(goal2_closed)
        for _ in range(20):
            self.scene.step()

        # Lift up
        target_pos3 = np.array([x_cube, y_cube, z_cube + 0.30], dtype=float)
        goal3 = self.robot.inverse_kinematics(
            link=ee_link, pos=target_pos3, quat=communal_target_quat
        )

        if goal3 is None:
            print(f"[pick_up] IK failed for lift pose over {cube}")
            return False

        goal3 = np.array(goal3, dtype=float)
        goal3[-2:] = self.GRIPPER_CLOSE
        self.transitional_state = goal3

        path3 = self.planInterface.plan_path(
            qpos_goal=goal3,
            qpos_start=goal2_closed,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=75,
            attached_object=self.blocks_state.get(cube),
            planner="RRTConnect",
        )

        if not self.waypoint_plan(path3, cube):
            print(f"[pick_up] planning failed for lift move over {cube}")
            return False

        print(f"[pick_up] Finished pick-up of {cube}")
        self.held_cube = cube
        return True

    def stack(self, cube1: str, cube2: str) -> bool:
        # Alternate the wrist orientation to get a nicer tower
        self.stack_count += 1
        if self.stack_count % 2:
            target_quat1 = [1, 0, 0, 0]
            target_quat1 = (
                R.from_euler("z", 45, degrees=True) * R.from_quat(target_quat1)
            ).as_quat(scalar_first=True)
        else:
            target_quat1 = [0, 1, 0, 0]

        cube2_pos = self.get_cube_pos(cube2)
        target_x, target_y, target_z = map(float, cube2_pos[:3])

        # Record base positions (first layer) so we can snap higher cubes onto them
        if target_z < 0.05:
            self.tower_base_pos.append((target_x, target_y))

        # Snap target to nearest recorded base xy
        min_dist = math.inf
        for base in self.tower_base_pos:
            dist_to_base = math.sqrt((base[0] - target_x) ** 2 + (base[1] - target_y) ** 2)
            if dist_to_base > min_dist:
                continue
            min_dist = dist_to_base
            target_x, target_y = base

        print("[stack] snapped target xy:", target_x, target_y)

        target_cube_pos = np.array(
            [target_x, target_y, target_z + self.Z_DISTANCE_GAP + 0.03], dtype=float
        )

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
            qpos_start=self.transitional_state,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=100,
            attached_object=self.blocks_state.get(cube1),
            planner="RRTConnect",
        )

        if not self.waypoint_plan(stack_path, cube1):
            print(f"[Stack]: planning failed for motion involving initial cube grasped {cube1}")
            return False

        print(f"[Stacking]: Reached pre-positioning spot on top of block {cube2}")

        # Refine: move closer to the actual top
        cube_pos = np.array(
            [target_x + 0.001, target_y, target_z + self.Z_DISTANCE_GAP], dtype=float
        )
        q_refined = self.robot.inverse_kinematics(
            link=ee_link, pos=cube_pos, quat=target_quat1
        )
        refined_goal = np.array(q_refined, dtype=float)
        refined_goal[-2:] = self.GRIPPER_CLOSE

        self.robot.control_dofs_position(refined_goal)
        for _ in range(50):
            self.scene.step()

        # Open gripper
        letgo_goal = refined_goal.copy()
        letgo_goal[-2:] = self.GRIPPER_OPEN
        self.robot.control_dofs_position(letgo_goal)

        print(f"[Letting Gooo]: Successfully placed block on top of block {cube2}")

        # Move away
        holding_cube1_pos = self.get_cube_pos(cube1)
        move_x, move_y, move_z = map(float, holding_cube1_pos[:3])

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
            qpos_start=letgo_goal,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=50,
            attached_object=self.blocks_state.get(cube1),
            planner="RRTConnect",
        )

        if not self.waypoint_plan(move_away_path):
            print(f"[Moving Away]: planning failed for motion involving moving away from {cube1}")
            return False

        print(f"[Moving Away]: Successfully distanced ourselves from block {cube2}")
        self.transitional_state = move_away_goal
        self.held_cube = None
        return True

    def get_clear_spot(self, place_range) -> np.ndarray:
        clear_buffer = 0.06
        clear_spot_found = False

        place_x = -100.0
        place_y = -100.0
        while not clear_spot_found:
            place_x = rand.uniform(place_range[0][0], place_range[0][1])
            place_y = rand.uniform(place_range[1][0], place_range[1][1])

            for cube in self.blocks_state.keys():
                cube_x, cube_y, _ = self.get_cube_pos(cube)
                if math.sqrt((place_x - cube_x) ** 2 + (place_y - cube_y) ** 2) <= clear_buffer:
                    break
            else:
                clear_spot_found = True

        goal_pos = np.array([place_x, place_y, self.Z_DISTANCE_GAP + 0.02], dtype=float)
        return goal_pos

    def unstack(self, cube1: str, cube2: str) -> bool:
        # pick up the top cube
        if not self.pick_up(cube1):
            return False

        _ = self.get_cube_pos(cube2)  # only used to force printing / sanity check

        place_range = [[0.4, 0.7], [0.1, 0.2]]
        goal_pos = self.get_clear_spot(place_range)

        if goal_pos[0] < -10:
            print("[unstack] failed to find clear spot to place down cube")
            return False

        ee_link = self.robot.get_link(self.LINK_NAME)
        quat = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)

        place_goal = self.robot.inverse_kinematics(
            link=ee_link, pos=goal_pos, quat=quat
        )
        if place_goal is None:
            print(f"[unstack] IK failed for pre-place of cube {cube1}")
            return False

        place_goal = np.array(place_goal, dtype=float)
        place_goal[-2:] = self.GRIPPER_CLOSE

        pre_path = self.planInterface.plan_path(
            qpos_goal=place_goal,
            qpos_start=None,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=300,
            attached_object=self.blocks_state.get(cube1),
            planner="RRTStar",
        )

        if not self.waypoint_plan(pre_path, cube1):
            print(f"[unstack]: planning failed for placing down {cube1}")
            return False

        print("[unstack]: Reached clear position for place down")

        release_goal = place_goal.copy()
        release_goal[-2:] = self.GRIPPER_OPEN
        self.robot.control_dofs_position(release_goal)
        for _ in range(20):
            self.scene.step()

        retreat_goal_pos = goal_pos.copy()
        retreat_goal_pos[2] += 0.15

        retreat_goal = self.robot.inverse_kinematics(
            link=ee_link, pos=retreat_goal_pos, quat=quat
        )
        self.robot.control_dofs_position(retreat_goal)
        for _ in range(20):
            self.scene.step()

        print(f"[unstack] Successfully placed cube {cube1}")
        self.transitional_state = retreat_goal
        self.held_cube = None
        return True

    def _push_away_from_base(self, x, y, min_radius=0.30, margin=0.05):
      
        base_pos = np.asarray(self.robot.get_pos(), dtype=float)  
        bx, by = float(base_pos[0]), float(base_pos[1])

        dx = x - bx
        dy = y - by
        dist = math.sqrt(dx*dx + dy*dy)
        if dist >= min_radius:
            return x, y   

        push = (min_radius + margin) - dist
        x_new = x + push
        return x_new, y

    def adjacent(self, cube1: str, cube2: str, side: str) -> bool:

        cube2_pos = self.get_cube_pos(cube2)
        base_x, base_y, base_z = map(float, cube2_pos[:3])

        lateral_gap = 0.055  
        dx, dy = 0.0, 0.0
        if side == "right":
            dy = lateral_gap
        elif side == "left":
            dy = -lateral_gap
        elif side == "top":
            dx = lateral_gap
        else:
            print(f"[adjacent] Unknown side '{side}', defaulting to 'right'")
            dy = lateral_gap

        target_x = base_x + dx
        target_y = base_y + dy
        target_z = base_z  

        target_x, target_y = self._push_away_from_base(target_x, target_y)

        quat = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)
        ee_link = self.robot.get_link(self.LINK_NAME)

        SAFE_HOVER_Z = 0.45
        local_hover_z = max(target_z + self.Z_DISTANCE_GAP + 0.15, SAFE_HOVER_Z)

        place_z = target_z + 0.001

        high_hover_pos = np.array([target_x, target_y, local_hover_z], dtype=float)
        high_hover_goal = self.robot.inverse_kinematics(
            link=ee_link, pos=high_hover_pos, quat=quat
        )
        if high_hover_goal is None:
            print(f"[adjacent] IK failed for high hover over {cube2}")
            return False
        high_hover_goal = np.asarray(high_hover_goal, dtype=float)
        high_hover_goal[-2:] = self.GRIPPER_CLOSE

        start_q = getattr(self, "transitional_state", None)
        high_hover_path = self.planInterface.plan_path(
            qpos_goal=high_hover_goal,
            qpos_start=start_q,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=150,
            attached_object=self.blocks_state.get(cube1),
            planner="RRTConnect",
        )
        if not self.waypoint_plan(high_hover_path, cube1):
            print(f"[adjacent] planning failed to reach high hover with {cube1}")
            return False

        side_hover_pos = np.array([target_x, target_y, local_hover_z], dtype=float)
        side_hover_goal = self.robot.inverse_kinematics(
            link=ee_link, pos=side_hover_pos, quat=quat
        )
        if side_hover_goal is None:
            print(f"[adjacent] IK failed for side hover {side} of {cube2}")
            return False
        side_hover_goal = np.asarray(side_hover_goal, dtype=float)
        side_hover_goal[-2:] = self.GRIPPER_CLOSE

        side_hover_path = self.planInterface.plan_path(
            qpos_goal=side_hover_goal,
            qpos_start=high_hover_goal,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=60,
            attached_object=self.blocks_state.get(cube1),
            planner="RRTConnect",
        )
        if not self.waypoint_plan(side_hover_path, cube1):
            print(f"[adjacent] planning failed while moving {cube1} next to {cube2}")
            return False

        print(f"[adjacent] Reached hover pose {side} of {cube2}")

        place_pos = np.array([target_x, target_y, place_z], dtype=float)
        place_goal = self.robot.inverse_kinematics(
            link=ee_link, pos=place_pos, quat=quat
        )
        if place_goal is None:
            print(f"[adjacent] IK failed for final place pose of {cube1}")
            return False
        place_goal = np.asarray(place_goal, dtype=float)
        place_goal[-2:] = self.GRIPPER_CLOSE

        N = 80
        joint_path = []
        for alpha in np.linspace(0.0, 1.0, N):
            q = (1.0 - alpha) * side_hover_goal + alpha * place_goal
            joint_path.append(q)

        if not self.waypoint_plan(joint_path, cube1):
            print(f"[adjacent] direct descent failed for {cube1}")
            return False

        release_goal = place_goal.copy()
        release_goal[-2:] = self.GRIPPER_OPEN
        self.robot.control_dofs_position(release_goal)
        for _ in range(20):
            self.scene.step()
        print(f"[adjacent] Released {cube1} {side} of {cube2}")

        retreat_pos = place_pos.copy()
        retreat_pos[2] = local_hover_z
        retreat_goal = self.robot.inverse_kinematics(
            link=ee_link, pos=retreat_pos, quat=quat
        )
        if retreat_goal is None:
            print(f"[adjacent] IK failed for retreat after placing {cube1}")
            return False
        retreat_goal = np.asarray(retreat_goal, dtype=float)
        retreat_goal[-2:] = self.GRIPPER_OPEN

    
        retreat_path = self.planInterface.plan_path(
            qpos_goal=retreat_goal,
            qpos_start=release_goal,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=80,
            attached_object=None,
            planner="RRTConnect",
        )
        if not self.waypoint_plan(retreat_path):
            print(f"[adjacent] planning failed for retreat after placing {cube1}")
            return False

        print(f"[adjacent] Successfully placed {cube1} {side} of {cube2}")
        self.held_cube = None
        self.transitional_state = retreat_goal
        return True


    def adjacent_left(self, cube1: str, cube2: str) -> bool:
        return self.adjacent(cube1, cube2, side="left")

    def adjacent_right(self, cube1: str, cube2: str) -> bool:
        return self.adjacent(cube1, cube2, side="right")

    def adjacent_top(self, cube1: str, cube2: str) -> bool:
        return self.adjacent(cube1, cube2, side="top")
    

    def triangle_struct(self, top_cube: str, left_cube: str, right_cube: str) -> bool:

        left_pos  = self.get_cube_pos(left_cube)
        right_pos = self.get_cube_pos(right_cube)

        lx, ly, lz = map(float, left_pos[:3])
        rx, ry, rz = map(float, right_pos[:3])

        base_z = 0.5 * (lz + rz)

        mid_x = 0.5 * (lx + rx)
        mid_y = 0.5 * (ly + ry)

        CUBE_SIZE = 0.04

        quat = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)
        ee_link = self.robot.get_link(self.LINK_NAME)

        hover_z = base_z + CUBE_SIZE + self.Z_DISTANCE_GAP
        hover_pos = np.array([mid_x, mid_y, hover_z], dtype=float)

        place_z = base_z + CUBE_SIZE + 0.001  
        place_pos = np.array([mid_x, mid_y, place_z], dtype=float)

        hover_q = self.robot.inverse_kinematics(
            link=ee_link, pos=hover_pos, quat=quat
        )
        if hover_q is None:
            print(f"[triangle_struct] IK failed for hover above midpoint of {left_cube},{right_cube}")
            return False

        hover_q = np.asarray(hover_q, dtype=float)
        hover_q[-2:] = self.GRIPPER_CLOSE

        if isinstance(self.transitional_state, (list, tuple, np.ndarray)) and len(self.transitional_state):
            q_start = np.asarray(self.transitional_state, dtype=float)
        else:
            q_start = np.asarray(self.robot.get_qpos(), dtype=float)

        hover_path = self.planInterface.plan_path(
            qpos_goal=hover_q,
            qpos_start=q_start,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=120,
            attached_object=self.blocks_state.get(top_cube),
            planner="RRTConnect",
        )
        if not self.waypoint_plan(hover_path, top_cube):
            print(f"[triangle_struct] planning failed to reach hover above midpoint")
            return False

        print(f"[triangle_struct] Reached hover above midpoint of {left_cube} and {right_cube}")

        place_q = self.robot.inverse_kinematics(
            link=ee_link, pos=place_pos, quat=quat
        )
        if place_q is None:
            print(f"[triangle_struct] IK failed for final place pose of {top_cube}")
            return False

        place_q = np.asarray(place_q, dtype=float)
        place_q[-2:] = self.GRIPPER_CLOSE

        N = 80
        joint_path = []
        for alpha in np.linspace(0.0, 1.0, N):
            q = (1.0 - alpha) * hover_q + alpha * place_q
            joint_path.append(q)

        if not self.waypoint_plan(joint_path, top_cube):
            print(f"[triangle_struct] vertical descent failed for {top_cube}")
            return False

        release_q = place_q.copy()
        release_q[-2:] = self.GRIPPER_OPEN
        self.robot.control_dofs_position(release_q)
        for _ in range(25):
            self.scene.step()

        print(f"[triangle_struct] Placed {top_cube} bridging {left_cube} and {right_cube}")

        retreat_pos = place_pos.copy()
        retreat_pos[2] = hover_z
        retreat_q = self.robot.inverse_kinematics(
            link=ee_link, pos=retreat_pos, quat=quat
        )
        if retreat_q is None:
            print(f"[triangle_struct] IK failed for retreat after placing {top_cube}")
            return False

        retreat_q = np.asarray(retreat_q, dtype=float)
        retreat_q[-2:] = self.GRIPPER_OPEN

        retreat_path = []
        for alpha in np.linspace(0.0, 1.0, 60):
            q = (1.0 - alpha) * release_q + alpha * retreat_q
            retreat_path.append(q)

        if not self.waypoint_plan(retreat_path):
            print(f"[triangle_struct] retreat interpolation failed")
            return False

        self.transitional_state = retreat_q
        self.held_cube = None
        print(f"[triangle_struct] Bridge structure completed.")
        return True


    def preds_to_pddl(self, preds, base_problem_file: str) -> None:
        with open(base_problem_file, "r") as file:
            contents = file.read()
            init_and_goal = contents.split("/")

        curr_state = "(:init\n"
        for pred in preds:
            curr_state += pred + "\n"
        curr_state += ")"

        full_text = init_and_goal[0] + curr_state + init_and_goal[1]
        full_problem_file = base_problem_file[:-5] + "_full.pddl"

        with open(full_problem_file, "w") as file:
            file.write(full_text)

    def parse_symbolic_plan(self, plan: str):
        string2action = {
            "pick-up": self.pick_up,
            "stack": self.stack,
            "adjacent-left": self.adjacent_left,
            "adjacent-right": self.adjacent_right,
            "adjacent-top": self.adjacent_top,
            # "unstack": self.unstack,  # keep if/when you use it
        }

        cleaned_plan = plan.replace("(", "").replace(")", "")
        steps = cleaned_plan.split("\n")
        actions = []
        for step in steps:
            if not step.strip():
                continue

            step_cleaned = step.split()
            print(step_cleaned)
            primitive = string2action[step_cleaned[0]]
            # handle names like red3 -> 'r3'
            args = [color[0] + color[-1] if color[-1].isdigit() else color[0]
                    for color in step_cleaned[2:]]
            actions.append((primitive, args))
            print("actions", actions)
        return actions

    def iterative_plan_and_execute(self, base_problem_file, blockstate):
        full_problem_file = base_problem_file[:-5] + "_full.pddl"
        solution_file = full_problem_file + ".soln"

        while True:
            preds = lift_scene_to_predicates(self.robot, self.blocks_state)
            print("------------------preds--------------")
            print(preds)
            self.preds_to_pddl(preds, base_problem_file)
            run_symbolic_taskplan(full_problem_file)

            with open(solution_file, "r") as file:
                plan = file.read()

            actions = self.parse_symbolic_plan(plan)
            if not len(actions):
                break

            action, args = actions[0]
            action(*args)


if __name__ == "__main__":
    # quick manual test harness (if you ever want to use it)
    test_plan = """(pick-up r magenta)
    (stack r magenta cyan)
    (pick-up r yellow)
    (stack r yellow magenta)
    (pick-up r green)
    (stack r green blue)
    (pick-up r red)
    (stack r red green)"""
