import sys
import math
import random
import numpy as np
import genesis as gs
import random as rand
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
        
        # Initiate a transitional state for smoother path planning between primitives (first one based on starting position)
        self.transitional_state = np.asarray(self.robot.get_qpos(), dtype=float)   
        self.held_cube = None
        self.tower_base_pos = []
        self.stack_count = 0 

        # Common distance between cube moving to grab and gripper.      
        self.Z_DISTANCE_GAP = 0.15
        self.TURN_90 = False
        self.COMMUNAL_QUAT = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)

        self.GRIPPER_OPEN = np.array([0.035], dtype=np.float32) #0.035
        self.GRIPPER_CLOSE = np.array([0.0167], dtype=np.float32) #0.0167 #Lower means tighter grip
        self.EE_LINK = self.robot.get_link("hand")
        self.LINK_NAME = "hand"
        self.finger1_idx = self.robot.get_joint("finger_joint1").dofs_idx_local
        self.finger2_idx = self.robot.get_joint("finger_joint2").dofs_idx_local


    def get_block_pos(self, block_color: str, BlocksState: Dict[str, Any]) -> np.array:
        #print(block_color)
        name_colors = {"r": "Red", "g": "Green", "b": "Blue", "y": "Yellow","m": "Magenta", "c": "Cyan","r2": "Red2", "g2": "Green2", "b2": "Blue2",
                       "r3": "Red3", "r4": "Red4", "r5": "Red5","r6": "Red6", "r7": "Red7", "r8": "Red8","r9": "Red9", "r10": "Red10"}
        that_block = BlocksState[block_color]
        block_position = that_block.get_pos()
        #print(f"{name_colors[block_color]} block position (x, y, z):", block_position)
        return block_position

    def get_cube_pos(self, cube: str) -> np.ndarray:
        return self.get_block_pos(cube, self.blocks_state)

    def waypoint_plan(self, path, cube: str = None) -> bool:
        """Execute a list of waypoints, updating the attached cube pose explicitly."""
        if not path:
            print("There are no waypoints to plot or animate.")
            return False

        for waypoint in path:
            if cube:
                self.robot.control_dofs_position(self.GRIPPER_CLOSE, self.finger1_idx)
                self.robot.control_dofs_position(self.GRIPPER_CLOSE, self.finger2_idx)
            else:
                self.robot.control_dofs_position(self.GRIPPER_OPEN, self.finger1_idx)
                self.robot.control_dofs_position(self.GRIPPER_OPEN, self.finger2_idx)
            waypoint = np.asarray(waypoint, dtype=np.float32)
            self.robot.control_dofs_position(waypoint)
            self.scene.step()
        return True
    
    def move_to_safe_pre_pose(self, cube: str = None, position: np.array = None) -> bool:
        #position: Any = np.array([0.0, -0.5, -0.2, -1.0, 0.0, 1.00, 0.5, 0.02, 0.02], dtype=float)
        # If we want to go to grab a cube, move to a safe pre_grasp position
        if position is None:
            cube_pos = self.get_cube_pos(cube)
            x_cube, y_cube, z_cube = map(float, cube_pos[:3])
            safe_pos = np.array([x_cube, y_cube, z_cube + self.Z_DISTANCE_GAP], dtype=float)
        
        # Incorporate poisition plz
        else: 
            #safe_pos = np.array([0.55, 0.0, 0.55], dtype=float)
            print("SAVING NEW POSITION ALSO")
            x_pose, y_pose, z_pose = position[0], position[1], position[2]
            safe_pos = np.array([x_pose, y_pose, z_pose + 0.2], dtype=float)
            print(f"Pre Place Position of cube {cube} is: {[position[0], position[1], position[2] + 0.2]}")

        if self.TURN_90:
            target_quat1 = [1, 0, 0, 0]
            target_quat1 = (R.from_euler("z", 90, degrees=True) * R.from_quat(target_quat1)).as_quat(scalar_first=True)

        # Actually calculate IK for the pre_pose and parse at once.
        safe_q = np.asarray(self.robot.inverse_kinematics(link=self.EE_LINK, 
                                                          pos=safe_pos, 
                                                          quat=self.COMMUNAL_QUAT),#target_quat1 if self.TURN_90 else self.COMMUNAL_QUAT),
                            dtype=float)
        
        # Check that IK worked, if not go to origin
        if safe_q is None:
            print("[Safe_Pre_Pose]: IK failed for safe positioning, falling back to starting pose.")
            safe_q = np.array(
                [0.0, -0.5, -0.2, -1.0, 0.0, 1.00, 0.5, 0.02, 0.02], dtype=float)

        # Check for cube to determine gripper functionality
        if self.held_cube and cube:
            safe_q[-2:] = self.GRIPPER_CLOSE
        else:
            safe_q[-2:] = self.GRIPPER_OPEN
        
        # Not needed anymore since we always store the initial position or update it at the end.
        # current_q = np.asarray(self.robot.get_qpos(), dtype=float)

        safe_path = self.planInterface.plan_path(
            qpos_goal=safe_q,
            qpos_start=self.transitional_state,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=200,
            attached_object=self.blocks_state.get(self.held_cube) if cube else None,   
            planner="RRTConnect",
        )
        
        # Animate through the path but send cube also and print status
        if not self.waypoint_plan(safe_path, self.held_cube if self.held_cube else None):
            print("[Safe_Pre_Pose]: Planning to safe position failed.")
            return False
        else:
            print("[Safe_Pre_Pose]: Planning to safe position Succeeded.")
            self.transitional_state = safe_q

        return True

    def pick_up(self, cube: str) -> bool:

        if not self.move_to_safe_pre_pose(cube):
            print(f"[pick_up]: Could not move to safe pre-pick pose to get {cube}")
            return False
        
        # Get the cubes actual position since Pregrasp was taken care of already.
        cube_pos = self.get_cube_pos(cube)
        x_cube, y_cube, z_cube = map(float, cube_pos[:3])
        target_pos1 = np.array([x_cube, y_cube, z_cube + 0.11], dtype=float)

        # Get IK values for where the block is positioned.
        goal1 = np.array(self.robot.inverse_kinematics(link=self.EE_LINK, 
                                                       pos=target_pos1, 
                                                       quat=self.COMMUNAL_QUAT),
                        dtype=float)

        gs.logger.info(f"The IK joint values for this goal position are:{goal1}")

        # Check that IK calculations were successful.
        if goal1 is None:
            print(f"[pick_up]: IK calculations failed for direct grasp of {cube}")
            return False

        # Specify gripper orientation, open right now.
        goal1[-2:] = self.GRIPPER_OPEN

        # Planning path for direct grasp of cube (moving down to pinch).
        path1 = self.planInterface.plan_path(
            qpos_goal=goal1,
            qpos_start=self.transitional_state,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=200,
            attached_object=None,
            planner="RRTConnect",
        )

        # Check that path planning was successful.
        if not self.waypoint_plan(path1):
            print(f"[pick_up]: planning failed for pinching {cube}")
            return False
        else:
            print(f"[pick_up]: Reached pinching spot of {cube}")

        # Close gripper and apply control from now on but animate, 
        #       no need for path planning right now since we're only close the fingers
        goal2_closed = goal1.copy()
        goal2_closed[-2:] = self.GRIPPER_CLOSE
        self.robot.control_dofs_position(goal2_closed)
        for _ in range(20):
            self.scene.step()

        # Lift up
        target_pos3 = np.array([x_cube, y_cube, z_cube + 0.30], dtype=float)
        goal3 = np.array(self.robot.inverse_kinematics(link=self.EE_LINK, 
                                              pos=target_pos3, 
                                              quat=self.COMMUNAL_QUAT),
                        dtype=float)
        
        # Check that IK was successfull for lifting up cube
        if goal3 is None:
            print(f"[pick_up]: Failed lift position IK calculations, holding {cube}")
            return False

        # Gripper is still in the closed position for lift goal
        goal3[-2:] = self.GRIPPER_CLOSE

        path3 = self.planInterface.plan_path(
            qpos_goal=goal3,
            qpos_start=goal2_closed,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=75,
            attached_object=self.blocks_state.get(cube),
            planner="RRTConnect",
        )

        # check that pickup was completely successfull
        if not self.waypoint_plan(path3, cube):
            print(f"[pick_up]: Planning failed for final pickup of {cube}")
            return False
        else:
            print(f"[pick_up]: Successfully picked-up {cube} cube!")
            self.held_cube = cube
            self.transitional_state = goal3

        return True

    def put_down(self, cube: str, position: np.array = None) -> bool:
        
        if position is None:
            goal_pos = self.get_clear_spot([[0.4, 0.7], [0.4, 0.6]])
        else:
            #self.move_to_safe_pre_pose(cube, position)   

            x_pose, y_pose, z_pose = position[0], position[1], position[2]
            goal_pos = np.array([x_pose, y_pose, z_pose], dtype=float)
            print(f"Place Position of cube {cube} is: {goal_pos}")

        target_quat1 = [0,1,0,0]
            
        if self.TURN_90:
           target_quat1 = [1, 0, 0, 0]
           target_quat1 = (R.from_euler("z", 90, degrees=True) * R.from_quat(target_quat1)).as_quat(scalar_first=True)
        
        place_goal = self.robot.inverse_kinematics(
            link=self.EE_LINK, pos=goal_pos, quat=target_quat1 if self.TURN_90 else self.COMMUNAL_QUAT
        )

        if place_goal is None:
            print(f"[unstack] IK failed for pre-place of cube {cube}")
            return False
        
        place_goal = np.array(place_goal, dtype=float) #target_quat1 if self.TURN_90 else self.COMMUNAL_QUAT),   

        place_goal[-2:] = self.GRIPPER_CLOSE

        pre_path = self.planInterface.plan_path(
            qpos_goal=place_goal,
            qpos_start=self.transitional_state,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=250,
            attached_object=self.blocks_state.get(self.held_cube),
            planner="RRTConnect",
        )

        if not self.waypoint_plan(pre_path, cube):
            print(f"[Put_Down]: planning failed for placing down cube {cube}")
            return False

        print("[unstack]: Reached clear position for place down")

        self.robot.control_dofs_position(place_goal)
        for _ in range(20):
            self.scene.step()

        final_place_open = place_goal.copy()
        final_place_open[-2:] = self.GRIPPER_OPEN
        self.robot.control_dofs_position(final_place_open)
        for _ in range(20):
            self.scene.step()

        self.transitional_state = final_place_open
        self.held_cube = None
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

        #print("[stack]: Snapped to target xy:", target_x, target_y)

        # Get target cube position and consider this the reachable Pre_Pose
        #target_cube_pos = np.array([target_x, target_y, target_z + self.Z_DISTANCE_GAP + 0.03], dtype=float)

        # Reach a pre-place position for destination cube
        self.move_to_safe_pre_pose(cube2)

        # Refine: move closer to the actual top of the cube to stack on
        cube_pos = np.array(
            [target_x + 0.0045, target_y + 0.0021, target_z + self.Z_DISTANCE_GAP + 0.011], dtype=float
        )
        q_refined = self.robot.inverse_kinematics(
            link=self.EE_LINK, pos=cube_pos, quat=target_quat1
        )
        refined_goal = np.array(q_refined, dtype=float)
        refined_goal[-2:] = self.GRIPPER_CLOSE

        refined_path = self.planInterface.plan_path(
            qpos_goal=refined_goal,
            qpos_start=self.transitional_state,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=150,  # 3s duration
            attached_object=self.blocks_state.get(cube1),
            planner="RRTConnect",
        )
        if not self.waypoint_plan(refined_path, cube1):
            print(f"[Perfecting approximation]: Getting better IK to top of cube failed.")
            return False
        else:
            print(f"[Perfecting approximation]: Successfully got closer to top of block {cube1}")
        
        # Open gripper and stop applying control from now on but animate, 
        #       no need for path planning right now since we're only close the fingers
        final_goal_open = refined_goal.copy()
        final_goal_open[-2:] = self.GRIPPER_OPEN
        self.robot.control_dofs_position(final_goal_open)
        for _ in range(20):
            self.scene.step()
        self.transitional_state = final_goal_open

        # Move away time
        holding_cube1_pos = self.get_cube_pos(cube1)
        move_x, move_y, move_z = map(float, holding_cube1_pos[:3])

        move_away_target = np.array(
            [move_x, move_y, move_z + self.Z_DISTANCE_GAP + 0.22], dtype=float
        )

        move_away_goal = self.robot.inverse_kinematics(
            link=self.EE_LINK, pos=move_away_target, quat=target_quat1
        )
        if move_away_goal is None:
            print(f"[Moving Away]: IK failed when moving away from {cube1}")
            return False

        move_away_goal = np.array(move_away_goal, dtype=float)

        move_away_path = self.planInterface.plan_path(
            qpos_goal=move_away_goal,
            qpos_start=self.transitional_state,
            timeout=5.0,
            smooth_path=True,
            num_waypoints=50,
            attached_object=None,
            planner="RRTConnect",
        )

        if not self.waypoint_plan(move_away_path):
            print(f"[Moving Away]: planning failed for motion involving moving away from {cube1}")
            return False

        print(f"[Moving Away]: Successfully distanced ourselves from block {cube2}")
        self.transitional_state = move_away_goal
        self.held_cube = None
        return True
    
    def spot_is_clear(self, x, y, clear_buffer = 0.1) -> bool:
        for cube in self.blocks_state.keys():
            cube_x, cube_y, _ = self.get_cube_pos(cube)
            if math.sqrt((x - cube_x) ** 2 + (y - cube_y) ** 2) <= clear_buffer:
                return False
        
        return True

    def get_clear_spot(self, place_range,max_iterations=100) -> np.ndarray:

        place_x = -100.0
        place_y = -100.0
        for _ in range(max_iterations):
            place_x = rand.uniform(place_range[0][0], place_range[0][1])
            place_y = rand.uniform(place_range[1][0], place_range[1][1])

            if self.spot_is_clear(place_x, place_y):
                return np.array([place_x, place_y, self.Z_DISTANCE_GAP], dtype=float)
            
        
        raise Exception("Error: No clear spot found to put cube down")
    
    def unstack(self, cube1:str, cube2:str) -> bool:
        # Wrapper for pick_up so passing two arguments doesn't throw an error
        if not self.pick_up(cube1):
            print("[Unstack]: Pick up of cube to unstack failed.")
            return False
        
        return True


    # def put_down(self, cube1:str) -> bool:


    #     place_range = [[0.3, 0.5], [-0.35, 0.35]]
    #     goal_pos = self.get_clear_spot(place_range)

    #     if goal_pos[0] < -10:
    #         print("[put-down] failed to find clear spot to place down cube")
    #         return False

    #     ee_link = self.robot.get_link(self.LINK_NAME)
    #     quat = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)

    #     place_goal = self.robot.inverse_kinematics(
    #         link=ee_link, pos=goal_pos, quat=quat
    #     )
    #     if place_goal is None:
    #         print(f"[put-down] IK failed for pre-place of cube {cube1}")
    #         return False

    #     place_goal = np.array(place_goal, dtype=float)
    #     place_goal[-2:] = self.GRIPPER_CLOSE

    #     pre_path = self.planInterface.plan_path(
    #         qpos_goal=place_goal,
    #         qpos_start=self.transitional_state,
    #         timeout=5.0,
    #         smooth_path=True,
    #         num_waypoints=100,
    #         attached_object=self.blocks_state.get(cube1),
    #         planner="RRTConnect",
    #     )

    #     if not self.waypoint_plan(pre_path, cube1):
    #         print(f"[put-down]: planning failed for placing down {cube1}")
    #         return False

    #     print("[put-down]: Reached clear position for place down, refining placement")

    #     self.robot.control_dofs_position(place_goal)
    #     for _ in range(20):
    #         self.scene.step()


    #     release_goal = place_goal.copy()
    #     release_goal[-2:] = self.GRIPPER_OPEN
    #     self.robot.control_dofs_position(release_goal)
    #     for _ in range(20):
    #         self.scene.step()

    #     retreat_goal_pos = goal_pos.copy()
    #     retreat_goal_pos[2] += 0.15

    #     retreat_goal = self.robot.inverse_kinematics(
    #         link=ee_link, pos=retreat_goal_pos, quat=quat
    #     )
    #     self.robot.control_dofs_position(retreat_goal)
    #     for _ in range(20):
    #         self.scene.step()

    #     print(f"[put-down] Successfully placed cube {cube1}")
    #     self.transitional_state = retreat_goal
    #     self.held_cube = None
    #     return True

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

        SAFE_HOVER_Z = 0.45
        local_hover_z = max(target_z + self.Z_DISTANCE_GAP + 0.15, SAFE_HOVER_Z)

        place_z = target_z + 0.001

        high_hover_pos = np.array([target_x, target_y, local_hover_z], dtype=float)
        high_hover_goal = self.robot.inverse_kinematics(
            link=self.EE_LINK, pos=high_hover_pos, quat=self.COMMUNAL_QUAT
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
            num_waypoints=300,
            attached_object=self.blocks_state.get(cube1),
            planner="RRTConnect",
        )
        if not self.waypoint_plan(high_hover_path, cube1):
            print(f"[adjacent] planning failed to reach high hover with {cube1}")
            return False

        side_hover_pos = np.array([target_x, target_y, local_hover_z], dtype=float)
        side_hover_goal = self.robot.inverse_kinematics(
            link=self.EE_LINK, pos=side_hover_pos, quat=self.COMMUNAL_QUAT
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
            num_waypoints=300,
            attached_object=self.blocks_state.get(cube1),
            planner="RRTConnect",
        )
        if not self.waypoint_plan(side_hover_path, cube1):
            print(f"[adjacent] planning failed while moving {cube1} next to {cube2}")
            return False

        print(f"[adjacent] Reached hover pose {side} of {cube2}")

        place_pos = np.array([target_x, target_y, place_z], dtype=float)
        place_goal = self.robot.inverse_kinematics(
            link=self.EE_LINK, pos=place_pos, quat=self.COMMUNAL_QUAT
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
            link=self.EE_LINK, pos=retreat_pos, quat=self.COMMUNAL_QUAT
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
            num_waypoints=280,
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

        hover_z = base_z + CUBE_SIZE + self.Z_DISTANCE_GAP
        hover_pos = np.array([mid_x, mid_y, hover_z], dtype=float)

        place_z = base_z + CUBE_SIZE + 0.001  
        place_pos = np.array([mid_x, mid_y, place_z], dtype=float)

        hover_q = self.robot.inverse_kinematics(
            link=self.EE_LINK, pos=hover_pos, quat=self.COMMUNAL_QUAT
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
            link=self.EE_LINK, pos=place_pos, quat=self.COMMUNAL_QUAT
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
            link=self.EE_LINK, pos=retreat_pos, quat=self.COMMUNAL_QUAT
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

    def place_right_of(self, cube1: str, cube2: str) -> bool: # place Cube1 is to the right of cube2
        self.TURN_90 = True
        #if not self.pick_up(cube1):
        #    print(f"[Right-of Pickup]: Could not pickup {cube1}")
        #    return False
        
        # Get the cube position and adjust placement to be to the right of the block
        cube_pos = self.get_cube_pos(cube2)
        x_cube, y_cube, z_cube = map(float, cube_pos[:3])
        target_pos1 = np.array([x_cube, y_cube - 0.08, z_cube + 0.13], dtype=float)
        print(f"Position of cube {cube2} is: {x_cube, y_cube, z_cube}")

        if not self.put_down(cube1, target_pos1):
            print(f"[Right-of PutDown]: Could not put down {cube1} next to {cube2}")
            self.TURN_90 = False
            return False
        else:
            print(f"[Right-of PutDown]: Successfully placed {cube1} to the right of {cube2}!")
            self.TURN_90 = False

    def place_middle_behind(self, cube1, cube2, cube3) -> bool:
        self.TURN_90 = False
        # Cube1 is to the right of Cube2 and you need to place cube 3 behind cube1 and 2 but between them both
        
        # Get Cube 1 position and just add an offset in the y and
        #        x direction to place cube3 behind them both.
        cube_pos = self.get_cube_pos(cube1)
        x_cube, y_cube, z_cube = map(float, cube_pos[:3])
        target_pos1 = np.array([x_cube - 0.045, y_cube + 0.019, z_cube+self.Z_DISTANCE_GAP], dtype=float)
        print(f"Place Position of cube {cube3} is: {target_pos1}")

        if not self.put_down(cube3, target_pos1):
            print(f"[Place-middle-behind PutDown]: Could not put down {cube3} behind any cube.")
            #self.TURN_90 = False
            return False
        else:
            print(f"[Place-middle-behind]: Successfully placed {cube3} behind cube {cube1} and cube {cube2}!")
            #self.TURN_90 = False
        
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
            "put-down": self.put_down,
            "stack": self.stack,
            "unstack": self.unstack,
            "adjacent-left": self.adjacent_left,
            "adjacent-right": self.adjacent_right,
            "adjacent-top": self.adjacent_top,
            "put-down": self.put_down,  # keep if/when you use it
            "place-right-of": self.place_right_of,
            "place-middle-behind": self.place_middle_behind,
        }

        cleaned_plan = plan.replace("(", "").replace(")", "")
        steps = cleaned_plan.split("\n")
        actions = []
        for step in steps:
            if not step.strip():
                continue

            step_cleaned = step.split()
            print(step_cleaned)
            '''
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
            '''
            primative = string2action[step_cleaned[0]]
            args = []
            #args = [color[0] for color in step_cleaned[2:]]
            name_colors = {"r": "red", "g": "green", "b": "blue", "y": "yellow","m": "magenta", "c": "cyan", "o": "orange", "r2": "red2", "g2": "green2", "b2": "blue2",
                       "r3": "red3", "r4": "red4", "r5": "red5","r6": "red6", "r7": "red7", "r8": "red8","r9": "red9", "r10": "red10"}
            
            for color in step_cleaned[2:]:
                for key, full_color_name in name_colors.items():
                    if (full_color_name == color):
                        #print(color[0])
                        args.append(color[0])
            #print(args)
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
    # quick manual test harness (if you ever want to use it)
    test_plan = """(pick-up r magenta)
    (stack r magenta cyan)
    (pick-up r yellow)
    (stack r yellow magenta)
    (pick-up r green)
    (stack r green blue)
    (pick-up r red)
    (stack r red green)"""
