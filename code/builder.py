import sys
import planning
import numpy as np
import genesis as gs
from typing import Any, Dict, Tuple
import gc
from symbolic_taskplan import run_symbolic_taskplan
from scenes import create_scene_6blocks, create_scene_stacked, goal3tower
from motion_primitives import MotionPrimitives


# Ensure Genesis is initialized before building scenes
if len(sys.argv) > 1 and sys.argv[1] == "gpu":
    gs.init(backend=gs.gpu, logging_level='Warning', logger_verbose_time=False)
else:
    gs.init(backend=gs.cpu, logging_level='Warning', logger_verbose_time=False)

# build the scene using the factory
# scene, builder, BlocksState = create_scene_6blocks()
# scene, builder, BlocksState = create_scene_stacked()
scene, builder, BlocksState = goal3tower()


# After scene is built and links/geoms exist update friction parameters:
right_finger = builder.get_link("right_finger")
left_finger  = builder.get_link("left_finger")

for geom in right_finger.geoms:
    geom.set_friction(2.0)

for geom in left_finger.geoms:
    geom.set_friction(2.0)

for cube in BlocksState.values():
    for geom in cube.geoms:
        geom.set_friction(2.0)


# set control gains
# Note: the following values are tuned for achieving best behavior with builder
# Typically, each new robot would have a different set of parameters.
# Sometimes high-quality URDF or XML file would also provide this and will be parsed.
builder.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
)
builder.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
)
builder.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
)



'''
# move to a fixed pre-grasp pose
qpos = builder.inverse_kinematics(
    link=builder.get_link("hand"),
    pos=np.array([0.65, 0.0, 0.15]),
    quat=np.array([0, 1, 0, 0]),
)
# Creating a planner ofr our robot to interface with
# Set the same gripper open goal position as an example.

qpos[-2:] = 0.04
planner = planning.PlannerInterface(builder, scene)
builder_path = planner.plan_path(
    qpos_goal=qpos,
    qpos_start=None,
    timeout=5.0,
    smooth_path=True,
    num_waypoints=200,  # 5s duration
    attached_object=None,
    planner="RRT"
)


red_block_pos = get_block_pos("r", BlocksState)
green_block_pos = get_block_pos("g", BlocksState)
blue_block_pos = get_block_pos("b", BlocksState)
yellow_block_pos = get_block_pos("y", BlocksState)
magenta_block_pos = get_block_pos("m", BlocksState)
cyan_block_pos = get_block_pos("c", BlocksState)
'''

solve_motion = MotionPrimitives(builder, BlocksState, scene)
solve_motion.iterative_plan_and_execute("../pddl/goal3_blocksworld_problem.pddl", BlocksState)

# if (run_symbolic_taskplan()):
#     print("Successfully solved the PDDL problem in parts.")
#     print("Moving to execute the symbolic plan accordingly...")
    
    # solve_motion = MotionPrimitives(builder, BlocksState, scene)
    # solve_motion.execute_symbolic_plan("../pddl/goal3_blocksworld_problem.pddl.soln", BlocksState)
    #builder_path = solve_motion.complete_waypoint_path
    #solve_motion.waypoint_plan()

'''
# gripper open pos
#qpos[-2:] = 0.04
#path = builder.plan_path(
#    qpos_goal=qpos,
#    num_waypoints=300,  # 2s duration
#)
# execute the planned path
#for waypoint in path:
#    builder.control_dofs_position(waypoint)
#    scene.step()

#print("The robots path as tensor values:", builder_path)

for i, waypoint in enumerate(builder_path):
    waypoint = np.asarray(waypoint, dtype=np.float32)
    print(f"Waypoint {i}:", waypoint, "Shape:", np.shape(waypoint),
          "Min:", np.min(waypoint), "Max:", np.max(waypoint))
    
#waypoint = np.asarray(waypoint, dtype=np.float32)

for waypoint in builder_path:
    waypoint = np.asarray(waypoint, dtype=np.float32)
    if np.any(waypoint < builder.q_limit[0]) or np.any(waypoint > builder.q_limit[1]):
        print("Bounds violation in executed waypoint:", waypoint)


# execute the planned path using the prescribed planner
for i, waypoint in enumerate(builder_path):
    try:
        builder.control_dofs_position(waypoint)
        scene.step()
    except Exception as e:
        print(f"Error at waypoint {i}:", e)
        break
'''

gc.collect()