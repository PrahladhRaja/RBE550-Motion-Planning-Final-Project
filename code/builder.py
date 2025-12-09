import sys, os
import planning
import numpy as np
import genesis as gs
from typing import Any, Dict, Tuple
import gc
from symbolic_taskplan import run_symbolic_taskplan
import scenes
from motion_primitives import MotionPrimitives

SCRIPT_PATH = os.path.realpath(__file__)
SCRIPT_DIRECTORY = os.path.dirname(SCRIPT_PATH)
SOLVED_PDDLS = os.path.join(os.path.dirname(SCRIPT_DIRECTORY), "pddl")

def display_menu() -> None:
    print("\n----------------- Goal Selection Menu ----------------")
    print("Please select one of the following scenes to solve.\n")
    print("\t1. Building Two Towers")
    print("\t2. Building a Five Block Tower")
    print("\t3. Attempt the Tallest Tower!")
    print("\t4. Build special structure #1")
    print("\t5. Build special structure #2")
    print("\t6. Exit")
    print("------------------------------------------------------\n\n")

def set_friction_in_scene(aRobot: any, aBlocksState: any) -> None:
    # Only after scene is built and links/geoms exist can we update friction parameters:
    right_finger = aRobot.get_link("right_finger")
    left_finger  = aRobot.get_link("left_finger")

    for geom in right_finger.geoms:
        geom.set_friction(2.0)

    for geom in left_finger.geoms:
        geom.set_friction(2.0)

    for cube in aBlocksState.values():
        for geom in cube.geoms:
            geom.set_friction(2.0)

def solve_goal(builder: any, BlocksState: any, scene: any, pddl_problem: str) -> None:
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

    solve_motion = MotionPrimitives(builder, BlocksState, scene)
    #set_friction_in_scene(builder, BlocksState)
    solve_this_pddl = os.path.join(SOLVED_PDDLS, pddl_problem)
    solve_motion.execute_symbolic_plan(solve_this_pddl, BlocksState)

def ensure_scenes_are_built() -> None:
    # Ensure Genesis is initialized before building scenes
    if len(sys.argv) > 1 and sys.argv[1] == "gpu":
        gs.init(backend=gs.gpu, logging_level='Warning', logger_verbose_time=False)
    else:
        gs.init(backend=gs.cpu, logging_level='Warning', logger_verbose_time=False)


def main():
    print(f"Script directory: {SCRIPT_DIRECTORY}")
    print(f"Script directory Parent: {os.path.dirname(SCRIPT_DIRECTORY)}")
    print(f"PDDL solved directory locations: {SOLVED_PDDLS}")

    display_menu()

    user_choice = input("Goal Choice: ")
    match user_choice:
        case '1':
            # build the scene using the factory
            ensure_scenes_are_built()
            scene, builder, BlocksState = scenes.create_scene_6blocks()
            solve_goal(builder, BlocksState, scene, "blocksworld_problem_full.pddl.soln")

        case '2':
            ensure_scenes_are_built()
            scene, builder, BlocksState = scenes.create_scene_6blocks()
            solve_goal(builder, BlocksState, scene, "goal2_blocksworld_problem.pddl.soln")
            
        case '3':
            ensure_scenes_are_built()
            scene, builder, BlocksState = scenes.creates_scene_extrablocks()
            solve_goal(builder, BlocksState, scene, "goal3_blocksworld_problem.pddl.soln")
        
        case '4':
            ensure_scenes_are_built()
            scene, builder, BlocksState = scenes.goal4p1()
            #scene, builder, BlocksState = scenes.creates_scene_extrablocks()
            solve_goal(builder, BlocksState, scene, "goal4p1_blocksworld_problem_full.pddl.soln")
        
        case '5':
            ensure_scenes_are_built()
            scene, builder, BlocksState = scenes.creates_scene_extrablocks()
            solve_goal(builder, BlocksState, scene, "goal4p2_blocksworld_problem.pddl.soln")

        case '6':
            print("Goodbye now!")
        case _:
            print("How did you get here? Thats not a valid choice.")

    gc.collect()

if __name__ == "__main__":
    main()
