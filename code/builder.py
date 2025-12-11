import sys, os
import planning
import numpy as np
import genesis as gs
import subprocess
from typing import Any, Dict, Tuple
import gc
from symbolic_taskplan import run_symbolic_taskplan
import scenes
from motion_primitives import MotionPrimitives

SCRIPT_PATH = os.path.realpath(__file__)
SCRIPT_DIRECTORY = os.path.dirname(SCRIPT_PATH)
PDDLS_LOCATIONS = os.path.join(os.path.dirname(SCRIPT_DIRECTORY), "pddl")

command = "pyperplan benchmarks/tpp/domain.pddl benchmarks/tpp/task01.pddl"

def solve_pddl_problem(problem_file_name: str, domain_file_name: str = '/blocksworld_domain_123.pddl') -> str:


    if os.path.exists(PDDLS_LOCATIONS + '/' + problem_file_name + '.soln'):
        print(f"This solution has already been found and is saved as: {problem_file_name + 'soln'}")
        return os.path.join(PDDLS_LOCATIONS, problem_file_name + ".soln")
    try:
        domain = PDDLS_LOCATIONS + domain_file_name
        problem = PDDLS_LOCATIONS + '/' + problem_file_name
        cmd = ["python3","-m","pyperplan","-s","astar","-H","hff",domain,problem]
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print("Command output: Success! Specified PDDL Solved!")
        print(result.stdout)
        if result.stderr:
            print("Command error:")
            print(result.stderr)
    except subprocess.CalledProcessError as e:
        print(f"Command failed with exit code {e.returncode}:")
        print(e.stderr)

    # Return the solution file name as a full path
    print(os.path.join(PDDLS_LOCATIONS, problem_file_name + ".soln"))
    return os.path.join(PDDLS_LOCATIONS, problem_file_name + ".soln")

def scene_selection_menu() -> str:
    print("\n----------------- Scene Selection Menu ----------------")
    print("Please select one of the following scenes to start from.\n")
    print("\t1. Scene one: 6 Cubes on Table")
    print("\t2. Scene two: 6 Stacked Cubes")
    print("\t3. Scene with more cubes!")
    print("\t4. Scene with only 10 Red cubes!")
    print("\t5. Only a few.")
    print("\t6. Exit")
    print("------------------------------------------------------\n\n")

    return input("Goal Choice: ")

def goal_selection_menu() -> str:
    print("\n----------------- Goal Selection Menu ----------------")
    print("Please select one of the following goals to accomplish.\n")
    print("\t1. Building Two Towers")
    print("\t2. Building a Five Block Tower")
    print("\t3. Attempt the Tallest Tower!")
    print("\t4. Build special structure #1")
    print("\t5. Build special structure #2")
    print("\t6. Exit")
    print("------------------------------------------------------\n\n")

    return input("Goal Choice: ")

def solve_goal(builder: any, BlocksState: any, scene: any, pddl_problem_soln: str) -> None:
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
    plan_this_pddl = os.path.join(PDDLS_LOCATIONS, pddl_problem_soln)
    solve_motion.execute_symbolic_plan(plan_this_pddl, BlocksState)

def ensure_scenes_are_built(user_choice_scene: int) -> Tuple[Any, Any, Any]:
    # Ensure Genesis is initialized before building scenes
    if len(sys.argv) > 1 and sys.argv[1] == "gpu":
        gs.init(backend=gs.gpu, logging_level='Warning', logger_verbose_time=False)
    else:
        gs.init(backend=gs.cpu, logging_level='Warning', logger_verbose_time=False)

    match user_choice_scene:
        case '1':
            scene, builder, BlocksState = scenes.create_scene_6blocks()
        case '2':
            scene, builder, BlocksState = scenes.create_scene_stacked()
        case '3':
            scene, builder, BlocksState = scenes.creates_scene_extrablocks()
        case '4':
            scene, builder, BlocksState = scenes.goal4p1()
        case '5':
            scene, builder, BlocksState = scenes.creates_scene_extrablocks()
        case '6':
            print("Goodbye now!")
        case _:
            print("How did you get here? Thats not a valid choice.")
    
    return scene, builder, BlocksState


def main():
    print(f"Script directory: {SCRIPT_DIRECTORY}")
    print(f"Script directory Parent: {os.path.dirname(SCRIPT_DIRECTORY)}")
    print(f"PDDL directory locations: {PDDLS_LOCATIONS}")

    user_choice_scene = scene_selection_menu()
    user_choice_goal = goal_selection_menu()

    scene, builder, BlocksState = ensure_scenes_are_built(user_choice_scene)

    match user_choice_goal:
        case '1':
            # build the scene using the factory
            if user_choice_scene == '1':
                pddl_solution = solve_pddl_problem("blocksworld_problem.pddl")
            elif user_choice_scene == '2':
                pddl_solution = solve_pddl_problem("blocksworld_problem_stacked.pddl")
            else:
                print("This goal is only compatible with scene 1 and 2! Try again!")

        case '2':
            if user_choice_scene == '1':
                pddl_solution = solve_pddl_problem("goal2_blocksworld_problem.pddl")
            elif user_choice_scene == '2':
                pddl_solution = solve_pddl_problem("goal2_blocksworld_problem_stacked.pddl")
            else:
                print("This goal is only compatible with scene 1 and 2! Try again!")

        case '3':
            if user_choice_scene == '1' or '3':
                pddl_solution = solve_pddl_problem("goal3_blocksworld_problem.pddl.soln")
            else:
                print("This goal is only compatible with scene 1 and 3! Try again!")

        case '4':
            if user_choice_scene == '4':
                pddl_solution = solve_pddl_problem("goal4p1_blocksworld_problem_full.pddl.soln")
            else:
                print("This goal is only compatible with scene 4! Try again!")

        case '5':
            solve_goal(builder, BlocksState, scene, "goal4p2_blocksworld_problem.pddl.soln")
        case '6':
            print("Goodbye now!")
        case _:
            print("How did you get here? Thats not a valid choice.")

    solve_goal(builder, BlocksState, scene, pddl_solution)

    gc.collect()

if __name__ == "__main__":
    main()
