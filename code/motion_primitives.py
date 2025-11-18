def pick_up(block):
    """
    Generates a sequence of motion primitives to pick up block1 and place it on top of block2.

    Args:
        block: The block to be picked up.
    
    """
    print("Picking up block:", block)

def stack(block1, block2):
    """
    Generates a sequence of motion primitives to stack block1 on top of block2.

    Args:
        block1: The block to be stacked.
        block2: The block on which block1 will be stacked.
    
    """
    print("Stacking block:", block1, "on top of block:", block2)

def unstack(block):
    """
    Generates a sequence of motion primitives to unstack block1 from block2.

    Args:
        block1: The block to be unstacked.
    
    """
    print("Unstacking block:", block)


def parse_symbolic_plan(plan):
    """
    Parses a symbolic plan into a sequence of motion primitives.

    Args:
        plan: A list of symbolic actions.
    
    Returns:
        A list of motion primitives and their arguments.
    """
    string2action = {
        'pick-up': pick_up,
        'stack': stack,
        'unstack': unstack
        }
    
    cleaned_plan = plan.replace('(', '').replace(')', '')
    steps = cleaned_plan.split('\n')
    print(steps)
    actions = []
    for step in steps:
        step_cleaned = step.split()
        primative = string2action[step_cleaned[0]]
        args = [color[0] for color in step_cleaned[2:]]
        actions.append((primative, args))
    return actions

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

