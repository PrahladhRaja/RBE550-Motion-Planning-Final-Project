
import numpy as np

EPS = 0.01  # meters

def lift_scene_to_predicates(robot, blocks_state):
    print("block_state", blocks_state)
    """
    Returns (objects, facts) for BlocksWorld:
      objects: ['red','green','blue','yellow','magenta','cyan']
      facts:   ['(on red green)', '(ontable blue)', '(clear red)', '(handempty)']
    """
    # Map short keys to canonical names
    name_map = {'r':'red','g':'green','b':'blue','y':'yellow','m':'magenta','c':'cyan'}
    keys = list(blocks_state.keys())
    objs = [name_map[k] for k in keys]

    def size_of(k):
        # all cubes are 0.04 per scene setup
        return np.array([0.04, 0.04, 0.04])

    # positions of cube centers
    pos = {k: np.array(blocks_state[k].get_pos()) for k in keys}
    h = {k: size_of(k)[2] for k in keys}
    top_z  = {k: pos[k][2] + 0.5*h[k] for k in keys}
    base_z = {k: pos[k][2] - 0.5*h[k] for k in keys}
    table_z = 0.0  # plane at z=0 (the scenes place cube centers at 0.02 with height 0.04)

    def overlaps_xy(k1,k2):
        # cubes are axis-aligned; AABB overlap test in X–Y
        (x1,y1,_), (x2,y2,_) = pos[k1], pos[k2]
        s = 0.04
        return abs(x1-x2) <= s and abs(y1-y2) <= s

    facts = []

    # ON / ONTABLE
    for x in keys:
        placed_on = None
        for y in keys:
            if x == y: continue
            if overlaps_xy(x,y) and abs(base_z[x]-top_z[y]) < EPS:
                facts.append(f"(on {name_map[x]} {name_map[y]})")
                placed_on = y
                break
        if placed_on is None and abs(base_z[x]-table_z) < EPS:
            facts.append(f"(ontable {name_map[x]})")

    # CLEAR
    for y in keys:
        clear = True
        for x in keys:
            if x == y: continue
            if overlaps_xy(x,y) and abs(base_z[x]-top_z[y]) < EPS:
                clear = False; break
        if clear:
            facts.append(f"(clear {name_map[y]})")

    # hand state
    held = getattr(robot, "holding", None) or None
    if held is None:
        facts.append("(handempty r)")
    else:
        facts.append(f"(holding {name_map[held]})")

    print(facts)
    return facts
