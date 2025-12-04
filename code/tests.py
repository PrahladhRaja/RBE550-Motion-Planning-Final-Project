import sys
import planning
import numpy as np
import genesis as gs
from typing import Any, Dict, Tuple
import gc
from symbolic_taskplan import run_symbolic_taskplan
from scenes import create_scene_6blocks, create_scene_stacked
from motion_primitives import MotionPrimitives

if len(sys.argv) > 1 and sys.argv[1] == "gpu":
    gs.init(backend=gs.gpu, logging_level='Warning', logger_verbose_time=False)
else:
    gs.init(backend=gs.cpu, logging_level='Warning', logger_verbose_time=False)

scene, builder, BlocksState = create_scene_6blocks()
print(BlocksState)

run_symbolic_taskplan()