(define (problem goal2_blocksworld_tower)
    (:domain blocksworld)
    (:objects
    r - robot
    red green blue yellow magenta cyan - cube
    tower1 - tower)

    (:init
    (clear red)
    (clear green)
    (clear blue)
    (clear yellow)
    (clear magenta)
    (clear cyan)
    (ontable red)
    (ontable green)
    (ontable blue)
    (ontable yellow)
    (ontable magenta)
    (ontable cyan)
    (handempty r)
    )
    
    (:goal (and
      (clear magenta)

      (on magenta yellow)
      (on yellow blue)
      (on blue red)
      (on red green)

      (ontable green)
    ))
    )
    
