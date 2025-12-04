(define (problem goal2_blocksworld_tower)
    (:domain blocksworld)
    (:objects
    r - robot
    red green blue yellow magenta cyan - cube
    tower1 - tower)

    (:init
    (ontable red)
    (on green red)
    (on blue green)
    (on yellow green)
    (on magenta yellow)
    (on cyan magenta)
    (clear cyan)
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
    
