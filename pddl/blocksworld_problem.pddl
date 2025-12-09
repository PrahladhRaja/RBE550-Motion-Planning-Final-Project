(define (problem blocksworld)
    (:domain blocksworld)
    (:objects
    r - robot
    red green blue yellow magenta cyan - cube
    tower1 tower2 - tower)

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
      (on red green)
      (on green blue)
      (ontable blue)

      (on yellow magenta)
      (on magenta cyan)
      (ontable cyan)
    ))
    )
    