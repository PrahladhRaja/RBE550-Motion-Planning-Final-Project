(define (problem blocksworld)
    (:domain blocksworld)
    (:objects
    r - robot
    red green blue yellow magenta cyan - cube
    tower1 tower2 - tower)

(:init
(ontable red)
(ontable green)
(ontable blue)
(ontable yellow)
(ontable cyan)
(clear red)
(clear green)
(clear blue)
(clear yellow)
(clear magenta)
(clear cyan)
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
    
