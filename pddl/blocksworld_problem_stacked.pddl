(define (problem blocksworld)
    (:domain blocksworld)
    (:objects
    r - robot
    red green blue yellow magenta cyan - cube
    tower1 tower2 - tower)

    (:init
    (ontable red)
    (on green red)
    (on blue green)
    (on yellow blue)
    (on magenta yellow)
    (on cyan magenta)
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
    
