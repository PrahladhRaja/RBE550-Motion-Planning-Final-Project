(define (problem goal3_blocksworld_tower)
    (:domain blocksworld)
    (:objects
    r - robot
    red green blue yellow magenta cyan red2 green2 blue2 - cube
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
      (clear cyan)

      (on cyan magenta)
      (on magenta yellow)
      (on yellow blue)
      (on blue red)
      (on red green)

      (ontable green)
    ))
    )
    
