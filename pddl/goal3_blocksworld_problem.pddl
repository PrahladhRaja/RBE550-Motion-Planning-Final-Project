(define (problem goal3_blocksworld_tower)
    (:domain blocksworld)
    (:objects
    r - robot
    red green blue yellow magenta cyan red2 green2 blue2 - cube
    tower1 - tower)

/    
    (:goal (and
      (clear red2)
      ;; (clear green2)

      ;;(on green2 red2)
      (on red2 magenta)
      (on magenta yellow)
      (on yellow blue)
      (on blue red)
      (on red green)

      (ontable green)
    ))
    )
    
