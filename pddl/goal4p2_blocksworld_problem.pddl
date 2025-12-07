(define (problem goal4p1_blocksworld)
    (:domain blocksworld)
    (:objects
    r - robot
    red blue orange - cube
    )

/    
    (:goal (and
      (clear red)
      (clear blue)
      (clear orange)

      ;;(on green2 red2)
      (on red2 magenta)
      (on magenta yellow)
      (on yellow blue)
      (on blue red)
      (on red green)

      (ontable green)
    ))
    )
    
