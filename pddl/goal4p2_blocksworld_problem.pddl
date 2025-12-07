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

      

      (ontable red)
      (ontable blue)
      (ontable orange)
    ))
    )
    
