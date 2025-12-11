(define (problem goal4p2_blocksworld)
  (:domain blocksworld)
  (:objects
    r - robot
    red blue orange - cube
  )

  (:init
    (ontable red)
    (ontable blue)
    (ontable orange)

    (clear red)
    (clear blue)
    (clear orange)

    (handempty r)
  )
  
  (:goal 
    (and
      (on red blue)
      (adjacent-right blue orange)
      (ontable blue)
      (ontable orange)
      (clear red)
      (handempty r)
    )
  )
)
    
