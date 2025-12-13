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
      (right-of blue orange)

      (behind red blue)

      (touches red blue)
      (touches red orange)
      
      (ontable red)
      (ontable blue)
      (ontable orange)

      (clear red)
      (clear blue)
      (clear orange)

      (handempty r)
    )
  )
)
    
