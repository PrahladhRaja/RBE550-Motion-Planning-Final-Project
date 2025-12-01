(define (problem goal4p1_blocksworld)
  (:domain blocksworld)

  (:objects
    r - robot
    red red2 red3 red4 red5 red6 red7 red8 red9 red10 - cube
  )

  (:init
    ;; initial config – you should match your scene
    (ontable red)
    (ontable red2)
    (ontable red3)
    (ontable red4)
    (ontable red5)
    (ontable red6)
    (ontable red7)
    (ontable red8)
    (ontable red9)
    (ontable red10)
    (clear red)
    (clear red2)
    (clear red3)
    (clear red4)
    (clear red5)
    (clear red6)
    (clear red7)
    (clear red8)
    (clear red9)
    (clear red10)
    (handempty r)
  )

  (:goal
    (and
      (clear red10)
      (clear red7)
      (clear red9)

      (on red10 red8)
      (on red7 red2)
      (on red8 red3)
      (on red9 red5)


      (ontable red)
      (ontable red2)
      (ontable red3)
      (ontable red4)
      (ontable red5)
      (ontable red6)

      ;; adjacency pattern around the central cube
      (adjacent-top  red2   red)
      (adjacent-top  red3   red2)
      (adjacent-right red4 red2)
      (adjacent-right red5 red3)
      (adjacent-right red6 red5)

      (adjacent-top   red8 red7)
      (adjacent-right red9 red8)

      (clear red)
      (clear red4)
      (clear red6)

      (handempty r)
    )
  )
)
