(define (domain blocksworld)
    (:requirements :strips :typing)
    (:types cube tower robot)

    (:predicates 
     (ontable ?x - cube)
     (on ?x - cube ?y - cube)
     (clear ?x - cube)
     (handempty ?r - robot)
     (holding ?r - robot ?x - cube)
     (ontower ?x - cube ?t - tower)

     (adjacent-left ?x - cube ?y - cube)
     (adjacent-right ?x - cube ?y - cube)
     (adjacent-top ?x - cube ?y - cube)
     (triangle-three ?x - cube ?top - cube ?mid - cube)
     )

    (:action pick-up
     :parameters (?r - robot ?x - cube)
     :precondition (and (ontable ?x) 
                        (clear ?x) 
                        (handempty ?r))
     :effect (and (holding ?r ?x)
             (not (ontable ?x))
             (not (clear ?x))
             (not (handempty ?r))))
    
     (:action put-down
      :parameters (?r - robot ?x - cube)
      :precondition (and (holding ?r ?x))
      :effect (and (ontable ?x)
                   (clear ?x)
                   (handempty ?r)
              (not (holding ?r ?x))))

    (:action stack
     :parameters (?r - robot ?x - cube ?y - cube)
     :precondition (and (holding ?r ?x) 
                        (clear ?y))
     :effect (and (on ?x ?y)
                  (clear ?x)
                  (handempty ?r)
             (not (holding ?r ?x))
             (not (clear ?y))))
    
     (:action unstack
     :parameters (?r - robot ?x - cube ?y - cube)
     :precondition (and (on ?x ?y) 
                        (clear ?x) 
                        (handempty ?r))
     :effect (and (holding ?r ?x)
                  (not (handempty ?r))
                  (not (clear ?x))
                  (clear ?y)
             (not (on ?x ?y))))     

    (:action adjacent-left
    :parameters (?r - robot ?x - cube ?y - cube)
    :precondition (and  (holding ?r ?x) 
                        (ontable ?y))
    :effect (and (adjacent-left ?x ?y)
                 (ontable ?x)
                 (clear ?x)
                 (handempty ?r)
                 (not (holding ?r ?x))))

    (:action adjacent-right
    :parameters (?r - robot ?x - cube ?y - cube)
    :precondition (and (holding ?r ?x)
                        (ontable ?y))
    :effect (and (adjacent-right ?x ?y)
                 (ontable ?x)
                 (clear ?x)
                 (handempty ?r)
                 (not (holding ?r ?x))))

    (:action adjacent-top
    :parameters (?r - robot ?x - cube ?y - cube)
    :precondition (and (holding ?r ?x) (ontable ?y))
    :effect (and (adjacent-top ?x ?y)
                 (ontable ?x)
                 (clear ?x)
                 (handempty ?r)
                 (not (holding ?r ?x))))

    (:action triangle-three
    :parameters (?r - robot ?x - cube ?top -  cube ?mid - cube)
    :precondition (and (handempty ?r)(ontable ?x)
                  (clear ?x)
                  (clear ?top)
                  (clear ?mid)
                  (adjacent-top ?x ?top))
    :effect (and (not (ontable ?x))
            (on ?mid ?x)
            (clear ?x)
            (handempty ?r)
  )
)        
             
)