(define (domain blocksworld)
    (:requirements :strips :typing :fluents :adl :derived-predicates)
    (:types cube tower robot)

    (:predicates 
     (ontable ?x - cube)
     (on ?x - cube ?y - cube)
     (clear ?x - cube)
     (handempty ?r - robot)
     (holding ?r - robot ?x - cube)
     (touches ?x - cube ?y - cube)
     (behind ?x - cube ?y - cube)               ; Cube x is behind of cube y
     (disjoint ?x - cube ?y - cube)
     (ontower ?x - cube ?t - tower)

     (adjacent-left ?x - cube ?y - cube)
     ;(adjacent-right ?x - cube ?y - cube)
     (right-of ?x - cube ?y - cube)            ; Cube x is to the right of cube y
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
                  (touches ?x ?y)
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
                  (not (touches ?x ?y))
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

    (:action place-right-of
        :parameters (?r - robot ?x - cube ?y - cube)
        :precondition (and (holding ?r ?x)
                           (ontable ?y)
                           (clear ?y))
        :effect (and 
                (not (holding ?r ?x))
                (not (behind ?x ?y))
                (handempty ?r)
                (ontable ?x)
                (clear ?x)
                (touches ?x ?y)
                (right-of ?x ?y))
    )

    (:action place-middle-behind
        :parameters (?r - robot ?x - cube ?y - cube ?z - cube)
        :precondition (and (holding ?r ?z)
                           (right-of ?x ?y)
                           (touches ?x ?y)
                           (ontable ?x)
                           (ontable ?y)
                           (clear ?x)
                           (clear ?y))
        :effect (and (not (holding ?r ?z))
                     (ontable ?z)
                     (clear ?z)
                     (behind ?z ?x)
                     (handempty ?r)
                     (touches ?z ?y)
                     (touches ?z ?x)
                     (not (right-of ?z ?x)))
    )
    
    

    ;(:action adjacent-right
    ;:parameters (?r - robot ?x - cube ?y - cube)
    ;:precondition (and (holding ?r ?x)
    ;                    (ontable ?y))
    ;:effect (and (adjacent-right ?x ?y)
    ;             (ontable ?x)
    ;             (clear ?x)
    ;             (handempty ?r)
    ;             (not (holding ?r ?x))))

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
            (handempty ?r)))   
)