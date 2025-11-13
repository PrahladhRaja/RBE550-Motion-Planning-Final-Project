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
     )

    (:action pick-up
     :parameters (?r - robot ?x - cube)
     :precondition (and (ontable ?x) (clear ?x) (handempty ?r))
     :effect (and (holding ?r ?x)
             (not (ontable ?x))
             (not (clear ?x))
             (not (handempty ?r))))
    
    (:action put-down
     :parameters (?r - robot ?x - cube)
     :precondition (holding ?r ?x)
     :effect (and (ontable ?x)
                  (clear ?x)
                  (handempty ?r)
             (not (holding ?r ?x))))

    (:action stack
     :parameters (?r - robot ?x - cube ?y - cube)
     :precondition (and (holding ?r ?x) (clear ?y))
     :effect (and (on ?x ?y)
                  (clear ?x)
                  (handempty ?r)
             (not (holding ?r ?x))
             (not (clear ?y))))
    
     (:action unstack
     :parameters (?r - robot ?x - cube ?y - cube)
     :precondition (and (on ?x ?y) (clear ?x) (handempty ?r))
     :effect (and (holding ?r ?x)
                  (clear ?y)
             (not (on ?x ?y))
             (not (clear ?x))
             (not (handempty ?r))))
             
             
)