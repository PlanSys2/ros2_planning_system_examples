(define (domain factory)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
zone
piece
car
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(ready_to_pick ?r - robot)
(ready_to_release ?r - robot ?p - piece ?z - zone)
(gripper_available ?r - robot)



(robot_at ?r - robot ?z - zone)
(piece_at_zone ?p - piece ?z - zone)
(piece_at_robot ?p - piece ?r - robot)

(piece_is_wheel ?p - piece)
(piece_is_body_car ?p - piece)
(piece_is_steering_wheel ?p - piece)

(piece_not_used ?p - piece)

(is_assembly_zone ?z - zone)
(is_recharge_zone ?z - zone)

(car_assembled ?c - car)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - zone)
    :duration ( = ?duration 20)
    :condition (and
        (at start(robot_at ?r ?z1))
        )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
    )
)

(:durative-action pick
    :parameters (?r - robot ?p - piece ?z1 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (over all (robot_at ?r ?z1))
        (at start(piece_at_zone ?p ?z1))
        (at start(ready_to_pick ?r))
    )
    :effect (and
        (at start(not(ready_to_pick ?r)))
        (at start(not(piece_at_zone ?p ?z1)))
        (at end(piece_at_robot ?p ?r))
        (at start(not(gripper_available ?r)))
    )
)

(:durative-action prepick
    :parameters (?r - robot ?p - piece ?z1 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (over all (gripper_available ?r))
        (at end (robot_at ?r ?z1))
        (at end(piece_at_zone ?p ?z1)) )
    :effect (and
        (at end(ready_to_pick ?r))
    )
)

(:durative-action release
    :parameters (?r - robot ?p - piece ?z1 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (over all (robot_at ?r ?z1))
        (at start(piece_at_robot ?p ?r))
        (at start(ready_to_release ?r ?p ?z1))
    )
    :effect (and
        (at start(not(ready_to_release ?r ?p ?z1)))
        (at end(piece_at_zone ?p ?z1))
        (at start(not(piece_at_robot ?p ?r)))
        (at end(gripper_available ?r))
    )
)

(:durative-action prerelease
    :parameters (?r - robot ?p - piece ?z - zone)
    :duration ( = ?duration 5)
    :condition (and
      (over all (piece_at_robot ?p ?r))
      (at end (robot_at ?r ?z))
    )
    :effect (and
        (at end(ready_to_release ?r ?p ?z))
    )
)

(:durative-action assemble
    :parameters (?r - robot ?z - zone ?p1 ?p2 ?p3 - piece ?c - car)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_assembly_zone ?z))
        (over all(robot_at ?r ?z))

        (at start(piece_at_zone ?p1 ?z))
        (at start(piece_at_zone ?p2 ?z))
        (at start(piece_at_zone ?p3 ?z))

        (at start(piece_not_used ?p1))
        (at start(piece_not_used ?p2))
        (at start(piece_not_used ?p3))

        (at start(piece_is_wheel ?p1))
        (at start(piece_is_body_car ?p2))
        (at start(piece_is_steering_wheel ?p3))
    )
    :effect (and
        (at start(not(piece_not_used ?p1)))
        (at start(not(piece_not_used ?p2)))
        (at start(not(piece_not_used ?p3)))
        (at end(car_assembled ?c))
    )
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
