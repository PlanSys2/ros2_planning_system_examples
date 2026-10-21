; Assembly with arms: a variant of bt_example to analyze parallel execution.
;
; Each trip is split in a long move and a short approach, and each pick and release has an arm
; preparation. The preparation needs (heading ?r ?z), which the START of approach makes true,
; and it stays true until the robot leaves: the plan prepares the arm during the approach.
; STNBTBuilder starts it as soon as the approach starts; SimpleBTBuilder, that only waits for
; actions to end, starts it after the approach. Both complete the plan; STN takes less time.
;
; Each robot handles one kind of piece (assigned), which keeps the problem easy for the planner.
(define (domain factory_arm)
(:requirements :strips :typing :adl :durative-actions)
(:types robot zone piece car)
(:predicates
  (robot_available ?r - robot)
  (robot_at ?r - robot ?z - zone)
  (near ?r - robot ?z - zone)
  (heading ?r - robot ?z - zone)
  (arm_free ?r - robot)
  (arm_ready ?r - robot ?p - piece)
  (holding ?r - robot ?p - piece)
  (release_ready ?r - robot ?p - piece)
  (piece_at ?p - piece ?z - zone)
  (piece_is_wheel ?p - piece)
  (piece_is_body_car ?p - piece)
  (piece_is_steering_wheel ?p - piece)
  (piece_not_used ?p - piece)
  (is_assembly_zone ?z - zone)
  (assigned ?r - robot ?z - zone)
  (car_assembled ?c - car))

(:durative-action move
  :parameters (?r - robot ?z1 ?z2 - zone)
  :duration (= ?duration 10)
  :condition (and (at start (robot_at ?r ?z1)) (at start (robot_available ?r)))
  :effect (and (at start (not (robot_at ?r ?z1))) (at start (not (robot_available ?r)))
               (at start (not (heading ?r ?z1))) (at end (near ?r ?z2))))

(:durative-action approach
  :parameters (?r - robot ?z - zone)
  :duration (= ?duration 3)
  :condition (and (at start (near ?r ?z)))
  :effect (and (at start (not (near ?r ?z))) (at start (heading ?r ?z))
               (at end (robot_at ?r ?z)) (at end (robot_available ?r))))

(:durative-action prepare_pick
  :parameters (?r - robot ?p - piece ?z - zone)
  :duration (= ?duration 3)
  :condition (and (at start (heading ?r ?z)) (at start (assigned ?r ?z)) (at start (piece_at ?p ?z))
                  (at start (arm_free ?r)))
  :effect (and (at start (not (arm_free ?r))) (at end (arm_ready ?r ?p))))

(:durative-action pick
  :parameters (?r - robot ?p - piece ?z - zone)
  :duration (= ?duration 2)
  :condition (and (at start (robot_at ?r ?z)) (at start (arm_ready ?r ?p))
                  (at start (piece_at ?p ?z)) (at start (robot_available ?r)))
  :effect (and (at start (not (robot_available ?r))) (at start (not (arm_ready ?r ?p)))
               (at start (not (piece_at ?p ?z)))
               (at end (holding ?r ?p)) (at end (robot_available ?r))))

(:durative-action prepare_release
  :parameters (?r - robot ?p - piece ?z - zone)
  :duration (= ?duration 3)
  :condition (and (at start (heading ?r ?z)) (at start (holding ?r ?p))
                  (at start (is_assembly_zone ?z)))
  :effect (and (at end (release_ready ?r ?p))))

(:durative-action release
  :parameters (?r - robot ?p - piece ?z - zone)
  :duration (= ?duration 2)
  :condition (and (at start (robot_at ?r ?z)) (at start (release_ready ?r ?p))
                  (at start (robot_available ?r)))
  :effect (and (at start (not (robot_available ?r))) (at start (not (release_ready ?r ?p)))
               (at start (not (holding ?r ?p)))
               (at end (piece_at ?p ?z)) (at end (arm_free ?r)) (at end (robot_available ?r))))

(:durative-action assemble
  :parameters (?r - robot ?z - zone ?p1 ?p2 ?p3 - piece ?c - car)
  :duration (= ?duration 5)
  :condition (and (at start (is_assembly_zone ?z)) (at start (robot_at ?r ?z))
                  (at start (robot_available ?r))
                  (at start (piece_at ?p1 ?z)) (at start (piece_at ?p2 ?z)) (at start (piece_at ?p3 ?z))
                  (at start (piece_not_used ?p1)) (at start (piece_not_used ?p2))
                  (at start (piece_not_used ?p3))
                  (at start (piece_is_wheel ?p1)) (at start (piece_is_body_car ?p2))
                  (at start (piece_is_steering_wheel ?p3)))
  :effect (and (at start (not (robot_available ?r)))
               (at start (not (piece_not_used ?p1))) (at start (not (piece_not_used ?p2)))
               (at start (not (piece_not_used ?p3)))
               (at end (car_assembled ?c)) (at end (robot_available ?r))))
)
