; Battlefield between two robot armies, seen by one of them.
;
; Each army runs its own PlanSys2 with this domain. Its problem holds its own robots (mine) and
; the enemies it knows (hostile). An arbiter keeps the true state: it decides the outcome of
; each shot, removes destroyed robots and tells each army what it sees.
;
; The planner is optimistic: a shot destroys its target, and a zone nobody has explored is
; empty. When reality differs (a miss, enemies found on arrival), the problem changes and
; PlanSys2 replans.
;
; Robots: drone (flies anywhere, surveys and spots for artillery), infantry (the only one that
; captures control points), tank (armored, assaults defended zones), artillery (fires at a
; connected zone, only while a drone spots it). Drones fly over the battle: they neither block
; ground moves nor hold a zone, and nothing on the ground can hit them.
(define (domain combat)
(:requirements :strips :typing :negative-preconditions :existential-preconditions
  :fluents :durative-actions)

(:types
  zone robot - object
  drone ground - robot
  tank soft - ground
  infantry artillery - soft
)

(:predicates
  (mine ?r - robot)
  (hostile ?r - robot)
  (available ?r - robot)
  (at ?r - robot ?z - zone)
  (destroyed ?r - robot)
  (connected ?z1 ?z2 - zone)
  (explored ?z - zone)
  (spotted ?z - zone)
  (base ?z - zone)
  (control_point ?z - zone)
  (owned ?z - zone)
  (enemy_owned ?z - zone)
)

(:functions
  (distance ?z1 ?z2 - zone)
  (speed ?r - robot)
  (ammo ?r - robot)
  (battery ?d - drone)
  (value ?r - robot)      ; euros the robot costs
  (op_cost ?r - robot)    ; euros per action
  (shot_cost ?r - robot)  ; euros per shot
  (spent)                 ; euros spent, the metric
)

; Ground move to an explored zone with no known enemy.
(:durative-action move
  :parameters (?r - ground ?from ?to - zone)
  :duration (= ?duration (/ (distance ?from ?to) (speed ?r)))
  :condition (and
    (at start (mine ?r))
    (at start (available ?r))
    (at start (at ?r ?from))
    (at start (connected ?from ?to))
    (at start (explored ?to))
    (at start (not (exists (?e - ground) (and (hostile ?e) (at ?e ?to))))))
  :effect (and
    (at start (not (available ?r)))
    (at start (not (at ?r ?from)))
    (at end (at ?r ?to))
    (at end (available ?r))
    (at end (increase (spent) (op_cost ?r)))))

; Ground move to an unexplored zone: what is there is known on arrival. It risks the robot.
(:durative-action advance
  :parameters (?r - ground ?from ?to - zone)
  :duration (= ?duration (/ (distance ?from ?to) (speed ?r)))
  :condition (and
    (at start (mine ?r))
    (at start (available ?r))
    (at start (at ?r ?from))
    (at start (connected ?from ?to))
    (at start (not (explored ?to))))
  :effect (and
    (at start (not (available ?r)))
    (at start (not (at ?r ?from)))
    (at end (at ?r ?to))
    (at end (explored ?to))
    (at end (available ?r))
    (at end (increase (spent) (+ (op_cost ?r) (* 0.2 (value ?r)))))))

; A tank enters a zone with known enemies.
(:durative-action assault_tank
  :parameters (?r - tank ?from ?to - zone)
  :duration (= ?duration (/ (distance ?from ?to) (speed ?r)))
  :condition (and
    (at start (mine ?r))
    (at start (available ?r))
    (at start (at ?r ?from))
    (at start (connected ?from ?to))
    (at start (explored ?to))
    (at start (exists (?e - ground) (and (hostile ?e) (at ?e ?to)))))
  :effect (and
    (at start (not (available ?r)))
    (at start (not (at ?r ?from)))
    (at end (at ?r ?to))
    (at end (available ?r))
    (at end (increase (spent) (+ (op_cost ?r) (* 0.1 (value ?r)))))))

; Infantry enters a zone with known enemies, but never against a tank.
(:durative-action assault_infantry
  :parameters (?r - infantry ?from ?to - zone)
  :duration (= ?duration (/ (distance ?from ?to) (speed ?r)))
  :condition (and
    (at start (mine ?r))
    (at start (available ?r))
    (at start (at ?r ?from))
    (at start (connected ?from ?to))
    (at start (explored ?to))
    (at start (exists (?e - ground) (and (hostile ?e) (at ?e ?to))))
    (at start (not (exists (?t - tank) (and (hostile ?t) (at ?t ?to))))))
  :effect (and
    (at start (not (available ?r)))
    (at start (not (at ?r ?from)))
    (at end (at ?r ?to))
    (at end (available ?r))
    (at end (increase (spent) (+ (op_cost ?r) (* 0.3 (value ?r)))))))

; Drones fly to any zone, connected or not.
(:durative-action fly
  :parameters (?d - drone ?from ?to - zone)
  :duration (= ?duration 5)
  :condition (and
    (at start (mine ?d))
    (at start (available ?d))
    (at start (at ?d ?from))
    (at start (>= (battery ?d) 10)))
  :effect (and
    (at start (not (available ?d)))
    (at start (not (at ?d ?from)))
    (at start (decrease (battery ?d) 10))
    (at end (at ?d ?to))
    (at end (available ?d))
    (at end (increase (spent) (op_cost ?d)))))

; The arbiter answers with the enemies in the zone.
(:durative-action survey
  :parameters (?d - drone ?z - zone)
  :duration (= ?duration 3)
  :condition (and
    (at start (mine ?d))
    (at start (available ?d))
    (at start (>= (battery ?d) 5))
    (over all (at ?d ?z)))
  :effect (and
    (at start (not (available ?d)))
    (at start (decrease (battery ?d) 5))
    (at end (explored ?z))
    (at end (available ?d))
    (at end (increase (spent) (op_cost ?d)))))

; The drone hovers over a zone and designates targets for the artillery, within reach of the
; enemy there (10 % of its value).
(:durative-action spot
  :parameters (?d - drone ?z - zone)
  :duration (= ?duration 12)
  :condition (and
    (at start (mine ?d))
    (at start (available ?d))
    (at start (explored ?z))
    (at start (>= (battery ?d) 10))
    (over all (at ?d ?z)))
  :effect (and
    (at start (not (available ?d)))
    (at start (decrease (battery ?d) 10))
    (at start (spotted ?z))
    (at end (not (spotted ?z)))
    (at end (available ?d))
    (at end (increase (spent) (+ (op_cost ?d) (* 0.1 (value ?d)))))))

(:durative-action recharge
  :parameters (?d - drone ?z - zone)
  :duration (= ?duration 20)
  :condition (and
    (at start (mine ?d))
    (at start (available ?d))
    (at start (base ?z))
    (over all (at ?d ?z)))
  :effect (and
    (at start (not (available ?d)))
    (at end (assign (battery ?d) 100))
    (at end (available ?d))))

; Light weapons: infantry only hurts soft targets in its zone, in close combat: it risks being
; shot back (15 % of its value).
(:durative-action attack_infantry
  :parameters (?r - infantry ?t - soft ?z - zone)
  :duration (= ?duration 3)
  :condition (and
    (at start (mine ?r))
    (at start (available ?r))
    (at start (hostile ?t))
    (at start (at ?t ?z))
    (at start (>= (ammo ?r) 1))
    (over all (at ?r ?z)))
  :effect (and
    (at start (not (available ?r)))
    (at start (decrease (ammo ?r) 1))
    (at end (not (at ?t ?z)))
    (at end (destroyed ?t))
    (at end (available ?r))
    (at end (increase (spent) (+ (+ (op_cost ?r) (shot_cost ?r)) (* 0.15 (value ?r)))))))

; The tank hurts any ground target in its zone; its armor keeps the risk low (1 % of its value).
(:durative-action attack_tank
  :parameters (?r - tank ?t - ground ?z - zone)
  :duration (= ?duration 3)
  :condition (and
    (at start (mine ?r))
    (at start (available ?r))
    (at start (hostile ?t))
    (at start (at ?t ?z))
    (at start (>= (ammo ?r) 1))
    (over all (at ?r ?z)))
  :effect (and
    (at start (not (available ?r)))
    (at start (decrease (ammo ?r) 1))
    (at end (not (at ?t ?z)))
    (at end (destroyed ?t))
    (at end (available ?r))
    (at end (increase (spent) (+ (+ (op_cost ?r) (shot_cost ?r)) (* 0.01 (value ?r)))))))

; Artillery fires at a connected zone while a drone spots it: from afar, with no risk.
(:durative-action bombard
  :parameters (?r - artillery ?t - ground ?from ?to - zone)
  :duration (= ?duration 4)
  :condition (and
    (at start (mine ?r))
    (at start (available ?r))
    (at start (hostile ?t))
    (at start (at ?t ?to))
    (at start (connected ?from ?to))
    (at start (>= (ammo ?r) 1))
    (over all (at ?r ?from))
    (over all (spotted ?to)))
  :effect (and
    (at start (not (available ?r)))
    (at start (decrease (ammo ?r) 1))
    (at end (not (at ?t ?to)))
    (at end (destroyed ?t))
    (at end (available ?r))
    (at end (increase (spent) (+ (op_cost ?r) (shot_cost ?r))))))

; Only infantry captures, and only a zone with no known enemy.
(:durative-action capture
  :parameters (?r - infantry ?z - zone)
  :duration (= ?duration 5)
  :condition (and
    (at start (mine ?r))
    (at start (available ?r))
    (at start (control_point ?z))
    (at start (not (owned ?z)))
    (over all (at ?r ?z))
    (over all (not (exists (?e - ground) (and (hostile ?e) (at ?e ?z))))))
  :effect (and
    (at start (not (available ?r)))
    (at end (owned ?z))
    (at end (not (enemy_owned ?z)))
    (at end (available ?r))
    (at end (increase (spent) (op_cost ?r)))))
)
