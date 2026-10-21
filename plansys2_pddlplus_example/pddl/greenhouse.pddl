(define (domain greenhouse)
  (:requirements :strips :typing :negative-preconditions :equality :existential-preconditions
    :universal-preconditions :conditional-effects :derived-predicates :durative-actions
    :duration-inequalities :numeric-fluents :object-fluents :continuous-effects :time
    :timed-initial-literals :constraints :preferences)
  (:types
    robot room tank plant tool
  )
  (:constants
    dock - room
    feeder - tool
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?from ?to - room)
    (generator_in ?l - room)
    (cable ?from ?to - room)
    (socket ?from ?to - room)
    (powered ?l - room)
    (tank_in ?t - tank ?l - room)
    (valve_open ?t - tank)
    (full ?t - tank)
    (window_in ?l - room)
    (window_open ?l - room)
    (heat_alarm ?l - room)
    (plant_in ?p - plant ?l - room)
    (tended ?p - plant)
    (fertilized ?p - plant)
    (tool_at ?t - tool ?l - room)
  )
  (:functions
    (battery ?r - robot)
    (level ?t - tank)
    (flow ?t - tank)
    (capacity ?t - tank)
    (temperature ?l - room) - number
    (tool ?r - robot) - tool
  )
  ; Power comes from the generator, room after room, along the cables
  (:derived (powered ?l - room)
    (generator_in ?l))
  (:derived (powered ?l - room)
    (exists (?m - room) (and (cable ?m ?l) (powered ?m))))

  (:durative-action move
    :parameters (?r - robot ?from ?to - room)
    :duration (= ?duration 3)
    :condition (and
      (at start (robot_at ?r ?from))
      (at start (>= (battery ?r) 15))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (decrease (battery ?r) (* #t 2))
    )
  )
  ; Only at the dock, for as long as the planner chooses
  (:durative-action recharge
    :parameters (?r - robot)
    :duration (and (>= ?duration 1) (<= ?duration 10))
    :condition (and
      (over all (robot_at ?r dock))
    )
    :effect (and
      (increase (battery ?r) (* #t 5))
    )
  )
  (:action pick_tool
    :parameters (?r - robot ?t - tool ?l - room)
    :precondition (and (robot_at ?r ?l) (tool_at ?t ?l) (not (= (tool ?r) ?t)))
    :effect (and (assign (tool ?r) ?t) (not (tool_at ?t ?l)))
  )
  (:action plug
    :parameters (?r - robot ?from ?to - room)
    :precondition (and (robot_at ?r ?from) (socket ?from ?to) (not (cable ?from ?to)))
    :effect (and (cable ?from ?to))
  )
  (:action open_valve
    :parameters (?r - robot ?t - tank ?l - room)
    :precondition (and (robot_at ?r ?l) (tank_in ?t ?l) (powered ?l) (not (valve_open ?t))
      (not (full ?t)))
    :effect (and (valve_open ?t))
  )
  (:action open_window
    :parameters (?r - robot ?l - room)
    :precondition (and (robot_at ?r ?l) (window_in ?l) (not (window_open ?l)))
    :effect (and (window_open ?l))
  )
  ; With the feeder, the plant is fertilized too
  (:action tend
    :parameters (?r - robot ?p - plant ?l - room)
    :precondition (and (robot_at ?r ?l) (plant_in ?p ?l) (not (tended ?p)))
    :effect (and
      (tended ?p)
      (when (= (tool ?r) feeder) (fertilized ?p))
    )
  )

  (:process filling
    :parameters (?t - tank)
    :precondition (and (valve_open ?t))
    :effect (and (increase (level ?t) (* #t (flow ?t))))
  )
  (:process warming
    :parameters (?l - room)
    :precondition (and (window_in ?l) (not (window_open ?l)))
    :effect (and (increase (temperature ?l) (* #t 0.5)))
  )
  ; The float switch closes the valve of a full tank
  (:event float_switch
    :parameters (?t - tank)
    :precondition (and (valve_open ?t) (>= (level ?t) (capacity ?t)))
    :effect (and (not (valve_open ?t)) (full ?t))
  )
  (:event alarm
    :parameters (?l - room)
    :precondition (and (not (heat_alarm ?l)) (>= (temperature ?l) 35))
    :effect (and (heat_alarm ?l))
  )
)
