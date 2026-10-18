(define (domain greenhouse)
  (:requirements :strips :typing :durative-actions :numeric-fluents :continuous-effects
    :duration-inequalities :negative-preconditions :time)
  (:types
    robot room tank
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?from ?to - room)
    (has_charger ?l - room)
    (tank_in ?t - tank ?l - room)
    (valve_open ?t - tank)
    (full ?t - tank)
    (window_in ?l - room)
    (window_open ?l - room)
    (heat_alarm ?l - room)
  )
  (:functions
    (battery ?r - robot)
    (level ?t - tank)
    (flow ?t - tank)
    (capacity ?t - tank)
    (temperature ?l - room)
  )
  (:durative-action move
    :parameters (?r - robot ?from ?to - room)
    :duration (= ?duration 4)
    :condition (and
      (at start (robot_at ?r ?from))
      (at start (>= (battery ?r) 20))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (decrease (battery ?r) (* #t 2))
    )
  )
  (:durative-action recharge
    :parameters (?r - robot ?l - room)
    :duration (and (>= ?duration 1) (<= ?duration 10))
    :condition (and
      (over all (robot_at ?r ?l))
      (over all (has_charger ?l))
    )
    :effect (and
      (increase (battery ?r) (* #t 5))
    )
  )
  (:action open_valve
    :parameters (?r - robot ?t - tank ?l - room)
    :precondition (and (robot_at ?r ?l) (tank_in ?t ?l) (not (valve_open ?t)))
    :effect (and (valve_open ?t))
  )
  (:action open_window
    :parameters (?r - robot ?l - room)
    :precondition (and (robot_at ?r ?l) (window_in ?l) (not (window_open ?l)))
    :effect (and (window_open ?l))
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
