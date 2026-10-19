(define (problem watering)
  (:domain greenhouse)
  (:objects
    rover - robot
    corridor nursery - room
    cistern - tank
    orchid fern - plant
    hose - tool
  )
  (:init
    (robot_at rover dock)
    (connected dock corridor)
    (connected corridor dock)
    (connected corridor nursery)
    (connected nursery corridor)
    (generator_in dock)
    (cable dock corridor)
    (socket corridor nursery)
    (tank_in cistern nursery)
    (window_in nursery)
    (plant_in orchid nursery)
    (plant_in fern nursery)
    (tool_at hose dock)
    (= (tool rover) hose)
    (= (battery rover) 10)
    (= (level cistern) 0)
    (= (flow cistern) 1)
    (= (capacity cistern) 6)
    (= (temperature nursery) 20)
    ; The feeder is delivered at the dock, and the water pressure goes up
    (at 6 (tool_at feeder dock))
    (at 8 (= (flow cistern) 2))
  )
  (:goal (and
    (forall (?p - plant) (tended ?p))
    (fertilized orchid)
    (full cistern)
    (window_open nursery)
    (preference home (robot_at rover dock))
  ))
  ; Never the heat alarm, and always a reserve of battery: the executor follows them
  (:constraints (and
    (always (not (heat_alarm nursery)))
    (always (>= (battery rover) 5))
  ))
  (:metric minimize (+ (total-time) (* 10 (is-violated home))))
)
