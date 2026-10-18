(define (problem watering)
  (:domain greenhouse)
  (:objects
    rover - robot
    dock nursery - room
    cistern - tank
  )
  (:init
    (robot_at rover dock)
    (has_charger dock)
    (connected dock nursery)
    (connected nursery dock)
    (tank_in cistern nursery)
    (window_in nursery)
    (= (battery rover) 10)
    (= (level cistern) 0)
    (= (flow cistern) 2)
    (= (capacity cistern) 6)
    (= (temperature nursery) 28)
  )
  (:goal (and
    (full cistern)
    (window_open nursery)
    (not (heat_alarm nursery))
    (robot_at rover dock)
  ))
)
