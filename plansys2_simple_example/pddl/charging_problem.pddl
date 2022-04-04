(define (problem charging-problem)
  (:domain charging)

  ;; Instantiate the objects.
  (:objects
    leia - robot
    entrance kitchen bedroom dining bathroom chargingroom - waypoint
  )

  (:init
    ; Define the initial state predicates.
    (connected entrance dining)
    (connected dining entrance)

    (connected dining kitchen)
    (connected kitchen dining)

    (connected dining bedroom)
    (connected bedroom dining)

    (connected bathroom bedroom)
    (connected bedroom bathroom)

    (connected chargingroom kitchen)
    (connected kitchen chargingroom)

    (charger_at chargingroom)
    (robot_at leia kitchen)

    ; Define the initial functions.
    (= (speed leia) 5)
    (= (max_range leia) 10)
    (= (state_of_charge leia) 20)

    (= (distance entrance dining) 2)
    (= (distance dining entrance) 2)
    (= (distance dining kitchen) 2)
    (= (distance kitchen dining) 2)
    (= (distance kitchen chargingroom) 1)
    (= (distance chargingroom kitchen) 1)
    (= (distance dining bedroom) 2)
    (= (distance bedroom dining) 2)
    (= (distance bedroom bathroom) 2)
    (= (distance bathroom bedroom) 2)
  )

  (:goal (and
    (patrolled bathroom)
    (patrolled kitchen)
    (patrolled entrance)
  ))

  (:metric
    minimize (total-time)
  )
)