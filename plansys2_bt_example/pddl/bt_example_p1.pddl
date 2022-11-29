(define (problem factory_1)
  (:domain factory)
  (:objects
    r2d2 - robot
    wheels_zone steering_wheels_zone body_car_zone assembly_zone - zone
    wheel_1 body_car_1 steering_wheel_1 - piece
    wheel_2 body_car_2 steering_wheel_2 - piece
    wheel_3 body_car_3 steering_wheel_3 - piece
    car_1 car_2 car_3 - car
  )
  (:init
    (robot_at r2d2 assembly_zone)
    (is_assembly_zone assembly_zone)

    (piece_at_zone wheel_1 wheels_zone)
    (piece_at_zone body_car_1 body_car_zone)
    (piece_at_zone steering_wheel_1 steering_wheels_zone)

    (piece_is_wheel wheel_1)
    (piece_is_body_car body_car_1)
    (piece_is_steering_wheel steering_wheel_1)
  
    (piece_not_used wheel_1)
    (piece_not_used wheel_2)
    (piece_not_used wheel_3)
    (piece_not_used body_car_1)
    (piece_not_used body_car_2)
    (piece_not_used body_car_3)
    (piece_not_used steering_wheel_1)
    (piece_not_used steering_wheel_2)
    (piece_not_used steering_wheel_3)

    (not_in_place wheel_1)
    (not_in_place wheel_2)
    (not_in_place wheel_3)
    (not_in_place body_car_1)
    (not_in_place body_car_2)
    (not_in_place body_car_3)
    (not_in_place steering_wheel_1)
    (not_in_place steering_wheel_2)
    (not_in_place steering_wheel_3)


    (gripper_available r2d2)
  )

  ;; The goal is to have both packages delivered to their destinations:
  (:goal (and
    ;; (piece_at_robot steering_wheel_1 r2d2)
    ;; (piece_at_zone steering_wheel_1 assembly_zone) 
    ;; (piece_at_zone body_car_1 assembly_zone) 
    (car_assembled car_1) 
    )
  )
  )
