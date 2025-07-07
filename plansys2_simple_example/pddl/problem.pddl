( define ( problem problem_1 )
( :domain simple )
( :objects
	robot1  - robot
  bedroom kitchen bathroom sitting_room corridor - room
)
( :init
	( robot_at robot1 corridor )

  ( connected sitting_room corridor )
  ( connected corridor sitting_room )
  ( connected bedroom corridor )
  ( connected corridor bedroom )
  ( connected bedroom bathroom )
  ( connected bathroom bedroom )
  ( connected sitting_room kitchen )
  ( connected kitchen sitting_room )

  ( charging_point_at corridor)
  ( battery_low robot1)
)
( :goal
	( and
		( robot_at robot1 kitchen )
	))
)
