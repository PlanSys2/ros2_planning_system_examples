# What one side knows of the battle, from the arbiter
string side
string base

# The map, known to everyone
string[] zones
string[] connection_from
string[] connection_to
float64[] connection_distance
string[] control_points

RobotState[] robots    # its own
RobotState[] enemies   # those it has seen
string[] explored
string[] owned
string[] enemy_owned

string winner          # the side that has won; empty while the battle goes on
