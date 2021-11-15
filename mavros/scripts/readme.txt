 rosrun mavros mavcmd int 18 1 0 5 0 37.2272186 -77.4395296 15

to send a mav cmd message to do circle, we must send::w

18	MAV_CMD_NAV_LOITER_TURNS	Loiter around this MISSION for X turns
Mission Param #1	Turns
Mission Param #2	Empty
Mission Param #3	Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
Mission Param #4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
Mission Param #5	Latitude
Mission Param #6	Longitude
Mission Param #7	Altitude



