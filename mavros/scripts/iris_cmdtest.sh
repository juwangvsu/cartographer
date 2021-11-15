for i in {1..25}
do
   #rosrun mavros mavcmd camera_trigger 0  0 5 #not working
   #rosrun mavros mavcmd trigger_control
   #rosservice call /mavros/cmd/arming '{value: true}'
   rosrun mavros mavcmd int 31010 1050 1500 $i $i 0 0 0
   # send a specifi command, command id: 300, 4 5 4 5 param 1-4, 
   echo "send mavlink msg the $ith times, throttle 1050, roll 1600, cmd type: MAV_CMD_USER_1 (31010)"
   echo "throttle thresh : reverse<1200, forward > 1700, roll threshold ]1300 1700["
done
#rosservice call /mavros/cmd/arming '{value: true}'
#rosrun mavros mavcmd landcur 0  0 
