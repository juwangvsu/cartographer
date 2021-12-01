#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosbag
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu, PointCloud2
from visualization_msgs.msg import MarkerArray
import json
import yaml
import csv

def callback(trajmsg):
    global trajsub

    markerlen = len(trajmsg.markers)
    pointslen = len(trajmsg.markers[markerlen-1].points)
    print (len(trajmsg.markers[2].points))
    print (trajmsg.markers[2].points)
    
    #dataj=msg2json(trajmsg)
    #dataj=msg2json(trajmsg.markers[markerlen-1])
    with open('traj_turtlebot3d_carto.csv', 'w') as outfile:
        fieldnames = ['t', 'x', 'y', 'z','#trajectory from carto at 15 secs']
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow({'t': trajmsg.markers[markerlen-1].header.stamp.secs})
        for i in range(pointslen):
            writer.writerow({'x': trajmsg.markers[markerlen-1].points[i].x, 'y': trajmsg.markers[markerlen-1].points[i].y, 'z': trajmsg.markers[markerlen-1].points[i].z})
    trajsub.unregister()

def msg2json(msg):
   ''' Convert a ROS message to JSON format'''
   y = yaml.load(str(msg))
   return json.dumps(y,indent=4)

if __name__ == "__main__":
    global trajsub
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('listener', anonymous=True)
    trajsub = rospy.Subscriber("/trajectory_node_list", MarkerArray, callback)
    rospy.spin()

