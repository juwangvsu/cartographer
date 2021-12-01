#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosbag
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
import json
import yaml
import csv

def callback(odommsg):
    global odomsub, outfile, prevmsg_t, writer
    msg_t = odommsg.header.stamp.secs + 1.0*odommsg.header.stamp.nsecs/1000000000
    if (msg_t-prevmsg_t) < 0.2:
        return
    #dataj=msg2json(trajmsg)
    #dataj=msg2json(trajmsg.markers[markerlen-1])
    writer.writerow({'t': msg_t, 'x': odommsg.pose.pose.position.x, 'y': odommsg.pose.pose.position.y, 'z': odommsg.pose.pose.position.z})
    prevmsg_t = msg_t
    print('msg time: ', prevmsg_t)
    #odomsub.unregister()

if __name__ == "__main__":
    global odomsub, prevmsg_t, outfile, writer
    prevmsg_t=0
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('listener', anonymous=True)
    odomsub = rospy.Subscriber("/odom", Odometry, callback)
    outfile= open('odom_turtlebot3d_carto.csv', 'w')
    fieldnames = ['t', 'x', 'y', 'z','#odom gt up to 15 secs']
    writer = csv.DictWriter(outfile, fieldnames=fieldnames)
    writer.writeheader()
    rospy.spin()

