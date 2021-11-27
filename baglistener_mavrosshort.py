#!/usr/bin/env python
# convert /tmpimu to /mavros/imu/data with frame_id=imu_link

import rospy
from std_msgs.msg import String
import rosbag
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu, PointCloud2


def callback(imudata):
   global imupub 
   imudata.header.frame_id='imu_link'
   imudata.linear_acceleration.z = -imudata.linear_acceleration.z
   imupub.publish(imudata)

def listener():
    global imupub , pt2pub

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    imupub = rospy.Publisher('/mavros/imu/data', Imu, queue_size=10)
    rospy.Subscriber("/tmpimu", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
