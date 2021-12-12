#!/usr/bin/env python
# convert /tmpimu to /mavros/imu/data with frame_id=imu_link
# for mavros_realsense_short.bag to mavrosshort_transcode_2.bag

import rospy
from std_msgs.msg import String
import rosbag
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu, PointCloud2


def callback_pt2(pt2msg):
   global pt2pub, msgt1_prev
   msgt1 =pt2msg.header.stamp.secs + 1.0*pt2msg.header.stamp.nsecs/1000000000
   #print('t1, t1_prev: ', msgt1, msgt1_prev)
   if msgt1 < msgt1_prev:
       #print('time glitch')
       return
   msgt1_prev=msgt1
   pt2pub.publish(pt2msg)

def callback(imumsg):
   global imupub, msgt2_prev 
   msgt2 =imumsg.header.stamp.secs + 1.0*imumsg.header.stamp.nsecs/1000000000
   if msgt2 < msgt2_prev:
       print('time glitch')
       return
   msgt2_prev=msgt2
   imumsg.header.frame_id='imu_link'
   print('imu tm:', imumsg.header.stamp.secs,imumsg.header.stamp.nsecs)
   imupub.publish(imumsg)

def listener():
    global imupub , pt2pub, msgt1_prev, msgt2_prev
    msgt1_prev=-1.0
    msgt2_prev=-1.0

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    imupub = rospy.Publisher('/mavros/imu/data', Imu, queue_size=10)
    rospy.Subscriber("/tmpimu", Imu, callback)
    pt2pub = rospy.Publisher('/camera/depth/points2',PointCloud2, queue_size=10)
    rospy.Subscriber("/tmppt2", PointCloud2, callback_pt2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
