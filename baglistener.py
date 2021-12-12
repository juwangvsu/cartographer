# change the frame_id of imu and pt2.
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosbag
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu, PointCloud2

# change the frame_id of imu and pt2.

def callback2(pt2data):
   global pt2pub 
   pt2data.header.frame_id='camera_depth_optical_frame'
   pt2pub.publish(pt2data)

def callback(imudata):
   global imupub 
   imudata.header.frame_id='imu_link'
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
    pt2pub = rospy.Publisher('/camera/depth/points2', PointCloud2, queue_size=10)
    rospy.Subscriber("/imu", Imu, callback)
    rospy.Subscriber("/horizontal_laser_3d", PointCloud2, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
