# rewrite the bag file, change frame_id, preserve the time stamp
import rosbag
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu, PointCloud2
import rospy
bag = rosbag.Bag('b3_rerecord.bag')
rospy.init_node('playbag', anonymous=True)
pt2pub = rospy.Publisher('/camera/depth/points2', PointCloud2, queue_size=10)
rate = rospy.Rate(1) # 1 Hz
for topic, msg, t in bag.read_messages(topics=['/imu', '/horizontal_laser_3d']):
    #print(msg.header)
    #print(msg._type)
    if msg._type == 'sensor_msgs/PointCloud2':
        #print('get pointcloud2 msg')
        print(msg.header)
       # msg.header.frame_id='camera_depth_optical_frame'
        pt2pub.publish(msg)
    rate.sleep();
    
bag.close()
