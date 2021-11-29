# rewrite the bag file, change frame_id, preserve the time stamp
import rosbag
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu, PointCloud2
import rospy
rospy.init_node('playbag', anonymous=True)
imupub = rospy.Publisher('/imu', Imu, queue_size=10)
rate = rospy.Rate(1) # 1 Hz
cnt=0
msg = Imu()
msgz = Imu()
msgz.linear_acceleration.x=0
msgz.linear_acceleration.y=0
msgz.linear_acceleration.z= 9.8 
msgz.header.frame_id = 'imu_link'
msgx = Imu()
msgx.linear_acceleration.x= 2 
msgx.linear_acceleration.y=0
msgx.linear_acceleration.z= 0
msgx.header.frame_id = 'imu_link'
msgy = Imu()
msgy.linear_acceleration.x=0
msgy.linear_acceleration.y= 4
msgy.linear_acceleration.z= 0
msgy.header.frame_id = 'imu_link'
while not rospy.is_shutdown():
    print(msg.header.frame_id)
    print(msg.angular_velocity)
    print(msg.linear_acceleration)
    if cnt%3 == 0:
        msg = msgx
    elif cnt%3 ==1:
        msg = msgy
    else:
        msg = msgz

    imupub.publish(msg)
    rate.sleep();
    cnt = cnt +1
    
