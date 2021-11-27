# rewrite the bag file, change frame_id, preserve the time stamp
import rosbag
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu, PointCloud2
import rospy
bag2 = rosbag.Bag('test.bag', 'w')
bag = rosbag.Bag('b3_rerecord.bag')
start_time = bag._chunks[0].start_time #rospy.Time
end_time = bag._chunks[-1].end_time #rospy.Time
print('start_time:', start_time, 'end_time:', end_time)
for topic, msg, t in bag.read_messages(topics=['/imu', '/horizontal_laser_3d']):
    #print(msg.header)
    #print(msg._type)
    if msg._type == 'sensor_msgs/Imu':
        #print('get imu msg')
        msg.header.frame_id='base_link'
        bag2.write('/mavros/imu/data', msg)
    if msg._type == 'sensor_msgs/PointCloud2':
        #print('get pointcloud2 msg')
        msg.header.frame_id='camera_depth_optical_frame'
        bag2.write('/camera/depth/points2', msg)
#bag2._chunks[0].start_time = start_time #rospy.Time
#bag2._chunks[-1].end_time = end_time
print('chunk # for the two bags"', len(bag._chunks), len(bag2._chunks))
for i in range(len(bag._chunks)-1):
    #print(i)
    bag2._chunks[i].start_time = bag._chunks[i].start_time
    bag2._chunks[i].end_time = bag._chunks[i].end_time

metadata_msg = String(data='my metadata')
bag2.write('/metadata', metadata_msg, rospy.Time(end_time.secs))
bag2.write('/metadata', metadata_msg, rospy.Time(end_time.secs))
print('curr chunk end_time:', bag2._curr_chunk_info.end_time)
bag2.flush()
bag2._curr_chunk_info.end_time=rospy.Time(end_time.secs)
print('curr chunk end_time:', bag2._curr_chunk_info.end_time)
bag2.close()
print('curr chunk end_time:', bag2._curr_chunk_info.end_time)
