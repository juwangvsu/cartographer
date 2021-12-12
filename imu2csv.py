#convert /mavros/imu/data from mavrosshort..bag to csv file to examine if
#imu x-axis data make sense

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
import sys
import tf
from tf.transformations import *

fdigis=6

def callbackimu(imumsg):
    global imusub, outfile, prevmsg_t, writer, prevodom
    if prevodom is None:
        return
    q0=[prevodom.pose.pose.orientation.x, prevodom.pose.pose.orientation.y,prevodom.pose.pose.orientation.z,prevodom.pose.pose.orientation.w]
    Rm=quaternion_matrix(q0)
    imu_acc_body = [imumsg.linear_acceleration.x,imumsg.linear_acceleration.y,imumsg.linear_acceleration.z,1]
    imu_acc_odom = numpy.matmul(Rm,imu_acc_body)
    msg_t = imumsg.header.stamp.secs + 1.0*imumsg.header.stamp.nsecs/1000000000
    writer.writerow({'t': msg_t, 'type':'imu', 'x': round(prevodom.pose.pose.position.x,4), 'y': round(prevodom.pose.pose.position.y,4), \
        'z': round(prevodom.pose.pose.position.z, 4), \
        'qx':round(prevodom.pose.pose.orientation.x, fdigis), \
        'qy':round(prevodom.pose.pose.orientation.y, fdigis), \
        'qz':round(prevodom.pose.pose.orientation.z, fdigis), \
        'qw':round(prevodom.pose.pose.orientation.w, fdigis), \
        'ax':round(imumsg.linear_acceleration.x,fdigis),
        'ay':round(imumsg.linear_acceleration.y,fdigis),
        'az':round(imumsg.linear_acceleration.z, fdigis),
        'ax_o':round(imu_acc_odom[0],fdigis),
        'ay_o':round(imu_acc_odom[1],fdigis),
        'az_o':round(imu_acc_odom[2],fdigis),
        'an_x':round(imumsg.angular_velocity.x,fdigis),
        'an_y':round(imumsg.angular_velocity.y,fdigis),
        'an_z':round(imumsg.angular_velocity.z,fdigis)})
#    print('msg time: ', prevmsg_t, 'preodom.x', prevodom.pose.pose.position.x)

def callback(odommsg):
    global odomsub, outfile, prevmsg_t, writer, prevodom
    msg_t = odommsg.header.stamp.secs + 1.0*odommsg.header.stamp.nsecs/1000000000
    if (msg_t-prevmsg_t) < 0.1:
        return
    writer.writerow({'t': msg_t, 'type':'odom', 
        'x': round(odommsg.pose.pose.position.x,fdigis), 
        'y': round(odommsg.pose.pose.position.y,fdigis), 
        'z': round(odommsg.pose.pose.position.z,fdigis), 
        'qx':round(odommsg.pose.pose.orientation.x,fdigis), 
        'qy':round(odommsg.pose.pose.orientation.y,fdigis), 
        'qz':round(odommsg.pose.pose.orientation.z,fdigis), 
        'qw':round(odommsg.pose.pose.orientation.w, fdigis)})
    outfile.flush()
    prevmsg_t = msg_t
    prevodom = odommsg
    print('msg time: ', prevmsg_t, 'odom.x', odommsg.pose.pose.position.x)

if __name__ == "__main__":
    global odomsub, prevmsg_t, outfile, writer,imusub, prevodom
    prevodom = None
    arglen=len(sys.argv)
    filename = 'odom_turtlebot3d_gazebo.csv'
    if arglen>1:
        filename = sys.argv[1]
    prevmsg_t=0
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('listener', anonymous=True)
    odomsub = rospy.Subscriber("/odom", Odometry, callback)
    imusub = rospy.Subscriber("/mavros/imu/data", Imu, callbackimu)
    outfile= open(filename, 'w')
    fieldnames = ['t', 'type', 'x', 'y', 'z','qx', 'qy', 'qz', 'qw', 'ax', 'ay', 'az', 'an_x', 'an_y', 'an_z', 'ax_o', 'ay_o', 'az_o', 'x_', 'y_', 'z_', '#odom gt up to 15 secs, ax_o: acc_x in odom frame, x_ pose estimated from imu_acc value']
    writer = csv.DictWriter(outfile, fieldnames=fieldnames)
    writer.writeheader()
    prevodom=Odometry()
    prevodom.pose.pose.position.x=0
    prevodom.pose.pose.position.y=0
    prevodom.pose.pose.position.z=0
    prevodom.pose.pose.orientation.x=0
    prevodom.pose.pose.orientation.y=0
    prevodom.pose.pose.orientation.z=0
    prevodom.pose.pose.orientation.w=1
    rospy.spin()

