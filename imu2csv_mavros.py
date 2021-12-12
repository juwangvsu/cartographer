#convert /mavros/imu/data and /camera/imu from mavros_realsense_short.bag to csv file to examine if
#imu x-axis data make sense
#usage: python imu2csv_mavros.py test11.csv
#rosbag play mavros_realsense_short.bag
# /mavros/imu/data frame_id base_link, 
# /camera/imu   frame_id: camera_imu_optical_frame, which has z-axis pointing
# 
# to screen, x to right, y point down.
# time stamp of the two imu are different, choose the one from /mavros/imu/data
# test11.ods shows that imu_link x-axis is negative of base_link x-axis
#       /camera/imu after transformed to base_link using the assumed camera_imu_optical_frame pose is inline with base_link

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
#callback for /camera/imu
#prevodom2 is camera link pose, which is used to transform imu data
# to base_link frame
# camera_link z= base x
# camera_link y= base -z 
# camera_link x= base -y
# q0= [-0.5, 0.5, -0.5, 0.5]
# Rm=[0 0 1; -1 0 0; 0 -1 0]

def callbackimu2(imumsg):
    global imusub, outfile, prevmsg_t, writer, prevodom2, imumsg1_prev, imu_acc_odom_prev, msg_t_prev
    if prevodom2 is None:
        return
    if msg_t_prev is None:
        return
    q0=[prevodom2.pose.pose.orientation.x, prevodom2.pose.pose.orientation.y,prevodom2.pose.pose.orientation.z,prevodom2.pose.pose.orientation.w]
    Rm=quaternion_matrix(q0)
    imu_acc_body = [imumsg.linear_acceleration.x,imumsg.linear_acceleration.y,imumsg.linear_acceleration.z,1]
    imu_acc_odom = numpy.matmul(Rm,imu_acc_body)
    imu_acc_odom_prev = imu_acc_odom
    msg_t = msg_t_prev
    #imumsg.header.stamp.secs + 1.0*imumsg.header.stamp.nsecs/1000000000
    writer.writerow({'t': msg_t, 'type':'imu', 'x': round(prevodom.pose.pose.position.x,4), 'y': round(prevodom.pose.pose.position.y,4), \
        'z': round(prevodom.pose.pose.position.z, 4), \
        'qx':round(prevodom.pose.pose.orientation.x, fdigis), \
        'qy':round(prevodom.pose.pose.orientation.y, fdigis), \
        'qz':round(prevodom.pose.pose.orientation.z, fdigis), \
        'qw':round(prevodom.pose.pose.orientation.w, fdigis), \
        'ax':round(imumsg1_prev.linear_acceleration.x,fdigis),
        'ay':round(imumsg1_prev.linear_acceleration.y,fdigis),
        'az':round(imumsg1_prev.linear_acceleration.z, fdigis),
        'ax_2':round(imu_acc_odom[0],fdigis),
        'ay_2':round(imu_acc_odom[1],fdigis),
        'az_2':round(imu_acc_odom[2],fdigis),
        'an_x':round(imumsg.angular_velocity.x,fdigis),
        'an_y':round(imumsg.angular_velocity.y,fdigis),
        'an_z':round(imumsg.angular_velocity.z,fdigis)})
#    print('msg time: ', prevmsg_t, 'preodom.x', prevodom.pose.pose.position.x)

def callbackimu(imumsg):
    global imusub, outfile, prevmsg_t, writer, prevodom, imumsg1_prev, imu_acc_odom_prev, msg_t_prev
    if prevodom is None:
        return
    imumsg1_prev=imumsg
    q0=[prevodom.pose.pose.orientation.x, prevodom.pose.pose.orientation.y,prevodom.pose.pose.orientation.z,prevodom.pose.pose.orientation.w]
    Rm=quaternion_matrix(q0)
    imu_acc_body = [imumsg.linear_acceleration.x,imumsg.linear_acceleration.y,imumsg.linear_acceleration.z,1]
    imu_acc_odom = numpy.matmul(Rm,imu_acc_body)
    msg_t = imumsg.header.stamp.secs + 1.0*imumsg.header.stamp.nsecs/1000000000
    msg_t_prev=msg_t
    writer.writerow({'t': msg_t, 'type':'imu', 'x': round(prevodom.pose.pose.position.x,4), 'y': round(prevodom.pose.pose.position.y,4), \
        'z': round(prevodom.pose.pose.position.z, 4), \
        'qx':round(prevodom.pose.pose.orientation.x, fdigis), \
        'qy':round(prevodom.pose.pose.orientation.y, fdigis), \
        'qz':round(prevodom.pose.pose.orientation.z, fdigis), \
        'qw':round(prevodom.pose.pose.orientation.w, fdigis), \
        'ax':round(imumsg.linear_acceleration.x,fdigis),
        'ay':round(imumsg.linear_acceleration.y,fdigis),
        'az':round(imumsg.linear_acceleration.z, fdigis),
        'ax_2':round(imu_acc_odom_prev[0],fdigis),
        'ay_2':round(imu_acc_odom_prev[1],fdigis),
        'az_2':round(imu_acc_odom_prev[2],fdigis),
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
    global odomsub, prevmsg_t, outfile, writer,imusub, prevodom, prevodom2, imumsg1_prev, imu_acc_odom_prev, msg_t_prev

    prevodom = None
    msg_t_prev = None
    imumsg1_prev=Imu()
    imu_acc_odom_prev=numpy.zeros(3)
    arglen=len(sys.argv)
    filename = 'test10.csv'
    if arglen>1:
        filename = sys.argv[1]
    prevmsg_t=0
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('listener', anonymous=True)
    odomsub = rospy.Subscriber("/odom", Odometry, callback)
    imusub = rospy.Subscriber("/mavros/imu/data", Imu, callbackimu)
    imusub2 = rospy.Subscriber("/camera/imu", Imu, callbackimu2)
    outfile= open(filename, 'w')
    fieldnames = ['t', 'type', 'x', 'y', 'z','qx', 'qy', 'qz', 'qw', 'ax', 'ay', 'az', 'an_x', 'an_y', 'an_z', 'ax_2', 'ay_2', 'az_2', 'x_', 'y_', 'z_', '# value']
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
    prevodom2=Odometry()
    prevodom2.pose.pose.orientation.x=-0.5
    prevodom2.pose.pose.orientation.y=0.5
    prevodom2.pose.pose.orientation.z=-0.5
    prevodom2.pose.pose.orientation.w=0.5
    rospy.spin()

