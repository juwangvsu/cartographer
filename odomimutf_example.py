import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R
r = R.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
r.as_quat()
r.as_matrix()

import tf.transformations
from geometry_msgs.msg import PointStamped
import rospy
tp = PointStamped()
tp.header.frame_id='base_footprint'
tp.header.stamp.secs=2994
tf.transformPoint('odom',tp)

rospy.init_node('listener', anonymous=True)
tl = tf.TransformListener()

tl.transformPoint('odom',tp)

quaternion_matrix(quat2)


