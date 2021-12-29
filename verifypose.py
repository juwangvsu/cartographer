#!/usr/bin/env python
# usage: python verifypose.py [gen|demo1|demo2|demo3]
# generate transformed cloud and viewing
# copyright Ju Wang
import yaml
import numpy
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
import json
import yaml
import csv
import sys
import tf
from tf.transformations import *
import glob
import pcl
import visualization

pcdfilelist = glob.glob("map*.pcd")  # original point cloud in cam frame
pcdfilelist_bodyrel = glob.glob("rel-body-map*.pcd") # point cloud in body frame
pcdfilelist_body = glob.glob("bodymap*.pcd") # point cloud in body frame of the previous seqno 
yamlfilelist = glob.glob("map*.yaml")
# q0= [-0.5, 0.5, -0.5, 0.5]
# Rm=[0 0 1; -1 0 0; 0 -1 0]
test_xyz = [1,1,1, 1]

# interactive check two original cloud
def democheckpcds():
    while True:
        print("enter two seq number to select two cloud fle: ")
        input_a = raw_input()
        if input_a=='q':
            return
        seq1=int(input_a.split()[0])
        seq2=int(input_a.split()[1])
        cloud1 = pcl.load(pcdfilelist[seq1])
        cloud2 = pcl.load(pcdfilelist[seq2])
        visualization.show2cloud(cloud1, cloud2, blocking=False)


# interactive check two cloud, cloud1 is in its body frame, cloud2 is transformed to cloud1's body frame, both are pre-calculated from gen mode
def democheckpcds_bodyrel():
    print("show two consective clouds, cloud 2 in cloud1's body frame: ")
    while True:
        print("enter the seq number to select cloud fle: ")
        input_a = raw_input()
        if input_a=='q':
            return
        seq1=int(input_a.split()[0])
        print(pcdfilelist_body[seq1],pcdfilelist_bodyrel[seq1])
        cloud1 = pcl.load(pcdfilelist_body[seq1])
        cloud2 = pcl.load(pcdfilelist_bodyrel[seq1])
        visualization.show2cloud(cloud1, cloud2, blocking=False)

def sortfilelist(flist):
#assume file name format: xxxx_##.yyya
    seqnlist=[]
    if len(flist)==0:
        return None
    fn=flist[0]
    fnprefix = fn.split('.')[0].split('_')[0]
    fnpostfix=fn.split('.')[1]
    for fn in flist:
        seqn = int(fn.split('.')[0].split('_')[1])
        seqnlist.append(seqn)
    seqnlist.sort()
    sortedflist=[]
    for seqn in seqnlist:
        fn=fnprefix+'_'+str(seqn)+'.'+fnpostfix
        sortedflist.append(fn)
    return sortedflist

# transform from cam frame to body frame
# input numpy array homo points
# euler angle of cam frame: [z-x-y order, -90, -90, 0], 
#   equvalent [x-y-z order: -90, 90, 0]
# quat [ -0.5, 0.5, -0.5, 0.5 ]
# rotation matrix:[  0.0000000,  0.0000000,  1.0000000;
#                   -1.0000000,  0.0000000,  0.0000000;
#                   0.0000000, -1.0000000,  0.0000000 ]
def  cam_to_body(parray_cam_h, pcsize):
    q_cam = [ -0.5, 0.5, -0.5, 0.5 ]
    Rm_cam=quaternion_matrix(q_cam)
    parray_body_h = numpy.array(parray_cam_h, copy=True)  
#    print('parray_cam_h.size ', parray_cam_h.size)
    for i in range(1,pcsize):
            parray_body_h[i] = numpy.matmul(Rm_cam,parray_body_h[i]) 
    return parray_body_h

# transform from  frame1 to frame0
# Rm, rot matrix
# t0 translation
# return pc in world frame, homo representation
def  frame1_to_frame0(parray_body_h, Rm, t0, pcsize):
        # transfrom from body frame to world frame
        parray_wf_h = numpy.array(parray_body_h, copy=True)
        #parray_wf_h = numpy.pad(parray_wf, ((0,0),(0,1)), mode='constant', constant_values=1)
        for i in range(1,pcsize):
            parray_wf_h[i] = numpy.matmul(Rm,parray_body_h[i]) + t0
        return parray_wf_h

def loadpcd(pcdfilename="mapdata_6.pcd"):
    p = pcl.load(pcdfilename)
    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k (50)
    fil.set_std_dev_mul_thresh (1.0)
    fil.filter().to_file("test.pcd".encode('utf-8'))
    return p

def pcd_worldframe(yamlfn="mappose_6.yaml",pcdfn="mapdata_6.pcd"):
  with open(yamlfn, "r") as stream:
    try:
        pose0=yaml.safe_load(stream)
        print(pose0['header'])
        t0=[pose0['pose']['position']['x'], pose0['pose']['position']['y'], pose0['pose']['position']['z'], 0]
        #t0=[-pose0['pose']['position']['z'], -pose0['pose']['position']['y'], pose0['pose']['position']['x'], 0]
        q0=[pose0['pose']['orientation']['x'], pose0['pose']['orientation']['y'], pose0['pose']['orientation']['z'],pose0['pose']['orientation']['w']]
        Rm=quaternion_matrix(q0)
        print(yamlfn, ' q0: ', q0 , 't0: ', t0)

        test_xyz_world_h = numpy.matmul(Rm,test_xyz)
        test_xyz_world = test_xyz_world_h[0:4] + t0
        #print('test_xyz_world_h: ', test_xyz_world_h, 'test_xyz_world: ', test_xyz_world)
        p = loadpcd(pcdfn)
        parray_cam = numpy.asarray(p)

        #expand column
        parray_cam_h = numpy.pad(parray_cam, ((0,0),(0,1)), mode='constant', constant_values=1)

        # transform to body frame
        parray_body_h = cam_to_body(parray_cam_h, p.size)
        pp_body =pcl.PointCloud(parray_body_h[:,[0,1,2]])
        pp_body.to_file("body"+pcdfn.encode('utf-8'))

        # transfrom from body frame to world frame (frame0)
        parray_wf_h = frame1_to_frame0(parray_body_h, Rm, t0, p.size)
        pp=pcl.PointCloud(parray_wf_h[:,[0,1,2]])
        pp.to_file("wf"+pcdfn.encode('utf-8'))

    except yaml.YAMLError as exc:
        print(exc)

#calculate the 2nd frame's pcd and pose w.r.t the body frame of previous frame 
# p_w = Rm0 * P_f0 + t0
# p_w = Rm1 * P_f1 + t1
# p_f0 = Rm0^-1 (Rm1 * p_f1 +t1 -t0)
# p_f0 = Rm0^-1 * Rm1 * p_f1 + Rm0^-1 *(t1-t0)
# so frame1 pose in frame0 is described by 
# Rm2 = Rm0^-1 * Rm1, t2 = Rm0^-1 *(t1-t0)
def pcd_relframe(yamlfn1="mappose_6.yaml",pcdfn1="mapdata_6.pcd", yamlfn2="mappose_7.yaml",pcdfn2="mapdata_7.pcd"):
     stream1=open(yamlfn1, "r")
     stream2=open(yamlfn2, "r")
     pose1=yaml.safe_load(stream1)
     t1=[pose1['pose']['position']['x'], pose1['pose']['position']['y'], pose1['pose']['position']['z'], 0]
     q1=[pose1['pose']['orientation']['x'], pose1['pose']['orientation']['y'], pose1['pose']['orientation']['z'],pose1['pose']['orientation']['w']]
     pose2=yaml.safe_load(stream2)
     t2=[pose2['pose']['position']['x'], pose2['pose']['position']['y'], pose2['pose']['position']['z'], 0]
     q2=[pose2['pose']['orientation']['x'], pose2['pose']['orientation']['y'], pose2['pose']['orientation']['z'],pose2['pose']['orientation']['w']]
     Rm1=quaternion_matrix(q1)
     Rinv1 = numpy.linalg.inv(Rm1)
     Rm2=quaternion_matrix(q2)
     print(yamlfn1, ' q1: ', q1 , 't1: ', t1)
     Rm3 = numpy.matmul(Rinv1, Rm2) 
     t3 = numpy.matmul (Rinv1 , (numpy.array(t2)-numpy.array(t1)))
     q3=quaternion_from_matrix(Rm3)
     yamlfn3="rel-"+yamlfn2
     print(yamlfn3, ' q2: ', q2 , 't2: ', t2)
     stream3 = open(yamlfn3, "w")
     stream4 = open(yamlfn3+".txt", "w") # pure text format: x,y,z,qx,qy,qz,qw
     stream4.write(str(t3[0:3]).strip('[]')+"\n")
     stream4.write(str(q3).strip('[]'))
     stream4.close()

     pose3=dict.copy(pose2)
     pose3['pose']['position']['x']=float(t3[0])
     pose3['pose']['position']['y']=float(t3[1])
     pose3['pose']['position']['z']=float(t3[2])
     pose3['pose']['orientation']['x'] = float(q3[0])
     pose3['pose']['orientation']['y'] = float(q3[1])
     pose3['pose']['orientation']['z'] = float(q3[2])
     pose3['pose']['orientation']['w'] = float(q3[3])
     yaml.safe_dump(pose3,stream3)
     stream3.close()

     p = loadpcd(pcdfn2)
     parray_cam = numpy.asarray(p)
     parray_cam_h = numpy.pad(parray_cam, ((0,0),(0,1)), mode='constant', constant_values=1)

     # transform to body frame
     parray_body_h = cam_to_body(parray_cam_h, p.size)
     pp_body =pcl.PointCloud(parray_body_h[:,[0,1,2]])

     # transform to frame 1 and save
     parray_wf_h = frame1_to_frame0(parray_body_h, Rm3, t3, p.size)
     pp=pcl.PointCloud(parray_wf_h[:,[0,1,2]])
     pp.to_file("rel-body-"+pcdfn2.encode('utf-8'))

#calculate pcd and pose w.r.t the body frame of previous point cloud
def pose_pcd_rel(yamlfilelist,pcdfilelist):
  for i in range(1,len(pcdfilelist)-1):
      pcd_relframe(yamlfilelist[i],pcdfilelist[i],yamlfilelist[i+1],pcdfilelist[i+1])

def pose_pcd_world(yamlfilelist,pcdfilelist):
  for i in range(1,len(pcdfilelist)):
      pcd_worldframe(yamlfilelist[i], pcdfilelist[i])

if __name__ == "__main__":
  pcdfilelist = sortfilelist(pcdfilelist)
  pcdfilelist_body = sortfilelist(pcdfilelist_body)
  print(pcdfilelist_body)
  pcdfilelist_bodyrel = sortfilelist(pcdfilelist_bodyrel)
  print(pcdfilelist_bodyrel)
  yamlfilelist = sortfilelist(yamlfilelist)
  #print(pcdfilelist)
  #print(yamlfilelist)
  arglen=len(sys.argv)
  cmd = "inspect"
  if arglen>1:
      cmd = sys.argv[1]
  if cmd=="gen":
      pose_pcd_world(yamlfilelist,pcdfilelist)
      pose_pcd_rel(yamlfilelist,pcdfilelist)
  if cmd=="demo1":
      print("demo1, show  original clouds")
      cloud2 = pcl.load(pcdfilelist[0])
      visualization.displaycloud(cloud2)
  if cmd=="demo2":
      print("demo2, interactively show two original clouds")
      democheckpcds()
  if cmd=="demo3":
      print("demo3")
      democheckpcds_bodyrel()
