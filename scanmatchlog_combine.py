#combine scanmatch_log.csv with odom data from scan_hrt.csv
#for each scanmatch log line, find the odom record < 0.05 sec apart

#input:
#   test/scanmatch_log.txt
#   test/scan_hrt.csv
#output:
#   test/scanmatch_log_odom.txt

#run this at carto main folder.
# 1/31/2022 Ju Wang
from tf.transformations import *

import csv
with open('test/scanmatch_log.txt') as f0:
    reader0 = csv.reader(f0)
    scanmatchlog = list(reader0)
with open('test/scan_hrt.csv') as f:
    reader1 = csv.reader(f)
    odomdata = list(reader1)
f2= open('test/scanmatch_log_odom.txt', 'w+')
writer = csv.writer(f2)

def find_matching_odom(tm):
    for j in range(1, len(odomdata)):
        tt1 = float(odomdata[j][0])
        if abs(tt1-tm)<0.08:
            return odomdata[j]
smlog_2=[]
#add head
smlog_2.append(['time,init_x,	y,	z,	qw,	qx,	qy,	qz,	smx,	y,	z,	qw,	qx,	qy,	qz,	t,		odomx,	y,	z,	qx,	qy,	qz,	qw,	eulerpre_x_angle,	yangle,	zangle,	euler_odom_xangle,	yangle,	zangle'])
#smlog_2.append(scanmatchlog[0]+odomdata[0])
for i in range(1,len(scanmatchlog)):
    tt = float(scanmatchlog[i][0])/1000000000
    odom_m = find_matching_odom(tt)
    if odom_m is None:
        print("no match for", scanmatchlog[i])
    else:
        quat_pred=[float(scanmatchlog[i][6]), float(scanmatchlog[i][7]),float(scanmatchlog[i][8]),float(scanmatchlog[i][5])]
        ea_pred = euler_from_quaternion(quat_pred)
        quat_odom=[float(odom_m[5]), float(odom_m[6]),float(odom_m[7]),float(odom_m[8])]
        ea_odom = euler_from_quaternion(quat_odom)
        smlog_2.append(scanmatchlog[i] + odom_m + ['ea_pred'] + list(ea_pred) + ['ea_odom'] + list(ea_odom))


for i in range(0,len(smlog_2)):
    writer.writerow(smlog_2[i])

f2.close()
