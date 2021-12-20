#!/bin/bash
# convert pcd file to ply for meshlab use
# run: cd cartographer/test/mavros_realsense_pcd; ../../pcd2pcl_clean.sh
#pcl_converter -f ascii mapdata_3.pcd mapdata_3.ply
#awk '{if($1 ~ /^[^nan]/) print $0;}' mapdata_3.ply > mapdata_3_.ply
yourfilenames=`ls mapdata*.pcd`
mkdir -p tmp
rm tmp/*
for eachfile in $yourfilenames
do
   fname=$eachfile
   echo $eachfile
   #IFS='.'
   readarray -d . -t strarr <<< "$eachfile"
   #IFS='_'
   readarray -d _ -t strarr2 <<< "${strarr[0]}"
   echo ${strarr2[1]}
   pclfname=${strarr[0]}".ply"
   echo $pclfname
   cd tmp
   pcl_converter -f ascii "../"$fname $pclfname
   awk '{if($1 ~ /^[^nan]/) print $0;}' $pclfname > ${strarr[0]}"_.ply"
   linenum=$(wc -l <${strarr[0]}"_.ply")
   linenum=$(($linenum-11))
   sed -i "5s/.*/element vertex $linenum/" ${strarr[0]}"_.ply"
   mv ${strarr[0]}"_.ply" ../
   cd ..
done
