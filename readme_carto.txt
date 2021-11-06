------newpcamd install cartographer ----------------------
install:

update apt key for ros, otherwise cartographer-rviz pkg not found
	sudo apt-key list #confirm ros key expired
	sudo apt-key del "C1CF 6E31 E6BA DE88 68B1  72B4 F42E D6FB AB17 C654"
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-ros-msgs ros-melodic-cartographer-rviz
cd /media/student/data5; mkdir cartographer;


dataset:

wget  https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
wget https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
cd Documents; ln -sn /media/student/data5/cartographer

test:

roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Documents/cartographer/cartographer_paper_deutsches_museum.bag
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Documents/cartographer/b3-2016-04-05-14-14-00.bag

