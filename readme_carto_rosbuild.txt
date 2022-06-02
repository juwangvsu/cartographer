------11/27/21 homepc build  cartographer_ros from src----------------------
/media/student/data6/catkin_ws
using ninja tool
        catkin_make_isolated --install --use-ninja
        this will download both cartographer and cartographer_ros
        and build cartographer_ros
        source install_isolated/setup.bash
                to run carto node build here
        PCL link issue:

# To Build  Cartographer seperately (to access test)
cd /media/student/data6/catkin_ws/src/cartographer
mkdir build
cd build
cmake .. -G Ninja

