cd ../..
#catkin_make clean
source devel/setup.bash

# Make the deps first
catkin_make --pkg wam_srvs
catkin_make --pkg wam_msgs

# Make the rest
catkin_make

# Install everything
catkin_make install
