workspace_name="project_ws"

basepath=$(cd `dirname $0`; pwd)
cd $basepath

source ./devel/setup.bash
./kill.sh


sudo chmod 777 /dev/ttyUSB*

cd $basepath
rosrun flea3 flea3_async_trigger &
sleep 2
rosrun xsens_imu_driver xsens_imu_driver &
sleep 1
rosrun gnss_driver gnss_driver &
sleep 1
roslaunch velodyne_pointcloud VLP16_points.launch &
sleep 2

# rosbag record
#rostopic hz /velodyne_points &
#rosbag record /velodyne_points &
#rosbag record /velodyne_points /imu/data &
#rosbag record /image_raw &
#rosbag record /image_raw /imu/data &
#rosbag record /gnss/data &
#rosbag record /imu/data /velodyne_points /image_raw /gnss/data &
#rosbag record /imu/data /velodyne_points /image_raw /gnss/data /imu/magneticField /imu/pressure &



