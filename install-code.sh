#
#By He Guojian 
#2019-02
#

#define vars
workspace_name="project_ws"
basepath=$(cd `dirname $0`; pwd)

# 1) create catkin workspace
echo "---->1. Create catkin space...\n"
#judge if workspace exists?
if [ -d ~/$workspace_name/src ]; then
	echo -n "workspace already exists, overlap ? [y/n]:"
	read res
	if [ $res == "n" ]; then
		exit 0
	else
		rm -fr ~/$workspace_name/
	fi
fi

#create  workspace
mkdir -p ~/$workspace_name/src
cd ~/$workspace_name/src
catkin_init_workspace

cd ~/$workspace_name/
catkin_make

source ~/$workspace_name/devel/setup.bash

#check if success
if echo $ROS_PACKAGE_PATH |grep -a $workspace_name; then
	echo "Successfully create workspace!"
else
	echo "Create workspace failed!"
	exit 1
fi


#2) create pkgs and copy files
echo "---->2. Create pckgs and copy files...\n"

#copy files into the folders
echo ========================$basepath
cd $basepath
cp -rf ./velodyne-master/ ~/$workspace_name/src
cp -rf ./camera_base-master/ ~/$workspace_name/src
cp -rf ./camera_info/ ~/$workspace_name/src
cp -rf ./flea3-master/ ~/$workspace_name/src
cp -rf ./gnss_driver/ ~/$workspace_name/src
cp -rf ./xsens_imu_driver/ ~/$workspace_name/src

cp -rf kill.sh ~/$workspace_name/
cp -rf run-boost.sh ~/$workspace_name/
cp -rf run-all.sh ~/$workspace_name/
cp -rf run-laser-relocalization.sh ~/$workspace_name/

mkdir -p ~/$workspace_name/log/


#3) catkin make
echo "---->3. Catkin make...\n"
cd ~/$workspace_name/
#catkin_make -DCATKIN_WHITELIST_PACKAGES="velodyne-master"
#catkin_make -DCATKIN_WHITELIST_PACKAGES="xsens_imu_driver"
#catkin_make -DCATKIN_WHITELIST_PACKAGES="laser_slam_algorithm"
catkin_make 
source devel/setup.bash
echo "---->OK!   Install Completely.\n"
