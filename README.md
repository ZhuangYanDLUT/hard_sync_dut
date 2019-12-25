# hard_sync_dut
## Description
**hard_sync_dut** is the software driver of hard-synchronization framework for 3D-M-Box. A typical software driver of hard-synchronization must be implemented carefully to deal with problems such as **the absence of PPS signal**, **the transmission delay** and **data index association for multi-sensor data acquisition**.  
**This source code can be easily expand to other similar systems with slightly modification**.
  
  
**(1) The corresponding hardware of this source code is:**  
  
![image](/imgs/1.png)
  
  
**(2) The hardware connection is organized as:**  
  
![image](/imgs/2.png)
  
  
**(3) The ROS nodes graph is:**  
  
![image](/imgs/3.png)
  
  
  
**(4) Other issues:**  
>(a) Queue buffer mechanism is used to deal with the transmission delay among different ROS nodes.  
>(b) “velodyne-master” is originally Velodyne ROS driver downloaded from official website. However, we have modified the package in order to accomplish hard-synchronization of LiDAR with other sensors in the system. The detailed modifications are listed as follows:  
>>1) convert.cc of velodyne_pointcloud, extracting timestamp from velodyne packet is added.
>>2) input.cc, the time of a velodyne scan is modified by using time of its first packat (originally last packet).
>>3) convert.cc, before sending /velodyne_points message, the timestamp is adjusted according to the GNSS information.
  
  
## Usage
The detailed steps for installation are listed as follows:  
(1) Install Ubuntu (Ubuntu16.04).  
(2) Install ROS (Kinetic).  
(3) Install drivers needed by Velodyne:  
```
sudo apt-get install libpcap-dev
```
  
(4) Install drivers needed by flycapture cameras (please change this step according to your own camera hardware):  
1) install depandency
```
sudo apt-get install libraw1394-11 libavcodec-ffmpeg56    \
libavformat-ffmpeg56 libswscale-ffmpeg3 libswresample-ffmpeg1   \
libavutil-ffmpeg54 libgtkmm-2.4-dev libglademm-2.4-dev   \
libgtkglextmm-x11-1.2-dev libusb-1.0-0
```  
2) add “raw1394” to the system file: /etc/modules
3) Copy flycapture2-2.13.3.31-amd64-pkg_xenial.tgz (or the newest version) into Ubuntu system，extract (tar –xzvf flycapture2-2.13.3.31-amd64-pkg_xenial.tgz), cd into the folder and run the installation script：```sudo sh install_flycapture.sh```.  
  
(5) Install ROS serial driver (sudo apt-get install ros-<distro>-serial):
```
sudo apt-get install ros-kinetic-serial
```
  
(6) Install driver needed by Xsens Mti IMU:  
1) ```sudo apt-get install sharutils```  
2) run the script (with sudo authority): mtsdk_linux_4.8.sh. The script can be download from Xsens Homepage.  
  
(7) Install GNSS USB driver (our GNSS board is Novatel OEM718D, please change this step according to your own GNSS hardware): Copy Linux USB Driver--ngpsusbpackage.tar.gz to Ubuntu system and extract, run the ngps-install script in the extracted forlder.  
  
(8) Configure IP (192.168.1.xxx) for the computer in order to connect with Velodyne (192.168.1.201).  
  
(9) Download this source code (hard_sync_dut) and run the installation script install-code.sh in the folder. It will automatically create a catking workspace named project_ws, and install all the ROS package into the working space.  
  
(10) run the run-all.sh script in the catking workspace to start all the drivers.  
  
