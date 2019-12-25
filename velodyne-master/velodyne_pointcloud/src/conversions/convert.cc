/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"
#include "hque.h"
#include <pcl_conversions/pcl_conversions.h>

//是否输出调试信息到文件
//#define ENABLE_OUTPUT_DEBUG_INFO

namespace velodyne_pointcloud
{
	
	HQue<double> lidarTimeQue(40);
	HQue<std_msgs::Header> syninQue(10);
	HQue<double> syninTimeQue(10);
	
	
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
	
	//硬同步相关
	isValidGnssHasCome = false;
	timeImuSys = timeImu = timeGpsUTC = -1.0;
	lidar_internal_time = 0.0;
	imu_imusys = 0.0;
	hour_circle = 0;
	last_lidar_internal_time = 0.0;
	lidar_syn_ =  node.subscribe("/lidar_sync", 5, &Convert::processLidarSynin, (Convert *) this);
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    velodyne_rawdata::VPointCloud::Ptr
      outMsg(new velodyne_rawdata::VPointCloud());
	 
	 if(scanMsg->packets.size() > 0)
		last_lidar_internal_time = lidar_internal_time;
	
    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
        double ret = data_->unpack(scanMsg->packets[i], *outMsg);  //修改，提取LIDAR内部时间戳并返回
		if(i == 0)   //取第一个数据包的时间戳
			lidar_internal_time = ret;
    }
	
	// ===========  赋予时间戳的逻辑 ==================
	double final_time;
	//无硬件同步
	if(timeImu < 0.0)  
	{
		final_time = scanMsg->header.stamp.toSec();  //系统时间
	}
	//硬件同步，且出现过有效的GPS UTC (注意，lidar内部时间、gps utc时间，都是0-3600秒范围)
	else if(isValidGnssHasCome)  
	{
		if(lidar_internal_time<1.0 && last_lidar_internal_time>3599.0)
			hour_circle++;
		final_time = lidar_internal_time - lastValidTimeGpsUTC + hour_circle*3600.0 + lastValidTimeIMU;  //使用以GPS utc为桥梁计算的差值
	}
	//硬件同步，且一直未出现过有效的GPS UTC
	else 
	{
		final_time = imu_imusys + scanMsg->header.stamp.toSec(); //使用以系统时间为桥梁计算的差值imu_imusys
	}
	
#ifdef ENABLE_OUTPUT_DEBUG_INFO		
	std::ofstream myout;
	myout.open("/home/robot/lidar-info.txt", std::ios::app);
	myout<<std::fixed<<"sys time: "<<scanMsg->header.stamp.toSec()<<", lidar time: "<<lidar_internal_time<<", ";
	if(timeImu < 0.0) myout<<"sys as final."<<std::endl;
	else if(isValidGnssHasCome) myout<<std::fixed<<"bridge gps utc, final time: "<<final_time<<std::endl;
	else myout<<std::fixed<<"bridge sys, final time: "<<final_time<<std::endl;
	myout.close();
#endif
	
	// outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
	//outMsg->header.stamp是uint64_t，单位微秒
	outMsg->header.stamp = final_time * 1e6;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;
	//============================================

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);
    output_.publish(outMsg);
  }
  
  
  
  
void Convert::processLidarSynin(const gnss_driver::lidar_sync_msg::ConstPtr &_syncMsg)
{
	timeImuSys = _syncMsg->timeImuSys;
	timeImu = _syncMsg->header.stamp.toSec();
	timeGpsUTC = _syncMsg->timeGpsUTC;
	
	 //有效GPS UTC (>2.0 <3597.0的意思是，不要在时间整点边界处同步，防止 刚刚确定有效的gps就出现套圈现象，容易出问题)
	if(timeGpsUTC > 2.0 && timeGpsUTC<3597.0 && fabs(lidar_internal_time-timeGpsUTC) < 0.5) 
	{
		lastValidTimeIMU = timeImu;
		lastValidTimeGpsUTC = timeGpsUTC;
		hour_circle = 0;  //清0
		isValidGnssHasCome = true;
	}
	if( ! isValidGnssHasCome) //没出现过有效GPS UTC
	{
		imu_imusys = timeImu - timeImuSys;
	}
	
#ifdef ENABLE_OUTPUT_DEBUG_INFO		
	std::ofstream myout;
	myout.open("/home/robot/lidar-info.txt", std::ios::app);
	myout<<std::fixed<<"==> synin ("<<_syncMsg->header.seq<<")  comes, GPS UTC: "<<timeGpsUTC<<", last lidar time: "<<lidar_internal_time<<std::endl;
	myout.close();
#endif
}
  
  

  /*
// ===============================================  
void Convert::processSynin(const std_msgs::HeaderConstPtr& syninMsg)
{
	syninTimeQue.push(ros::Time::now().toSec());
	syninQue.push(*syninMsg);
}

void Convert::f_align_time(const double &lidar_time)
{
	//入队列
	lidarTimeQue.push(lidar_time);
	
	//遍历每一个待处理的lidar（采集时间），找到对应的synin
	int lidarQueCnt = 0;
	for(int i=lidarTimeQue.front(); i!=lidarTimeQue.rear(); i=lidarTimeQue.get_next(i))
	{
		lidarQueCnt++;
		
		//遍历imu synin的消息队列，寻找当前lidar时间与哪个synin对应
		int syninQueCnt = 0;
		int findj = -1;
		for(int j=syninQue.front(); j!=syninQue.rear(); j=syninQue.get_next(j))
		{			
			if(lidarTimeQue[i] < (syninTimeQue[j]-0.005))  //不用找了，lidar太滞后了
			{ break; }	
			
			syninQueCnt++;	
			if(lidarTimeQue[i] >= (syninTimeQue[j]-0.005) 
				&& lidarTimeQue[i] < (syninTimeQue[j]+0.05)
				)  //找到了
			{
				findj = j;
				break;
			}
		}
		
		
		//情况1：找到了，获取synin传递过来的时间戳
		if(findj >= 0)
		{
			//这两个量，就是本函数的输出
			newestAlignIMUTime = syninQue[findj].stamp.toSec();
			newestAlignLidarTime = lidarTimeQue[i];
			
			
			//清理syninQue
			while(syninQueCnt--)
			{ 
				syninQue.pop_front();
				syninTimeQue.pop_front();
			}
		}
		//情况2：当前的lidar太超前了，意味着以后所有的lidar也会超前，
		//所以暂先退出，等多累积点synin，下一次再处理
		else if(syninQueCnt==syninQue.size())//包括syninQue.size()为0的情况
		{
			//std::cout<<"current lidar comes too fast!!"<<std::endl;
			syninQue.clear();
			syninTimeQue.clear();
			lidarQueCnt--;   //表示当前的还没处理
			break;
		}
		//情况3：当前的lidar太滞后了，则此帧lidar不要了（一般是中间多余的lidar）
		else if(syninQueCnt==0)
		{
			//std::cout<<"Error, current lidar comes too slow!!"<<std::endl;
			continue;
		}
		//情况4：还没遍历完，lidar的时间就已经小于synin的采集时间了，
		//意味着当前这个lidar，永远找不到对应的帧了，很可能因为有之前的synin msg没有配对上
		else
		{
			std::cout<<"Error, current lidar have no corresponding synin msg!!"<<std::endl;
			//清syninQue
			while(syninQueCnt--)
			{ 
				syninQue.pop_front(); 
				syninTimeQue.clear();
			}
			continue;
		}
	}//end 遍历每一个lidar
	
	//处理完，删除lidarTimeQue中已处理过的
	while(lidarQueCnt--)
	{
		lidarTimeQue.pop_front(); 
	}
}
// ===============================================  
*/

} // namespace velodyne_pointcloud
