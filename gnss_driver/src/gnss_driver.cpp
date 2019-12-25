#include <cstdio>
#include <cerrno>
#include <ros/ros.h>
#include <serial/serial.h>
#include <ros/duration.h>
#include <fstream>
#include "gps_nmea.h"
#include "gnss_driver/gpgga_msg.h"
#include "gnss_driver/gprmc_msg.h"
#include "gnss_driver/lidar_sync_msg.h"
#include "hque.h"

#include "xsens_imu_driver/synin.h"


//是否输出调试信息到文件
//#define ENABLE_OUTPUT_DEBUG_INFO

//gnss队列，与synin队列
HQue<double> gnssTimeQue(20);
HQue<double> gnssUtcQue(20);
HQue<xsens_imu_driver::synin> syninQue(10);
bool isSyninAlreadyCame = false;

ros::Publisher lidar_sync_pub;

//最近一次对齐时，synin带来的时间戳，及所对应gnss数据的采集系统时间
double newestAlignIMUTime = -1.0;
double newestAlignGnssTime = -1.0;


bool findGnssPort(serial::Serial &_sp)
{
	//创建timeout, 5个值分别为
	//inter_byte_timeout_ , read_timeout_constant_ , read_timeout_multiplier_ , 
	//write_timeout_constant_ , write_timeout_multiplier_ 
	serial::Timeout to(20, 0, 0, 50, 0);  
	_sp.setBaudrate(115200);
	_sp.setTimeout(to);
	_sp.setBytesize(serial::bytesize_t::eightbits);						//数据位 fivebits sixbits sevenbits eightbits
	_sp.setParity(serial::parity_t::parity_none);						//奇偶校验parity_none parity_odd parity_even parity_mark..
	_sp.setStopbits(serial::stopbits_t::stopbits_one);				//停止位 stopbits_one stopbits_two stopbits_one_point_five 
	_sp.setFlowcontrol(serial::flowcontrol_t::flowcontrol_none);	//flowcontrol_none, flowcontrol_software, flowcontrol_hardware 
	
	char *pPorts[] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyUSB4", "/dev/ttyUSB5", "/dev/ttyUSB6",
							"/dev/ttyUSB7", "/dev/ttyUSB8", "/dev/ttyUSB9"};
	std::string str_test = "log version\r";  //用来测试串口返回值的命令字符串
	uint8_t buffer[1024];  //接收缓冲区
							
	bool isFind = false;
	for(int i=0; i<10; i++)
	{
		//std::cout<<"checking "<<pPorts[i]<<"..."<<std::endl;
		_sp.setPort(pPorts[i]);
		try { _sp.open(); }
		catch(serial::IOException& e) {}
		
		if(_sp.isOpen())
		{
			size_t nn = _sp.write(str_test);  
			if(nn >= 11)
			{
				ros::Duration(0.1).sleep();
				size_t n = _sp.available();
				if(n>0 && n<1000)
				{
					n = _sp.read(buffer, n);
					if(n >0 && n<1000)
					{
						buffer[n] = '\0';
						string tmpstring = (char*)buffer;
						//std::cout<<tmpstring<<std::endl;
						 //串口返回字符，带OK，应该就是GNSS板卡了
						int findid = tmpstring.find("OK", 0); 
						if (findid != string::npos)  //找到了
						{
							isFind = true;
							std::cout<<"Find GNSS device, port: "<<pPorts[i]<<std::endl;
							break;
						}
						else _sp.close();
					}
					else _sp.close();
				}
				else _sp.close();
			}
			else _sp.close();
		}
	}
	return isFind;
}


void imuSyninHandler(const xsens_imu_driver::syninConstPtr& syninMsg)
{
	syninQue.push(*syninMsg);
	if(!isSyninAlreadyCame)
		isSyninAlreadyCame = true;
}

//寻找与gnss数据对齐的synin消息，以更新时间戳基准
void f_align_time(const double &_currentTime, const double &_gpsUTC)
{
	//入队列
	gnssTimeQue.push(_currentTime);
	gnssUtcQue.push(_gpsUTC);
	
	//遍历每一个待处理的gnss cnt，找到对应的synin
	int gnssQueCnt = 0;
	for(int i=gnssTimeQue.front(); i!=gnssTimeQue.rear(); i=gnssTimeQue.get_next(i))
	{
		gnssQueCnt++;
		
		//遍历imu synin的消息队列，寻找当前gnss与哪个synin对应
		int syninQueCnt = 0;
		int findj = -1;
		for(int j=syninQue.front(); j!=syninQue.rear(); j=syninQue.get_next(j))
		{			
			if(gnssTimeQue[i] < (syninQue[j].timeSys-0.05))  //不用找了，gnss太滞后了
			{ break; }	
			
			syninQueCnt++;	
			if(gnssTimeQue[i] >= (syninQue[j].timeSys-0.05)
					&& gnssTimeQue[i] < (syninQue[j].timeSys+0.05)
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
			newestAlignIMUTime = syninQue[findj].header.stamp.toSec();
			newestAlignGnssTime = gnssTimeQue[i];
			//std::cout<<std::fixed;
			//std::cout<<"==> Bingo, use synin ("<<syninQue[findj].header.seq<<") to adjust time ("<<newestAlignGnssTime<<")."<<std::endl;

#ifdef ENABLE_OUTPUT_DEBUG_INFO	
			std::ofstream myout;
			myout.open("/home/robot/gnss-info.txt", std::ios::app);
			myout<<std::fixed<<"==> Bingo, use synin ("<<syninQue[findj].header.seq+1<<") to adjust time ("<<newestAlignGnssTime<<")."<<std::endl;
			myout.close();
#endif
			
			//=== 向lidar发送时间差消息 =====
			gnss_driver::lidar_sync_msg m_lidarsync;
			m_lidarsync.header =  syninQue[findj].header;  //IMU硬件时间
			m_lidarsync.timeImuSys =  syninQue[findj].timeSys;		//IMU系统时间
			m_lidarsync.timeGpsUTC =  gnssUtcQue[i];						//GPS UTC
			lidar_sync_pub.publish(m_lidarsync);
			//======================
			
			//清理syninQue
			while(syninQueCnt--)
			{ 
				syninQue.pop_front();
			}
		}
		//情况2：当前的gnss太超前了，意味着以后所有的gnss也会超前，
		//所以暂先退出，等多累积点synin，下一次再处理
		else if(syninQueCnt==syninQue.size())//包括syninQue.size()为0的情况
		{
			//std::cout<<"current gnss comes too fast!!"<<std::endl;
			syninQue.clear();
			gnssQueCnt--;   //表示当前的还没处理
			break;
		}
		//情况3：当前的gnss太滞后了，则此帧gnss不要了（一般是中间多余的gnss）
		else if(syninQueCnt==0)
		{
			//std::cout<<"Error, current gnss comes too slow!!"<<std::endl;
			continue;
		}
		//情况4：还没遍历完，gnss的cnt就已经小于synin的了，
		//意味着当前这个gnss，永远找不到对应的帧了，很可能因为synin msg丢了（正常不会）
		else
		{
			std::cout<<"Error, current gnss have no corresponding synin msg!!"<<std::endl;
			//清syninQue
			while(syninQueCnt--)
			{ 
				syninQue.pop_front(); 
			}
			continue;
		}
	}//end 遍历每一个gnss
	
	//处理完，删除gnssTimeQue中已处理过的
	while(gnssQueCnt--)
	{
		gnssTimeQue.pop_front(); 
		gnssUtcQue.pop_front(); 
	}
}

 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "gnss_driver");
	ros::NodeHandle n;
	ros::Subscriber subImuSynin = n.subscribe<xsens_imu_driver::synin> 
                                  ("/imu/synin_msg", 5, imuSyninHandler);
	ros::Publisher chatter_pub = n.advertise<gnss_driver::gpgga_msg>("/gnss/data", 1);
	//ros::Publisher chatter_pub1 = n.advertise<gnss_driver::gprmc_msg>("/gnss/data", 1);
	lidar_sync_pub = n.advertise<gnss_driver::lidar_sync_msg>("/lidar_sync", 1);
	
	//定义serial对象，尝试寻找gnss串口，并打开
	serial::Serial sp;
	if( ! findGnssPort(sp))
	{
		std::cout<<"Could not find GNSS device, exit!"<<std::endl;
		return -1;
	}

	//一些变量
	uint8_t buffer[10240];  //接收缓冲区
	CCopeNmea m_nmea;
	gnss_driver::gpgga_msg m_gpgga;
	//gnss_driver::gprmc_msg m_gprmc;
	ros::Rate loop_rate(50);
	double current_secs;
	int gnsscnt = 0;
	bool sysInit = false;
	
	
	//设置Gnss板卡的PPS、输出语句
	ros::Duration(0.02).sleep();
	std::string str0 = "COM COM1 9600 N 8 1 N OFF ON\r";  //激光要求波特率9600
	size_t nn = sp.write(str0);  
	ros::Duration(0.01).sleep();
	std::string str1 = "log com1 gprmc ontime 1\r";  //向激光雷达输出gprmc
	nn = sp.write(str1);  
	ros::Duration(0.01).sleep();
	std::string str2 = "PPSCONTROL ENABLE POSITIVE 1.0 30000\r";   //30ms
	nn = sp.write(str2);  
	ros::Duration(0.01).sleep();
	std::string str3= "log gpgga ontime 0.2\r";
	nn = sp.write(str3);
	
	//循环接收串口数据
	while(ros::ok())
	{
		//获取缓冲区内的字节数
		size_t n = sp.available();

		if(n>0)
		{
			//读出原始数据
			n = sp.read(buffer, n);
			if(n > 300)  //若缓冲区积攒的太多了则丢弃（一般刚开始时候）
				continue;
			else if(n > 0)
			{
				buffer[n] = '\0';
				//解析nmea语句，返回值ret代表解析出了几条完整的nmea语句
				int ret = m_nmea.CopeData((char*)buffer); 
				
				//==刚开始的几个，容易有问题，去掉==
				if(!sysInit)
				{
					gnsscnt++;
					if(gnsscnt >5)
						sysInit = true;
					continue;
				}
				//=======================
				
				//for(int i=0; i<ret; i++) //不用for循环了，因为根据此板卡特性，输出频率较低，这里ret会一直等于1
				if(ret > 0)
				{
					int i = 0; 
					
					//获取系统时间
					current_secs = ros::Time::now().toSec();
					//std::cout<<std::fixed<<"current system time:"<<current_secs<<std::endl;
#ifdef ENABLE_OUTPUT_DEBUG_INFO	
					std::ofstream myout;
					myout.open("/home/robot/gnss-info.txt", std::ios::app);
					myout<<std::fixed<<"gnss comes, sys time: "<<current_secs;
#endif

					if(isSyninAlreadyCame)  //若开启了同步，根据synin消息校正时间戳current_secs
					{
						//============  更新时间戳基准 ==================
						f_align_time(current_secs,  m_nmea.p_gpgga[i].timeUTC);  //寻找匹配对，更新校正基准
						if(newestAlignIMUTime < 0.0)
							//刚开始的一两帧会找不到对齐的synin，只能先用系统时间了(ros这个函数获取的时间也不带时区)
							//减8毫秒是稍微补偿一下，和imu的时间戳基准尽量一致
							current_secs = current_secs - 0.008; 
						else
							current_secs = current_secs - newestAlignGnssTime + newestAlignIMUTime;
						//std::cout<<"final gnss time: "<<current_secs<<std::endl<<std::endl;
						//=========================================
					}

#ifdef ENABLE_OUTPUT_DEBUG_INFO						
					myout<<std::fixed<<",  pub time: "<<current_secs<<std::endl;
					myout.close();
#endif

					//发布消息
					//m_gpgga.header.stamp = ros::Time::now();
					m_gpgga.header.stamp = ros::Time().fromSec(current_secs);
					m_gpgga.header.frame_id = "/GNSS";
					m_gpgga.head            = "$GPGGA";
					m_gpgga.timeUTC         = m_nmea.p_gpgga[i].timeUTC;
					
					m_gpgga.latitude        = m_nmea.p_gpgga[i].latitude;
					m_gpgga.northOrSouth    = m_nmea.p_gpgga[i].northOrSouth;
					m_gpgga.longitude       = m_nmea.p_gpgga[i].longitude;
					m_gpgga.eastOrWest      = m_nmea.p_gpgga[i].eastOrWest;
					m_gpgga.GPSstatus       = m_nmea.p_gpgga[i].GPSstatus;
					m_gpgga.satelliteNum    = m_nmea.p_gpgga[i].satelliteNum;
					m_gpgga.HDOP            = m_nmea.p_gpgga[i].HDOP;
					m_gpgga.altitude        = m_nmea.p_gpgga[i].altitude;
					m_gpgga.altitudeUnit    = m_nmea.p_gpgga[i].altitudeUnit;
					m_gpgga.geiod           = m_nmea.p_gpgga[i].geiod;
					m_gpgga.geiodUnit       = m_nmea.p_gpgga[i].geiodUnit;
					m_gpgga.rtkAge          = m_nmea.p_gpgga[i].rtkAge;
					m_gpgga.rtkID           = m_nmea.p_gpgga[i].rtkID;

					chatter_pub.publish(m_gpgga);
				}
			}
		}
	
		ros::spinOnce();
		loop_rate.sleep();
	}

	//设置Gnss板卡，取消输出
	std::string str4 = "PPSCONTROL DISABLE\r";
	nn = sp.write(str4);
	ros::Duration(0.01).sleep();

	std::string str5 = "unlog gpgga\r";
	nn = sp.write(str5);
	ros::Duration(0.01).sleep();
	
	std::string str6 = "unlog com1 gprmc\r";
	nn = sp.write(str6);
	ros::Duration(0.01).sleep();

	std::string str7 = "saveconfig\r";
	nn = sp.write(str7);
	ros::Duration(0.01).sleep();
	
	//关闭串口
    sp.close();
    return 0;
}
