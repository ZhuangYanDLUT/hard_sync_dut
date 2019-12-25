//=============================================================================
// Copyright \A9 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: AsyncTriggerEx.cpp 316528 2017-02-22 00:03:53Z alin $
//=============================================================================

#include "stdafx.h"

#if defined(LINUX32) || defined(LINUX64)
#define LINUX
#endif

#ifdef LINUX
#include <time.h>
#include <unistd.h>
#endif

//#include <flycapture/FlyCapture2.h>
//#include <iostream>
//#include <sstream>
//#include <string>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "flea3/flea3_setting.h"
#include <sensor_msgs/fill_image.h>
//#include <utility>
#include <sensor_msgs/image_encodings.h>
#include "hque.h"



// Software trigger the camera instead of using an external hardware trigger
//#define SOFTWARE_TRIGGER_CAMERA

//是否输出调试信息到文件
//#define ENABLE_OUTPUT_DEBUG_INFO

using namespace FlyCapture2;
using namespace std;

//图像队列 与 imu synout队列（经测试，一般img队列有3个，syn有1个，img快2个）
HQue<sensor_msgs::Image> imgQue(6);
HQue<uint32_t> imgCounterQue(6);
HQue<std_msgs::Header> synoutQue(10);

//暂时没用，从imu送出脉冲信号，到相机开始生成一帧图像，需经历的时间，按理说需要补偿到图像时间戳上
double imgGenTimeBias = 0.0;  

void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion(&fc2Version);

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "."
            << fc2Version.build;
    cout << version.str() << endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;
}

void PrintCameraInfo(CameraInfo *pCamInfo)
{
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number - " << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl
         << endl;
}

void PrintError(Error error) { error.PrintErrorTrace(); }

bool CheckSoftwareTriggerPresence(Camera *pCam)
{
    const unsigned int k_triggerInq = 0x530;

    Error error;
    unsigned int regVal = 0;

    error = pCam->ReadRegister(k_triggerInq, &regVal);

    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return false;
    }

    if ((regVal & 0x10000) != 0x10000)
    {
        return false;
    }

    return true;
}

bool PollForTriggerReady(Camera *pCam)
{
    const unsigned int k_softwareTrigger = 0x62C;
    Error error;
    unsigned int regVal = 0;

    do
    {
        error = pCam->ReadRegister(k_softwareTrigger, &regVal);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return false;
        }

    } while ((regVal >> 31) != 0);

    return true;
}

bool FireSoftwareTrigger(Camera *pCam)
{
    const unsigned int k_softwareTrigger = 0x62C;
    const unsigned int k_fireVal = 0x80000000;
    Error error;

    error = pCam->WriteRegister(k_softwareTrigger, k_fireVal);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return false;
    }

    return true;
}
//设置快门时间
bool SetShutterTime(Camera *pCam)
{
    //Declarea Property struct. 
    Property prop; 
    //Define the property to adjust. 
    prop.type=SHUTTER; 
    //Ensure the property is on.
    prop.onOff=true; 
    //Ensure auto-adjust mode is off. 
	prop.autoManualMode=false; 
	//Ensure the property is set up to use absolute value control. 	
	prop.absControl=true; 
	//Set the absolute value of shutter to ms. 
	prop.absValue=4; 
	//Set the property. 
	Error error;
	pCam->SetProperty( &prop);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return false;
    }

    return true;
}

void imuSynoutHandler(const std_msgs::HeaderConstPtr& synoutMsg)
{
	synoutQue.push(*synoutMsg);
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "flea3_async_trigger");
    ros::NodeHandle n;
	ros::Subscriber subImuSynout = n.subscribe<std_msgs::Header> 
                                  ("/imu/synout_msg", 5, imuSynoutHandler);
    ros::Publisher chatter_pub = n.advertise<sensor_msgs::Image>("/image_raw", 2);
    PrintBuildInfo();

	//====================准备工作开始===========================
    Error error;
    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }
    cout << "Number of cameras detected: " << numCameras << endl;
    if (numCameras < 1)
    {
        cout << "Insufficient number of cameras... exiting" << endl;
        return -1;
    }

    PGRGuid guid;
    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

	 // Connect to the camera
    Camera cam;
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Power on the camera
    const unsigned int k_cameraPower = 0x610;
    const unsigned int k_powerVal = 0x80000000;
    error = cam.WriteRegister(k_cameraPower, k_powerVal);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

	// Wait for camera to complete power-up
    const unsigned int millisecondsToSleep = 100;
    unsigned int regVal = 0;
    unsigned int retries = 10;
    do
    {
#if defined(_WIN32) || defined(_WIN64)
        Sleep(millisecondsToSleep);
#elif defined(LINUX)
        struct timespec nsDelay;
        nsDelay.tv_sec = 0;
        nsDelay.tv_nsec = (long)millisecondsToSleep * 1000000L;
        nanosleep(&nsDelay, NULL);
#endif
        error = cam.ReadRegister(k_cameraPower, &regVal);
        if (error == PGRERROR_TIMEOUT)
        {
            // ignore timeout errors, camera may not be responding to
            // register reads during power-up
        }
        else if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }
        retries--;
    } while ((regVal & k_powerVal) == 0 && retries > 0);

	
    // Check for timeout errors after retrying
    if (error == PGRERROR_TIMEOUT)
    {
        PrintError(error);
        return -1;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    PrintCameraInfo(&camInfo);

#ifndef SOFTWARE_TRIGGER_CAMERA
    // Check for external trigger support
    TriggerModeInfo triggerModeInfo;
    error = cam.GetTriggerModeInfo(&triggerModeInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    if (triggerModeInfo.present != true)
    {
        cout << "Camera does not support external trigger! Exiting..." << endl;
        return -1;
    }
#endif

    // Get current trigger settings
    TriggerMode triggerMode;
    error = cam.GetTriggerMode(&triggerMode);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Set camera to trigger mode 0
    triggerMode.onOff = true;
    triggerMode.mode = 0;
    triggerMode.parameter = 0;
	triggerMode.polarity = 1;  //上升沿

#ifdef SOFTWARE_TRIGGER_CAMERA
    // A source of 7 means software trigger
    triggerMode.source = 7;
#else
    // Triggering the camera externally using source 0.
    triggerMode.source = 0;
#endif

    error = cam.SetTriggerMode(&triggerMode);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }


	//设置快门时间、分辨率
	SetShutterTime(&cam);
    //设置图像分辨率
	const Mode k_fmt7Mode = MODE_0;
	const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_MONO8;
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = 1280;
    fmt7ImageSettings.height = 1024;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

	
	 // Validate the settings to make sure that they are valid
    bool valid;
    Format7PacketInfo fmt7PacketInfo;
    error = cam.ValidateFormat7Settings(
        &fmt7ImageSettings, &valid, &fmt7PacketInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }
    if (!valid)
    {
        // Settings are not valid
        cout << "Format7 settings are not valid" << endl;
        return -1;
    }

    // Set the settings to the camera
    error = cam.SetFormat7Configuration(
        &fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }


    // Poll to ensure camera is ready
    bool retVal = PollForTriggerReady(&cam);
    if (!retVal)
    {
        cout << endl;
        cout << "Error polling for trigger ready!" << endl;
        return -1;
    }

    // Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration(&config);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Set the grab timeout to xxx ms.
    config.grabTimeout = 20;
    error = cam.SetConfiguration(&config);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Camera is ready, start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

#ifdef SOFTWARE_TRIGGER_CAMERA
    if (!CheckSoftwareTriggerPresence(&cam))
    {
        cout << "SOFT_ASYNC_TRIGGER not implemented on this camera! Stopping "
                "application"
             << endl;
        return -1;
    }
#else
    cout << "Trigger the camera by sending a trigger pulse to GPIO"
         << triggerMode.source << endl;
#endif
//====================准备工作结束=========================


//====================循环接收图像=========================
	sensor_msgs::Image msg;
	msg.header.frame_id = "/flea3";
	uint32_t imgCounter = 0;
	
    Image image;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {

#ifdef SOFTWARE_TRIGGER_CAMERA
        // Check that the trigger is ready
        PollForTriggerReady(&cam);
        cout << "Press the Enter key to initiate a software trigger" << endl;
        cin.ignore();

        // Fire software trigger
        bool retVal = FireSoftwareTrigger(&cam);
        if (!retVal)
        {
            cout << endl;
            cout << "Error firing software trigger" << endl;
            return -1;
        }
#endif

        // Grab an image
        error = cam.RetrieveBuffer(&image);
        if (error != PGRERROR_OK)
        {
            //PrintError(error);
            continue;
        }
		/*PNGOption png;
		char *s1 = "/home/robot/test_img.png";
		image.Save(s1, &png);*/
		

		//图像转换为ROS消息形式
		const auto bits_per_pixel = image.GetBitsPerPixel();
		//string encoding = MONO8;
		string encoding = flea3::PixelFormatToEncoding(bits_per_pixel);
		
		//image.Convert(PIXEL_FORMAT_MONO8, &convertedImage);
		//msg.header.stamp = ros::Time::now();
		sensor_msgs::fillImage(msg, encoding, image.GetRows(),
									image.GetCols(), image.GetStride(),
									image.GetData());
									
		/*===============
		新的图像虽然已经生成，但由于Ubuntu及ROS调度的不可预见性，
		最新的synout msg很可能此节点还没有收到，
		因此需要队列机制缓冲一下，如果没收到，就先等一等。
		================*/

		//新的图像入队列
		imgQue.push(msg);
		imgCounterQue.push(++imgCounter);
		//std::cout<<"image: "<<std::fixed<<imgCounter<<", system time:"<<ros::Time::now().toSec()<<std::endl;
		
#ifdef ENABLE_OUTPUT_DEBUG_INFO
		std::ofstream myout;
		myout.open("/home/robot/camera-info.txt", std::ios::app);
		myout<<std::fixed<<"image "<<imgCounter<<" comes!"<<std::endl;
		myout.close();
#endif
		
		//遍历每一个待处理的图像，找到对应的synout
		int imgQueCnt = 0;
		for(int i=imgCounterQue.front(); i!=imgCounterQue.rear(); i=imgCounterQue.get_next(i))
		{
			imgQueCnt++;
			
			
			//遍历imu synout的消息队列，寻找当前图像与哪个synout对应（即 计数一致）
			int synoutQueCnt = 0;
			int findj = -1;
			for(int j=synoutQue.front(); j!=synoutQue.rear(); j=synoutQue.get_next(j))
			{			
				if(imgCounterQue[i] < synoutQue[j].seq)  //不用找了，图像太滞后了
				{ break; }	
				
				synoutQueCnt++;	
				if(imgCounterQue[i] == synoutQue[j].seq)  //找到了
				{
					findj = j;
					break;
				}
			}
			
			
			//情况1：找到了，获取synout传递过来的时间戳，发布图像
			if(findj >= 0)
			{
				imgQue[i].header.stamp = synoutQue[findj].stamp;
				//imgQue[i].header.stamp = ros::Time().fromSec(synoutQue[findj].stamp.toSec() + imgGenTimeBias);
				chatter_pub.publish(imgQue[i]);  //正式发布图像消息
				//std::cout<<"publish image: "<<imgCounterQue[i]<<", final time: "<<
				//			std::fixed<<imgQue[i].header.stamp.toSec()<<std::endl<<std::endl;
				
#ifdef ENABLE_OUTPUT_DEBUG_INFO
				std::ofstream myout;
				myout.open("/home/robot/camera-info.txt", std::ios::app);
				myout<<std::fixed<<"publish image: "<<imgCounterQue[i]<<",  time: "<<imgQue[i].header.stamp.toSec()<<std::endl;
				myout.close();
#endif
				
				//清synoutQue
				while(synoutQueCnt--)
				{ synoutQue.pop_front(); }
			}
			//情况2：当前的图像太超前了，意味着以后所有的图像也会超前，
			//所以暂先退出，等多累积点synout，下一次再处理
			else if(synoutQueCnt==synoutQue.size())//包括synoutQue.size()为0的情况
			{
				//std::cout<<"current image comes too fast!!"<<std::endl;
				synoutQue.clear();
				imgQueCnt--;   //表示当前的还没处理
				break;
			}
			//情况3：当前的图像太滞后了，没办法，此帧图像不能要了（正常不会出现此情况）
			else if(synoutQueCnt==0)
			{
				std::cout<<"Error, current image comes too slow!!"<<std::endl;
				continue;
			}
			//情况4：还没遍历完，图像的cnt就已经小于synout的了，
			//意味着当前这个图像，永远找不到对应的帧了，很可能因为synout msg丢了（正常不会）
			else
			{
				std::cout<<"Error, current image have no corresponding synout msg!!"<<std::endl;
				//清synoutQue
				while(synoutQueCnt--)
				{ synoutQue.pop_front(); }
				continue;
			}
		}//end 遍历每一个图像

		
		//处理完，删除imgQue中已处理过的（共删除imgQueCnt个）
		while(imgQueCnt--)
		{
			imgQue.pop_front(); 
			imgCounterQue.pop_front(); 
		}
		
        ros::spinOnce();
        loop_rate.sleep();
    }//end while (ros::ok())
	
	
	//==================程序结束，收尾工作================
    // Turn trigger mode off.
    triggerMode.onOff = false;
    error = cam.SetTriggerMode(&triggerMode);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }
    cout << endl;
    cout << "Finished grabbing images" << endl;

    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    cout << "Done!" << endl;
    //cin.ignore();

    return 0;
}
