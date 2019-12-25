
killall record


#kill velodyne drivers
kill $(ps aux|grep "velodyne"|awk '{print $2}')


#killall -9 flea3_async_trigger
rosnode kill flea3_async_trigger
#killall -9 xsens_imu_driver
rosnode kill xsens_imu_driver
#killall -9 gnss_driver
rosnode kill gnss_driver

#killall -9 server





