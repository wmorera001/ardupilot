/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_HAL_LINUX_ROS_H__
#define __AP_HAL_LINUX_ROS_H__

#include <AP_HAL_Linux.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "ros/ros.h"


class Linux::LinuxROS {
public:
	LinuxROS();
	bool init();
	void _ros_timer_tick();

protected:
	ros::NodeHandle n;
	ros::Publisher chatter_pub;
	ros::Rate loop_rate(10);
	int count;

};

#endif // CONFIG_HAL_BOARD
#endif // __AP_HAL_LINUX_ROS_H__
