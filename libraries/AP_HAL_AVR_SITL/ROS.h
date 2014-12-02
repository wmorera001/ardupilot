/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_HAL_AVR_SITL_ROS_H__
#define __AP_HAL_AVR_SITL_ROS_H__

#include "AP_HAL_AVR_SITL.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#include "ros/ros.h"


class AVR_SITL::SITLROS {
public:
	SITLROS();
	bool init();
	void _ros_thread_routine();

protected:
	ros::NodeHandle n;
	ros::Publisher chatter_pub;
	int count;
};

#endif // CONFIG_HAL_BOARD
#endif // __AP_HAL_AVR_SITL_ROS_H__
