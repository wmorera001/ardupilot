/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       Coded by Víctor Mayoral Vilches <victor@erlerobot.com>
 *       Erle Robotics
 *
 *       This class a way to interact natively with the Robot Operative System (ROS)
 *		 using its native API. Take in account that in order to use this code a ROS
 *		 distro should be installed in your system. Refer to ros.org for instructions.
 *
 */

#include <AP_HAL.h>
#include "ROS.h"
#include "std_msgs/String.h"
#include <sstream>


#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

using namespace Linux;
extern const AP_HAL::HAL& hal;


LinuxROS::LinuxROS()
{

}

bool LinuxROS::init()
{
   // Init a talker node
   ros::init(NULL, NULL, "talker");
   // register to the "chatter" topic
   chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
   // number of messages sent
   count = 0;
}

void LinuxROS::_ros_timer_tick()
{
    ros::Rate loop_rate(10);
    
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;  
}

#endif // CONFIG_HAL_BOARD
