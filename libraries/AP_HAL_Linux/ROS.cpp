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
 *       ROS.cpp
 *       Coded by Víctor Mayoral Vilches <victor@erlerobot.com>
 *       Erle Robotics
 *
 *       This class a way to interact natively with the Robot Operative System (ROS)
 *		 using its native API. Take in account that in order to use this code a ROS
 *		 distro should be installed in your system. Refer to ros.org for instructions.
 *
 */

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

using namespace Linux;


#endif // CONFIG_HAL_BOARD
