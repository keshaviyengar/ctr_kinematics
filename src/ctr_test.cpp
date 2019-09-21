// This file is part of CTR, a kinematics library for concentric tube robots
//
// Copyright (C) 2017 Konrad Leibrandt <konrad.lei@gmx.de>
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <Erl/Utility/debug.h>
#include <ros/ros.h>
#include <ctr_kinematics/ctr_robot.h>
#include <ctr_kinematics/ctr_kinematics.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

typedef double Real;
typedef CTR::Robot<Real> Robot_t;

int main(int _argc, char *_argv[])
{
    ros::init(_argc, _argv, "ctr_kinematics_node");
    ros::NodeHandle n;

    CTRKinematics ctr_kinematics(n);

    ros::Rate loop_rate(10);

    // every few seconds sample a new joint and publish
    while (ros::ok())
    {
        ctr_kinematics.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
