/**
 * @copyright (c) 2019, Sandeep Kota 
 * BSD 3-Clause License
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * @file turtleWalker.cpp
 * @author Sandeep Kota
 * @version 1.0
 * @brief TurtleWalker object implementation file
 * 
 */

#include <iostream>
#include "turtleWalker.hpp"

turtleWalker::turtleWalker() {
    minDistance = 0.5;
    obstacle = false;
    ///  Publish velocity data into the node.
    vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 500);
    /// Subscribe to the laser scan message.
    scan = nh.subscribe <sensor_msgs::LaserScan> ("/scan", 500, \
                                 &turtleWalker::scanCallback, this);
    resetBot();
}

turtleWalker::~turtleWalker() {
    resetBot();
}

void turtleWalker::resetBot() {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
}

bool turtleWalker::getObstacle() {
    return obstacle;
}

void turtleWalker::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    obstacle = false;
    ///  Check if the laser scan has an object within the min distance.
    for (float m : msg->ranges) {
        if (m < minDistance) {
            obstacle = true;
            ROS_INFO("Obstacle detected at %f distance!", m);
        }
    }
}

void turtleWalker::moveBot() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ///  If obstacle is found, rotate the robot.
        if (getObstacle() == true) {
            resetBot();
            msg.linear.x = 0.0;
            msg.angular.z = 0.5;
            ///  If obstacle is still found, rotate in opposite direction.
            if (getObstacle() == true) {
                    resetBot();
                    msg.linear.x = 0.0;
                    msg.angular.z = -0.5;
            }
        ///  Else move the robot forward.
        } else {
            ROS_INFO("Path Clear! Moving forward!");
            msg.linear.x = 0.2;
            msg.angular.z = 0.0;
        }
        ///  Publish the velocity to the robot.
        vel.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
