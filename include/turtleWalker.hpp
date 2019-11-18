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
 * @file turtleWalker.hpp
 * @author Sandeep Kota
 * @version 1.0
 * @brief TurtleWalker object include file
 * 
 */

#ifndef INCLUDE_TURTLEWALKER_HPP_
#define INCLUDE_TURTLEWALKER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief This class describes a turtle walker.
 */
class turtleWalker {
 private:
    bool obstacle;
    geometry_msgs::Twist msg;
    ros::Publisher vel;
    ros::Subscriber scan;
    double minDistance;
    ros::NodeHandle nh;

 public:
    /**
     * @brief Constructs a new instance of turtleWalker class.
     */
    turtleWalker();

    /**
     * @brief Destroys the object turtleWalker class.
     */
    ~turtleWalker();

    /**
     * @brief Gets the obstacle information.
     * @return True/False Depending on whether an object is found or not.
     */
    bool getObstacle();

    /**
     * @brief Callback function for subscriber function.
     * @param[in] msg The message subscribed from laser scan topic.
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief Resets the robot velocities.
     */
    void resetBot();

    /**
     * @brief Moves the robot in the environment.
     */
    void moveBot();
};

#endif /* INCLUDE_TURTLEWALKER_HPP_ */
