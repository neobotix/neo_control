/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TeleopNeo
{
public:
  TeleopNeo();
  ros::NodeHandle nh_;
  void sendCmd();
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  int linear_x, linear_y, angular_z;
  double last_linear_x[20], last_linear_y[20], last_angular_z[20];
  double l_scale_x, l_scale_y, a_scale_z;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  geometry_msgs::Twist vel;
  ros::Time lastCmd;
  ros::Duration timeOut;
  bool activeJoy;
  bool deadman;
  int deadman_button;
};


TeleopNeo::TeleopNeo():
  linear_x(1), linear_y(0), angular_z(2)
{
  activeJoy = false;
  deadman = false;
  lastCmd = ros::Time(0);
  double timeout;
  if(nh_.hasParam("timeOut"))
  {
  	nh_.getParam("timeOut", timeout);
  }
  else
  {
  	ROS_INFO("kein TimeOut gesetzt, verwende Standard");
  	timeout = 5.0;
  }
  if(nh_.hasParam("deadman_button"))
  {
  	nh_.getParam("deadman_button", deadman_button);
  }
  else
  {
  	ROS_INFO("kein deadmanbutton gesetzt, verwende Standard");
  	deadman_button = 5;
  }
  ROS_INFO("TimeOut: %f ", timeout);
  timeOut = ros::Duration(timeout);
  nh_.param("axis_linear_x", linear_x, linear_x);
  nh_.param("scale_linear_x", l_scale_x, l_scale_x);
  nh_.param("axis_linear_y", linear_y, linear_y);
  nh_.param("scale_linear_y", l_scale_y, l_scale_y);
  nh_.param("axis_angular_z", angular_z, angular_z);
  nh_.param("scale_angular_z", a_scale_z, a_scale_z);
 
  ROS_INFO("DeadmanButton: %i ", deadman_button);

  ROS_INFO("started joystick drive with ");

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopNeo::joyCallback, this);

}

void TeleopNeo::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  for(int i = 18; i >= 0; i--)
  {
	last_linear_x[i+1] = last_linear_x[i];
	last_linear_y[i+1] = last_linear_y[i];
	last_angular_z[i+1] = last_angular_z[i];
  }
  
  last_linear_x[0] = l_scale_x*joy->axes[linear_x];
  last_linear_y[0] = l_scale_y*joy->axes[linear_y];
  last_angular_z[0] = a_scale_z*joy->axes[angular_z];
  ROS_INFO("X: %f",last_linear_x[0]);
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.angular.z = 0;
  for(int g = 0; g < 20; g++)
  {
	vel.linear.x = vel.linear.x + last_linear_x[g];
	vel.linear.y = vel.linear.y + last_linear_y[g];
	vel.angular.z = vel.angular.z + last_angular_z[g];
  }
  vel.linear.x = (vel.linear.x/20);
  vel.linear.y = (vel.linear.y/20);
  vel.angular.z = (vel.angular.z/20);
  activeJoy = true;
  /*
  vel.angular.z = a_scale_z*joy->axes[angular_z];
  vel.linear.x = l_scale_x*joy->axes[linear_x];
  vel.linear.y = l_scale_y*joy->axes[linear_y];
  */
  deadman = (bool)joy->buttons[deadman_button];
  lastCmd = ros::Time::now();
}

void TeleopNeo::sendCmd()
{
  if((ros::Time::now() - lastCmd < timeOut) || deadman == true)
  {
    vel_pub_.publish(vel);
  }
  else
  {
    if(activeJoy)
    {
      vel.angular.z = 0;
      vel.linear.x = 0;
      vel.linear.y = 0;
      vel_pub_.publish(vel);
      activeJoy = false;
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_Neo");
  TeleopNeo teleop_Neo;

  ros::Rate loop_rate(50); // Hz
  while(teleop_Neo.nh_.ok())
  {
    teleop_Neo.sendCmd();
    loop_rate.sleep();
    ros::spinOnce();
  }
}
