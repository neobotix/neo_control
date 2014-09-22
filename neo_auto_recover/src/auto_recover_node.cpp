/*********************************************************************
 * Softw
are License Agreement (BSD License)
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
/*
Used Parameters:

ButtonANr = Number of Button on Joystic for the init service
ButtonBNr = Number of Button on Joystic for the recover service

*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <neo_msgs/EmergencyStopState.h>
#include <cob_srvs/Trigger.h>


class auto_recover
{
public:
  auto_recover();
  ros::NodeHandle n;
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void EMSCallback(const neo_msgs::EmergencyStopState::ConstPtr& msg);
  
  bool button;
  int button_A_nr;
  int button_B_nr;
  ros::Subscriber joy_sub;
  ros::Subscriber EMS_sub;
  ros::ServiceClient client_init;
  ros::ServiceClient client_recover;
  cob_srvs::Trigger trigger;

};

auto_recover::auto_recover()
{
	button = false;
	EMS_sub = n.subscribe("/emergency_stop_state", 1, &auto_recover::EMSCallback, this);
	joy_sub = n.subscribe("/joy", 1, &auto_recover::joyCallback, this);
	client_init = n.serviceClient<cob_srvs::Trigger>("/init");
	client_recover = n.serviceClient<cob_srvs::Trigger>("/recover");
	//get Parameter from parameter-server
	if(n.hasParam("ButtonANr"))
  	{
  		n.getParam("ButtonANr", button_A_nr);
  	}
  	else
  	{
  		ROS_WARN("ButtonANr not found, using standard: 0");
  		button_A_nr = 0;
  	}
	if(n.hasParam("ButtonBNr"))
  	{
  		n.getParam("ButtonBNr", button_B_nr);
  	}
  	else
  	{
  		ROS_WARN("ButtonBNr not found, using standard: 1");
  		button_A_nr = 1;
  	}
}

void auto_recover::EMSCallback(const neo_msgs::EmergencyStopState::ConstPtr& msg)
{
	int EM_state = 0;
	EM_state = msg->emergency_state;
	if(EM_state == msg->EMCONFIRMED)
	{
		ROS_INFO("Found confirmed EMStop -> Autorecover Platform");
		if(client_recover.call(trigger))
		{
			ROS_INFO("Autorecover OK!");
		}
		else
		{
			ROS_INFO("Autorecover failed!");
		}
	}
}

void auto_recover::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//Knopf-A
	button = (bool)joy->buttons[button_A_nr];
	if(button == true)
	{
		ROS_INFO("Button A pressed");
		//init platform
		if(client_init.call(trigger))
		{
			ROS_INFO("Init OK!");
		}
		else
		{
			ROS_INFO("Init failed!");
		}
	}
	//Knopf-B
  	button = (bool)joy->buttons[button_B_nr];
	if(button == true)
	{
		ROS_INFO("Button B pressed");
		//recover platform
		if(client_recover.call(trigger))
		{
			ROS_INFO("Recover OK!");
		}
		else
		{
			ROS_INFO("Recover failed!");
		}
	}
}
int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "neo_auto_recover");
  auto_recover autorec;
  ros::Rate loop_rate(20); // Hz
  while(autorec.n.ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
}
