/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <nsra_control/nsra_hw_interface.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <vector>
#include <nsra_odrive_interface/nsra_control_step.h>

namespace nsra_control
{

NSRAHWInterface::NSRAHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : nsra_controller::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("nsra_hardware_interface", "NSRAHWInterface Ready.");

  drive_pub1 = nh_.advertise<std_msgs::Float64>("drive_pub1", 5);
  drive_pub2 = nh_.advertise<std_msgs::Float64>("drive_pub2", 5);
  drive_pub3 = nh_.advertise<std_msgs::Float64>("drive_pub3", 5);
  drive_pub4 = nh_.advertise<std_msgs::Float64>("drive_pub4", 5);
  drive_pub5 = nh_.advertise<std_msgs::Float64>("drive_pub5", 5);
  drive_pub6 = nh_.advertise<std_msgs::Float64>("drive_pub6", 5);

  axis_step = nh_.advertise<nsra_odrive_interface::nsra_control_step>("axis_step", 5);
  
   try
  {
    ser.setPort("/dev/controller");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
  }

  if(ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
  }
  else {
        ROS_ERROR_STREAM("Port failed! ");
  }
  
  for(int i = 0; i <= 5; i++) {
    saved_pos.push_back(0);
  }
}

void NSRAHWInterface::read(ros::Duration &elapsed_time)
{
  for (size_t i = 0; i < num_joints_; i++) {
    joint_position_[i] = saved_pos[i];
  }
}

void NSRAHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);
  nsra_odrive_interface::nsra_control_step msg_step;
  const int BUFFER_SIZE = 13;
  unsigned char data[BUFFER_SIZE];
  //std::cout << num_joints_ << std::endl;
  for (size_t i = 0; i < num_joints_ - 2; i++) {
    double pi = 2*acos(0.0);
    std_msgs::Float64 msg;
    int steps;
    if(i == 0)
    {
      msg.data = joint_position_command_[i]*(1024000*0.885)/pi/8192;
      drive_pub1.publish(msg);
      steps = round(joint_position_command_[i]*12500*0.885/pi);
      msg_step.axis1 = steps;
    } else if(i == 1)
    {
      msg.data = joint_position_command_[i]*(1024000*0.885)/pi/8192;
      drive_pub2.publish(msg);
      steps = round(joint_position_command_[i]*12500*0.885/pi);
      msg_step.axis2 = steps;
    } else if(i == 2)
    {
      msg.data = joint_position_command_[i]*(-204800)/pi/8192;
      drive_pub3.publish(msg);
      steps = round(joint_position_command_[i]*(-2500)/pi);
      msg_step.axis3 = steps;
    } else if(i == 3)
    {
      msg.data = joint_position_command_[i]*204800/pi/8192;
      drive_pub4.publish(msg);
      steps = round(joint_position_command_[i]*2500/pi);
      msg_step.axis4 = steps;
    } else if(i == 4)
    {
      msg.data = joint_position_command_[i]*204800/pi/8192;
      drive_pub5.publish(msg);
      steps = round(joint_position_command_[i]*2500/pi);
      msg_step.axis5 = steps;
    } else if(i == 5)
    {
      msg.data = joint_position_command_[i]*327680/pi/8192;
      drive_pub6.publish(msg);
      steps = round(joint_position_command_[i]*4000/pi);
      msg_step.axis6 = steps;
    }

    data[i*2] = ((uint16_t)(steps + 32000) >> 0) & 0xFF;
    data[i*2+1] = ((uint16_t)(steps + 32000) >> 8) & 0xFF;

    saved_pos[i] = joint_position_command_[i];
  }
  axis_step.publish(msg_step);

  if(joint_position_command_[6] > 0.01) 
  {
    data[12] = (uint8_t)1;
  } else 
  {
    data[12] = (uint8_t)0;
  }
  saved_pos[6] = joint_position_command_[6];
  saved_pos[7] = joint_position_command_[7];

  uint32_t crc = CRC::Calculate(data, BUFFER_SIZE, CRC::CRC_32());

  unsigned char crc_data[17];
  for(int n = 0; n < 13; n++) {
    crc_data[n] = data[n];
  }
  crc_data[13] = ((uint32_t)crc >> 0) & 0xFF;
  crc_data[14] = ((uint32_t)crc >> 8) & 0xFF;
  crc_data[15] = ((uint32_t)crc >> 16) & 0xFF;
  crc_data[16] = ((uint32_t)crc >> 24) & 0xFF;

  std::string result;
  base64::encode(result, crc_data);
  result.insert(0, 1, '\n');
  
  unsigned char message[result.length()];
  strcpy((char*)message, result.c_str());
  
  try { 
    ser.write(message, result.length());
  } catch (char *excp)
  {
    ROS_ERROR_STREAM("Error!");
  }
  
}

void NSRAHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  // pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ---------------------------------------------------- 
}

}  // namespace
