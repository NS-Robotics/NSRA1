/*****************************************************************************
*  BSD 3-Clause License
* 
*  Copyright (c) 2020, Noa Sendlhofer
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
* 
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
* 
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
* 
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/* Author: Noa Sendlhofer
   Desc:   Part of the NSRA Robot Vision stack. Bottle detection MoveIt! Interface.
*/


#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nsra_robot_vision/stereo_camera_coords.h"
#include <vector>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>


std::vector<double> x;
std::vector<double> y;
std::vector<double> z;

int nmb_prv_objs = 0;
bool exec_flag = false;
int table_height = 200;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{

  posture.joint_names.resize(2);
  posture.joint_names[0] = "Slider8";
  posture.joint_names[1] = "Slider9";
  
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(2);
  
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{

  posture.joint_names.resize(2);
  posture.joint_names[0] = "Slider8";
  posture.joint_names[1] = "Slider9";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.02;
  posture.points[0].positions[1] = 0.02;
  posture.points[0].time_from_start = ros::Duration(2);
  
}

void empty_bottle(moveit::planning_interface::MoveGroupInterface& move_group, double px, double py, double pz, int rot)
{

    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);

    std::vector< double > current_state = move_group.getCurrentRPY();

    current_state[0] += rot;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(current_state[0],current_state[1],current_state[2]);
    myQuaternion = myQuaternion.normalize();

    geometry_msgs::Quaternion quat = tf2::toMsg(myQuaternion);

    geometry_msgs::Pose target_pose1;

    target_pose1.orientation.x = quat.x;
    target_pose1.orientation.y = quat.y;
    target_pose1.orientation.z = quat.z;
    target_pose1.orientation.w = quat.w;

    target_pose1.position.x = px;
    target_pose1.position.y = py;
    target_pose1.position.z = pz;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("pick_place", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();
    move_group.stop();
    move_group.clearPoseTargets();
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, int index)
{

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "world";
  tf2::Quaternion orientation;
  orientation.setRPY(M_PI / 2, 0, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = y[index]/1000 - 0.16;
  grasps[0].grasp_pose.pose.position.y = x[index]/1000;
  grasps[0].grasp_pose.pose.position.z = z[index]/1000 - 0.07;


  grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
  grasps[0].pre_grasp_approach.direction.vector.x = 0.7;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;


  grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
  grasps[0].post_grasp_retreat.direction.vector.z = 0.2;
  grasps[0].post_grasp_retreat.min_distance = 0.2;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;


  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table");

  move_group.pick("object" + std::to_string(index), grasps);

}

void place(moveit::planning_interface::MoveGroupInterface& group, int index)
{

  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "world";
  tf2::Quaternion orientation;

  orientation.setRPY(0, 0, 0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  place_location[0].place_pose.pose.position.x = 0.5;
  place_location[0].place_pose.pose.position.y = 0.6;
  place_location[0].place_pose.pose.position.z = z[index]/1000 - 0.1;

  place_location[0].pre_place_approach.direction.header.frame_id = "world";
  place_location[0].pre_place_approach.direction.vector.z = -0.2;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  place_location[0].post_place_retreat.direction.header.frame_id = "world";
  place_location[0].post_place_retreat.direction.vector.z = 0.5;
  place_location[0].post_place_retreat.min_distance = 0.2;
  place_location[0].post_place_retreat.desired_distance = 0.4;

  openGripper(place_location[0].post_place_posture);

  group.setSupportSurfaceName("table");

  group.place("object" + std::to_string(index), place_location);

}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  nmb_prv_objs = x.size() + 1;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(x.size() + 1);

  collision_objects[0].id = "table";
  collision_objects[0].header.frame_id = "world";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.8;
  collision_objects[0].primitives[0].dimensions[1] = 0.8;
  collision_objects[0].primitives[0].dimensions[2] = table_height/1000.0;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.40;
  collision_objects[0].primitive_poses[0].position.y = 0.27;
  collision_objects[0].primitive_poses[0].position.z = table_height/2000.0;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  for(int i = 1; i < x.size() + 1; i++)
  {

    collision_objects[i].header.frame_id = "world";
    collision_objects[i].id = "object" + std::to_string(i - 1);

    collision_objects[i].primitives.resize(1);
    collision_objects[i].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
    collision_objects[i].primitives[0].dimensions.resize(3);
    collision_objects[i].primitives[0].dimensions[0] = 0.2;
    collision_objects[i].primitives[0].dimensions[1] = 0.02;

    collision_objects[i].primitive_poses.resize(1);
    collision_objects[i].primitive_poses[0].position.x = y[i-1]/1000;
    collision_objects[i].primitive_poses[0].position.y = x[i-1]/1000;
    collision_objects[i].primitive_poses[0].position.z = z[i-1]/1000 - 0.1;
    collision_objects[i].primitive_poses[0].orientation.w = 1.0;

    collision_objects[i].operation = collision_objects[2].ADD;
  }

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void scene_callback(const nsra_robot_vision::stereo_camera_coords& data)
{
  if(exec_flag) {
    return;
  }

  ros::param::get("/table_height", table_height);

  x = data.x;
  y = data.y;
  z = data.z;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::vector<std::string> object_ids;
  object_ids.push_back("table");
  for (int i = 1; i < nmb_prv_objs; i++)
  {
    object_ids.push_back("object" + std::to_string(i - 1));
  }
  planning_scene_interface.removeCollisionObjects(object_ids);

  addCollisionObjects(planning_scene_interface);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nsra_moveit_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  nh.getParam("/table_height", table_height);

  ros::Subscriber sub = nh.subscribe("nsra/stereo_camera_coords", 1, scene_callback);

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("nsra");
  group.setPlanningTime(45.0);

  while(true)
  {
    
    ros::WallDuration(1.0).sleep();

    std::string var;
    std::cin >> var;
    int val;
    try {
      val = std::stoi(var);
    } catch (int e)
    {
      val = 100;
    }
    if(val >= 0 && val < nmb_prv_objs - 1)
    {
      exec_flag = true;

      pick(group, val);

      ros::WallDuration(1.0).sleep();

      empty_bottle(group, 200.0/1000, 400.0/1000, 570.0/1000, 0);
      empty_bottle(group, 200.0/1000, 400.0/1000, 570.0/1000, -90);
      ros::WallDuration(8.0).sleep();
      empty_bottle(group, 200.0/1000, 400.0/1000, 570.0/1000, 45);

      ros::WallDuration(1.0).sleep();

      place(group, val);

      exec_flag = false;

    } else if(var == "end")
    {
      break;
    }
  }

  ros::waitForShutdown();
  return 0;

}