/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{

	ros::init(argc, argv, "move_robot");

	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

		sleep(10.0);
		 static const std::string PLANNING_GROUP = "manipulator";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

		// We will use the :planning_scene_interface:`PlanningSceneInterface`
		// class to add and remove collision objects in our "virtual world" scene
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		
		//add test collision box
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = move_group.getPlanningFrame();
		collision_object.id = "box1";
		//specify box size
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[0] = 1;
		primitive.dimensions[1] = 1;
		primitive.dimensions[2] = 0.1;
		//specify box position/orientation
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0;
		box_pose.position.y = 0;
		box_pose.position.z = 0;
		//add box to collision object
		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;
		//add box collision object to vector of Collision Objects
		std::vector<moveit_msgs::CollisionObject> collision_objects;
		collision_objects.push_back(collision_object);
		//add vector of collision bjects to the "world"
		planning_scene_interface.addCollisionObjects(collision_objects);


		const robot_state::JointModelGroup* joint_model_group =
			move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

		 move_group.setStartStateToCurrentState();

		geometry_msgs::Pose target_pose1 = move_group.getCurrentPose().pose;
		target_pose1.position.z = 0.5;
		target_pose1.position.y = 0;
		target_pose1.position.x = -0.5;

		 		ROS_INFO_NAMED("tutorial", "Current Pose After move x:%f y:%f z:%f", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z );

	 	move_group.setPoseTarget(target_pose1);

		move_group.plan(my_plan);

		move_group.move();

	ros::shutdown();

	return 0;
}
