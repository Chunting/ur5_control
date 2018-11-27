
// ROS
#include <ros/ros.h>
#include <iostream>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <std_msgs/Float32.h>

ros::Publisher pub_to_gripper;


void OpenGripper(){
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output Grip;

  Grip.rACT = 1;
  Grip.rPR = 0;
  Grip.rGTO = 1;
  Grip.rSP  = 255;
  Grip.rFR = 150;
  Grip.rATR =0;

  pub_to_gripper.publish(Grip);
}

void CloseGripper(){
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output Grip;

  Grip.rACT = 1;
  Grip.rPR = 255;
  Grip.rGTO = 1;
  Grip.rSP  = 255;
  Grip.rFR = 150;
  Grip.rATR =0;  
  pub_to_gripper.publish(Grip);
}

void InitGripper(){

  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output Grip;

  Grip.rACT = 1;
  Grip.rPR = 0;
  Grip.rGTO = 1;
  Grip.rSP  = 255;
  Grip.rFR = 150;
  Grip.rATR =0;

  pub_to_gripper.publish(Grip);
}

void Move_ur5(const geometry_msgs::PointConstPtr& Centroid){
    std::cout << "MOVE!" << std::endl;

		geometry_msgs::Point Centroid_XYZ = *Centroid; 
		static const std::string PLANNING_GROUP = "manipulator";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

		// We will use the :planning_scene_interface:`PlanningSceneInterface` ee_link
		// class to add and remove collision objects in our "virtual world" scene
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		//add rviz tools
		namespace rvt = rviz_visual_tools;
		moveit_visual_tools::MoveItVisualTools visual_tools("ur5");
		visual_tools.deleteAllMarkers();

		visual_tools.loadRemoteControl();

		Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
 		text_pose.translation().z() = 1.75;
		visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);


		//add test collision box
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = move_group.getPlanningFrame();
		collision_object.id = "box1";
		//specify box size
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[0] = 2;
		primitive.dimensions[1] = 2;
		primitive.dimensions[2] = 0.1;
		//specify box position/orientation
		geometry_msgs::Pose box_pose;
  		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0;
  		box_pose.position.y = 0;
  		box_pose.position.z = -0.05;

		//add box to collision object
		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;
		//add box collision object to vector of Collision Objects
		std::vector<moveit_msgs::CollisionObject> collision_objects;
		collision_objects.push_back(collision_object);
		//add vector of collision bjects to the "world"
		sleep(2);
		planning_scene_interface.addCollisionObjects(collision_objects);

		move_group.setStartStateToCurrentState();

		geometry_msgs::Pose target_pose1 = move_group.getCurrentPose().pose;
		//target_pose1.orientation.x = 0;
		
		OpenGripper();
		sleep(2);

		target_pose1.position.z = Centroid_XYZ.z + 0.25;
		target_pose1.position.y = Centroid_XYZ.y;
		target_pose1.position.x = Centroid_XYZ.x;
		

		ROS_INFO_NAMED("tutorial", "Current Pose After move x:%f y:%f z:%f w:%f", move_group.getCurrentPose().pose.position.x, move_group.getCurrentPose().pose.position.y, move_group.getCurrentPose().pose.position.z, move_group.getCurrentPose().pose.orientation.w );

	 	move_group.setPoseTarget(target_pose1);

		move_group.move();

		target_pose1.position.z -= 0.1;


	 	move_group.setPoseTarget(target_pose1);		
		move_group.move();

		CloseGripper();
		sleep(2);

		//back to home
		target_pose1.position.z = 0.6;
		target_pose1.position.y = 0.1;
		target_pose1.position.x = -0.5;

	 	move_group.setPoseTarget(target_pose1);

		move_group.plan(my_plan);

		move_group.move();

		OpenGripper();
		sleep(2);
}


int main(int argc, char** argv)
{
	sleep(10);
	std::cout << "INIT" << std::endl;
	ros::init(argc, argv, "move_robot");
	ros::NodeHandle node_handle;
	ros::Subscriber sub = node_handle.subscribe ("output_point", 1, &Move_ur5);
	pub_to_gripper = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output> ("Robotiq2FGripperRobotOutput", 1);
	InitGripper();
   	ros::AsyncSpinner spinner(2);
	spinner.start();
	  ros::waitForShutdown();
  return 0; 

}