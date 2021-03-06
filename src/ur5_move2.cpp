

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
		
		target_pose1.position.z = 0.3 ;
		target_pose1.position.y = 0.1;
		target_pose1.position.x = -0.5;
		

		ROS_INFO_NAMED("tutorial", "Current Pose After move x:%f y:%f z:%f w:%f", move_group.getCurrentPose().pose.position.x, move_group.getCurrentPose().pose.position.y, move_group.getCurrentPose().pose.position.z, move_group.getCurrentPose().pose.orientation.w );

	 	move_group.setPoseTarget(target_pose1);

		move_group.plan(my_plan);

		visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  		visual_tools.trigger();

  		// Wait for MoveGroup to recieve and process the collision object message
		visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to position");
		sleep(2);
		move_group.move();

		target_pose1.position.z -= 0.1;

	 	move_group.setPoseTarget(target_pose1);		
		move_group.move();


		//back to home
		target_pose1.position.z = 0.6;
		target_pose1.position.y = 0.4;
		target_pose1.position.x = -0.5;

	 	move_group.setPoseTarget(target_pose1);

		move_group.plan(my_plan);
		visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  		visual_tools.trigger();
		visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move back");

		move_group.move();

	ros::shutdown();

	return 0;
}