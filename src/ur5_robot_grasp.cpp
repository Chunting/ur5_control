
// ROS
#include <ros/ros.h>
#include <iostream>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <std_msgs/Float32.h>
ros::Publisher pub_to_gripper;

void Gripper_move(){

  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output Grip;

  Grip.rACT = 0;
  Grip.rPR = 0;
  Grip.rGTO = 0;
  Grip.rSP  = 0;
  Grip.rFR = 0;
  Grip.rATR =0;

  pub_to_gripper.publish(Grip);
  sleep(2);

  Grip.rACT = 1;
  Grip.rPR = 0;
  Grip.rGTO = 1;
  Grip.rSP  = 255;
  Grip.rFR = 150;
  Grip.rATR =0;

  pub_to_gripper.publish(Grip);

  sleep(2);

  for(int i=0;i<10;i++){
  Grip.rPR = 255;
  pub_to_gripper.publish(Grip);

  sleep(2);
  Grip.rPR = 0;
  pub_to_gripper.publish(Grip);
  sleep(2);
  }

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_test");
  ros::NodeHandle nh;
  //ros::Subscriber Gripper = nh.subscribe ("gripper_pos", 1, cloud_cb);
  pub_to_gripper = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output> ("Robotiq2FGripperRobotOutput", 1);

	ros::AsyncSpinner spinner(1);
	spinner.start();
  Gripper_move();

  return 0;
}

// BEGIN_TUTORIAL
// CALL_SUB_TUTORIAL table1
// CALL_SUB_TUTORIAL table2
// CALL_SUB_TUTORIAL object
//
// Pick Pipeline
// ^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL pick1
// openGripper function
// """"""""""""""""""""
// CALL_SUB_TUTORIAL open_gripper
// CALL_SUB_TUTORIAL pick2
// closedGripper function
// """"""""""""""""""""""
// CALL_SUB_TUTORIAL closed_gripper
// CALL_SUB_TUTORIAL pick3
//
// Place Pipeline
// ^^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL place
// END_TUTORIAL