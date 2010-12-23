#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <this_basic/SetGoal.h>
#include <this_basic/SetPose.h>
#include <this_basic/SetVel.h>
#include <this_basic/SetState.h>
#include "std_msgs/Int32.h"
#include "this_defines.h"

ros::Publisher goal_pub;
ros::Publisher pose_pub;
ros::Publisher vel_pub;
ros::Publisher user_control_pub;

void publish_state(int state)
{
  std_msgs::Int32 msg;
  msg.data = state;
  
  user_control_pub.publish(msg);
}

void publish_vel(int speed, int turn)
{
  geometry_msgs::Pose2D msg;
  msg.x = (float)speed;
  msg.theta = (float)turn;
  
  vel_pub.publish(msg);
}

void publish_goal(float x, float y, float theta)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "/map_cu";
    
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0;
    
  float r = sqrt(2-(2*cos(theta)));

  if(r == 0)
    msg.pose.orientation.w = 1;
  else
    msg.pose.orientation.w = sin(theta)/r;
  
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = r/2;
      
  goal_pub.publish(msg);
}

void publish_pose(float x, float y, float theta)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "/map_cu";
    
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0;
    
  float r = sqrt(2-(2*cos(theta)));

  if(r == 0)
    msg.pose.orientation.w = 1;
  else
    msg.pose.orientation.w = sin(theta)/r;
  
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = r/2;
      
  pose_pub.publish(msg);
}

bool set_new_pose(this_basic::SetPose::Request  &req,
                  this_basic::SetPose::Response &res)
{
  publish_pose(req.x,req.y,req.theta);
  res.success = true;
  return res.success;
}

bool set_new_goal(this_basic::SetGoal::Request  &req,
                  this_basic::SetGoal::Response &res )
{
  publish_goal(req.x,req.y,req.theta);
  res.success = true;
  return res.success;
}

bool set_new_vel(this_basic::SetVel::Request  &req,
                 this_basic::SetVel::Response &res)
{
  publish_vel(req.speed,req.turn);
  ROS_INFO("published new vel! (%d,%d)",req.speed,req.turn);
  res.success = true;
  return res.success;
}

bool set_new_state(this_basic::SetState::Request  &req,
                   this_basic::SetState::Response &res)
{
  publish_state(req.value);
  ROS_INFO("published new state! %d",req.value);
  res.success = true;
  return res.success;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle n;
  goal_pub = n.advertise<geometry_msgs::PoseStamped>("/cu/reset_goal_cu", 1);
  ros::ServiceServer new_goal_service = n.advertiseService("set_new_goal", set_new_goal);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/cu/user_pose_cu",1);
  ros::ServiceServer new_pose_service = n.advertiseService("set_new_pose",set_new_pose);
  
  vel_pub = n.advertise<geometry_msgs::Pose2D>("/cu/user_control_cu",1);
  ros::ServiceServer new_velocity_service = n.advertiseService("set_new_vel",set_new_vel);
  user_control_pub = n.advertise<std_msgs::Int32>("/cu/user_state_cu",1);
  ros::ServiceServer new_control_state_service = n.advertiseService("set_new_state",set_new_state);
  
  ROS_INFO("set_new_goal service started");
  ros::Rate loop_rate(5);
  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}

