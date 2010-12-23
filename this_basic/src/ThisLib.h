#ifndef THIS_LIB_H
#define THIS_LIB_H

#include <ros/ros.h>
#include "ArmLib.h"
#include <this_basic/SetPose.h>
#include <this_basic/SetGoal.h>
#include <this_basic/SetState.h>
#include <this_basic/SetVel.h>
#include <irobot_create_rustic/Speeds.h>
#include <sensor_msgs/LaserScan.h>
#include <localization_cu/GetPose.h>
#include <std_msgs/Int32.h>

ros::ServiceClient set_new_pose_client;
ros::ServiceClient set_new_vel_client;
ros::ServiceClient set_new_state_client;
ros::ServiceClient set_new_goal_client;
ros::ServiceClient get_pose_client;
ros::ServiceClient get_goal_client;

bool CREATE_IS_MOVING = false;
float FORWARD_DIST = 0.0;

void kill_this();
bool set_new_goal(float x, float y, float theta);
bool set_new_state(int state);
bool set_new_vel(int speed, int turn);
void drive_backward(float dist);
void drive_forward(float dist);
void turn_left(float degrees);
void turn_right(float degrees);
bool path_exists();

double game_time()
{
  return ros::Time::now().toSec()-START_TIME.toSec();
}

bool at_goal()
{ 
  localization_cu::GetPose pose;
  localization_cu::GetPose goal;
  if(get_pose_client.call(pose) && get_goal_client.call(goal))
  {
    float x = pose.response.pose.pose.position.x - goal.response.pose.pose.position.x;
    float y = goal.response.pose.pose.position.y - pose.response.pose.pose.position.y;
    float dist = sqrt(x*x+y*y);
    return (dist < .1);
  }
  else return false;
}

bool score_object()
{
  if(gsp(CLAW) == OPEN) return false;
  if(game_time() < PHASE_ONE_TIME)
  {
    //sap(0,10,gsp(CLAW),45,10,0);
    //usleep(100000);
    //ssp(BASE,90);
    //usleep(100000);
    sap(70,UP_POSITION);
    usleep(1000000);
    sap(90,OUT_POSITION);
    usleep(1000000);
    ssp(CLAW,OPEN);
    usleep(1000000);
    sap(0,UP_POSITION);
    //usleep(100000);
    sap(REST_POSITION);
    //usleep(100000);
    std::cout << "done\n";
  }
  else if(game_time() < PHASE_TWO_TIME)
  {
    set_new_state(0);
    set_new_goal(SECOND_FIELD,3.14);
    
    ros::spinOnce();
    if(!ROBOT_HAS_PATH)
    {
      set_new_goal(SECOND_FIELD,4.74);
      sleep(1);
      wait_till_arrived();
    }
    set_new_goal(SECOND_FIELD,3.14);
    sleep(1);
    wait_till_arrived();
    if(!at_goal())
    {
      sap(REST_POSITION);
      while(!at_goal()) ros::spinOnce();
      wait_till_arrived();
      ORIENTATION = 3.14;
    }
    //usleep(100000);
    //ssp(BASE,90);
    usleep(1000000);
    sap(70,UP_POSITION);
    usleep(1000000);
    sap(90,OUT_POSITION);
    usleep(1000000);
    ssp(CLAW,OPEN);
    usleep(1000000);
    sap(45,UP_POSITION);
    //usleep(100000);
    sap(REST_POSITION);
    //usleep(100000);
    std::cout << "phase two\n";
  }
  /*else if(game_time() < GAME_DURATION)
  {
    set_new_goal(MIDDLE,4.74);
    sap(REST_POSITION);
    cmvision::Blob b;
    while(!at_goal()) 
    {
      while(CREATE_IS_MOVING && ros::ok())
      {
        set_new_state(2);
        set_new_vel(0,0);
        ros::spinOnce();
      }
      if(grab_blob_special(b))
      {
        set_new_state(0);
        i = 6;
        //score_object();
      }
      else i = 0;
      ros::spinOnce();
        }
      }
    }
    ros::spinOnce();
    wait_till_arrived();
    ORIENTATION = 4.74;
    //sap(0,10,gsp(CLAW),30,10,0);
    //usleep(1000000);
    sap(0,UP_POSITION);
    usleep(1000000);
    sap(0,OUT_POSITION);
    usleep(1000000);
    ssp(CLAW,OPEN);
    usleep(1000000);
    sap(0,UP_POSITION);
    //usleep(100000);
    sap(REST_POSITION);
    //usleep(100000);
    std::cout << "phase three\n";
  }*/
  else 
  {
    kill_this();
    std::cout << "something went wrong..." << std::endl;
  }
  return true;
}

void wait_for_light()
{
  std::cout << "waiting for light sensor to be greater than " << LIGHT_THRESH << std::endl;
  int light_value = 0;
  //int light_value_2 = 0;
  while(light_value < LIGHT_THRESH && ros::ok())
  {
    light_value = get_light();
    //usleep(10000);
    //light_value_2 = get_light();
    //usleep(10000);
    if(light_value > LIGHT_THRESH && light_value > 0 && light_value < 100)
    {
    }
    else
    {
      std::cout << "caught an exception! value 1 = " << light_value << std::endl;
      light_value = 0;
    }
  }
  std::cout << "light tripped!\n";
}

bool set_new_pose(float x, float y, float theta)
{
  this_basic::SetPose srv;
  srv.request.x = x;
  srv.request.y = y;
  srv.request.theta = theta;
  ORIENTATION = theta;
  if(set_new_pose_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("Set new position to x=%f, y=%f, theta=%f",x,y,theta);
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_new_pose service did not work");
    #endif
  }
  return srv.response.success;
}

bool set_new_goal(float x, float y, float theta)
{
  this_basic::SetGoal srv;
  srv.request.x = x;
  srv.request.y = y;
  srv.request.theta = theta;
  if(set_new_goal_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("Set new goal to x=%f, y=%f, theta=%f",x,y,theta);
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_new_goal service did not work");
    #endif
  }
  return srv.response.success;
}

bool set_new_state(int state)
{
  this_basic::SetState srv;
  srv.request.value = state;
  if(set_new_state_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("Set new state to x=%d",state);
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_new_state service did not work");
    #endif
  }
  return srv.response.success;
}

bool set_new_vel(int change_speed, int change_turn)
{
  this_basic::SetVel srv;
  srv.request.speed = change_speed;
  srv.request.turn = change_turn;
  if(set_new_vel_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("Set new velocity to current speed + (%d), current_turn + (%d)",change_speed,change_turn);
    #endif
  }
  else
  {
    //#if(DEBUG_MODE)
    ROS_INFO("set_new_vel service did not work");
    //#endif
  }
  return srv.response.success;
}

void speeds_subscriber_callback(const irobot_create_rustic::SpeedsConstPtr& msg)
{
  if(msg->forward == 0.0 && msg->rotate == 0.0)
    CREATE_IS_MOVING = false;
  else CREATE_IS_MOVING = true;
  //ROS_INFO("CREATE_IS_MOVING = %d",CREATE_IS_MOVING);
}

void laser_subscriber_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  int count = 0;
  for(int i = 290;i <= 300 && ros::ok();i++)
  {
    FORWARD_DIST += msg->ranges[i];
    count++;
  }
  FORWARD_DIST /= count;
  std::cout << "distance from forward = " << FORWARD_DIST << std::endl;
}

void system_subscriber_callback(const std_msgs::Int32::ConstPtr& msg)
{
  ROBOT_HAS_PATH = (msg->data != 3);
}

void wait_till_moving()
{
  std::cout << "waiting till moving!\n";
  while(!CREATE_IS_MOVING && ros::ok())
  {
    usleep(100000);
    ros::spinOnce();
  }
  std::cout << "moving!\n";
}

void wait_till_arrived()
{
  std::cout << "waiting till arrived!\n";
  while(CREATE_IS_MOVING && ros::ok())
  {
    usleep(100000);
    ros::spinOnce();
  }
  std::cout << "arrived!\n";
}

void kill_this()
{
  set_new_state(1);
  sap(END_POSITION);
  while(ros::ok())
  {
    std::cout << "the game is over!" << std::endl;
    ros::spinOnce();
  }
}

void timer_callback(const ros::TimerEvent& event)
{
  std::cout << "timer event!" << std::endl;
  kill_this();
}

void turn_right(float degrees)
{
  set_new_state(2);
  set_new_vel(0,-1);
  usleep((int)(degrees*PIE*260.)/(180.*100.)*1000000);
  set_new_vel(0,0);
  //set_new_state(0);
}

void turn_left(float degrees)
{
  set_new_state(2);
  set_new_vel(0,1);
  usleep((int)(degrees*PIE*260.)/(180.*100.)*1000000);
  set_new_vel(0,0);
  set_new_vel(0,0);
  //set_new_state(0);
}

void drive_forward(float distance)
{
  set_new_state(2);
  set_new_vel(-1,0);
  set_new_vel(-1,0);
  set_new_vel(-1,0);
  usleep(distance*500.*100);
  set_new_vel(0,0);
  set_new_vel(0,0);
  //set_new_state(0);
}

void drive_backward(float distance)
{
  set_new_state(2);
  set_new_vel(1,0);
  set_new_vel(1,0);
  set_new_vel(1,0);
  usleep(distance*500.*100);
  set_new_vel(0,0);
  set_new_vel(0,0);
  //set_new_state(0);
}

void stop()
{
  set_new_state(2);
  set_new_vel(0,0);
  set_new_vel(0,0);
}

void e_stop()
{
  set_new_state(1);
}

bool scan_for_color(int channel,int direction)
{
  cmvision::Blob b;
  float offset = 0;
  sap(0,10,OPEN,15,10,0);
  if(direction == right)
  { 
    offset = -90;
    for(int i = 0;i >= -90 && ros::ok();i-=10)
    {
      ssp(BASE,(float)i);
      std::cout << "set position to " << offset+(float)i << std::endl;
      if(blob_is_valid(get_biggest_blob(channel)))
      {
        return true;
      }
      ros::spinOnce();
    }
  }
  else if(direction == forward) 
  {  
    offset = -45;
    ssp(BASE,-45.);
    usleep(300000);
    for(int i = 0;i <= 90 && ros::ok();i+=10)
    {
      ssp(BASE,offset+(float)i);
      //usleep(100000);
      std::cout << "set position to " << offset+(float)i << std::endl;
      if(blob_is_valid(get_biggest_blob(channel)))
      {
        return true;
      }
      ros::spinOnce();
    }
  } 
  else if(direction == left)
  {
    for(int i = 0;i <= 90 && ros::ok();i+=10)
    {
      ssp(BASE,offset+(float)i);
      //usleep(100000);
      std::cout << "set position to " << offset+(float)i << std::endl;
      if(blob_is_valid(get_biggest_blob(channel)))
      {
        return true;
      }
    }
    ros::spinOnce();
  }
  return false;
}

void center_on_color(int channel)
{
  cmvision::Blob b = get_biggest_blob(channel);
  while(CREATE_IS_MOVING && ros::ok())
  {
    set_new_state(2);
    set_new_vel(0,0);
    ros::spinOnce();
  }
  
  while(blob_is_valid(b) && !blob_is_centered(b) && ros::ok())
  {
    if(b.x < CENTER_X-CENTER_X_OFFSET)
      turn_left(degrees_from_middle(b));
    else
      turn_right(-degrees_from_middle(b));
    ros::spinOnce();
    usleep(500000);
    b = get_biggest_blob(channel);
  }
}

bool pick_up_botguy()
{
  return grab_blob(get_biggest_blob(botguy));
}

bool go_to_color(int channel)
{
  //record general direction of blob (from scan)
  float angle_found = gsp(BASE);
  cmvision::Blob b;
  b.x = -1;
  ssp(BASE,0);
  bool moving_flag = false;
  
  //turn to face general direction of blob
  if(angle_found > 0) turn_left(angle_found);
  else turn_right(-angle_found);
  
  //update the blob being tracked
  b = get_biggest_blob(channel);
  
  center_on_color(channel);
  ros::spinOnce();
  set_new_state(2);
  set_new_vel(-1,0);
  moving_flag = true;
  while(FORWARD_DIST > .25 && ros::ok())
  {
    if(blob_is_valid(b) && !blob_is_centered(b))
    {
      set_new_vel(0,0);
      center_on_color(channel);
      moving_flag = false;
      std::cout << "center on blob" << std::endl;
    }
    if(!CREATE_IS_MOVING)
    {
      moving_flag = true;
      std::cout << "go forward" << std::endl;
      set_new_vel(-1,0); 
    }
    if(b.bottom == 239)
    {
      ssp(WRIST_FLEX,0);
    }
    if(!blob_is_valid(b)) return false;
    usleep(50000);
    b = get_biggest_blob(channel);
    ros::spinOnce();
  }
  while(CREATE_IS_MOVING && ros::ok())
  {
    set_new_state(2);
    set_new_vel(0,0);
    ros::spinOnce();
  }
  
  return true;
}

void turn_left_fast(float sleep)
{
  set_new_state(2);
  set_new_vel(0,1);
  set_new_vel(0,1);
  set_new_vel(0,1);
  usleep(sleep*1000000);
  set_new_vel(0,0);
}

void turn_left_right(float sleep)
{
  set_new_state(2);
  set_new_vel(0,-1);
  set_new_vel(0,-1);
  set_new_vel(0,-1);
  usleep(sleep*1000000);
  set_new_vel(0,0);
}

void swipe_buckets()
{
  turn_right(45.);
}

bool score_object_bucket_run()
{
  if(gsp(CLAW) == OPEN) return false;
  else if(game_time() < GAME_DURATION)
  {
    ros::spinOnce();
    sap(-75,UP_POSITION);
    usleep(1000000);
    sap(-90,OUT_POSITION);
    usleep(1000000);
    ssp(CLAW,OPEN);
    usleep(1000000);
    sap(0,UP_POSITION);
    //usleep(100000);
    sap(REST_POSITION);
    //usleep(100000);
    return true;
  }
  return false;
}
#endif
