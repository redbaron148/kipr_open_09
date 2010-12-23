//#include <ros/ros.h>
//#include "ArmLib.h"
#include "ThisLib.h"
#include <sensor_msgs/LaserScan.h>

//#define DEBUG_MODE 0

int main(int argc, char **argv)
{
  //init the node and create a node handle
  ros::init(argc, argv, "test_find_grab_ball_client");
  ros::NodeHandle n;
  
  //setting up the service clients to use the arm and blob services
  blob_client = n.serviceClient<this_basic::GetBlob>("get_blob");
  blobs_client = n.serviceClient<this_basic::GetBlobs>("get_blobs");
  set_position_client = n.serviceClient<crustcrawler_arm::SetPosition>("set_position");
  set_servo_position_client = n.serviceClient<crustcrawler_arm::SetServoPosition>("set_servo_position");
  get_servo_position_client = n.serviceClient<crustcrawler_arm::GetServoPosition>("get_servo_position");
  set_all_servo_positions_client = n.serviceClient<crustcrawler_arm::SetAllServoPositions>("set_all_servo_positions");
  set_new_pose_client = n.serviceClient<this_basic::SetPose>("set_new_pose");
  set_new_goal_client = n.serviceClient<this_basic::SetGoal>("set_new_goal");
  set_new_state_client = n.serviceClient<this_basic::SetState>("set_new_state");
  set_new_vel_client = n.serviceClient<this_basic::SetVel>("set_new_vel");
  get_light_client = n.serviceClient<crustcrawler_arm::GetLight>("get_light");
  get_pose_client = n.serviceClient<localization_cu::GetPose>("/cu/get_pose_cu");
  get_goal_client = n.serviceClient<localization_cu::GetPose>("/cu/get_goal_cu");
  ros::Subscriber speeds_subscriber = n.subscribe("speeds_bus",1,speeds_subscriber_callback);
  ros::Subscriber hokuyo_laser = n.subscribe("/scan",1,laser_subscriber_callback);
  ros::Subscriber system_check = n.subscribe("/cu/system_state_cu",1,system_subscriber_callback);
  //begin code execution
  
  cmvision::Blob b;
  sap(START_POSITION);
  set_new_pose(HOME, 1.57);
  
  wait_for_light();
  std::cout << "starting node\n";
  START_TIME = ros::Time::now();
  int count = 0;
  count = 0;
  
  //ros::Timer timmer = n.createTimer(ros::Duration(177.),timer_callback);
  //sleep(START_TIME_OFFSET);
  
  //set_new_goal(FIRST_FIELD, 3.14);
  /*set_new_state(2);
  set_new_vel(1,0);
  sleep(2);
  set_new_vel(0,0);
  set_new_state(0);*/
  sap(REST_POSITION);
  
  //while(!at_goal() && ros::ok()) ros::spinOnce();
 // wait_till_arrived();
  
  while(ros::ok())
  {
    b = center_on_blob("size");
    if(grab_blob(b)) 
    {
      score_object();
    }
  }
  /*
  set_new_goal(SECOND_FIELD, 3.14);
  sap(REST_POSITION);
  
  while(!at_goal() && ros::ok()) ros::spinOnce();
  wait_till_arrived();
  
  while(game_time() < PHASE_TWO_TIME && ros::ok())
  {
    b = center_on_blob("size");
    if(grab_blob(b)) 
    {
      score_object();
    }
  }
  
  wait_till_arrived();
  int scan_direction = left;

  /*while(game_time() < GAME_DURATION && ros::ok())
  {
    if(ORIENTATION == 4.74)
    {
      scan_direction = right;
    }
    else if(ORIENTATION == 1.57)
    {
      scan_direction = left; 
    }
    if(scan_for_color(botguy,scan_direction))
    {
      //go to safe distance to grab botguy
      set_new_state(0);
      set_new_goal(SAFE_SCAN,3.14);
      while(!at_goal()) ros::spinOnce();
      wait_till_arrived();
      
      //if you still see botguy
      if(scan_for_color(botguy,forward))
      {
        if(go_to_color(botguy))
        {
          while(CREATE_IS_MOVING && ros::ok())
  	  {
            set_new_state(2);
            set_new_vel(0,0);
            ros::spinOnce();
          }
          for(int i = 0;i < 5;i++)
          {
            b = center_on_blob("center");
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
              score_object();
            }
            else i = 0;
          }
        }
      }
    }
    else
    {
      set_new_state(0);
      set_new_goal(SAFE_SCAN,3.14);
      while(!at_goal()) ros::spinOnce();
      wait_till_arrived();
      
      drive_backward(70.);
      drive_forward(40.);
      
      while(CREATE_IS_MOVING && ros::ok())
      {
        set_new_state(2);
        set_new_vel(0,0);
        ros::spinOnce();
      }
    }
  }
  
  //set_new_goal(MIDDLE,3.14);
  //while(!at_goal()) ros::spinOnce();
  //sleep(3);
  
  if(scan_for_water())
  {
    go_to_color(blue);
    count = 0;
    while(count < 1) 
    {
      b = center_on_blob("center");
      if(grab_blob(b)) 
      {
        score_object();
        count++;
      }
    }
  }
  else
  {
    set_new_goal(SECOND_FIELD, 3.14);
    sap(REST_POSITION);
    while(!at_goal()) ros::spinOnce();
    sleep(2);
    while(game_time() < GAME_DURATION)
    {
      sap(REST_POSITION);
      //ssp(WRIST_FLEX,0);
      b = center_on_blob("size");
      if(grab_blob(b)) score_object();
      set_new_goal(SECOND_FIELD, 3.14);
      while(!at_goal()) ros::spinOnce();
    }
    kill_this();
  }
  */
  
  //while(!at_goal()) ros::spinOnce();
  //sleep(2);
  
  //while(game_time() < PHASE_THREE_TIME) ros::spinOnce();
  
  //set_new_goal(POSTS,3.14);
  
  //ros::spin();*/
  return 0;
}
