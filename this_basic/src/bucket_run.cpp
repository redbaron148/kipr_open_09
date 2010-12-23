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
  //begin code execution
  
  cmvision::Blob b;
  sap(SWIPE_START_POSITION);
  //set_new_pose(HOME, 1.57);
  
  wait_for_light();
  std::cout << "starting node\n";
  START_TIME = ros::Time::now();
  
  ros::Timer timmer = n.createTimer(ros::Duration(177.),timer_callback);
  
  set_new_state(2);
  set_new_vel(-1,0);
  ssp(WRIST_FLEX,40);
  set_new_vel(-1,0);
  set_new_vel(-1,0);
  usleep(500000);
  set_new_vel(1,0);
  set_new_vel(1,0);
  set_new_vel(-1,-1);
  set_new_vel(-1,-1);
  set_new_vel(-1,-1);
  sap(SWIPE_MIDDLE);
  usleep(2100000);
  set_new_vel(1,1);
  set_new_vel(1,1);
  set_new_vel(-1,0);
  set_new_vel(-1,0);
  set_new_vel(-1,0);
  usleep(550000);
  set_new_vel(0,0);
  set_new_state(0);
  ssp(BASE,-20);
  turn_right(60.);
  sleep(30);
  
  sap(SWIPE_DOWN);
  ssp(CLAW,OPEN);
  ssp(WRIST_TWIST,90);
  ssp(WRIST_TWIST,-90);
  
  sleep(1);
  
  sap(REST_POSITION);

  set_new_state(2);
  set_new_vel(1,0);
  sleep(1);
  set_new_vel(0,0);
  
  ssp(WRIST_FLEX,40);
  turn_right(110.);
  drive_forward(7.);
  turn_left(85.);
  drive_forward(25.);
  set_new_vel(1,0);
  usleep(500000);
  set_new_vel(0,0);
  ssp(WRIST_FLEX,0);
  //drive_backward(1);
  
  while(game_time() < GAME_DURATION && ros::ok())
  {
    b = center_on_blob("bottom");
    if(grab_blob_bucket_run(b)) 
    {
      score_object_bucket_run();
    }
  }
  
  return 0;
}
