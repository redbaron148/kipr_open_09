#include <ros/ros.h>
#include <this_basic/GetBlob.h>
#include <this_basic/GetBlobs.h>
#include "this_defines.h"
#include <cmvision/Blobs.h>
#include <../../crustcrawler_arm/srv/cpp/crustcrawler_arm/SetPosition.h>
#include <../../crustcrawler_arm/srv/cpp/crustcrawler_arm/SetServoPosition.h>
#include <../../crustcrawler_arm/srv/cpp/crustcrawler_arm/GetServoPosition.h>
#include <../../crustcrawler_arm/srv/cpp/crustcrawler_arm/SetAllServoPositions.h>

#define DEBUG_MODE 1

ros::ServiceClient blob_client;
ros::ServiceClient blobs_client;
ros::ServiceClient set_position_client;
ros::ServiceClient set_servo_position_client;
ros::ServiceClient get_servo_position_client;
ros::ServiceClient set_all_servo_positions_client;

float distance_from_middle(const cmvision::Blob &b);
float degrees_from_middle(const cmvision::Blob &b);
float calculate_distance_from_robot(const cmvision::Blob &b);
bool blob_is_valid(const cmvision::Blob &b);
bool blob_is_centered(const cmvision::Blob &b);
bool blob_is_centered_arm(const cmvision::Blob &b);
cmvision::Blob& center_on_blob(std::string type);
bool set_position(float x, float y, float z, float attack_angle);
bool set_position(float x, float y, float z);
bool sap(float value_0, float value_1, float value_2, float value_3, float value_4, float value_5);
bool grab_blob(const cmvision::Blob &b);

cmvision::Blob getBlob(std::string request);
cmvision::Blobs getBlobs();

bool ssp(int s, float degrees);
float gsp(int s);

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

  //begin code execution
  
  cmvision::Blob b = getBlob("size");
  
  //ssp(BASE,30);
  //ssp(BASE,-30);
  ssp(CLAW,100);
    b = center_on_blob("center");
    //sleep(1);
    //b = getBlob("size");
  //}
  std::cout << "distance from robot = " << calculate_distance_from_robot(b) << std::endl;
  std::cout << "grab_blob returned " << grab_blob(b) << std::endl;
  //sleep(1);
  //ssp(CLAW,0);
  //sap(0,45,100,0,65,0);
  //ssp(BASE,-60);
  //ssp(BASE,0);
  //set_position(0,calculate_distance_from_robot(b),3);
  //sleep(1);
  //for(int i = 20;i>=10;i--)
    //set_position(0,i,3);
  //sleep(3);
  //ssp(CLAW,0);
  //usleep(50000);
  sap(0,0,gsp(CLAW),0,0,0);
  
  return 0;
}

bool grab_blob(const cmvision::Blob &b)
{
  if(blob_is_valid(b) && blob_is_centered(b))
  {
    float dist = calculate_distance_from_robot(b)+2;
    float angle = gsp(BASE)*(PIE/180.);
    float x = -dist*sin(angle);
    float y = dist*cos(angle);
    float z = 10;
  
    ssp(CLAW,100);
  
    ROS_INFO("set position to x=%f, y=%f", x, y);
  
    set_position(x,y,z);
    usleep(200000);
    set_position(x,y,z-6);
  
    ssp(CLAW,0);
  
    return true;
  }
  return false;
}

float distance_from_middle(const cmvision::Blob &b)
{
  return abs(b.x-CENTER_X);
}

float degrees_from_middle(const cmvision::Blob &b)
{
  return (((float)IMAGE_FOV/2.) - (float)IMAGE_FOV/(float)IMAGE_WIDTH*(float)b.x);
}

float degrees_from_arm(const cmvision::Blob &b)
{
  return (((float)IMAGE_FOV/2.) - (float)IMAGE_FOV/(float)IMAGE_WIDTH*(float)b.x);
}

float calculate_distance_from_robot(const cmvision::Blob &b)
{
  return 31.342718265*pow(0.99555,b.y);
}

bool blob_is_valid(const cmvision::Blob &b)
{
  std::cout << "blob validity = " << (b.x != -1) << std::endl;
  return (b.x != -1);
}

bool blob_is_centered_arm(const cmvision::Blob &b)
{
  return (b.x < ARM_CENTER_X+CENTER_X_OFFSET && b.x > ARM_CENTER_X-CENTER_X_OFFSET);
}

bool blob_is_centered(const cmvision::Blob &b)
{
  std::cout << "blob centered = " << (b.x < CENTER_X+CENTER_X_OFFSET && b.x > CENTER_X-CENTER_X_OFFSET) << std::endl;
  return (b.x < CENTER_X+CENTER_X_OFFSET && b.x > CENTER_X-CENTER_X_OFFSET);
}

/*cmvision::Blob& center_on_blob(std::string type)
{
  cmvision::Blob b = getBlob(type);

  int start_position = gsp(BASE);
  int new_position = 0;
  int current_position = gsp(BASE);
  
  while(blob_is_valid(b) && !blob_is_centered_arm(b))
  {
    #if(DEBUG_MODE)
    ROS_INFO("blob x = %d  degrees from center = %f",b.x,degrees_from_arm(b)/CAMERA_P);
    #endif
    new_position = current_position+degrees_from_middle(b)/CAMERA_P;
    ssp(BASE,new_position);
    current_position = new_position;
    
    b = getBlob(type);
  }
  
  if(!blob_is_valid(b))
  {
    ssp(BASE,start_position);

    b = getBlob(type);
    if(!blob_is_valid(b))
      return b;
  }
  
  return b;
}*/

cmvision::Blob& center_on_blob(std::string type)
{
  cmvision::Blob b = getBlob(type);

  int start_position = gsp(BASE);
  int new_position = 0;
  int current_position = gsp(BASE);
  
  while(blob_is_valid(b) && !blob_is_centered(b))
  {
    #if(DEBUG_MODE)
    ROS_INFO("blob x = %d  degrees from center = %f",b.x,degrees_from_middle(b)/CAMERA_P);
    #endif
    new_position = current_position+degrees_from_middle(b)/CAMERA_P;
    ssp(BASE,new_position);
    current_position = new_position;
    
    b = getBlob(type);
  }
  
  if(!blob_is_valid(b))
  {
    ssp(BASE,start_position);

    b = getBlob(type);
    if(!blob_is_valid(b))
      return b;
  }
  
  return b;
}

cmvision::Blobs getBlobs()
{
  this_basic::GetBlobs srv;
  if(blobs_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("recieved blobs from get_blobs service");
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("get_blobs service did not return the blobs");
    #endif
  }
  return srv.response.blobs;
}

cmvision::Blob getBlob(std::string request)
{
  this_basic::GetBlob srv;
  srv.request.type = request;
  if(blob_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("recieved blob from get_blob service");
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("get_blob service did not return blob");
    #endif
    srv.response.blob.x=-1;
  }
  return srv.response.blob;
}

bool set_position(float x, float y, float z, float attack_angle)
{
  crustcrawler_arm::SetPosition srv;
  srv.request.x = x;
  srv.request.y = y;
  srv.request.z = z;
  srv.request.attack_angle = attack_angle;
  if(set_position_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("set arm to position x=%f, y=%f, z=%f, a=%f",x,y,z,attack_angle);
    usleep(400000);
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_arm_position service did not work");
    #endif
  }
  return srv.response.success;
}

bool ssp(int s, float degrees)
{
  crustcrawler_arm::SetServoPosition srv;
  srv.request.servo_index = s;
  srv.request.value = degrees;
  float old_pos = gsp(s);
  if(set_servo_position_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("set %s servo to %f degrees, time slept = %f,old servo pos = %f",servo[s].c_str(),degrees,(SERVO_SPEED*abs(old_pos-degrees)/60.),old_pos);
    #endif
    usleep(SERVO_SPEED*abs(old_pos-degrees)/7.*100000);
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_servo_position service did not work");
    #endif
  }
  return srv.response.success;
}

bool set_position(float x, float y, float z)
{
  float angle = -90.;
  float distance = sqrt((x*x)+(y*y));
  float dst_ratio = (distance-MIN_ARM_REACH)/(MAX_ARM_REACH-MIN_ARM_REACH);
  angle *= dst_ratio;
  if(angle < -60.) angle = -60.;
  
  return set_position(x,y,z,(-90.)-angle);
}

bool sap(float value_0, float value_1, float value_2, float value_3, float value_4, float value_5)
{
  crustcrawler_arm::SetAllServoPositions srv;
  srv.request.value_0 = value_0;
  srv.request.value_1 = value_1;
  srv.request.value_2 = value_2;
  srv.request.value_3 = value_3;
  srv.request.value_4 = value_4;
  srv.request.value_5 = value_5;
  if(set_all_servo_positions_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("set all servo positions");
    usleep(600000);
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_all_servo_positions service did not work");
    #endif
  }
  return srv.response.success;
}

float gsp(int s)
{
  crustcrawler_arm::GetServoPosition srv;
  srv.request.servo_index = s;
  if(get_servo_position_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("get %s servo degrees",servo[s].c_str());
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_servo_position service did not work");
    #endif
  }
  return srv.response.value;
}
