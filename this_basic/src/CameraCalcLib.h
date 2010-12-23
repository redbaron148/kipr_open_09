#ifndef CAMERA_CALC_LIB_H
#define CAMERA_CALC_LIB_H

#include <ros/ros.h>
#include <this_basic/GetBlob.h>
#include <this_basic/GetBlobs.h>
#include "this_defines.h"
#include <cmvision/Blobs.h>

ros::ServiceClient blob_client;
ros::ServiceClient blobs_client;

float distance_from_middle(const cmvision::Blob &b);
float degrees_from_middle(const cmvision::Blob &b);
float calculate_distance_from_robot(const cmvision::Blob &b);
bool blob_is_valid(const cmvision::Blob &b);
bool blob_is_centered(const cmvision::Blob &b);
bool blob_is_centered_arm(const cmvision::Blob &b);
int what_is_blob(const cmvision::Blob &b);

cmvision::Blob getBlob(std::string request);
cmvision::Blobs getBlobs();

bool blob_is_pom(const cmvision::Blob &b)
{
  return (b.area < MAX_POM_SIZE && 
         (b.channel == orange || b.channel == greenFoam || b.channel == yellow) &&
         (b.right-b.left < MAX_POM_WIDTH));
}

/*
 * returns the number of what the blob b is
 */
int what_is_blob(const cmvision::Blob &b)
{
  if(b.channel == orange || b.channel == greenFoam)
  {
    if(b.area < MAX_POM_SIZE)
    {
      if(b.channel == orange)
        return orangePom;
      else return greenPom;
    }
    else 
    {
      if(b.channel == orange)
        return orangeFoam;
      else return greenFoam;
    }
  }
  return b.channel;
}

/*
 * returns the number of pixels from the center of the image to the center of 
 * the blob passed.
 */
float distance_from_middle(const cmvision::Blob &b)
{
  return abs(b.x-CENTER_X);
}

/*
 *  returns the degrees from the middle the blob is estimated to be at.
 */
float degrees_from_middle(const cmvision::Blob &b)
{
  return (((float)IMAGE_FOV/2.) - (float)IMAGE_FOV/(float)IMAGE_WIDTH*(float)b.x);
}

/*
 * returns the degrees form the middle (as defined by the center of the arm) the
 * blob is estimated to be at.
 */
/*float degrees_from_arm(const cmvision::Blob &b)
{
  return ;
}*/

/*
 * estimates the distance the blob is from the robot. function is derived from 
 * testing scenarios when the arm is on and set to position 0,0,0,0,0,0
 */
float calculate_distance_from_robot(const cmvision::Blob &b)
{
  if(!blob_is_pom(b))
    return 33.14*pow(0.9958,b.y);
  else
    return 33.61*pow(0.995971,b.y);
}

/*
 * returns if the given blob is valid or not (if x is set to -1).
 */
bool blob_is_valid(const cmvision::Blob &b)
{
  //std::cout << "blob validity = " << (b.x != -1) << std::endl;
  return ((int)b.x != -1);
}

/*
 * returns if the blob is centered relative to the arm.
 */
bool blob_is_centered_arm(const cmvision::Blob &b)
{
  return (b.x < ARM_CENTER_X+CENTER_X_OFFSET && b.x > ARM_CENTER_X-CENTER_X_OFFSET);
}

/*
 * returns if the blob is centered in the center of the image
 */
bool blob_is_centered(const cmvision::Blob &b)
{
  //std::cout << "blob centered = " << (b.x < CENTER_X+CENTER_X_OFFSET && b.x > CENTER_X-CENTER_X_OFFSET) << std::endl;
  return (b.x < CENTER_X+CENTER_X_OFFSET && b.x > CENTER_X-CENTER_X_OFFSET);
}

/*
 * returns a cmvision blobs object (array of blob)
 */
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

/*
 * returns a cmvision blob
 */
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

cmvision::Blob get_biggest_blob(int ch)
{
  cmvision::Blobs bs = getBlobs();
  cmvision::Blob b;
  b.x = -1;
  //std::cout << "i can see " << bs.blob_count << " blobs\n";
  for(int i = bs.blob_count-1;i >= 0;i--)
  {
    if(bs.blobs[i].channel == ch && bs.blobs[i].area > b.area)
      b = bs.blobs[i];
      //std::cout << "one blob is " << channel[(int)bs.blobs[i].channel] << std::endl;
  }
  return b;
}

bool color_is_present(int channel)
{
  cmvision::Blob b = get_biggest_blob(channel);
  return blob_is_valid(b);
}

cmvision::Blob get_lowest_blob()
{
  cmvision::Blobs bs = getBlobs();
  int choice = 0;
  for(int i = bs.blob_count-1; i > 0; i--)
  {
    if(bs.blobs[i].bottom >= bs.blobs[choice].bottom)
      choice = i;
  }
  return bs.blobs[choice];
}

#endif
