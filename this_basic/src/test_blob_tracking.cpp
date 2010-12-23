#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <cmath>
#include <this_basic/GetBlob.h>
#include <this_basic/GetBlobs.h>
#include "this_defines.h"

using namespace std;

int largest_blob(const cmvision::BlobsConstPtr& msg);
int distance_from_middle(const cmvision::Blob msg);
int middle_blob(const cmvision::BlobsConstPtr& msg);
int right_blob(const cmvision::BlobsConstPtr& msg);
int left_blob(const cmvision::BlobsConstPtr& msg);

cmvision::Blobs blobs;

int largest_blob()
{
  if(blobs.blob_count < 2)
    return blobs.blob_count-1;
  int j = 0;
  for(int i = blobs.blob_count-1;i > 0;i--)
  {
    if(blobs.blobs[i].area > blobs.blobs[j].area)
      j = i;
  }
  return j;
}

int left_blob()
{
  if(blobs.blob_count < 2)
    return blobs.blob_count-1;
  int j = 0;
  for(int i = blobs.blob_count-1;i > 0;i--)
  {
    if(blobs.blobs[i].x < blobs.blobs[j].x)
      j = i;
  }
  return j;
}

int right_blob()
{
  if(blobs.blob_count < 2)
    return blobs.blob_count-1;
  int j = 0;
  for(int i = blobs.blob_count-1;i > 0;i--)
  {
    if(blobs.blobs[i].x > blobs.blobs[j].x)
      j = i;
  }
  return j;
}

int middle_blob()
{
  if(blobs.blob_count < 2)
    return blobs.blob_count-1;
  int j = 0;
  for(int i = blobs.blob_count-1;i > 0;i--)
  {
    if(distance_from_middle(blobs.blobs[i]) < distance_from_middle(blobs.blobs[j]))
      j = i;
  }
  return j;
}

int bottom_blob()
{
  if(blobs.blob_count < 2)
    return blobs.blob_count-1;
  int j = 0;
  for(int i = blobs.blob_count-1;i > 0;i--)
  {
    if(blobs.blobs[i].bottom > blobs.blobs[j].bottom)
      j = i;
  }
  return j;
}

int distance_from_middle(const cmvision::Blob msg)
{
  return abs((long int)msg.x-160);
}

bool get_blobs(this_basic::GetBlobs::Request &req,
                       this_basic::GetBlobs::Response &res)
{
  res.blobs = blobs;
  return true;
}

bool get_blob(this_basic::GetBlob::Request  &req,
              this_basic::GetBlob::Response &res )
{
  if(blobs.blob_count != 0)
  {
    if(req.type == "left")
    {
      res.blob = blobs.blobs[left_blob()];
      //ROS_INFO("LEFT!!!");
    }
    else if(req.type == "right")
    {
      res.blob = blobs.blobs[right_blob()];
      //ROS_INFO("RIGHT!!!");
    }
    else if(req.type == "middle" || req.type == "center")
    {
      res.blob = blobs.blobs[middle_blob()];
      //ROS_INFO("middle...");
    }
    else if(req.type == "size" || req.type == "largest" || req.type == "biggest")
    {
      res.blob = blobs.blobs[largest_blob()];
      //ROS_INFO("SIZE!!!!");
    }
    else if(req.type == "bottom" || req.type == "lowest")
    {
      res.blob = blobs.blobs[bottom_blob()];
    }
    else return false;
    return true;
  }
  return false;
}

void chatterCallback(const cmvision::BlobsConstPtr& msg)
{
  blobs.blob_count = msg->blob_count;
  blobs.blobs = msg->blobs;
  blobs.header = msg->header;
  blobs.image_width = msg->image_width;
  blobs.image_height = msg->image_height;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blob_listener");
  ros::NodeHandle n;
  ros::Subscriber chatter_sub = n.subscribe("blobs", 1, chatterCallback);
  ros::ServiceServer get_blob_service = n.advertiseService("get_blob", get_blob);
  ros::ServiceServer get_blobs_service = n.advertiseService("get_blobs",get_blobs);
  ros::Rate loop_rate(7);
  ros::spin();
}
