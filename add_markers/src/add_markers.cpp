#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

ros::Publisher marker_pub;
visualization_msgs::Marker marker;
bool pickup_reached = false;
bool dropoff_reached = false;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{  
    if(pickup_reached == false){
       marker.pose.position.x = 3;
       marker.pose.position.y = 1;
       marker_pub.publish(marker);
	
       float x1 = -1;
       float y1 = 3;
       float x2 = msg->pose.pose.position.x;
       float y2 = msg->pose.pose.position.y;
       float dist = sqrt( pow(x2 - x1, 2)+ pow(y2 - y1,2) );
  
       if(dist <= 0.7 ){
		ROS_INFO("Pick up zone reached! %f", dist);
		marker.action = visualization_msgs::Marker::DELETE;
		marker_pub.publish(marker);
		pickup_reached = true;}
    }
    else if(dropoff_reached == false){
	float x1 = 5;
	float y1 = 0;
  	float x2 = msg->pose.pose.position.x;
  	float y2 = msg->pose.pose.position.y;
	float dist = sqrt( pow(x2 - x1, 2)+ pow(y2 - y1,2) );

	if(dist <= 0.7 ){
	   ROS_INFO("Drop off zone reached! %f", dist);
           marker.pose.position.x = 0;
   	   marker.pose.position.y = -5;
           marker.action = visualization_msgs::Marker::ADD;
   	   marker_pub.publish(marker);
	   dropoff_reached = true;}
     }
 }

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
 
    ros::spin();
    return 0;
}

