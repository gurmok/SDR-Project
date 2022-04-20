/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <stdlib.h>
using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int a = 0;

    int count = scan->scan_time / scan->time_increment;
    printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	if(degree > -5 && degree< 5) {
        printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);
        a++;
    };
    if(a == count) {
        double x1 = odom->pose.pose.position.x;
        double y1 = odom->pose.pose.position.y;
        double z1 = odom->pose.pose.position.z;
        double w1 = odom->pose.pose.orientation.w;
        string x2 = to_string(x1);
        string y2 = to_string(y1);
        string z2 = to_string(z1);
        string w2 = to_string(w1);        
        string string1("rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: ");
        string string2(", y: ");
        string string3(", z: ");
        string string4("}, orientation: {w: ");
        string string5("}}}'");
        string1 += x2;
        string1 += string2;
        string1 += y2;
        string1 += string3;
        string1 += z2;
        string1 += string4;
        string1 += w2;
        string1 += string5;
        system(string1);              //이게 아닌데...
    };
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ydlidar_client");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::spin();

    return 0;
}
