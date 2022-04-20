#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>      
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>


double L_encoder_pulse = 0.0;
double R_encoder_pulse = 0.0;
double alpha_enc = 3.927;
double PI = 3.141592;
double Wheel_r = 0.034;

double ang_vel = 0.0;

// Subscribe encoder pulse : topic name -> "/L_encoder_pulse", "/R_encoder_pulse"
void L_encoder_pulseCallback(const std_msgs::Float64::ConstPtr& msg)
{
    L_encoder_pulse = msg->data;

    //ROS_INFO("Encoder pulse: %.3lf", encoder_pulse);
}

void R_encoder_pulseCallback(const std_msgs::Float64::ConstPtr& msg)
{
    R_encoder_pulse = msg->data;

    //ROS_INFO("Encoder pulse: %.3lf", encoder_pulse);
}


// Subscribe imu topic -> topic name -> "/imu"
void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ang_vel = msg->angular_velocity.z;
}

// main funtion
int main(int argc, char** argv) {
    // ROS Node name: "odometry_publisher"
    ros::init(argc, argv, "sdr_odom_publisher");

    // NodeHandle Setup
	ros::NodeHandle n;

    // Odometry Publisher Setting
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);  
    tf::TransformBroadcaster odom_broadcaster;                              

    // Subscriber encoder & imu
    ros::Subscriber L_encoder_sub;
    ros::Subscriber R_encoder_sub;
    ros::Subscriber imu_sub;

    // 시작위치
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(20.0);
	while (n.ok()){
        //Callback function
        ros::spinOnce();    // 현재까지 요청된 모든 callBack함수를 모두 호출하고 코드 다음 줄로 넘어감

		current_time = ros::Time::now();

        L_encoder_sub = n.subscribe("/L_encoder_pulse", 10, L_encoder_pulseCallback); 
        R_encoder_sub = n.subscribe("/R_encoder_pulse", 10, R_encoder_pulseCallback); 
        imu_sub = n.subscribe("/imu", 20, imu_Callback);

        double vx_l = (L_encoder_pulse/10) * 2 * PI * Wheel_r;
        double vx_r = (R_encoder_pulse/10) * 2 * PI * Wheel_r;
        double vth = ang_vel;
        //ROS_INFO("Angular Velocity: %.3lf", ang_vel);

        //주행 거리 계산
        double dt = (current_time - last_time).toSec();
        
        // 1.05 = 환경에 따라 마찰과 미끄러짐을 상정한 상수
        double delta_x_l =  1.05*vx_l * cos(th) * dt;
        double delta_y_l =  1.05*vx_l * sin(th) * dt;
        double delta_x_r =  1.05*vx_r * cos(th) * dt;
        double delta_y_r =  1.05*vx_r * sin(th) * dt;

        double delta_th =  vth * dt;

        th += delta_th;
        x += (delta_x_l+delta_x_r)/2;
        y += (delta_y_l+delta_y_r)/2;
       

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);                                                                                       

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;    
		odom_trans.header.frame_id = "odom";       
		odom_trans.child_frame_id = "base_link";   

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;            
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";      
        odom.child_frame_id = "base_link";  

		//set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
        odom.twist.twist.linear.x = vx_l;
        odom.twist.twist.linear.x = vx_r;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);

		last_time = current_time;
        r.sleep();
	}
}
