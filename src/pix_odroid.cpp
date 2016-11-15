#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include <tf/transform_datatypes.h>

#include "pid.hpp"

using namespace std;

geometry_msgs::PoseStamped x_pose;
bool RESET_PID=false;

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.param(name, value, 0.0);
    return value;
}

void GetRPY(geometry_msgs::Quaternion Q, double* roll, double* pitch, double* yaw)
{
    tf::Quaternion tfq(Q.x, Q.y, Q.z, Q.w);
    tf::Matrix3x3 m(tfq);
    m.getRPY(roll[0],pitch[0],yaw[0]);
}

//#include </home/mahesh/catkin_ws/src/beginner_tutorials/src/Qualisys.h>
void x_position_Callback(const geometry_msgs::PoseStamped& CurrPose)
{
	x_pose = CurrPose;
	RESET_PID=true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_setpoints");
    ros::NodeHandle n;
 
    ros::Publisher pub_att = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",100);
    ros::Publisher pub_thr = n.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 100);
    ros::Subscriber sub_position = n.subscribe("/mavros/local_position/pose", 1000, x_position_Callback);
    ros::Rate loop_rate(100);
    ros::spinOnce();
    geometry_msgs::PoseStamped cmd_att;
    std_msgs::Float64 cmd_thr;
    int count = 1;
    double v[3]={1.0, 0.0, 0.0};
    double lambda = 0.3;
 
 
    double v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    double theta=0.0;
    
    PID m_pidX(get(n, "/pix_odroid/PIDs/X/kp"),
		      get(n, "/pix_odroid/PIDs/X/kd"),
		      get(n, "/pix_odroid/PIDs/X/ki"),
		      get(n, "/pix_odroid/PIDs/X/minOutput"),
		      get(n, "/pix_odroid/PIDs/X/maxOutput"),
		      get(n, "/pix_odroid/PIDs/X/integratorMin"),
		      get(n, "/pix_odroid/PIDs/X/integratorMax"),
		      "x");
    PID m_pidY(get(n, "/pix_odroid/PIDs/Y/kp"),
		      get(n, "/pix_odroid/PIDs/Y/kd"),
		      get(n, "/pix_odroid/PIDs/Y/ki"),
		      get(n, "/pix_odroid/PIDs/Y/minOutput"),
		      get(n, "/pix_odroid/PIDs/Y/maxOutput"),
		      get(n, "/pix_odroid/PIDs/Y/integratorMin"),
		      get(n, "/pix_odroid/PIDs/Y/integratorMax"),
		      "y");
    PID m_pidZ(get(n, "/pix_odroid/PIDs/Z/kp"),
		      get(n, "/pix_odroid/PIDs/Z/kd"),
		      get(n, "/pix_odroid/PIDs/Z/ki"),
		      get(n, "/pix_odroid/PIDs/Z/minOutput"),
		      get(n, "/pix_odroid/PIDs/Z/maxOutput"),
		      get(n, "/pix_odroid/PIDs/Z/integratorMin"),
		      get(n, "/pix_odroid/PIDs/Z/integratorMax"),
		      "z");
    PID m_pidYaw(get(n, "/pix_odroid/PIDs/Yaw/kp"),
		      get(n, "/pix_odroid/PIDs/Yaw/kd"),
		      get(n, "/pix_odroid/PIDs/Yaw/ki"),
		      get(n, "/pix_odroid/PIDs/Yaw/minOutput"),
		      get(n, "/pix_odroid/PIDs/Yaw/maxOutput"),
		      get(n, "/pix_odroid/PIDs/Yaw/integratorMin"),
		      get(n, "/pix_odroid/PIDs/Yaw/integratorMax"),
		      "yaw"); 

 
     
    while(ros::ok()){
        //Get Roll Pitch Yaw
        double roll, pitch, yaw;
        GetRPY(x_pose.pose.orientation, &roll, &pitch, &yaw);
        //Get desired
        double x_desired=0;
        double y_desired=0;
        double yaw_desired=0;
        //Convert into body frame
        double x_error_local=x_pose.pose.position.x-x_desired;
        double y_error_local=x_pose.pose.position.y-y_desired;
        double x_error_body=-cos(yaw)*x_error_local+sin(yaw)*y_error_local;
        double y_error_body=sin(yaw)*x_error_local+cos(yaw)*y_error_local;
        //PID controller
	    double roll_cmd  = m_pidY.update(0.0, y_error_body, 0);
       	double pitch_cmd = m_pidX.update(0.0, x_error_body, 0);
       	double yaw_cmd   = m_pidYaw.update(yaw, yaw_desired, 0);
       	double z_cmd     = m_pidZ.update(x_pose.pose.position.z, 1.5, 0);

        //Convert control effort to quaternion
        tf::Quaternion control_effort=tf::createQuaternionFromRPY(roll_cmd, pitch_cmd, yaw_cmd);
		
        //Create attitude command message
        cmd_att.header.stamp = ros::Time::now();
        cmd_att.header.seq=count;
        cmd_att.header.frame_id = 1;
        cmd_att.pose.position.x = 0.0;//0.0;//0.001*some_object.position_x;
        cmd_att.pose.position.y = 0.0;//0.001*some_object.position_y;
        cmd_att.pose.position.z = 0.0;//0.001*some_object.position_z;
     
        cmd_att.pose.orientation.x = control_effort[0];//sin(theta/2.0)*v[0]/v_norm;
        cmd_att.pose.orientation.y = control_effort[1];//sin(theta/2.0)*v[1]/v_norm;
        cmd_att.pose.orientation.z = control_effort[2];//sin(theta/2.0)*v[2]/v_norm;
        cmd_att.pose.orientation.w = control_effort[3];//cos(theta/2.0);

        //Create throttle command message
        cmd_thr.data = z_cmd;
       
        pub_att.publish(cmd_att);
        pub_thr.publish(cmd_thr);
        ros::spinOnce();
        count++;
        theta=0.3*sin(count/300.0);
        loop_rate.sleep();
   }
    
       
   return 0;
}
