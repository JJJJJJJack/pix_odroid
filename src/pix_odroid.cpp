#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <tf/transform_datatypes.h>

#include "pid.hpp"

//Define TEST_COMM to test communication
#define TEST_COMM

#define LOOP_RATE      100
#define IDLE_STATE     0
#define TAKEOFF_STATE  1
#define LANDING_STATE  2
#define AUTOMATIC      4
#define RC_MAX         1893
#define RC_MIN         1092
#define RC_MID         ((RC_MAX+RC_MIN)/2.0)
#define RC_RANGE       (RC_MAX-RC_MIN)

using namespace std;

geometry_msgs::PoseStamped x_pose;
mavros_msgs::State x_current_state;
mavros_msgs::RCIn x_RCIn;
geometry_msgs::PoseStamped x_goal;
bool RESET_PID=false, KILL_SWITCH=false;
double x_thrust=0, x_landing_height=0;
int x_state=IDLE_STATE;

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

//Callback functions
void x_position_Callback(const geometry_msgs::PoseStamped& CurrPose)
{
	x_pose = CurrPose;
	RESET_PID=true;
}
void state_Callback(const mavros_msgs::State& CurrState)
{
	x_current_state = CurrState;
}
void RCIn_Callback(const mavros_msgs::RCIn& CurrRCIn)
{
	x_RCIn = CurrRCIn;
}
void goal_Callback(const geometry_msgs::PoseStamped& Currgoal)
{
	x_goal = Currgoal;
}


//Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pix_odroid");
    ros::NodeHandle n;
    //Publish topics
    ros::Publisher pub_att = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude",100);
    ros::Publisher pub_thr = n.advertise<std_msgs::Float64>("mavros/setpoint_attitude/att_throttle", 100);
    //Subscribe topics
    ros::Subscriber sub_position = n.subscribe("mavros/local_position/pose", 1000, x_position_Callback);
    ros::Subscriber sub_state    = n.subscribe("mavros/state", 1000, state_Callback);
    ros::Subscriber sub_RCIn     = n.subscribe("mavros/rc/in", 1000, RCIn_Callback);
    ros::Subscriber sub_goal     = n.subscribe("goal", 1000, goal_Callback);
    //Service list
    ros::ServiceClient arming_client   = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate loop_rate(LOOP_RATE);
    ros::spinOnce();
    
    //Local variables
    geometry_msgs::PoseStamped cmd_att;
    std_msgs::Float64 cmd_thr;
    int count = 1;
    mavros_msgs::SetMode x_offb_set_mode;
    x_offb_set_mode.request.custom_mode="OFFBOARD";
    mavros_msgs::CommandBool x_arm_cmd;
    x_arm_cmd.request.value=true;
    
    PID x_pidX(get(n, "/pix_odroid/PIDs/X/kp"),
		      get(n, "/pix_odroid/PIDs/X/kd"),
		      get(n, "/pix_odroid/PIDs/X/ki"),
		      get(n, "/pix_odroid/PIDs/X/minOutput"),
		      get(n, "/pix_odroid/PIDs/X/maxOutput"),
		      get(n, "/pix_odroid/PIDs/X/integratorMin"),
		      get(n, "/pix_odroid/PIDs/X/integratorMax"),
		      "x");
    PID x_pidY(get(n, "/pix_odroid/PIDs/Y/kp"),
		      get(n, "/pix_odroid/PIDs/Y/kd"),
		      get(n, "/pix_odroid/PIDs/Y/ki"),
		      get(n, "/pix_odroid/PIDs/Y/minOutput"),
		      get(n, "/pix_odroid/PIDs/Y/maxOutput"),
		      get(n, "/pix_odroid/PIDs/Y/integratorMin"),
		      get(n, "/pix_odroid/PIDs/Y/integratorMax"),
		      "y");
    PID x_pidZ(get(n, "/pix_odroid/PIDs/Z/kp"),
		      get(n, "/pix_odroid/PIDs/Z/kd"),
		      get(n, "/pix_odroid/PIDs/Z/ki"),
		      get(n, "/pix_odroid/PIDs/Z/minOutput"),
		      get(n, "/pix_odroid/PIDs/Z/maxOutput"),
		      get(n, "/pix_odroid/PIDs/Z/integratorMin"),
		      get(n, "/pix_odroid/PIDs/Z/integratorMax"),
		      "z");
    PID x_pidYaw(get(n, "/pix_odroid/PIDs/Yaw/kp"),
		      get(n, "/pix_odroid/PIDs/Yaw/kd"),
		      get(n, "/pix_odroid/PIDs/Yaw/ki"),
		      get(n, "/pix_odroid/PIDs/Yaw/minOutput"),
		      get(n, "/pix_odroid/PIDs/Yaw/maxOutput"),
		      get(n, "/pix_odroid/PIDs/Yaw/integratorMin"),
		      get(n, "/pix_odroid/PIDs/Yaw/integratorMax"),
		      "yaw");
     
    
    //Wait for Fight Control Connection
    while(ros::ok() && x_current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ros::Time last_request = ros::Time::now(); 
    while(ros::ok()){
        //First set the flight mode as OFFBOARD
        if(x_current_state.mode != "OFFBOARD" && (ros::Time::now()-last_request>ros::Duration(5.0)) &&
            KILL_SWITCH==false){
            if(set_mode_client.call(x_offb_set_mode) && x_offb_set_mode.response.success)
                ROS_INFO("Offboard Mode Enabled");
            last_request = ros::Time::now(); 
        }else{
            if(!x_current_state.armed && (ros::Time::now()-last_request>ros::Duration(5.0)) && 
                KILL_SWITCH==false){
                if(arming_client.call(x_arm_cmd) && x_arm_cmd.response.success)
                    ROS_INFO("Robot Armed");
                last_request = ros::Time::now(); 
            }
        }
        
        //Takeoff and landing switch based on RCIn
        int roll_RC    = x_RCIn.channels[0];
        int pitch_RC   = x_RCIn.channels[1];
        int throttle_RC= x_RCIn.channels[2];
        int yaw_RC     = x_RCIn.channels[3];
        if(roll_RC<1100 && pitch_RC<1100 && throttle_RC<1100 && yaw_RC>1800){
            x_state=TAKEOFF_STATE;
        }
        if(roll_RC>1800 && pitch_RC<1100 && throttle_RC<1100 && yaw_RC>1800){
            x_state=LANDING_STATE;
        }
        //Kill the quad in the air
        if(yaw_RC==RC_MIN && throttle_RC==RC_MIN){
            mavros_msgs::SetMode x_manual_set_mode;
            x_manual_set_mode.request.custom_mode="MANUAL";
            set_mode_client.call(x_manual_set_mode);
            //Set kill switch to true to prevent re-arm
            KILL_SWITCH=true;
            x_state=IDLE_STATE;
            ROS_INFO("Emergency Kill");
        }
        //Save the quad in manual control need extra input
        
        //Control command
        double roll_cmd=0, pitch_cmd=0, yaw_cmd=0, z_cmd=0;

        //Get Roll Pitch Yaw
        double roll, pitch, yaw;
        GetRPY(x_pose.pose.orientation, &roll, &pitch, &yaw);
        //Get desired
        double x_desired=x_goal.pose.position.x;
        double y_desired=x_goal.pose.position.y;
        double z_desired=x_goal.pose.position.z;
        double yaw_desired=0;
        //Convert into body frame
        double x_error_local=x_pose.pose.position.x-x_desired;
        double y_error_local=x_pose.pose.position.y-y_desired;
        double x_error_body=-cos(yaw)*x_error_local+sin(yaw)*y_error_local;
        double y_error_body=sin(yaw)*x_error_local+cos(yaw)*y_error_local;
        
        switch(x_state){
            case TAKEOFF_STATE:{
                #ifdef TEST_COMM
                x_state=AUTOMATIC;
                #endif
       	        //Take off logic
                if(x_pose.pose.position.z>=0.05 || x_thrust>=0.9){
                    x_state=AUTOMATIC;
                    x_pidZ.setIntegral(x_thrust / x_pidZ.ki());
                    x_thrust = 0;
                    ROS_INFO("Takeoff Successfully! Automatic Control Now...");
                }else{
                    x_thrust+=0.1/LOOP_RATE;
                    z_cmd = x_thrust;
                }
            }break;
            case LANDING_STATE:{
                //Landing logic
                if(x_pose.pose.position.z>0.03){
                    x_landing_height-=0.5/LOOP_RATE;
                    z_desired+=x_landing_height;
                }else{
                    x_state=IDLE_STATE;
                }
            } //Intentional fall-thru
            case AUTOMATIC:{
                //PID controller
	            roll_cmd  = x_pidY.update(0.0, y_error_body, 0);
       	        pitch_cmd = x_pidX.update(0.0, x_error_body, 0);
               	yaw_cmd   = x_pidYaw.update(yaw, yaw_desired, 0);
               	z_cmd     = x_pidZ.update(x_pose.pose.position.z, z_desired, 0);
               	//RCIn Mixer
               	double remap_rc_roll = (roll_RC-RC_MID)/(RC_RANGE/2.0);
               	double remap_rc_pitch= (pitch_RC-RC_MID)/(RC_RANGE/2.0);
               	double remap_rc_yaw  = (yaw_RC-RC_MID)/(RC_RANGE/2.0);
               	roll_cmd  += remap_rc_roll;
               	pitch_cmd += remap_rc_pitch;
               	yaw_cmd   += remap_rc_yaw;
               	//Cancel PID input while test communivation
               	#ifdef TEST_COMM
               	    double remap_rc_throttle  = (throttle_RC-RC_MIN)/RC_RANGE;
                   	roll_cmd  = remap_rc_roll;
                   	pitch_cmd = remap_rc_pitch;
                   	yaw_cmd   = remap_rc_yaw;
                   	z_cmd     = remap_rc_throttle;
                #endif
            }break;
            case IDLE_STATE:{
                roll_cmd  = 0;
       	        pitch_cmd = 0;
               	yaw_cmd   = 0;
               	z_cmd     = 0;
            }break;
        }

        //Convert control effort to quaternion
        tf::Quaternion control_effort=tf::createQuaternionFromRPY(roll_cmd, pitch_cmd, yaw_cmd);
		
        //Create attitude command message
        cmd_att.header.stamp = ros::Time::now();
        cmd_att.header.seq=count;
        cmd_att.header.frame_id = 1;
        cmd_att.pose.position.x = 0.0;
        cmd_att.pose.position.y = 0.0;
        cmd_att.pose.position.z = 0.0;
        cmd_att.pose.orientation.x = control_effort[0];
        cmd_att.pose.orientation.y = control_effort[1];
        cmd_att.pose.orientation.z = control_effort[2];
        cmd_att.pose.orientation.w = control_effort[3];

        //Create throttle command message
        cmd_thr.data = z_cmd;
       
        pub_att.publish(cmd_att);
        pub_thr.publish(cmd_thr);
        //Remap local position to pose for trajectory_generate and record FIXME
        ros::spinOnce();
        count++;
        if(count%100 == 0)
            cerr<<".";
        loop_rate.sleep();
   }
    
       
   return 0;
}
