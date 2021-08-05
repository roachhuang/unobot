#ifndef robot_hardware_interface_h
#define robot_hardware_interface_h
// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <controller_manager/controller_manager.h>

//#include <std_msgs/Int32.h>
//#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
// #include <std_msgs/Float32MultiArray.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>
//#include <math.h>

// #include <rospy_tutorials/Floats.h>
// #include <chefbot_bringup/joint_state.h>

class MyRobot: public hardware_interface::RobotHW
{
public:
	// must be no return type for constructor
	MyRobot(ros::NodeHandle &nh);
	virtual void write(ros::NodeHandle nh, const ros::Time time, const ros::Duration period);
	virtual void read(const ros::Time time, ros::Duration period);	
	float degs[2];
	// int32_t lVel;
	// int32_t rWheelEnc;
	// int32_t lWheelEnc;

protected:
	// void lwheel_cb(const std_msgs::Int32& msg);
	// void rwheel_cb(const std_msgs::Int32& msg);
	void lwheel_deg_cb(const std_msgs::Float32 &msg);
	void rwheel_deg_cb(const std_msgs::Float32 &msg);

	virtual bool init(ros::NodeHandle &nh);
	float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
	double ticks2Rad(const int32_t &ticks);	
	// hardware_interface::JointStateInterface gives read access to all joint values
	// without conflicting with other controllers.
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::VelocityJointInterface velocity_joint_interface;

	// hardware_interface::PositionJointInterface inherits from
	// hardware_interface::JointCommandInterface and is used for reading and writing
	// joint positions. Because this interface reserves the joints for write access,
	// conflicts with other controllers writing to the same joints might occure.
	// To only read joint positions, avoid conflicts using
	// hardware_interface::JointStateInterface.
	// hardware_interface::PositionJointInterface jnt_pos_interface;
	hardware_interface::EffortJointInterface effort_joint_interface;
	// effort limit
	// joint_limits_interface::JointLimits limits;
	joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
	// joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;

	// Data member array to store the controller commands which are sent to the
	// robot's resources (joints, actuators)
	// 0->joint1 (right); 1->joint2 (left)
	// coz diff_drive_controller is a velcolity contorller, input and output are vel: rad/s
	double cmd[2];
	std::string joint_name[2]={"left_wheel_joint", "right_wheel_joint"};  
	std::size_t num_joints;
	// Data member arrays to store the state of the robot's resources (joints, sensors)
	double pos[2];
	double vel[2];
	double eff[2];
	// int encoder_ticks[2];

	double ppr;	// pulses per revolution
	double r;	// wheel_radius

	ros::Publisher v_pub;
	// ros::Publisher vl_pub;

	// std_msgs::Float32MultiArray pid_msg ={};
	
	// ros::Publisher motor_cmd_pub;
	//ros::Subscriber left_encoder_sub;
	//ros::Subscriber right_encoder_sub;
	//ros::Subscriber lwheel_enc_sub;
	ros::Subscriber lwheel_deg_sub;
	ros::Subscriber rwheel_deg_sub;
	// double left_motor_pos=0,right_motor_pos=0;
    // int left_prev_cmd, right_prev_cmd;
	double rad[2];
	// ros::ServiceClient client;
	// rospy_tutorials::Floats joints_pub;
	// chefbot_bringup::joint_state joint_read;
	ros::NodeHandle nh_;
};

#endif