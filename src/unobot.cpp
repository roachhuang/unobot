// #include <ros/ros.h>
#include <mobile_robot_base/unobot.h>
// #include <controller_manager/controller_manager.h>
// h/w interface
// ROS parameter loading
// #include <rosparam_shortcuts/rosparam_shortcuts.h>

// constant to compute velocities with (mm/sec)
// const float VELOCITY_COEFF = WHEEL_DIAMETER * M_PI /ppr * 1000000.0;

// to do: figure out if really need nh_
MyRobot::MyRobot(ros::NodeHandle &nh) : nh_(nh)
{
	ROS_INFO("Initializing roachbot Hardware Interface ...");

	// num_joints_ = joint_names_.size();
	//ROS_INFO("Number of joints: %d", (int)num_joints_);

	// pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino", 10);
	//vl_pub = nh.advertise<std_msgs::Float32>("lmotor_cmd", 10);
	v_pub = nh.advertise<rospy_tutorials::Floats>("motor_cmd", 10);
	// pid_pub = nh.advertise<std_msgs::Float32MultiArray>("pid", 10);
	
	// client = nh.serviceClient<chefbot_bringup::joint_state>("/read_joint_state");

	lwheel_deg_sub = nh.subscribe("/lwheel_deg", 10, &MyRobot::lwheel_deg_cb, this);	
	rwheel_deg_sub = nh.subscribe("/rwheel_deg", 10, &MyRobot::rwheel_deg_cb, this);

	// coz cb type is a class method https://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
	// left_encoder_sub = nh.subscribe("/lwheel", 1, &MyRobot::lwheel_cb, this);
	// right_encoder_sub = nh.subscribe("/rwheel", 1, &MyRobot::rwheel_cb, this);
	nh.getParam("PPR", ppr);
	nh.getParam("wheel_radius", r);
	// num_joints = joint_name.size();
	// lVel = rVel = lRad = rRad = 0;
	// left_prev_cmd = right_prev_cmd = 0;

	// Initialize the hardware interface
	init(nh_);
	// Wait for encoder messages being published
	// isReceivingMeasuredJointStates(ros::Duration(10));

	// client = nh_.serviceClient<diff_drive::joint_state>("/read_joint_state");
	// return true;
}

bool MyRobot::init(ros::NodeHandle &nh)
{
	ROS_INFO("Initializing DiffBot Hardware Interface ...");
	// Initialization of the robot's resources (joints, sensors, actuators) and
	// interfaces can be done here or inside init().
	// E.g. parse the URDF for joint names & interfaces, then initialize them
	// Create a JointStateHandle for each joint and register them with the
	// JointStateInterface.
	for (int i = 0; i < 2; i++)
	{
		hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(jointStateHandle);

		// Create velocity joint interface
		hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &cmd[i]);
		velocity_joint_interface.registerHandle(jointVelocityHandle);

		// Create Joint Limit interface
		joint_limits_interface::JointLimits limits;
		joint_limits_interface::getJointLimits(joint_name[i], nh_, limits);
		joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
		velocityJointSaturationInterface.registerHandle(jointLimitsHandle);
		degs[i]=pos[i] = vel[i] = eff[i] = rad[i] = 0;

		/*
		// Initialize the pid controllers for the motors using the robot namespace
		std::string pid_namespace = "pid/" + motor_names[i];
		ROS_INFO_STREAM("pid namespace: " << pid_namespace);
		ros::NodeHandle nh(root_nh, pid_namespace);
		// TODO implement builder pattern to initialize values otherwise it is hard to see which parameter is what.
		pids_[i].init(nh, 0.8, 0.35, 0.5, 0.01, 3.5, -3.5, false, max_velocity_, -max_velocity_);
		pids_[i].setOutputLimits(-max_velocity_, max_velocity_);
		*/
	}

	// Register the JointEffortInterface containing the read/write joints
	// with this robot's hardware_interface::RobotHW.
	registerInterface(&jnt_state_interface);
	registerInterface(&velocity_joint_interface);
	registerInterface(&velocityJointSaturationInterface);
	ROS_INFO("... Done Initializing DiffBot Hardware Interface: %f", ppr);
	return true;
}

void MyRobot::lwheel_deg_cb(const std_msgs::Float32 &msg)
{
	//delta degrees
	degs[0] = msg.data;
	// ROS_INFO("linput Deg: %d", ticks[0]);
}

void MyRobot::rwheel_deg_cb(const std_msgs::Float32 &msg){
	
	degs[1] = msg.data;
	//ROS_INFO("rVel: %f", degs[1]);
	// ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << msg.data);
}

float MyRobot::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double MyRobot::ticks2Rad(const int32_t &ticks)
{
	// Convert number of encoder ticks to angle in radians
	double rad = (double)ticks * (2.0 * M_PI / ppr);
	return rad;
	// ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
}

void MyRobot::read(ros::Time time, ros::Duration period)
{
	double norm_rad;
	ros::Duration elapsed_time = period;
	double lastPos = pos[0];
	/*      
	double wheel_angles[2];
	double wheel_angle_deltas[2];
	for (std::size_t i = 0; i < 2; i++)
	{
		wheel_angles[i] = ticks2Rad(ticks[i]);
		wheel_angle_deltas[i] = wheel_angles[i] - pos[i];
		pos[i] += wheel_angle_deltas[i];
		vel[i] = wheel_angle_deltas[i] / period.toSec();
	}
	*/
	// pos and vel are in Radians

	for (int i = 0; i < 2; i++)
	{
		// note: no norm is required
		rad[i] += angles::from_degrees((double)degs[i]);
		// norm_rad = rad[i];
		// ticks[i] = 0; //reset it coz ticks' cb may be slower than read's loop rate
		// rad/s
		vel[i] = (rad[i] - pos[i]) / elapsed_time.toSec();
		pos[i] = rad[i];
		eff[i] = 0;
		// ROS_INFO("rad, %f", rad[i]);
	}
	ROS_INFO("left: %.2f, %.2f \t right: %.2f, %.2f", pos[0], vel[0], pos[1], vel[1]);
}

// to do: stop motor if no push from twist_to_motor, otherwise pid goes crazy
void MyRobot::write(ros::NodeHandle nh, ros::Time time, ros::Duration period)
{
	rospy_tutorials::Floats v;
	// std_msgs::Float32 vl;
	// std_msgs::Float32MultiArray pid_msg;

	// velocityJointSaturationInterface.enforceLimits(period);

	// cmd is in rad/s
	// ROS_INFO("cmd, %f", cmd[0]);

	// convert radian to degree and send it to arduino
	// rad x 57.2958 = degrees
	v.data.clear();
	
	for(int i = 0; i < 2; i++){
		v.data.push_back((float)angles::to_degrees(cmd[i]));
		// velocity = mapFloat(velocity, -347, 347, -255, 255);
		// ROS_INFO("vel: %f", v.data[i]);
	}
	v_pub.publish(v);
	
	/* velocity = (int)cmd[1];
	velocity = (int)angles::to_degrees(cmd[1]);
	// ROS_INFO("cmd[0]=%d velocity=%d, cmd_[0], velocity);
	if (right_prev_cmd != velocity)
	{
		vr.data = velocity;
		vr_pub.publish(vr);
		//ROS_INFO("Writen successfully result=%d", result);
		right_prev_cmd = velocity;
	}
	*/
}

/*
double MyRobot::linearToAngular(const double &distance) 
{
	return distance / r;
}

double MyRobot::angularToLinear(const double &angle) 
{
	return angle * r;
}
*/

/* pid pseudo code
int pid()
{
	while (1)
	{
		err = setpoint - input;
		integral = integral + err * dt;
		derivative = (err - prev_err) / dt;
		output = Kp * err + Ki * integral + Kd * derivative;
		prev_err = err;
		wait(dt);
	}
}
*/

int main(int argc, char **argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "unobot_hardware_interface");

	// Create an instance of your robot so that this instance knows about all
	// the resources that are available.
	ros::NodeHandle nh;
	MyRobot robot = MyRobot(nh);

	// Create an instance of the controller manager and pass it the robot,
	// so that it can handle its resources.
	controller_manager::ControllerManager cm(&robot);

	// Setup a separate thread that will be used to service ROS callbacks.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Multiple threads for controller service cb & for the Service client cb used to get the feedback from ardiuno
	//ros::MultiThreadedSpinner spinner(2);
	//spinner.spin();

	// Setup for the control loop.
	ros::Time prev_time = ros::Time::now();
	ros::Rate rate(10.0); // 10 Hz rate

	while (ros::ok())
	{

		// Basic bookkeeping to get the system time in order to compute the control period.
		const ros::Time time = ros::Time::now();
		const ros::Duration period = time - prev_time;
		prev_time = time;
		// Execution of the actual control loop.		

		robot.read(time, period);
		// If needed, its possible to define transmissions in software by calling the
		// transmission_interface::ActuatorToJointPositionInterface::propagate()
		// after reading the joint states.
		cm.update(time, period);
		// In case of software transmissions, use
		// transmission_interface::JointToActuatorEffortHandle::propagate()
		// to convert from the joint space to the actuator space.
		robot.write(nh, time, period);

		// All these steps keep getting repeated with the specified rate.
		rate.sleep();
	}
	return 0;
}
