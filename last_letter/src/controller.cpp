#include "controller.hpp"

///////////////////////////////////
// Define attitude controller class
///////////////////////////////////

	///////////////////
	//Class Constructor
	BMcLAttitudeController::BMcLAttitudeController(ros::NodeHandle n)
	{
		//Initialize states
		tprev = ros::Time::now();
		states.header.stamp = tprev;
		
		//Subscribe and advertize
		subInp = n.subscribe("rawPWM",1,&BMcLAttitudeController::getInput, this);
		subState = n.subscribe("states",1,&BMcLAttitudeController::getStates, this);
		subEnv = n.subscribe("environment",1,&BMcLAttitudeController::getEnvironment, this);
		subRef = n.subscribe("refCommands",1,&BMcLAttitudeController::getReference, this);
		pubCtrl = n.advertise<last_letter::SimPWM>("ctrlPWM",1000);
		
		//Create roll to aileron controller
		if(!ros::param::getCached("roll2ail/p", P)) {ROS_FATAL("Invalid parameters for -roll2ail/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("roll2ail/i", I)) {ROS_FATAL("Invalid parameters for -roll2ail/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("roll2ail/d", D)) {ROS_FATAL("Invalid parameters for -roll2ail/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("roll2ail/max", satU)) {ROS_FATAL("Invalid parameters for -roll2ail/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("roll2ail/min", satL)) {ROS_FATAL("Invalid parameters for -roll2ail/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		N = 10;
		roll2Aileron = new PID(P, I, D, satU, satL, Ts, N);
		
		//Create roll to aileron controller
		if(!ros::param::getCached("yaw2roll/p", P)) {ROS_FATAL("Invalid parameters for -roll2ail/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/i", I)) {ROS_FATAL("Invalid parameters for -roll2ail/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/d", D)) {ROS_FATAL("Invalid parameters for -roll2ail/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/max", satU)) {ROS_FATAL("Invalid parameters for -roll2ail/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/min", satL)) {ROS_FATAL("Invalid parameters for -roll2ail/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		N = 10;
		yaw2Roll = new PID(P, I, D, satU, satL, Ts, N);
		
		//Create roll to aileron controller
		if(!ros::param::getCached("beta2rud/p", P)) {ROS_FATAL("Invalid parameters for -beta2rud/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("beta2rud/i", I)) {ROS_FATAL("Invalid parameters for -beta2rud/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("beta2rud/d", D)) {ROS_FATAL("Invalid parameters for -beta2rud/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("beta2rud/max", satU)) {ROS_FATAL("Invalid parameters for -beta2rud/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("beta2rud/min", satL)) {ROS_FATAL("Invalid parameters for -beta2rud/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		N = 10;
		beta2Rudder = new PID(P, I, D, satU, satL, Ts, N);
		
		//Create pitch to elevator controller
		if(!ros::param::getCached("pitch2elev/p", P)) {ROS_FATAL("Invalid parameters for -pitch2elev/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("pitch2elev/i", I)) {ROS_FATAL("Invalid parameters for -pitch2elev/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("pitch2elev/d", D)) {ROS_FATAL("Invalid parameters for -pitch2elev/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("pitch2elev/max", satU)) {ROS_FATAL("Invalid parameters for -pitch2elev/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("pitch2elev/min", satL)) {ROS_FATAL("Invalid parameters for -pitch2elev/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		N = 10;
		pitch2Elevator = new PID(P, I, D, satU, satL, Ts, N);
	}

	///////////////////
	//Class Destructor
	BMcLAttitudeController::~BMcLAttitudeController() {}
	
	/////////////////////
	//Lateral Controllers
	double BMcLAttitudeController::aileronControl() {
		euler = quat2euler(states.pose.orientation);
		double errYaw = refCommands.euler.z - euler.z;
		if (errYaw>M_PI) {errYaw-=2*M_PI;}
		if (errYaw<-M_PI) {errYaw+=M_PI;}
		return roll2Aileron->step( yaw2Roll->step( errYaw, 0.01) - euler.x, 0.01);
	}
	
	double BMcLAttitudeController::rudderControl() {
		geometry_msgs::Vector3 temp;
		temp.x = states.velocity.linear.x - environment.wind.x;
		temp.y = states.velocity.linear.y - environment.wind.y;
		temp.z = states.velocity.linear.z - environment.wind.z;
		airdata = getAirData(temp);
		return beta2Rudder->step( -airdata.z, 0.01);
	}

	/////////////////////
	//Longitudinal Controllers
	double BMcLAttitudeController::elevatorControl() {
		euler = quat2euler(states.pose.orientation);
		double errPitch = refCommands.euler.y - euler.y;
		if (errPitch>M_PI) {errPitch-=2*M_PI;}
		if (errPitch<-M_PI) {errPitch+=M_PI;}
		return pitch2Elevator->step( errPitch, 0.01);
	}
	
	////////////
	// Main Step
	void BMcLAttitudeController::step()
	{
		euler = quat2euler(states.pose.orientation);
		geometry_msgs::Vector3 temp;
		temp.x = states.velocity.linear.x - environment.wind.x;
		temp.y = states.velocity.linear.y - environment.wind.y;
		temp.z = states.velocity.linear.z - environment.wind.z;
		airdata = getAirData(temp);
//		std::cout << euler.x <<' '<< euler.y << ' '<< euler.z << std::endl;
		double output[4];
		output[0] = aileronControl();
		output[1] = elevatorControl();
		output[2] = input[2];
		output[3] = rudderControl();
		writePWM(output);
	}
	
	///////////
	//Utilities
	///////////
	
	void BMcLAttitudeController::getStates(last_letter::SimStates inpStates)
	{
		states = inpStates;
	}
	
	/////////////////////////////////////////////////
	//convert uS PPM values to control surface inputs
	void BMcLAttitudeController::getInput(last_letter::SimPWM inputMsg)
	{
		//Convert PPM to -1/1 ranges (0/1 for throttle)
		input[0] = (double)(inputMsg.value[0]-1500)/500;
		input[1] = (double)(inputMsg.value[1]-1500)/500;
		input[2] = (double)(inputMsg.value[2]-1000)/1000;
		input[3] = (double)(inputMsg.value[3]-1500)/500;
	}
	
	void BMcLAttitudeController::writePWM(double *output)
	{
		last_letter::SimPWM channels;
		channels.value[0] = (unsigned int)(output[0]*500+ 1500);
		channels.value[1] = (unsigned int)(output[1]*500+ 1500);
		channels.value[2] = (unsigned int)((output[2]+1)*500+ 1000);
		channels.value[3] = (unsigned int)(output[3]*500+ 1500);
		channels.header.stamp = ros::Time::now();
		pubCtrl.publish(channels);
	}
	
	/////////////////////////////////////////////////
	//Store environmental values
	void BMcLAttitudeController::getEnvironment(last_letter::Environment envUpdate)
	{
		environment = envUpdate;
	}
	
	/////////////////////////////////////////////////
	//Store environmental values
	void BMcLAttitudeController::getReference(last_letter::RefCommands refInp)
	{
		refCommands = refInp;
	}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

	ros::init(argc, argv, "controlNode");
	ros::NodeHandle n;

	ros::Duration(3).sleep(); //wait for other nodes to get raised
	double ctrlRate;
	ros::param::get("ctrlRate",ctrlRate); //frame rate in Hz
	ros::Rate spinner(ctrlRate);
	
	BMcLAttitudeController attCtrl(n);
	spinner.sleep();
	ROS_INFO("controlNode up");
	
	while (ros::ok())
	{
		ros::spinOnce();
		attCtrl.step();
		spinner.sleep();

//		if (isnan(uav.states.velocity.linear.x))
//		{		
//			ROS_FATAL("State NAN!");
//			break;
//		}
	}
	
	return 0;
	
}
