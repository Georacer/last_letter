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
		subInp = n.subscribe("rawPWM",1,&ModelPlane::getInput, this);
		subState = n.subscribe("states",1,&ModelPlane::getStates, this);
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
		roll2Aileron = PID(P, I, D, satU, satL, Ts, N);
		
		//Create roll to aileron controller
		if(!ros::param::getCached("yaw2roll/p", P)) {ROS_FATAL("Invalid parameters for -roll2ail/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/i", I)) {ROS_FATAL("Invalid parameters for -roll2ail/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/d", D)) {ROS_FATAL("Invalid parameters for -roll2ail/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/max", satU)) {ROS_FATAL("Invalid parameters for -roll2ail/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/min", satL)) {ROS_FATAL("Invalid parameters for -roll2ail/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		N = 10;
		yaw2Roll = last_letter::PID(P, I, D, satU, satL, Ts, N);
	}
	
	/////////////////////
	//Lateral Controllers
	/////////////////////
	
	
	///////////
	//Utilities
	///////////
	
	void BMcLAttitudeController::step()
	{
		euler = quat2euler(states.pose.orientation);
		double output[4];
		output[0] = roll2Aileron( yaw2Roll( euler.z));
		output[1] = input[1];
		output[2] = input[2];
		output[3] = input[3];
		writePWM(output);
	}
	
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
	
	attCtrl = BMcLAttitudeController(n);
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
