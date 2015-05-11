#include "controller_HCUAV.hpp"

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
		altDes = 0;

		//Subscribe and advertize
		subInp = n.subscribe("rawPWM",1,&BMcLAttitudeController::getInput, this);
		subState = n.subscribe("states",1,&BMcLAttitudeController::getStates, this);
		subEnv = n.subscribe("environment",1,&BMcLAttitudeController::getEnvironment, this);
		subRef = n.subscribe("refCommands",1,&BMcLAttitudeController::getReference, this);
		pubCtrl = n.advertise<last_letter_msgs::SimPWM>("ctrlPWM",1000);

		//Create roll to aileron controller

		//Create yaw to roll controller
		if(!ros::param::getCached("yaw2roll/p", P)) {ROS_FATAL("Invalid parameters for -yaw2roll/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/i", I)) {ROS_FATAL("Invalid parameters for -yaw2roll/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/d", D)) {ROS_FATAL("Invalid parameters for -yaw2roll/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/tau", Tau)) {ROS_FATAL("Invalid parameters for -yaw2roll/tau- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/max", satU)) {ROS_FATAL("Invalid parameters for -yaw2roll/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("yaw2roll/min", satL)) {ROS_FATAL("Invalid parameters for -yaw2roll/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		yaw2Roll = new PID(P, I, D, satU, satL, 0.0, Ts, Tau);

		//Create beta to rudder controller
		if(!ros::param::getCached("beta2rud/p", P)) {ROS_FATAL("Invalid parameters for -beta2rud/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("beta2rud/i", I)) {ROS_FATAL("Invalid parameters for -beta2rud/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("beta2rud/d", D)) {ROS_FATAL("Invalid parameters for -beta2rud/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("beta2rud/max", satU)) {ROS_FATAL("Invalid parameters for -beta2rud/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("beta2rud/min", satL)) {ROS_FATAL("Invalid parameters for -beta2rud/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		Tau = 0.1;
		beta2Rudder = new PID(P, I, D, satU, satL, 0.0, Ts, Tau);

		//Create pitch to elevator controller

		//Create altitude to pitch controller
		if(!ros::param::getCached("alt2pitch/p", P)) {ROS_FATAL("Invalid parameters for -alt2pitch/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("alt2pitch/i", I)) {ROS_FATAL("Invalid parameters for -alt2pitch/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("alt2pitch/d", D)) {ROS_FATAL("Invalid parameters for -alt2pitch/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("alt2pitch/max", satU)) {ROS_FATAL("Invalid parameters for -alt2pitch/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("alt2pitch/min", satL)) {ROS_FATAL("Invalid parameters for -alt2pitch/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("alt2pitch/tau", Tau)) {ROS_FATAL("Invalid parameters for -alt2pitch/tau- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		alt2Pitch = new PID(P, I, D, satU, satL, 0.0, Ts, Tau);
		// alt2Pitch = new APID(P, I, D, satU, satL, 0.0, Ts, Tau);

		//Create airspeed to pitch controller
		if(!ros::param::getCached("airspd2pitch/p", P)) {ROS_FATAL("Invalid parameters for -airspd2pitch/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2pitch/i", I)) {ROS_FATAL("Invalid parameters for -airspd2pitch/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2pitch/d", D)) {ROS_FATAL("Invalid parameters for -airspd2pitch/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2pitch/max", satU)) {ROS_FATAL("Invalid parameters for -airspd2pitch/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2pitch/min", satL)) {ROS_FATAL("Invalid parameters for -airspd2pitch/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2pitch/tau", Tau)) {ROS_FATAL("Invalid parameters for -airspd2pitch/tau- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		airspd2Pitch = new PID(P, I, D, satU, satL, 0.0, Ts, Tau);

		//Create airspeed to throttle controller
		if(!ros::param::getCached("airspd2throt/p", P)) {ROS_FATAL("Invalid parameters for -airspd2throt/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2throt/i", I)) {ROS_FATAL("Invalid parameters for -airspd2throt/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2throt/d", D)) {ROS_FATAL("Invalid parameters for -airspd2throt/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2throt/tau", Tau)) {ROS_FATAL("Invalid parameters for -airspd2throt/tau- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2throt/max", satU)) {ROS_FATAL("Invalid parameters for -airspd2throt/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2throt/min", satL)) {ROS_FATAL("Invalid parameters for -airspd2throt/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airspd2throt/neutral", trim)) {ROS_FATAL("Invalid parameters for -airspd2throt/neutral- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		airspd2Throt = new PID(P, I, D, satU, satL, trim, Ts, Tau);

		int alphaOrder = 2;
		int betaOrder = 1;
		double alpha[] = {0.9751804568, -1.97502451};
		double beta[] = {0.77646787e-4, 0.78300013e-4};
		pitchSmoother = new discrTF(alpha, alphaOrder, beta, betaOrder);
	}



	///////////////////
	//Class Destructor
	BMcLAttitudeController::~BMcLAttitudeController() {
		delete yaw2Roll;
		delete beta2Rudder;
		delete alt2Pitch;
		delete airspd2Pitch;
		delete airspd2Throt;
	}

	/////////////////////
	//Lateral Controllers
	double BMcLAttitudeController::aileronControl() {
		euler = quat2euler(states.pose.orientation);
		double errYaw = refCommands.euler.z - euler.z;
		double errRoll;
		static double x1=0;
		if (errYaw>M_PI) {errYaw-=2.0*M_PI;}
		if (errYaw<-M_PI) {errYaw+=2.0*M_PI;}
		double targetRoll = yaw2Roll->step(errYaw);
		// errRoll =  refCommands.euler.x - euler.x;
		errRoll = targetRoll - euler.x;
		double errP = 2.0*errRoll - states.velocity.angular.x;
		x1 += 0.01*6.325*errP;
		return std::max(std::min(3.162*x1 + 2.0*errP -0.02*states.velocity.angular.x,1.0),-1.0);
	}

	double BMcLAttitudeController::rudderControl() {
		// TODO: take accelerometer feedback
		//read orientation and subtract gravity
		double Reb[9];
		quat2rotmtx(states.pose.orientation,Reb);
		geometry_msgs::Vector3 gravVect;
		gravVect.z = 9.8051;
		geometry_msgs::Vector3 gravAcc = Reb/gravVect;
		double accError = -states.acceleration.linear.y + gravAcc.y;
		// std::cout << accError << std::endl;
		// return std::max(std::min( -0.003*(states.acceleration.linear.y - gravAcc.y),1.0),-1.0);
		return beta2Rudder->step( accError );
	}

	/////////////////////
	//Longitudinal Controllers
	double BMcLAttitudeController::elevatorControl() {
		double static altPrev=0;
		double static x1= 0;
		double errAlt = refCommands.altitude - states.geoid.altitude;
		double errVa = refCommands.airspeed - airdata.x;

		double errPitch;
		double refPitch;
		double static sumErrAlt = 0, sumErrVa = 0, sumErrAlt2 = 0, sumErrVa2 = 0;
		double errContr, contrOutput;
		euler = quat2euler(states.pose.orientation);

		double comRate = (refCommands.altitude - states.geoid.altitude)/5.0; // Using exponential profile
		comRate = std::min(2.8, std::max(comRate, -3.0));
		double climbRate = (states.geoid.altitude - altPrev)/0.01;
		altPrev = states.geoid.altitude;
		double errRate = comRate - climbRate;
		refPitch = alt2Pitch->step(errRate);
		errContr = errPitch - airspd2Pitch->step(errVa);
		sumErrVa += errContr*0.01;
		sumErrVa2 += sumErrVa*0.01;
		airspd2Pitch->Iterm += (10*errContr + 10*sumErrVa + 10*sumErrVa2)*0.01;

		errPitch = refPitch - euler.y;
		// errPitch =  refCommands.euler.y - euler.y;
		//
		// Compensator scheme based on Stevens/Lewis
		x1 += 0.01*0.9493*errPitch;
		return std::max(std::min(1.1878*x1 + 5.0*errPitch -0.6*states.velocity.angular.y,1.0),-1.0);
	}

	double BMcLAttitudeController::throttleControl() {
		double errAlt = refCommands.altitude - states.geoid.altitude;
		double static deltat;

		double errVa = refCommands.airspeed - airdata.x;
		deltat = airspd2Throt->step( errVa);
		// double forwarddelta = 0.00151*pow(refCommands.airspeed,2) -0.0896*refCommands.airspeed + 1.703;
		// std::cout<< deltat <<',' << forwarddelta << ','<< refCommands.airspeed << std::endl;
		// deltat += forwarddelta;
		// deltat = std::max(std::min(deltat,1.0),0.0);
		return deltat;
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
		double output[4];
		output[0] = aileronControl();
		output[1] = elevatorControl();
		output[2] = throttleControl();
		output[3] = rudderControl();
		writePWM(output);
	}

	///////////
	//Utilities
	///////////

	void BMcLAttitudeController::getStates(last_letter_msgs::SimStates inpStates)
	{
		states = inpStates;
	}

	/////////////////////////////////////////////////
	//convert uS PPM values to control surface inputs
	void BMcLAttitudeController::getInput(last_letter_msgs::SimPWM inputMsg)
	{
		//Convert PPM to -1/1 ranges (0/1 for throttle)
		input[0] = (double)(inputMsg.value[0]-1500)/500;
		input[1] = (double)(inputMsg.value[1]-1500)/500;
		input[2] = (double)(inputMsg.value[2]-1000)/1000;
		input[3] = (double)(inputMsg.value[3]-1500)/500;
		input[9] = (double)(inputMsg.value[9]-1000)/1000;
	}

	void BMcLAttitudeController::writePWM(double *output)
	{
		last_letter_msgs::SimPWM channels;
		channels.value[0] = (unsigned int)(output[0]*500+ 1500);
		channels.value[1] = (unsigned int)(output[1]*500+ 1500);
		channels.value[2] = (unsigned int)((output[2])*1000+ 1000);
		channels.value[3] = (unsigned int)(output[3]*500+ 1500);
		channels.value[9] = (unsigned int)(input[9]*1000 + 1000);
		channels.header.stamp = ros::Time::now();
		pubCtrl.publish(channels);
	}

	/////////////////////////////////////////////////
	//Store environmental values
	void BMcLAttitudeController::getEnvironment(last_letter_msgs::Environment envUpdate)
	{
		environment = envUpdate;
	}

	/////////////////////////////////////////////////
	//Store environmental values
	void BMcLAttitudeController::getReference(last_letter_msgs::RefCommands refInp)
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

	ros::WallDuration(3).sleep(); //wait for other nodes to get raised
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
