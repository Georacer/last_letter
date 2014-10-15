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
		pubCtrl = n.advertise<last_letter::SimPWM>("ctrlPWM",1000);
		
		//Create roll to aileron controller
		if(!ros::param::getCached("roll2ail/p", P)) {ROS_FATAL("Invalid parameters for -roll2ail/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("roll2ail/i", I)) {ROS_FATAL("Invalid parameters for -roll2ail/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("roll2ail/d", D)) {ROS_FATAL("Invalid parameters for -roll2ail/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("roll2ail/tau", Tau)) {ROS_FATAL("Invalid parameters for -roll2ail/tau- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("roll2ail/max", satU)) {ROS_FATAL("Invalid parameters for -roll2ail/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("roll2ail/min", satL)) {ROS_FATAL("Invalid parameters for -roll2ail/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		roll2Aileron = new PID(P, I, D, satU, satL, 0.0, Ts, Tau);
		
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
		if(!ros::param::getCached("pitch2elev/p", P)) {ROS_FATAL("Invalid parameters for -pitch2elev/p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("pitch2elev/i", I)) {ROS_FATAL("Invalid parameters for -pitch2elev/i- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("pitch2elev/d", D)) {ROS_FATAL("Invalid parameters for -pitch2elev/d- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("pitch2elev/max", satU)) {ROS_FATAL("Invalid parameters for -pitch2elev/max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("pitch2elev/min", satL)) {ROS_FATAL("Invalid parameters for -pitch2elev/min- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("pitch2elev/neutral", trim)) {ROS_FATAL("Invalid parameters for -pitch2elev/neutral- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ctrlRate", Ts)) {ROS_FATAL("Invalid parameters for -ctrlRate- in param server!"); ros::shutdown();}
		Ts = 1.0/Ts;
		Tau = 0.1;
		pitch2Elevator = new PID(P, I, D, satU, satL, trim, Ts, Tau);
		
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
		
		if(!ros::param::getCached("altSwitchThresh", altThresh)) {ROS_FATAL("Invalid parameters for -altSwitchThresh- in param server!"); ros::shutdown();}

		int alphaOrder = 2;
		int betaOrder = 1;
		double alpha[] = {0.9751804568, -1.97502451};
		double beta[] = {0.77646787e-4, 0.78300013e-4};
		pitchSmoother = new discrTF(alpha, alphaOrder, beta, betaOrder);
	}
	
	

	///////////////////
	//Class Destructor
	BMcLAttitudeController::~BMcLAttitudeController() {
		delete roll2Aileron;
		delete yaw2Roll;
		delete beta2Rudder;
		delete pitch2Elevator;
		delete alt2Pitch;
		delete airspd2Pitch;
		delete airspd2Throt;
	}
	
	/////////////////////
	//Lateral Controllers
	double BMcLAttitudeController::aileronControl() {
		euler = quat2euler(states.pose.orientation);
		double errYaw = refCommands.euler.z - euler.z;
		if (errYaw>M_PI) {errYaw-=2*M_PI;}
		if (errYaw<-M_PI) {errYaw+=2*M_PI;}
		double targetRoll = yaw2Roll->step(errYaw);
		return roll2Aileron->step( targetRoll - euler.x);
	}
	
	double BMcLAttitudeController::rudderControl() {
		return beta2Rudder->step( -airdata.z);
	}

	/////////////////////
	//Longitudinal Controllers
	double BMcLAttitudeController::elevatorControl() {
		double static altPrev=0;
		double errAlt = refCommands.altitude - states.geoid.altitude;
		double errVa = refCommands.airspeed - airdata.x;

		double errPitch;
		double refPitch;
		int static state = 0;
		double static sumErrAlt = 0, sumErrVa = 0, sumErrAlt2 = 0, sumErrVa2 = 0;
		double errContr, contrOutput;
		euler = quat2euler(states.pose.orientation);


		if (abs(errAlt) <= altThresh) { // within altitude hold zone
			// double refAlt = pitchSmoother->step(refCommands.altitude);
			// errAlt = refAlt - states.geoid.altitude;
			double comRate = (refCommands.altitude - states.geoid.altitude)/5.0;
			comRate = std::min(2.8, std::max(comRate, -5.0));
			// double climbRate = airdata.x*sin(airdata.y);
			double climbRate = (states.geoid.altitude - altPrev)/0.01;
			altPrev = states.geoid.altitude;
			double errRate = comRate - climbRate;
			// std::cout<< comRate << ',' << climbRate << std::endl;
			if (state) {
				state = 0;
				sumErrAlt = 0;
				sumErrAlt2 = 0;

			}
			// refPitch = alt2Pitch->step(errAlt, false, 0);
			// refPitch = alt2Pitch->step(errAlt);
			refPitch = alt2Pitch->step(errRate);			
			errContr = errPitch - airspd2Pitch->step(errVa);
			sumErrVa += errContr*0.01;
			sumErrVa2 += sumErrVa*0.01;
			airspd2Pitch->Iterm += (10*errContr + 10*sumErrVa + 10*sumErrVa2)*0.01;
		}
		else { // outside of altitude hold zone
			if (!state) {
				state = 1;
				sumErrVa = 0;
				sumErrVa2 = 0;
				pitchSmoother->init(refCommands.altitude, refCommands.altitude);
			}
			refPitch = airspd2Pitch->step(errVa);
			contrOutput = alt2Pitch->step(errAlt, true, refPitch);

		}
		// std::cout<< refPitch << std::endl;
		errPitch = refPitch - euler.y;
		// errPitch =  refCommands.euler.y - euler.y;
		// std::cout << "P /  I /  D / Iterm /  Out / Target :" << alt2Pitch->P << "/"<< alt2Pitch->I << "/"<< alt2Pitch->D << "/"<< alt2Pitch->Iterm << "/"<< contrOutput << "/" << refCommands.altitude;
		// std::cout << std::endl;

		// smooth out commanded pitch

		return pitch2Elevator->step(errPitch);
		// return pitch2Elevator->step(pitchSmoother->step(errPitch));
	}
	
	double BMcLAttitudeController::throttleControl() {
		double errAlt = refCommands.altitude - states.geoid.altitude;
		int static state = 0;
		double static deltat;
		
		if (abs(errAlt) <= altThresh) {
			double errVa = refCommands.airspeed - airdata.x;
			if (state != 0) {
				state = 0;
			}
			deltat = airspd2Throt->step( errVa);
		}
		else if (errAlt < altThresh) {
			if (state!=1) {
				state = 1;
				airspd2Throt->Iterm = deltat / airspd2Throt->I;
			}
			deltat = 0.0;
		}
		else {
			if (state!=2) {
				state = 2;
				airspd2Throt->Iterm = deltat / airspd2Throt->I;
			}
			deltat = 1.0;
		}
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
		input[9] = (double)(inputMsg.value[9]-1000)/1000;
	}
	
	void BMcLAttitudeController::writePWM(double *output)
	{
		last_letter::SimPWM channels;
		channels.value[0] = (unsigned int)(output[0]*500+ 1500);
		channels.value[1] = (unsigned int)(output[1]*500+ 1500);
		channels.value[2] = (unsigned int)((output[2])*500+ 1500);
		channels.value[3] = (unsigned int)(output[3]*500+ 1500);
		channels.value[9] = (unsigned int)(input[9]*1000 + 1000);
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
