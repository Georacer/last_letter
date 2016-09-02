//////////////////////////////
// Propulsion interfrace class
//////////////////////////////

// Constructor
Propulsion::Propulsion(ModelPlane * parent, int ID)
{
	parentObj = parent;
	id = ID;
	XmlRpc::XmlRpcValue list;
	int i;
	char paramMsg[50];
	sprintf(paramMsg, "motor%i/CGOffset", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	}
	CGOffset.x = list[0];
	CGOffset.y = list[1];
	CGOffset.z = list[2];

	sprintf(paramMsg, "motor%i/mountOrientation", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	}
	// !!! Order mixed because tf::Quaternion::setEuler seems to work with PRY, instead of YPR
	mountOrientation.y = list[0];
	mountOrientation.z = list[1];
	mountOrientation.x = list[2];

	theta = 0; // Initialize propeller angle

	sprintf(paramMsg, "motor%i/chanMotor", id);
	if(!ros::param::getCached(paramMsg, chanMotor)) {ROS_INFO("No MOTOR%i channel selected", id); chanMotor=-1;}
	sprintf(paramMsg, "motor%i/chanGimbal", id);
	if(!ros::param::getCached(paramMsg, chanGimbal)) {ROS_INFO("No GIMBAL%i channel selected", id); chanGimbal=-1;}
	sprintf(paramMsg, "motor%i/gimbalAngle_max", id);
	if(!ros::param::getCached(paramMsg, gimbalAngle_max)) {ROS_INFO("No GIMBALANGLE_MAX%i value selected", id); gimbalAngle_max=0.0;}

	inputMotor = 0.0;
	inputGimbal = 0.0;

	sprintf(paramMsg, "motor%i/rotationDir", id);
	if(!ros::param::getCached(paramMsg, rotationDir)) {ROS_INFO("No ROTATION_DIR%i value selected", id); rotationDir=1.0;}

}

// Destructor
Propulsion::~Propulsion()
{
	delete parentObj;
}

void Propulsion::getInput()
{
	char paramMsg[50];
	sprintf(paramMsg, "motor%i/gimbalAngle_max", id);
	ros::param::getCached(paramMsg, gimbalAngle_max);
	if (chanMotor>-1) {inputMotor = (double)(parentObj->input.value[chanMotor]-1000)/1000; }
	if (chanGimbal>-1) {inputGimbal = gimbalAngle_max * (double)(parentObj->input.value[chanGimbal]-1500)/500; }
}

// Engine physics step, container for the generic class
void Propulsion::stepEngine()
{
	rotateWind();
	updateRadPS();
	rotateProp();
	getForce();
	getTorque();
	rotateForce();
	rotateTorque();
}


// Convert the relateive wind from body axes to propeller axes
void Propulsion::rotateWind()
{

	tf::Quaternion tempQuat;
	// Construct transformation from body axes to mount frame
	tempQuat.setEuler(mountOrientation.z, mountOrientation.y, mountOrientation.x);

	body_to_mount.setOrigin(tf::Vector3(CGOffset.x, CGOffset.y, CGOffset.z));
	body_to_mount.setRotation(tempQuat);
	body_to_mount_rot.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	body_to_mount_rot.setRotation(tempQuat);

	// Construct transformation to apply gimbal movement. Gimbal rotation MUST be aligned with the resulting z-axis!
	// !!! Order mixed because tf::Quaternion::setEuler seems to work with PRY, instead of YPR
	tempQuat.setEuler(0.0, 0.0, inputGimbal);

	mount_to_gimbal.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	mount_to_gimbal.setRotation(tempQuat);
	mount_to_gimbal_rot.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	mount_to_gimbal_rot.setRotation(tempQuat);

	// Transform the relative wind from body axes to propeller axes
	tf::Vector3 bodyWind(parentObj->airdata.u_r, parentObj->airdata.v_r, parentObj->airdata.w_r);
	tf::Vector3 tempVect;
	tempVect = mount_to_gimbal_rot * (body_to_mount_rot * bodyWind);

	relativeWind.x = tempVect.getX();
	relativeWind.y = tempVect.getY();
	relativeWind.z = tempVect.getZ();

	normalWind = relativeWind.x;
	if (!std::isfinite(normalWind)) {ROS_FATAL("propulsion.cpp: NaN value in normalWind"); ros::shutdown();}
	// if (std::fabs(normalWind)>1e+160) {ROS_FATAL("propulsion.cpp/rotateWind: normalWind over 1e+160"); ros::shutdown();}
}

void Propulsion::rotateProp() // Update propeller angle
{
	theta += omega*parentObj->dt;
	if (theta > 2.0*M_PI) theta -= 2*M_PI;
	if (theta < 0.0) theta += 2*M_PI;

	tf::Quaternion tempQuat;
	// !!! Order mixed because tf::Quaternion::setEuler seems to work with PRY, instead of YPR
	tempQuat.setEuler(0.0, theta, 0.0);

	gimbal_to_prop.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	gimbal_to_prop.setRotation(tempQuat);
	gimbal_to_prop_rot.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	gimbal_to_prop_rot.setRotation(tempQuat);

	body_to_prop = body_to_mount * (mount_to_gimbal * gimbal_to_prop);
	body_to_prop_rot = body_to_mount_rot * (mount_to_gimbal_rot * gimbal_to_prop_rot);

	char prop_frame[50];
	sprintf(prop_frame, "propeller_%i", id);
	broadcaster.sendTransform(tf::StampedTransform(body_to_prop, ros::Time::now(), "base_link", prop_frame));

}

 // Convert the resulting force to the body axes
void Propulsion::rotateForce()
{

	tf::Vector3 tempVect(wrenchProp.force.x, wrenchProp.force.y, wrenchProp.force.z);
	tempVect = body_to_prop_rot * tempVect; // I'm not sure why this works and not inverted

	wrenchProp.force.x = tempVect.getX();
	wrenchProp.force.y = tempVect.getY();
	wrenchProp.force.z = tempVect.getZ();

}

// Convert the resulting torque to the body axes
void Propulsion::rotateTorque()
{

	tf::Vector3 tempVect(wrenchProp.torque.x, wrenchProp.torque.y, wrenchProp.torque.z);
	tempVect = body_to_prop_rot * tempVect;

	wrenchProp.torque.x = tempVect.getX();
	wrenchProp.torque.y = tempVect.getY();
	wrenchProp.torque.z = tempVect.getZ();


	// Convert the torque from the motor frame to the body frame
	double ratio = parentObj->kinematics.J[0] / (parentObj->kinematics.J[0] + parentObj->kinematics.mass * CGOffset.x*CGOffset.x);
	wrenchProp.torque.x =  ratio * wrenchProp.torque.x;
	ratio = parentObj->kinematics.J[4] / (parentObj->kinematics.J[4] + parentObj->kinematics.mass * CGOffset.y*CGOffset.y);
	wrenchProp.torque.y =  ratio * wrenchProp.torque.y;
	ratio = parentObj->kinematics.J[8] / (parentObj->kinematics.J[8] + parentObj->kinematics.mass * CGOffset.z*CGOffset.z);
	wrenchProp.torque.z =  ratio * wrenchProp.torque.z;

	// Add torque to to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	// Will potentially add the following code in the future, to support shift of CoG mid-flight
	// XmlRpc::XmlRpcValue list;
	// int i;
	// if(!ros::param::getCached("motor/CGOffset", list)) {ROS_FATAL("Invalid parameters for -/motor/CGOffset- in param server!"); ros::shutdown();}
	// for (i = 0; i < list.size(); ++i) {
	// 	ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	// 	CGOffset[i]=list[i];
	// }
	wrenchProp.torque.x = wrenchProp.torque.x + CGOffset.y*wrenchProp.force.z - CGOffset.z*wrenchProp.force.y;
	wrenchProp.torque.y = wrenchProp.torque.y - CGOffset.x*wrenchProp.force.z + CGOffset.z*wrenchProp.force.x;
	wrenchProp.torque.z = wrenchProp.torque.z - CGOffset.y*wrenchProp.force.x + CGOffset.x*wrenchProp.force.y;

	// std::cout << "torque: " << wrenchProp.torque.x << " " << wrenchProp.torque.y << " " << wrenchProp.torque.z << std::endl;

}

#include "noEngine.cpp"

#include "beardEngine.cpp"

#include "pistonEngine.cpp"

#include "electricEngine.cpp"
