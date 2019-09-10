// UavModelWrapper class definitions
#include "uav_model_ros.hpp"

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
// #include <sensor_msgs/JointState.h>


using namespace std;

//////////////////////////
// Define UavModelWrapper class
//////////////////////////

///////////////////
//Class Constructor
UavModelWrapper::UavModelWrapper (ros::NodeHandle n)
{
	// Get the UAV name from the parameter server and build the relevant UAV configuration struct
	string uavName;
	n.getParam("uav_name", uavName);
	// This will likely bloat the library size, but it's not too big of a deal.
    ROS_INFO("Loading uav configuration for %s", uavName.c_str());
	ConfigsStruct_t configStruct = loadModelConfig(uavName);
	// Parameter randomization it done internally in loadModelConfig

    ROS_INFO("Creating new UavModel");
	uavModel = new UavModel(configStruct); //Create a UavStruct by passing the configurations bundle struct

	if(!ros::param::getCached("/world/deltaT", dt)) {ROS_FATAL("Invalid parameters for -deltaT- in param server!"); ros::shutdown();}
	tprev = ros::Time::now();
	states.header.stamp = tprev;
	//Subscribe and advertize
	subInp = n.subscribe("ctrlPWM",1,&UavModelWrapper::getInput, this); //model control input subscriber
	pubState = n.advertise<last_letter_msgs::SimStates>("states",1000); //model states publisher
	pubEnv = n.advertise<last_letter_msgs::Environment>("environment",1000); //model states publisher
	pubForce = n.advertise<geometry_msgs::Vector3Stamped>("forceInput",1000); // forces publisher
	pubTorque = n.advertise<geometry_msgs::Vector3Stamped>("torqueInput",1000); // torques publisher
	pubLinAcc = n.advertise<geometry_msgs::Vector3Stamped>("linearAcc",1000); // Body frame linear acceleration - no corriolis effect
}

//Initialize states
void UavModelWrapper::init()
{
    uavModel->init();
}

///////////////////////////////////////
//make one step of the plane simulation
void UavModelWrapper::step(void)
{
	// perform step actions serially
    uavModel->step();

	// airdata.calcairdata();

	// dynamics.calcwrench();
	// kinematics.forceinput = dynamics.getforce();
	// kinematics.torqueinput = dynamics.gettorque();
	// kinematics.calcderivatives();
	// kinematics.integrator->propagation();

	tprev = ros::Time::now();

    // Access simulation state and other variables
    convertStates(uavModel->state, states);
    states.header.stamp = tprev;
    states.header.frame_id = "bodyFrame";
    convertEnvironment(uavModel->environmentModel.environment, environment);
    environment.header.stamp = tprev;
    environment.header.frame_id = "bodyFrame";
    convertRosVector3(uavModel->dynamics.getForce(), forceInput.vector);
    forceInput.header.stamp = tprev;
    forceInput.header.frame_id = "bodyFrame";
    convertRosVector3(uavModel->dynamics.getTorque(), torqueInput.vector);
    torqueInput.header.stamp = tprev;
    torqueInput.header.frame_id = "bodyFrame";
    convertRosVector3(uavModel->dynamics.getForce()/uavModel->kinematics.inertial.mass, linearAcc.vector);
    linearAcc.header.stamp = tprev;
    linearAcc.header.frame_id = "bodyFrame";

	//publish results
	pubState.publish(states);
    pubEnv.publish(environment);
	pubForce.publish(forceInput);
	pubTorque.publish(torqueInput);
	pubLinAcc.publish(linearAcc);

    // publish transforms
    if (tfBroadcastEnable)
    {
        broadcastTransforms();
    }
}

void UavModelWrapper::broadcastTransforms()
{
    Eigen::Vector3d translationEigen;
    Eigen::Quaterniond rotationEigen;
    tf::Vector3 translationTf;
    tf::Quaternion rotationTf;
    tf::Transform transform;

    // Send transformations for 
    transform.setOrigin( tf::Vector3(states.pose.position.x, states.pose.position.y, states.pose.position.z));
    transform.setRotation( tf::Quaternion(states.pose.orientation.x,states.pose.orientation.y,states.pose.orientation.z,states.pose.orientation.w));
    broadcaster.sendTransform(tf::StampedTransform(transform, states.header.stamp, "map", "base_link"));
    string frameName;

    for (int i=0; i<uavModel->dynamics.nMotors; i++)
    {
        translationEigen = uavModel->dynamics.propulsion[i]->body_to_prop.translation();
        rotationEigen = uavModel->dynamics.propulsion[i]->body_to_prop.rotation();
        convertTfVector3(translationEigen, translationTf);
        convertTfQuaternion(rotationEigen, rotationTf);
        transform.setOrigin(translationTf);
        transform.setRotation(rotationTf);
        frameName = "propeller"+i;
        broadcaster.sendTransform(tf::StampedTransform(transform, states.header.stamp, "base_link", frameName));
    }

    for (int i=0; i<uavModel->dynamics.nWings; i++)
    {
        translationEigen = uavModel->dynamics.aerodynamics[i]->body_to_gimbal.translation();
        rotationEigen = uavModel->dynamics.aerodynamics[i]->body_to_gimbal.rotation();
        convertTfVector3(translationEigen, translationTf);
        convertTfQuaternion(rotationEigen, rotationTf);
        transform.setOrigin(translationTf);
        transform.setRotation(rotationTf);
        frameName = "airfoil_"+i;
        broadcaster.sendTransform(tf::StampedTransform(transform, states.header.stamp, "base_link", frameName));
    }
}

/////////////////////////////////////////////////
// Pass PWM control input signals to uavModel
void UavModelWrapper::getInput(last_letter_msgs::SimPWM inputMsg)
{
	input = inputMsg;
    InputPwm_t modelInput;
    for (int i=0; i<12; i++)
    {
        modelInput.value[i] = inputMsg.value[i];
    }
    uavModel->setInputPwm(modelInput);
}

//////////////////
//Class destructor
UavModelWrapper::~UavModelWrapper ()
{
    delete uavModel;
}

void convertStates(const SimState_t simState, last_letter_msgs::SimStates &wrapperState)
{
    wrapperState.pose.position.x = simState.pose.position.x();
    wrapperState.pose.position.y = simState.pose.position.y();
    wrapperState.pose.position.z = simState.pose.position.z();
    wrapperState.pose.orientation.x = simState.pose.orientation.x();
    wrapperState.pose.orientation.y = simState.pose.orientation.y();
    wrapperState.pose.orientation.z = simState.pose.orientation.z();
    wrapperState.pose.orientation.w = simState.pose.orientation.w();
    wrapperState.velocity.linear.x = simState.velocity.linear.x();
    wrapperState.velocity.linear.y = simState.velocity.linear.y();
    wrapperState.velocity.linear.z = simState.velocity.linear.z();
    wrapperState.velocity.angular.x = simState.velocity.angular.x();
    wrapperState.velocity.angular.y = simState.velocity.angular.y();
    wrapperState.velocity.angular.z = simState.velocity.angular.z();
    wrapperState.acceleration.linear.x = simState.acceleration.linear.x();
    wrapperState.acceleration.linear.y = simState.acceleration.linear.y();
    wrapperState.acceleration.linear.z = simState.acceleration.linear.z();
    for (int i=0; i<4; i++)
    {
        // TODO: This crashese the simulation, debug it
        // wrapperState.rotorspeed[i] = simState.rotorspeed[i];
    }
    wrapperState.geoid.latitude = simState.geoid.latitude;
    wrapperState.geoid.longitude = simState.geoid.longitude;
    wrapperState.geoid.altitude = simState.geoid.altitude;
    wrapperState.geoid.velocity.x = simState.geoid.velocity.x();
    wrapperState.geoid.velocity.y = simState.geoid.velocity.y();
    wrapperState.geoid.velocity.z = simState.geoid.velocity.z();
}

void convertEnvironment(const Environment_t simEnv, last_letter_msgs::Environment &wrapperEnv)
{
    wrapperEnv.wind.x = simEnv.wind.x();
    wrapperEnv.wind.y = simEnv.wind.y();
    wrapperEnv.wind.z = simEnv.wind.z();
    wrapperEnv.density = simEnv.density;
    wrapperEnv.pressure = simEnv.pressure;
    wrapperEnv.temperature = simEnv.temperature;
    wrapperEnv.gravity = simEnv.gravity;
}

void convertRosVector3(const Eigen::Vector3d vector3Eigen, geometry_msgs::Vector3 &vector3Ros)
{
    vector3Ros.x = vector3Eigen.x();
    vector3Ros.y = vector3Eigen.y();
    vector3Ros.z = vector3Eigen.z();
}

void convertRosQuaternion(const Eigen::Quaterniond quatEigen, geometry_msgs::Quaternion &quatRos)
{
    quatRos.x = quatEigen.x();
    quatRos.y = quatEigen.y();
    quatRos.z = quatEigen.z();
    quatRos.w = quatEigen.w();
}

void convertTfVector3(const Eigen::Vector3d vector3Eigen, tf::Vector3 &vector3Tf)
{
    vector3Tf = tf::Vector3(vector3Eigen.x(), vector3Eigen.y(), vector3Eigen.z());
}

void convertTfQuaternion(const Eigen::Quaterniond quatEigen, tf::Quaternion &quatTf)
{
    quatTf = tf::Quaternion(quatEigen.x(), quatEigen.y(), quatEigen.z(), quatEigen.w());
}

