//////////////////////////////////
// Define PanosContactPoints class
//////////////////////////////////

#include "last_letter_lib/ground_reaction/ground_reaction.hpp"

namespace last_letter_lib {
namespace ground_reaction {

// Class constructor
PanosContactPoints::PanosContactPoints(string name) : GroundReaction(name)
{
	len=0.2;

	// Set coefficient of friction for each material
	frictForw[0] = 0.7; frictSide[0] = 0.7;
	frictForw[1] = 0.4; frictSide[1] = 0.4;
	frictForw[2] = 0.1; frictSide[2] = 1.0;
	frictForw[3] = 0.4; frictSide[3] = 0.4; //To update composite to ground firction coefficients!

	contact = false;
}

void PanosContactPoints::initialize_parameters()
{
    GroundReaction::initialize_parameters();

    const int num_contacts = 3;
    set_param<int>("contactPtsNo", num_contacts, false);

    // For each contact point, fill in their triplet coordinates.
    const double radius = 0.4;
    const double height = 0.2;
    const double contact_type = 2;
    const double spring_constant = 500;
    const double damping_constant = 25;
    for (int i = 0; i < num_contacts; i++)
    {
        const double coord_x = radius*cos(math_utils::deg_to_rad(120*i));
        const double coord_y = radius*sin(math_utils::deg_to_rad(120*i));
        const std::vector<double> entry = {coord_x, coord_y, height, contact_type, spring_constant, damping_constant};

        set_param<vector<double>>("contactPoint" + std::to_string(i+1), entry, false);
    }

    // Preallocate a total of 64 contact points.
    for (int i = num_contacts; i < 64; i++)
    {
        const std::vector<double> entry = {};
        set_param<vector<double>>("contactPoint" + std::to_string(i+1), entry, false);
    }
}

void PanosContactPoints::update_parameters()
{
	GroundReaction::update_parameters();

	// Read contact points number.
	contactPtsNo = get_param<double>("contactPtsNo");

	// Every contactPtsNo-sized container must be allocated here, where the
    // number of rows is known.
	spp.setZero(contactPtsNo, 1); // spring contraction
	sppprev.setZero(contactPtsNo, 1); // previous spring contraction
	cpi_up.setZero(3, contactPtsNo); // upper spring end matrix
	cpi_down.setZero(3, contactPtsNo); // lower spring end matrix
	spd.setZero(contactPtsNo, 1); // spring contraction speed

	// Create an appropriately sized matrix to contain contact point information
	pointCoords.setZero(3, contactPtsNo); // contact points coordinates in the body frame
	materialIndex.setZero(contactPtsNo, 1); // contact points material type index
	springIndex.setZero(2,contactPtsNo); // contact points spring characteristics
    for (int i = 0; i < contactPtsNo; i++)
    {
        vector<double> doubleVect = get_param<vector<double>>("contactPoint" + std::to_string(i+1));
		pointCoords(0, i) = doubleVect[0]; // Save body frame contact point coordinates.
		pointCoords(1, i) = doubleVect[1];
		pointCoords(2, i) = doubleVect[2];
		materialIndex(i) = doubleVect[3]; // A separate contact point material index array.
		springIndex(0, i) = doubleVect[4];
		springIndex(1, i) = doubleVect[5]; // And the spring constants.
    }
}

// Wrench calculation function
Vector3d PanosContactPoints::getForce(const SimState_t states, const WrenchSum_t /* wrenchSum */)
{
	double kFLong, kFLat;
	int i;
	contact = false;
	safe = true; // NaN protection flag
	Vector3d Euler;

	// Read vehicle orientation quaternion.
	// Vehicle quaternion refers to the Body-to-Earth rotation
	Quaterniond orientation = states.pose.orientation.conjugate(); // TODO: conjugate seems to be wrong here. See quat2euler definition
	Euler = quat2euler(orientation);

	Wrench_t tempE, totalE;
	Vector3d we,vpoint,Ve;

	// Get velocity in the inertial frame
	Ve = orientation.conjugate() * states.velocity.linear;
	// Read vehicle coordinates
	uavpos = states.pose.position;

	// Rotate contact points coordinates from body frame to earth frame
	// and place the spring ends to the contact points positions
	// Place the upper spring end to contact point
	cpi_up = Eigen::Translation<double, 3>(uavpos) * orientation.conjugate() * pointCoords;
	// Raise the upper spring end to obtain a visually pleasing result in RViz
	cpi_up.row(2).array() -= len;
	cpi_down = cpi_up;

	// Rotate body angular speeds to earth frame
	we = orientation.conjugate() * states.velocity.angular;

	// Main calculations for each contact point
	cpi_down.row(2).array() += len; // Place lower spring end "len" below upper spring end
	// Calculate force arm
	Eigen::MatrixXd dx = cpi_up - uavpos.replicate(1, contactPtsNo);

	for (i=0;i<contactPtsNo;i++)
	{
		if (cpi_down(2,i)>0) // if the lower spring end touches the ground
		{
			cpi_down(2,i)=0; // Force lower spring end to stay at ground level
			contact=true;
			spp(i)=len+cpi_up(2,i); // Calculate current spring contraction
			spd(i)=(spp(i)-sppprev(i))/dt; // Calculate current spring contraction spreed
			Vector3d dxElement = dx.col(i);
			vpoint = Ve + we.cross(dxElement); // Contact point Earth-frame speed due vehicle velocity and rotation

			// Calculate spring force. Acts only upon Earth z-axis
			double forceZ = -(springIndex(0,i)*spp[i] + springIndex(1,i)*spd[i]);
			// Make spring force negative or zero, as the spring lower end isn't fixed on the ground
			if (forceZ > 0)
				forceZ = 0;
			/** @todo model full spring contraction (possibly with exponential force curve after full contraction?) */

			int tempIndex = materialIndex[i];
			// Get longitudinal and lateral friction coefficients for this surface
			kFLong = frictForw[tempIndex];
			kFLat = frictSide[tempIndex];

			if ((inputBrake>0.9) && (i<3)) { // Apply breaks
				kFLong = 1.0;
			}

			// Convert friction coefficients to the Earth frame
			double trackAngle = Euler.z() - atan2(vpoint.y(), vpoint.x());
			if (i==2) { // Apply steeering on the 3rd contact point (assumed to be the turning wheel)
				trackAngle = trackAngle + inputSteer;
			}

			// Calculate the magnitude of the friction force in the Earth frame
			double frictionLong = fabs(sqrt(kFLong*(kFLong*cos(trackAngle)*cos(trackAngle)+kFLat*sin(trackAngle)*sin(trackAngle)))*forceZ);
			frictionLong = std::max(frictionLong - 0.05*frictionLong/sqrt(vpoint.x()*vpoint.x() + vpoint.y()*vpoint.y() + 0.001), 0.0); //Aply static friction
			frictionLong = frictionLong*cos(trackAngle);

			double frictionLat = fabs(sqrt(kFLat*(kFLat*sin(trackAngle)*sin(trackAngle)+kFLong*cos(trackAngle)*cos(trackAngle)))*forceZ);
			frictionLat = std::max(frictionLat - 0.05*frictionLat/sqrt(vpoint.x()*vpoint.x() + vpoint.y()*vpoint.y() + 0.001), 0.0); //Aply static friction
			frictionLat = frictionLat*sin(trackAngle);

			double forceX = -frictionLong*cos(Euler.z())-frictionLat*sin(Euler.z());
			double forceY = -frictionLong*sin(Euler.z())+frictionLat*cos(Euler.z());
			tempE.force = Vector3d(forceX, forceY, forceZ);

			// Add current contact point force contribution to the total
			totalE.force +=  tempE.force;
			// Calculate current contact point torque
			tempE.torque = dxElement.cross(tempE.force);
			// Add current contact point torque contribution to the total
			totalE.torque +=  tempE.torque;
		}

		// If there is no ground contact
		else {
			// Set spring contraction to zero
			spp(i) = 0;
			spd(i) = 0;
		}
		sppprev(i) = spp(i);
	}

	// Chech for rogue NaN results
    if (!totalE.force.allFinite())
	{
		safe = false; /** @todo remove the safe safety, it is not used*/
		throw runtime_error("State NAN in PanosGroundReactions force calculation!");
	}

	// if (isnan(totalE.torque.x) or isnan(totalE.torque.y) or isnan(totalE.torque.z)) {
    if (!totalE.torque.allFinite())
	{
		safe = false;
		throw runtime_error("State NAN in PanosGroundReactions torque calculation!");
	}

	// If there is a ground contact write the resulting wrench
	if (contact and safe) {
		wrenchGround.force= orientation * totalE.force;
		wrenchGround.torque= orientation * totalE.torque;
	}
	// Otherwise there is no wrench created
	else {
		wrenchGround.force = Vector3d::Zero();
		wrenchGround.torque = Vector3d::Zero();
	}

	return wrenchGround.force;
}

// Dummy torque calculation function
Vector3d PanosContactPoints::getTorque(SimState_t /* states */, const WrenchSum_t /* wrenchSum */)
{
	return wrenchGround.torque;
}

} // namespace ground_reaction
} // namespace last_letter_lib
