#pragma once

/// Ground reactions interface class

#include <iostream>
#include <memory>
#include <Eigen/Eigen>

#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"

using namespace std;
using Eigen::Vector3d;
using Eigen::Dynamic;
using namespace last_letter_lib::uav_utils;
using last_letter_lib::math_utils::quat2euler;
using last_letter_lib::programming_utils::ParameterManager;
using last_letter_lib::programming_utils::Parametrized;

namespace last_letter_lib {
namespace ground_reaction {

class GroundReaction: public Parametrized
{
public:
    GroundReaction(string name);
    virtual ~GroundReaction() {};

    virtual void initialize_parameters() override;
    virtual void update_parameters() override;
    virtual void set_dt(double dt_p);
    double dt{0.0025};
    Wrench_t wrenchGround;
    double inputSteer, inputBrake;
    double steerAngle_max;
    int chanSteer, chanBrake;
    void setInput(const Input &input);
    void setInputPwm(const InputPwm_t &input);
    virtual Vector3d getForce(const SimState_t states, const WrenchSum_t wrenchSum) = 0;
    virtual Vector3d getTorque(const SimState_t states, const WrenchSum_t wrenchSum) = 0;

private:
    // Store one normalized input, scaled where the model needs it.
    void set_steer(double steer) { inputSteer = steerAngle_max * steer; }
    void set_brake(double brake) { inputBrake = brake; }
};

/// No ground reactions, dummy class
class NoGroundReaction : public GroundReaction
{
public:
	///This is the NoGroundReaction constructor brief description
    NoGroundReaction(string name) : GroundReaction(name) {};
	~NoGroundReaction() {};
	Vector3d getForce(const SimState_t states, const WrenchSum_t wrenchSum);
	Vector3d getTorque(const SimState_t states, const WrenchSum_t wrenchSum);
};


class PanosContactPoints : public GroundReaction
{
public:
	PanosContactPoints(string name);
	~PanosContactPoints() {};
    void initialize_parameters() override;
    void update_parameters() override;

	Eigen::Matrix<double, 3, Dynamic> pointCoords, cpi_up, cpi_down;
	Eigen::Matrix<double, 2, Dynamic> springIndex;
	Eigen::VectorXd materialIndex, spp, sppprev, spd;

	double normVe;
	Vector3d uavpos, we;
	double len, frictForw[4], frictSide[4];
	bool contact, safe;
	int contactPtsNo{0};
	Vector3d getForce(const SimState_t states, const WrenchSum_t wrenchSum) override;
	Vector3d getTorque(const SimState_t states, const WrenchSum_t wrenchSum) override;
};


class PointFriction : public GroundReaction
{
	/// Does not include rolling wheel model
public:
	PointFriction(string name);
	~PointFriction() {};
    void initialize_parameters() override;
    void update_parameters() override;

	Eigen::Matrix<double, 3, Dynamic> pointCoords, cpi_up, cpi_down;
	Eigen::Matrix<double, 2, Dynamic> springIndex;
	Eigen::VectorXd materialIndex, spp, sppprev, spd;

	// double uavpos[3], normVe;
	double normVe;
	Vector3d uavpos, we;
	double len, frict[4];
	bool contact, safe;
	int contactPtsNo{0};
	Vector3d extForce, extTorque;
	Vector3d getForce(const SimState_t states, const WrenchSum_t wrenchSum) override;
	Vector3d getTorque(const SimState_t states, const WrenchSum_t wrenchSum) override;
};

std::unique_ptr<GroundReaction> buildGroundReaction(ParameterManager config);

} // namespace ground_reaction
} // namespace last_letter_lib
