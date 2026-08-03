#pragma once

#include <Eigen/Eigen>

#include <last_letter_lib/math_utils.hpp>
#include "last_letter_lib/prog_utils.hpp"
#include <last_letter_lib/uav_utils.hpp>

using namespace std;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using last_letter_lib::uav_utils::SimState_t;
using last_letter_lib::uav_utils::Geoid;
using last_letter_lib::uav_utils::Wrench_t;
using last_letter_lib::math_utils::Inertial;
using namespace last_letter_lib::programming_utils;

#define E2_EARTH Geoid::WGS84_e2
#define RP_EARTH Geoid::WGS84_Ra * (1.0 - E2_EARTH)

#define grav_const 3.986004418e14
#define grav_temp (2.0 / Geoid::WGS84_Ra) * (1.0 + Geoid::EARTH_flattening + (Geoid::EARTH_Omega * Geoid::EARTH_Omega) * (Geoid::WGS84_Ra * Geoid::WGS84_Ra) * RP_EARTH / grav_const)


namespace last_letter_lib
{
	class GravityModel
	{
	public:
        GravityModel() = default;
        ~GravityModel() = default;
        virtual void calcGravity(const SimState_t states)=0; // perform the gravity calculations
        Wrench_t getWrench(const SimState_t states, const Inertial inertial);
		void setG(double v) {_g = v;}

	private:
		double _g;
	};

	class GravitySimple : public GravityModel
	{
	public:
		GravitySimple();
		~GravitySimple() = default;
        void calcGravity(const SimState_t /*states*/) override {}; // perform the gravity calculations
	};

	class GravityClassic : public GravityModel
	{
	public:
		GravityClassic() = default;
		~GravityClassic() = default;
        void calcGravity(const SimState_t states) override; // perform the gravity calculations
	};

GravityModel *buildGravity(ParameterManager config);

} // namespace last_letter_lib
