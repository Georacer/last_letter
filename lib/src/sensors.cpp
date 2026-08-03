#include <Eigen/Eigen>

#include "last_letter_lib/sensors.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include <last_letter_lib/geo_mag_declination.hpp>

// using namespace std;
using namespace last_letter_lib::programming_utils;

using Eigen::Quaterniond;

namespace last_letter_lib
{

	//////////////////////
	// Define Sensor class
	//////////////////////

	Sensor::Sensor()
	{
	}

	Sensor::~Sensor()
	{
	}

	void Sensor::init(ParameterManager config)
	{
		read_base_parameters(config);
		read_parameters(config);
	}

	void Sensor::read_base_parameters(ParameterManager)
	{
	}

	// void Sensor::read_parameters(ParameterManager)
	// {
	// }

	void Sensor::update(const SimState_t state, const Environment_t environment)
	{
		uav_state_ = state;
		environment_ = environment;
		update_sensor_();
	}

	// void Sensor::update_sensor_()
	// {
	// }

	/**
	 * @brief Generate zero-mean white noise of given standard deviation.
	 *
	 * @param std_dev
	 * @return double
	 */
	double Sensor::generate_noise(double std_dev)
	{
		return std_dev * noise_distribution_(rn_generator_);
	}

	///////////////////
	// Define Imu class
	///////////////////

	Imu::Imu()
	{
		type = sensor_t::IMU;
	}

	Imu::~Imu()
	{
	}

	void Imu::read_parameters(ParameterManager config)
	{
		ParameterManager initConfig = config.filter("init");
		auto coordinates = initConfig.get<std::vector<double>>("coordinates");
		home_lat_deg_ = coordinates.at(0);
		home_lon_deg_ = coordinates.at(1);
	}

	void Imu::update_sensor_()
	{
		// Insert acceleration information. Gravity component needs to be removed, i.e. real-world accelerometer reading is expected.
		// Manually remove gravity component
		Quaterniond q_ned_link = uav_state_.pose.orientation;
		double g = 9.81;
		Vector3d gravity_ned = Vector3d(0, 0, -g);
		// Rotate gravity to body frame
		Vector3d gravity_b = q_ned_link * gravity_ned;
		// Needs conversion from FLU to Body-frame
		Vector3d acceleration_link = q_ned_link * uav_state_.acceleration.linear;
		Vector3d acceleration_imu = acceleration_link + gravity_b;
		accelerometer_reading.x() = acceleration_imu.x() + generate_noise(0.001);
		accelerometer_reading.y() = acceleration_imu.y() + generate_noise(0.001);
		accelerometer_reading.z() = acceleration_imu.z() + generate_noise(0.001);

		// Insert angular velocity information
		// Needs conversion from FLU to Body-frame
		Vector3d velocity_angular_link = q_ned_link * uav_state_.velocity.angular;
		gyroscope_reading.x() = velocity_angular_link.x() + generate_noise(0.001);
		gyroscope_reading.y() = velocity_angular_link.y() + generate_noise(0.001);
		gyroscope_reading.z() = velocity_angular_link.z() + generate_noise(0.001);

		// Insert magnetometer information
		// Magnetic field data from WMM2018 (10^5xnanoTesla (N, E D) n-frame), using the geo_mag_declination library
		// Magnetic field components are calculated by http://geomag.nrcan.gc.ca/mag_fld/comp-en.php

		// Generate magnetic declination and inclination (radians)
		float declination_rad = get_mag_declination(home_lat_deg_, home_lon_deg_) * M_PI / 180;
		float inclination_rad = get_mag_inclination(home_lat_deg_, home_lon_deg_) * M_PI / 180;
		// Generate magnetic strength (10^5xnanoTesla)
		float strength_ga_ct = get_mag_strength(home_lat_deg_, home_lon_deg_); // Get magnetic field strength in centi Tesla
		float strength_ga = 0.01f * strength_ga_ct;							   // Convert to Gauss
		// Resulting field in in NED
		float H = strength_ga * cosf(inclination_rad); // Magnetic field horizontal component
		float mag_Z = sinf(inclination_rad) * strength_ga;
		float mag_X = H * cosf(declination_rad);
		float mag_Y = H * sinf(declination_rad);
		Vector3d mag_e{mag_X, mag_Y, mag_Z};
		// Rotate magnetic field to body frame
		Vector3d mag_b = q_ned_link * mag_e;
		// Add noise
		magnetometer_reading.x() = mag_b.x() + generate_noise(0.002);
		magnetometer_reading.y() = mag_b.y() + generate_noise(0.002);
		magnetometer_reading.z() = mag_b.z() + generate_noise(0.002);
	}

	Barometer::Barometer()
	{
		type = sensor_t::BAROMETER;
	}

	Barometer::~Barometer()
	{
	}

	void Barometer::read_parameters(ParameterManager)
	{
	}

	void Barometer::update_sensor_()
	{
		// Insert barometer information
		// calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
		const float lapse_rate = 0.0065f;		   // reduction in temperature with altitude (Kelvin/m)
		const float temperature_msl = 288.0f;	   // temperature at MSL (Kelvin)
		float alt_msl = uav_state_.geoid.altitude; // TODO: initialize with correct home altitude
		float temperature_local = temperature_msl - lapse_rate * alt_msl;
		float pressure_ratio = powf((temperature_msl / temperature_local), 5.256f);
		const float pressure_msl = 101325.0f; // pressure at MSL
		float absolute_pressure = pressure_msl / pressure_ratio;
		// calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
		const float density_ratio = powf((temperature_msl / temperature_local), 4.256f);
		rho = 1.225f / density_ratio;
		// calculate pressure altitude
		float pressure_altitude = alt_msl;
		// calculate temperature in Celsius
		temperature_reading = temperature_local - 273.0 + generate_noise(0.1);
		// Add noise
		pressure_reading = absolute_pressure + generate_noise(0.00001);
		altitude_reading = pressure_altitude + generate_noise(0.002);
	}

	AirdataSensor::AirdataSensor()
	{
		type = sensor_t::AIRDATA;
	}

	AirdataSensor::~AirdataSensor()
	{
	}

	void AirdataSensor::read_parameters(ParameterManager)
	{
	}

	void AirdataSensor::update_sensor_()
	{
		barometer_.update(uav_state_, environment_);

		Quaterniond q_ned_link = uav_state_.pose.orientation;
		// Insert differential pressure information
		Vector3d airspeed_ned = uav_state_.velocity.linear - environment_.wind;
		Vector3d airspeed_body = q_ned_link * airspeed_ned;
		// Apply scaling factor error
		// airspeed_body /= 1.29;
		differential_pressure = 0.5f * barometer_.rho * powf(airspeed_body.x() + generate_noise(0.005), 2.0);
		static_pressure = barometer_.pressure_reading;
		temperature = barometer_.temperature_reading;
	}

	Gnss::Gnss()
	{
		type = sensor_t::GPS;
	}

	Gnss::~Gnss()
	{
	}

	void Gnss::read_parameters(ParameterManager config)
	{
		ParameterManager initConfig = config.filter("init");
		auto coordinates = initConfig.get<std::vector<double>>("coordinates");
		home_lat_deg_ = coordinates.at(0);
		home_lon_deg_ = coordinates.at(1);
	}

	void Gnss::update_sensor_()
	{
		// Assume 3D fix available always
		fix_type = 3;
		// Insert coordinate information
		// Small-angle approximation of the coordinates around the home location
		double p_north = uav_state_.pose.position.x() + generate_noise(0.1);
		double p_east = uav_state_.pose.position.y() + generate_noise(0.1);
		auto coords = uav_utils::reproject(p_north, p_east, home_lat_deg_, home_lon_deg_);
		latitude = coords.first;
		longitude = coords.second;
		altitude = uav_state_.geoid.altitude;
		eph = 1.0;
		epv = 1.0;
		evh = 1.0;
		evv = 1.0;

		// Insert inertial velocity information
		velocity_ned = uav_state_.velocity.linear;

		// Calculate course over ground
		double cog{atan2(uav_state_.velocity.linear.y(), uav_state_.velocity.linear.x())};
		course_over_ground = math_utils::wrap_to_2pi(cog);
		satellites = 10;
	}

	MavlinkHilStateQuaternion::MavlinkHilStateQuaternion()
	{
		type = sensor_t::MAVLINK_HIL_STATE_QUATERNION;
	}

	MavlinkHilStateQuaternion::~MavlinkHilStateQuaternion()
	{
	}

	void MavlinkHilStateQuaternion::read_parameters(ParameterManager config)
	{
		ParameterManager initConfig = config.filter("init");
		auto coordinates = initConfig.get<std::vector<double>>("coordinates");
		home_lat_deg_ = coordinates.at(0);
		home_lon_deg_ = coordinates.at(1);
	}

	void MavlinkHilStateQuaternion::update_sensor_()
	{
		attitude = uav_state_.pose.orientation;
		velocity_angular = attitude * uav_state_.velocity.angular; // Body-frame angular velocity
		acceleration_linear = attitude * uav_state_.acceleration.linear;
		Vector3d position_ned = uav_state_.pose.position;
		double p_north = position_ned.x();
		double p_east = position_ned.y();
		auto coords = uav_utils::reproject(p_north, p_east, home_lat_deg_, home_lon_deg_);
		latitude = coords.first;
		longitude = coords.second;
		altitude = uav_state_.geoid.altitude;
		velocity_ned = uav_state_.velocity.linear;
		Vector3d airspeed_ned = uav_state_.velocity.linear - environment_.wind;
		Vector3d airspeed_body = attitude * airspeed_ned;
		airspeed_indicated = airspeed_body.x() / 1.29;
		airspeed_true = airspeed_body.x();
	}

}
