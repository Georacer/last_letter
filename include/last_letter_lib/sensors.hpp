
///////////////////////////////////////
// Sensor class related declarations //
///////////////////////////////////////


#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/environment.hpp"

using Eigen::Vector3d;

using namespace last_letter_lib::uav_utils;
using namespace last_letter_lib::programming_utils;

namespace last_letter_lib
{

    enum class sensor_t
    {
        IMU = 0,
        AHRS,
        BAROMETER,
        AIRDATA,
        GPS,
        LIDAR,
        MAVLINK_HIL_STATE_QUATERNION
    };

    ////////////////////////////////////////
    // Sensor interface class declaration //
    ////////////////////////////////////////
    class Sensor
    {
    public:
        ////////////
        // Variables
        sensor_t type;
        const double noise_mean_{0};
        const double noise_stddev_{1};
        std::default_random_engine rn_generator_;
        std::normal_distribution<double> noise_distribution_{noise_mean_, noise_stddev_};

        ////////////
        // Functions
        Sensor();
        virtual ~Sensor();
        void init(ParameterManager config);
        void update(const SimState_t, const Environment_t); // General class for sensor updating
        void read_base_parameters(ParameterManager);
        double generate_noise(double std_dev);
        virtual void read_parameters(ParameterManager) = 0;

    protected:
        SimState_t uav_state_;      // Stored UAV state
        Environment_t environment_; // Stored environment state

    private:
        virtual void update_sensor_() = 0; // Interface class for running individual sensor code
    };

    class Imu : public Sensor
    {
    public:
        ////////////
        // Variables
        Vector3d accelerometer_reading;
        Vector3d gyroscope_reading;
        Vector3d magnetometer_reading;

        ////////////
        // Functions
        Imu();
        virtual ~Imu();
        void read_parameters(ParameterManager);

    private:
        void update_sensor_(); // Interface class for running individual sensor code
        float home_lat_deg_{0};
        float home_lon_deg_{0};
    };

    // class Ahrs : public Sensor
    // {
    // public:
    //     ////////////
    //     // Functions
    //     Ahrs(ParameterManager config);
    //     virtual ~Ahrs();

    // private:
    //     void update_sensor_(); // Interface class for running individual sensor code
    // };

    class Barometer : public Sensor
    {
    public:
        ////////////
        // Variables
        double temperature_reading;
        double pressure_reading;
        double altitude_reading;
        double rho;

        ////////////
        // Functions
        Barometer();
        virtual ~Barometer();
        void read_parameters(ParameterManager);

    private:
        void update_sensor_(); // Interface class for running individual sensor code
    };

    class AirdataSensor : public Sensor
    {
    public:
        ////////////
        // Variables
        double differential_pressure;
        double temperature;
        double static_pressure;
        double aoa;
        double aos;

        ////////////
        // Functions
        AirdataSensor();
        virtual ~AirdataSensor();
        void read_parameters(ParameterManager);

    private:
        Barometer barometer_;
        void update_sensor_(); // Interface class for running individual sensor code
    };

    class Gnss : public Sensor
    {
    public:
        ////////////
        // Variables
        int fix_type;
        double latitude{0};  // Latitude in degrees
        double longitude{0}; // Longitude in degrees
        double altitude{0};  // Altitude in meters
        double eph{1}, epv{1};
        Vector3d velocity_ned;
        double evh{1}, evv{1};
        double course_over_ground{0};
        int satellites{0};

        ////////////
        // Functions
        Gnss();
        virtual ~Gnss();
        void read_parameters(ParameterManager);

    private:
        float home_lat_deg_{0};
        float home_lon_deg_{0};

        ////////////
        // Functions
        void update_sensor_(); // Interface class for running individual sensor code
    };

    // class Lidar : public Sensor
    // {
    // public:
    //     ////////////
    //     // Functions
    //     Lidar(ParameterManager config);
    //     virtual ~Lidar();

    // private:
    //     void update_sensor_(); // Interface class for running individual sensor code
    // };

    class MavlinkHilStateQuaternion : public Sensor
    {
    public:
        ////////////
        // Variables
        Quaterniond attitude;         // NED to Body Frame quaterion
        Vector3d velocity_angular;    // Body-frame angular velocity
        Vector3d acceleration_linear; // Linear Acceleration in body-frame
        double latitude{0};
        double longitude{0};
        double altitude{0};
        Vector3d velocity_ned; // Linear velocity in NED frame
        double airspeed_indicated;
        double airspeed_true;

        ////////////
        // Functions
        MavlinkHilStateQuaternion();
        virtual ~MavlinkHilStateQuaternion();
        void read_parameters(ParameterManager);
        float home_lat_deg_{0};
        float home_lon_deg_{0};

    private:
        void update_sensor_(); // Interface class for running individual sensor code
    };

}
