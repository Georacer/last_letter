#ifndef MATH_UTILS_
#define MATH_UTILS_

#include <cstdio>
#include <iomanip>
#include <Eigen/Eigen>
#include <cmath>

#define R_earth 6378137.0
#define f_earth 1.0 / 298.257223563
#define e_earth sqrt(2.0 * f_earth - f_earth * f_earth)
#define DOUBLE_EPS 1e-9

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace last_letter_lib
{
	namespace math_utils
	{

		////////////
		// Functions

		template <typename T>
		double rad_to_deg(T angle)
		{
			return angle * 180 / M_PI;
		}
		template <typename T>
		double deg_to_rad(T angle)
		{
			return angle * M_PI / 180;
		}

		void quat_normalize(Quaterniond *q);
		void quat_normal_wcomplete(Quaterniond *q);
		void quat_equiv_wpos_get(Quaterniond *q);
		void quat_equiv_wneg_get(Quaterniond *q);
		void quat_inverse(const Quaterniond q, Quaterniond *q_inv);
		void quat_product(const Quaterniond a, const Quaterniond b, Quaterniond *c);

		Quaterniond euler2quat(const Vector3d euler);
		void quat2rotmtx(const Quaterniond q, double *rotmtx);
		Vector3d quat2euler(const Quaterniond q);
		void euler2rotmtx(const Vector3d euler, double *rotmtx);

		void quat_vector3_rotate(Quaterniond q, Vector3d v, Vector3d *res);
		double wrap_to_360(const double angle);
		double wrap_to_2pi(const double angle);
		double sign(const double value);

		void multi_mtx_mtx_3X3(double *a, double *b, double *res);
		void multi_mtx_mtx_3Xn(double *a, double *b, double *res, int n);
		void multi_mtxT_mtx_3X3(double *a, double *b, double *res);
		void multi_mtxT_mtx_3Xn(double *a, double *b, double *res, int n);
		int is_pos_def(double *R); // Check if 3x3 matrix is positive definite
		int is_pos_def(Eigen::Matrix3d R);

		template <typename T>
		const T &constrain(const T &x, const T &a, const T &b);
		template <typename T>
		T map(const T &x, const T &in_min, const T &in_max, const T &out_min, const T &out_max);
		template <typename T>
		T map_centered(const T &x, const T &in_min, const T &in_max, const T &out_min, const T &out_max);

		//////////////////////////
		// 3D Vector operations //
		//////////////////////////
		double vector3_norm(Vector3d a);
		void vector3_cross(Vector3d a, Vector3d b, Vector3d *c);
		Vector3d vector3_normalize(Vector3d a);

		// Vector3d operator+(const Vector3d& a, Vector3d& b);
		// Vector3d operator-(Vector3d& vec);
		// Vector3d operator-(Vector3d& a,Vector3d& b);
		Vector3d operator*(const double *mtx, Vector3d &vec);
		// Vector3d operator*(const double a, Vector3d& vec);
		Vector3d operator/(const double *mtx, Vector3d &vec);

		//////////
		// Classes

		class Vector3
		{
		public:
			Vector3(double x = 0, double y = 0, double z = 0);
			double get_x() const { return vector.x(); }
			void set_x(double v) { vector.x() = v; }
			double get_y() const { return vector.y(); }
			void set_y(double v) { vector.y() = v; }
			double get_z() const { return vector.z(); }
			void set_z(double v) { vector.z() = v; }
			Vector3d to_array() const { return vector; }
			std::vector<double> to_vector() const
			{
				return std::vector<double>{vector.x(), vector.y(), vector.z()};
			}
			double norm() const
			{
				return vector.norm();
			}
			Vector3 operator+(const Vector3 &v) const;
			Vector3 &operator+=(const Vector3 &v);
			Vector3 operator-() const;
			Vector3 operator-(const Vector3 &v) const;
			Vector3 &operator-=(const Vector3 &v);
			Vector3 operator*(double c) const;
			Vector3 &operator*=(double c);
			friend Vector3 operator*(double c, const Vector3 &v)
			{
				Vector3d res = c * v.vector;
				return Vector3(res.x(), res.y(), res.z());
			}
			bool operator==(const Vector3 &v) const;
			double operator[](const size_t idx) const;
			std::string to_str() const;
			std::string repr() const;

			Vector3d vector{0, 0, 0};
		};

		class EulerAngles;

		class UnitQuaternion : public Quaterniond
		{
		public:
			UnitQuaternion(double w = 1, double x = 0, double y = 0, double z = 0) : Quaterniond(w, x, y, z) { normalize(); }
			UnitQuaternion(Quaterniond q) : Quaterniond(q) {}
			UnitQuaternion(EulerAngles);
			UnitQuaternion(Matrix3d R) : Quaterniond(R) { normalize(); }
			UnitQuaternion(Vector3d, Vector3d, double eps = 1e-5);
			double get_w() const { return w(); }
			void set_w(double v) { w() = v; }
			double get_x() const { return x(); }
			void set_x(double v) { x() = v; }
			double get_y() const { return y(); }
			void set_y(double v) { y() = v; }
			double get_z() const { return z(); }
			void set_z(double v) { z() = v; }
			Vector4d get_coeffs() const { return Vector4d(w(), x(), y(), z()); }
			double real() { return this->w(); }
			Vector3d imag() { return Vector3d(this->x(), this->y(), this->z()); }
			UnitQuaternion flipped() { return UnitQuaternion(-this->w(), this->x(), this->y(), this->z()); }
			UnitQuaternion conjugate() const;
			Matrix4d to_prodmat();
			EulerAngles to_euler();
			Vector3d unitX();
			Vector3d unitY();
			Vector3d unitZ();
			// Matrix3d R_ib() { return toRotationMatrix(); }
			// Matrix3d R_bi() { return inverse().toRotationMatrix(); }
			Matrix3d R_ib() { return inverse().toRotationMatrix(); }
			Matrix3d R_bi() { return toRotationMatrix(); }
			Vector4d q_dot(Vector3d omega);
			bool is_unit() { return fabs(norm() - 1) < DOUBLE_EPS; }
			UnitQuaternion operator*(const UnitQuaternion &) const;
			UnitQuaternion operator*=(const UnitQuaternion &);
			using Quaterniond::operator*; // Unmask the multiplication operator from Quaterniond.
			friend last_letter_lib::math_utils::Vector3 operator*(const UnitQuaternion q, const last_letter_lib::math_utils::Vector3 v)
			{
				Vector3d res = q * v.vector;
				return last_letter_lib::math_utils::Vector3(res.x(), res.y(), res.z());
			}
			std::string to_str();
		};

		class EulerAngles
		{
		public:
			EulerAngles(double roll_p = 0, double pitch_p = 0, double yaw_p = 0, bool in_degrees = false);
			EulerAngles(UnitQuaternion);
			EulerAngles(Matrix3d R);
			bool operator==(const EulerAngles) const;
			bool operator==(const Vector3d) const;
			Vector3d to_vector();
			UnitQuaternion to_quaternion();
			Matrix3d R_roll();
			Matrix3d R_pitch();
			Matrix3d R_yaw();
			Matrix3d R_bi();
			Matrix3d R_ib();
			Matrix3d T_eb();
			Matrix3d T_be();
			std::string to_str();

			double roll;
			double pitch;
			double yaw;
		};

		EulerAngles build_euler_from_vector();

		class Polynomial
		{
		public:
			double *coeffs;

			Polynomial();
			virtual ~Polynomial();
			virtual double evaluate() { return 0; }
			virtual double evaluate(double) { return 0; }
			virtual double evaluate(double, double) { return 0; }
		};

		class Polynomial1D : public Polynomial
		{
		public:
			double coeffNo;

			Polynomial1D(int maxOrder, double *coeffArray);
			~Polynomial1D();
			double evaluate(double x);
		};

		class Polynomial2D : public Polynomial
		{
		public:
			double coeffNo1, coeffNo2;

			Polynomial2D(int maxOrder1, int maxOrder2, double *coeffArray);
			~Polynomial2D();
			double evaluate(double x, double y);
			// Important notes: maxOrder of variable y must be greater or equal to maxOrder of variable x
			// Pass the coefficient array with the following ordering (eg for maxOrder1=1, maxOrder2=3):
			// [00 01 02 03 10 11 12]
		};

		class Spline3 : public Polynomial
		{
		public:
			int breaksNo;
			double *breaks;

			Spline3(int breaksNoIn, double *breaksIn, double *coeffsIn);
			~Spline3();
			double evaluate(double x);
		};

		// Parameters for the definition of the Flight Envelope ellipsoid
		// Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
		struct Ellipsoid3DCoefficients_t
		{
			double A;
			double B;
			double C;
			double D;
			double E;
			double F;
			double G;
			double H;
			double I;
			double J;
		};

		class Ellipsoid3D
		{
		public:
			Ellipsoid3DCoefficients_t coeffs;

			Ellipsoid3D(const Ellipsoid3DCoefficients_t); // Constructor
			void update_coeffs(const Ellipsoid3DCoefficients_t);
			double evaluate(double x, double y, double z);
			Vector3d get_center();					// Get the ellipsoid center
			Vector3d project_point(Vector3d point); // Project a point onto the ellipsoid
		};

		// Discrete transfer function implementation
		// for strictly proper TFs
		class discrTF
		{
		public:
			///////////
			// Variables
			double *alpha, *beta;
			double *outputHist, *inputHist;
			int alphaOrder, betaOrder;

			///////////
			// Functions

			// Constructor
			discrTF(double *alphaIn, int alphaOrderIn, double *betaIn, int betaOrderIn);

			// Destructor
			~discrTF();

			// matrix reset
			void init(double restInp, double restOut);

			// main step
			double step(double input);
		};

		class Inertial
		{
		public:
			Inertial(double mass = 0.0);							 // Constructor with mass only.
			Inertial(double /*mass*/, std::vector<double> /*j_***/); // Constructor with either a 9-vector or a 3-vector.
			double mass;
			Eigen::Matrix3d tensor;
		};

		void WGS84_NM(double lat, double *NE, double *ME);

		/////////////////////////////////////////
		// Check for NaN in various structures //
		/////////////////////////////////////////
		bool isnan(Vector3d vec);
		bool isnan(Quaterniond q);
		bool isnan_mtx(double *R, int n);

		bool myisfinite(const Vector3d vec);
		bool myisfinite(const Quaterniond q);
		bool myisfinite_mtx(double *R, int n);

		///////////////////////////////////////
		// Numerical derivative functionalities
		///////////////////////////////////////

		using namespace Eigen;
		template <typename F>
		VectorXd derivative_partial_numerical(F funcPtr, VectorXd values)
		// Version without parameters
		{

			double de = 1e-4; // Numerical differentiation step

			uint num_inputs = values.size();

			VectorXd result = VectorXd(num_inputs);

			for (int i = 0; i < num_inputs; ++i)
			{
				// Setup the argument shift
				VectorXd eps = VectorXd::Zero(num_inputs);
				eps(i) = de;
				VectorXd x_plus = values + eps;
				VectorXd x_minus = values - eps;

				double f_plus, f_minus;

				f_plus = funcPtr(x_plus);
				f_minus = funcPtr(x_minus);

				result(i) = (f_plus - f_minus) / (2 * de);
			}

			return result;
		}

		template <typename F>
		VectorXd derivative_partial_numerical(F funcPtr, VectorXd values, VectorXd parameters)
		// Version with parameters
		{

			double de = 1e-4; // Numerical differentiation step

			uint num_inputs = values.size();

			VectorXd result = VectorXd(num_inputs);

			for (int i = 0; i < num_inputs; ++i)
			{
				// Setup the argument shift
				VectorXd eps = VectorXd::Zero(num_inputs);
				eps(i) = de;
				VectorXd x_plus = values + eps;
				VectorXd x_minus = values - eps;

				double f_plus, f_minus;

				f_plus = funcPtr(x_plus, parameters);
				f_minus = funcPtr(x_minus, parameters);

				result(i) = (f_plus - f_minus) / (2 * de);
			}

			return result;
		}

	} // namespace math_utils
} // namespace last_letter_lib

#endif
