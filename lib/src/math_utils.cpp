#include "iostream"
#include <stdexcept>

#include "last_letter_lib/math_utils.hpp"

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace last_letter_lib
{
    namespace math_utils
    {

        void quat_normalize(Quaterniond *q)
        {
            q->normalize();
        }

        void quat_normal_wcomplete(Quaterniond *q)
        {
            double x = q->x(), y = q->y(), z = q->z();
            double w = sqrt(1.0 - (x * x + y * y + z * z));

            q = new Quaterniond(w, x, y, z);
        }

        void quat_equiv_wpos_get(Quaterniond *q)
        {
            double x, y, z, w;
            if (q->w() < 0.0)
            {
                w = -(q->w());
                x = -(q->x());
                y = -(q->y());
                z = -(q->z());
            }
            q = new Quaterniond(w, x, y, z);
        }

        void quat_equiv_wneg_get(Quaterniond *q)
        {
            double x, y, z, w;
            if (q->w() > 0.0)
            {
                w = -(q->w());
                x = -(q->x());
                y = -(q->y());
                z = -(q->z());
            }
            q = new Quaterniond(w, x, y, z);
        }

        void quat_inverse(const Quaterniond q, Quaterniond *q_inv)
        {
            *q_inv = q.inverse();
        }

        void quat_product(const Quaterniond a, const Quaterniond b, Quaterniond *c)
        {
            double x, y, z, w;
            w = a.w() * b.w() - (a.x() * b.x() + a.y() * b.y() + a.z() * b.z());
            x = (a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y());
            y = (a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x());
            z = (a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w());
            *c = Quaterniond(w, x, y, z);
        }

        // Follow the Tait-Brian convention (yaw->pitch-roll)
        // Returns the Earth->Body orientation quaternion
        // i.e. v_b = q * v_e
        Quaterniond euler2quat(const Vector3d euler)
        {
            // // Alternate implementation
            // double x, y, z, w;
            // double cpsi, spsi, ctheta, stheta, cphi, sphi;
            // cpsi = cos (0.5 * euler.z()); spsi = sin (0.5 * euler.z());
            // ctheta = cos (0.5 * euler.y()); stheta = sin (0.5 * euler.y());
            // cphi = cos (0.5 * euler.x()); sphi = sin (0.5 * euler.x());
            // w = cphi*ctheta*cpsi + sphi*stheta*spsi;
            // x = sphi*ctheta*cpsi - cphi*stheta*spsi;
            // y = cphi*stheta*cpsi + sphi*ctheta*spsi;
            // z = cphi*ctheta*spsi - sphi*stheta*cpsi;
            // Quaterniond q = Quaterniond(w, x, y, z);

            Quaterniond q;
            q = Eigen::AngleAxis<double>(-euler.x(), Vector3d::UnitX()) * Eigen::AngleAxis<double>(-euler.y(), Vector3d::UnitY()) * Eigen::AngleAxis<double>(-euler.z(), Vector3d::UnitZ());

            return q;
        }

        void quat2rotmtx(const Quaterniond q, double *rotmtx)
        {
            double q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
            double q00 = q0 * q0, q11 = q1 * q1, q22 = q2 * q2, q33 = q3 * q3;
            double q01 = q0 * q1, q02 = q0 * q2, q03 = q0 * q3;
            double q12 = q1 * q2, q13 = q1 * q3;
            double q23 = q2 * q3;
            rotmtx[0] = q00 + q11 - q22 - q33;
            rotmtx[1] = 2.0 * (q12 - q03);
            rotmtx[2] = 2.0 * (q02 + q13);
            rotmtx[3] = 2.0 * (q12 + q03);
            rotmtx[4] = q00 - q11 + q22 - q33;
            rotmtx[5] = 2.0 * (q23 - q01);
            rotmtx[6] = 2.0 * (q13 - q02);
            rotmtx[7] = 2.0 * (q01 + q23);
            rotmtx[8] = q00 - q11 - q22 + q33;
        }

        Vector3d quat2euler(const Quaterniond q)
        // Based on Stevens & Lewis p. 42
        // Takes the body-to-Earth quaternion and produces the Euler anlges
        // CAUTION! This quaternion is the conjugate of the one returned from quat2euler
        {
            //	double rotmtx[9];
            Vector3d euler;
            //	quat2rotmtx(q, rotmtx);
            //	euler.x() = atan2(rotmtx[5],rotmtx[8]);
            //	euler.y() = -asin(rotmtx[2]);
            //	euler.z() = atan2(rotmtx[1],rotmtx[0]);
            //
            //	return euler;
            const double tol = 0.499;
            const double q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
            const double q00 = q0 * q0, q11 = q1 * q1, q22 = q2 * q2, q33 = q3 * q3;
            const double q01 = q0 * q1, q23 = q2 * q3;
            const double q12 = q1 * q2, q03 = q0 * q3;
            double test = q1 * q3 - q0 * q2;
            if (test < -tol)
            {
                euler(0) = 0.0;
                euler(1) = 0.5 * M_PI;
                euler(2) = 2.0 * atan2(q0, q3);
                return euler;
            }
            else if (test > +tol)
            {
                euler(0) = 0.0;
                euler(1) = -0.5 * M_PI;
                euler(2) = 2.0 * atan2(q0, q3);
                return euler;
            }
            else
            {
                euler(0) = atan2(2.0 * (q01 + q23), q00 - q11 - q22 + q33);
                euler(1) = asin(-2.0 * test);
                euler(2) = atan2(2.0 * (q12 + q03), q00 + q11 - q22 - q33);
                return euler;
            }
        }

        void euler2rotmtx(const Vector3d euler, double *rotmtx)
        {
            double cpsi, spsi, ctheta, stheta, cphi, sphi;
            /**** Calculate trigonometric values. ****/
            cpsi = cos(euler.z());
            spsi = sin(euler.z());
            ctheta = cos(euler.y());
            stheta = sin(euler.y());
            cphi = cos(euler.x());
            sphi = sin(euler.x());
            /**** Calculate rotation matrix. ****/
            rotmtx[0] = cpsi * ctheta;
            rotmtx[1] = sphi * cpsi * stheta - cphi * spsi;
            rotmtx[2] = cphi * cpsi * stheta + sphi * spsi;
            rotmtx[3] = spsi * ctheta;
            rotmtx[4] = sphi * spsi * stheta + cphi * cpsi;
            rotmtx[5] = cphi * spsi * stheta - sphi * cpsi;
            rotmtx[6] = -stheta;
            rotmtx[7] = sphi * ctheta;
            rotmtx[8] = cphi * ctheta;
        }

        void quat_vector3_rotate(Quaterniond q, Vector3d v, Vector3d *res)
        {
            Quaterniond vq, q_inv, qa, qb;
            vq = Quaterniond(0.0, v.x(), v.y(), v.z());
            quat_inverse(q, &q_inv);
            quat_product(q, vq, &qa);
            quat_product(qa, q_inv, &qb);
            *res = Vector3d(qb.x(), qb.y(), qb.z());
        }

        // Wrap an angle to the interval [0-360)
        double wrap_to_360(const double angle)
        {
            double a = angle;
            while (a < 0)
                a += 360.0;
            while (a >= 360.0)
                a -= 360.0;
            return a;
        }

        // Wrap an angle to the interval [0-2*pi)
        double wrap_to_2pi(const double angle)
        {
            double a = angle;
            while (a < 0)
                a += 2 * M_PI;
            while (a >= 2 * M_PI)
                a -= 2 * M_PI;
            return a;
        }

        double sign(const double value)
        {
            if (value >= 0) {
                return 1;
            } else {
                return -1;
            }
        }

        void multi_mtx_mtx_3X3(double *a, double *b, double *res)
        {
            int iii, jjj, kkk;
            for (iii = 0; iii < 3; ++iii)
            {
                for (jjj = 0; jjj < 3; ++jjj)
                {
                    res[iii * 3 + jjj] = 0.0;
                    for (kkk = 0; kkk < 3; ++kkk)
                        res[iii * 3 + jjj] += a[iii * 3 + kkk] * b[kkk * 3 + jjj];
                }
            }
        }

        void multi_mtx_mtx_3Xn(double *a, double *b, double *res, int n)
        {
            int iii, jjj, kkk;
            for (iii = 0; iii < 3; ++iii)
            {
                for (jjj = 0; jjj < n; ++jjj)
                {
                    res[iii * n + jjj] = 0.0;
                    for (kkk = 0; kkk < 3; ++kkk)
                        res[iii * n + jjj] += a[iii * 3 + kkk] * b[kkk * n + jjj];
                }
            }
        }

        void multi_mtxT_mtx_3X3(double *a, double *b, double *res)
        {
            int iii, jjj, kkk;
            for (iii = 0; iii < 3; ++iii)
            {
                for (jjj = 0; jjj < 3; ++jjj)
                {
                    res[iii * 3 + jjj] = 0.0;
                    for (kkk = 0; kkk < 3; ++kkk)
                        res[iii * 3 + jjj] += a[kkk * 3 + iii] * b[kkk * 3 + jjj];
                }
            }
        }

        void multi_mtxT_mtx_3Xn(double *a, double *b, double *res, int n)
        {
            int iii, jjj, kkk;
            for (iii = 0; iii < 3; ++iii)
            {
                for (jjj = 0; jjj < n; ++jjj)
                {
                    res[iii * n + jjj] = 0.0;
                    for (kkk = 0; kkk < 3; ++kkk)
                        res[iii * n + jjj] += a[kkk * 3 + iii] * b[kkk * n + jjj];
                }
            }
        }

        template <typename T>
        const T &constrain(const T &x, const T &a, const T &b)
        {
            if (x < a)
            {
                return a;
            }
            else if (x > b)
            {
                return b;
            }
            else
            {
                return x;
            }
        }
        // Explicit instantiations to allow using the library without need of its headers
        template const int &constrain<int>(const int &x, const int &a, const int &b);
        template const float &constrain<float>(const float &x, const float &a, const float &b);
        template const double &constrain<double>(const double &x, const double &a, const double &b);

        /**
         * @brief Rescale a number x from the range (in_min, in_max) to the range (out_min, out_max)
         *
         * @tparam T
         */
        template <typename T>
        T map(const T &x, const T &in_min, const T &in_max, const T &out_min, const T &out_max)
        {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
        // Explicit instantiations to allow using the library without need of its headers
        template int map<int>(const int &x, const int &in_min, const int &in_max, const int &out_min, const int &out_max);
        template float map<float>(const float &x, const float &in_min, const float &in_max, const float &out_min, const float &out_max);
        template double map<double>(const double &x, const double &in_min, const double &in_max, const double &out_min, const double &out_max);

        /**
         * @brief Rescale a number x from the range (in_min, in_max) to the range (out_min, out_max), but always preserve zero
         * This function differs form the simple map by splitting the input/output range around zero and applying the simple
         * map to each interval independently
         *
         * @tparam T
         */
        template <typename T>
        T map_centered(const T &x, const T &in_min, const T &in_max, const T &out_min, const T &out_max)
        {
            T zero{0};
            if (x < 0)
            {
                return map(x, in_min, zero, out_min, zero);
            }
            else
            {
                return map(x, zero, in_max, zero, out_max);
            }
        }
        // Explicit instantiations to allow using the library without need of its headers
        template float map_centered<float>(const float &x, const float &in_min, const float &in_max, const float &out_min, const float &out_max);
        template double map_centered<double>(const double &x, const double &in_min, const double &in_max, const double &out_min, const double &out_max);

        double vector3_norm(Vector3d a)
        {
            return a.norm();
        }

        void vector3_cross(Vector3d a, Vector3d b, Vector3d *c)
        {
            double x, y, z;
            x = a.y() * b.z() - a.z() * b.y();
            y = a.z() * b.x() - a.x() * b.z();
            z = a.x() * b.y() - a.y() * b.x();
            *c = Vector3d(x, y, z);
        }

        Vector3d vector3_normalize(Vector3d a)
        {
            return a.normalized();
        }

        // Vector3d operator+(const Vector3d& a, Vector3d& b)
        // {
        //   return a+b;
        // }

        // Vector3d operator-(Vector3d& vec)
        // {
        //   return -vec;
        // }

        // Vector3d operator-(Vector3d& a,Vector3d& b)
        // {
        //   return a-b;
        // }

        Vector3d operator*(const double *mtx, Vector3d &vec)
        {
            double x, y, z;
            x = mtx[0] * vec.x() + mtx[1] * vec.y() + mtx[2] * vec.z();
            y = mtx[3] * vec.x() + mtx[4] * vec.y() + mtx[5] * vec.z();
            z = mtx[6] * vec.x() + mtx[7] * vec.y() + mtx[8] * vec.z();
            Vector3d c = Vector3d(x, y, z);

            return c;
        }

        // Vector3d operator*(const double a, Vector3d& vec)
        // {
        //   return a*vec;
        // }

        Vector3d operator/(const double *mtx, Vector3d &vec)
        {
            double x, y, z;
            x = mtx[0] * vec.x() + mtx[3] * vec.y() + mtx[6] * vec.z();
            y = mtx[1] * vec.x() + mtx[4] * vec.y() + mtx[7] * vec.z();
            z = mtx[2] * vec.x() + mtx[5] * vec.y() + mtx[8] * vec.z();
            Vector3d c = Vector3d(x, y, z);

            return c;
        }

        ////////////////////
        // Class Definitions
        ////////////////////

        Vector3::Vector3(double x, double y, double z)
        {
            vector.x() = x;
            vector.y() = y;
            vector.z() = z;
        }
        Vector3 Vector3::operator+(const Vector3 &v) const
        {
            Vector3d other_vector = v.to_array();
            return Vector3(
                vector.x() + other_vector.x(),
                vector.y() + other_vector.y(),
                vector.z() + other_vector.z());
        }
        Vector3 &Vector3::operator+=(const Vector3 &v)
        {
            Vector3d other_vector = v.to_array();
            vector += other_vector;
            return *this;
        }
        Vector3 Vector3::operator-() const
        {
            return Vector3(-vector.x(), -vector.y(), -vector.z());
        }
        Vector3 Vector3::operator-(const Vector3 &v) const
        {
            Vector3d other_vector = v.to_array();
            return Vector3(
                vector.x() - other_vector.x(),
                vector.y() - other_vector.y(),
                vector.z() - other_vector.z());
        }
        Vector3 &Vector3::operator-=(const Vector3 &v)
        {
            Vector3d other_vector = v.to_array();
            vector -= other_vector;
            return *this;
        }
        Vector3 Vector3::operator*(double c) const
        {
            return Vector3(c * vector.x(), c * vector.y(), c * vector.z());
        }
        Vector3 &Vector3::operator*=(double c)
        {
            vector *= c;
            return *this;
        }
        bool Vector3::operator==(const Vector3 &v) const
        {
            Vector3d other_vec = v.to_array();
            return (vector.x() == other_vec.x()) && (vector.y() == other_vec.y()) && (vector.z() == other_vec.z());
        }
        double Vector3::operator[](const size_t idx) const
        {
            return (to_vector()).at(idx);
        }
        std::string Vector3::to_str() const
        {
            std::stringstream ss;
            ss << vector;
            return ss.str();
        }
        std::string Vector3::repr() const
        {
            std::stringstream ss;
            ss << "Vector3(" << vector.x() << ", " << vector.y() << ", " << vector.z() << ")";
            return ss.str();
        }

        UnitQuaternion::UnitQuaternion(EulerAngles euler)
        {

            UnitQuaternion q_yaw(cos(euler.yaw / 2), 0, 0, sin(euler.yaw / 2));
            UnitQuaternion q_pitch(cos(euler.pitch / 2), 0, sin(euler.pitch / 2), 0);
            UnitQuaternion q_roll(cos(euler.roll / 2), sin(euler.roll / 2), 0, 0);
            auto new_q = q_yaw * q_pitch * q_roll;

            w() = new_q.w();
            x() = new_q.x();
            y() = new_q.y();
            z() = new_q.z();
        }
        UnitQuaternion::UnitQuaternion(Vector3d src, Vector3d dst, double eps)
        {

            double dt = src.dot(dst);
            Vector3d cr = src.cross(dst);

            if ((cr.norm() < eps) && (dt < 0))
            {
                cr.x() = abs(cr.x());
                cr.y() = abs(cr.y());
                cr.z() = abs(cr.z());
                if (cr.x() < cr.y())
                {
                    if (cr.x() < cr.z())
                    {
                        cr = Vector3d(1, 0, 0);
                    }
                    else
                    {
                        cr = Vector3d(0, 0, 1);
                    }
                }
                else
                {
                    if (cr.y() < cr.z())
                    {
                        cr = Vector3d(0, 1, 0);
                    }
                    else
                    {
                        cr = Vector3d(0, 1, 1);
                    }
                }
                w() = 0;
                cr = src.cross(cr);
            }
            else
            {
                w() = dt + sqrt(src.norm() * src.norm() * dst.norm() * dst.norm());
            }

            x() = cr.x();
            y() = cr.y();
            z() = cr.z();

            normalize();
        }

        UnitQuaternion UnitQuaternion::conjugate() const
        {
            auto q = Quaterniond::conjugate();
            return UnitQuaternion(q.w(), q.x(), q.y(), q.z());
        }

        Matrix4d UnitQuaternion::to_prodmat()
        {
            Matrix4d m;
            m << w(), -x(), -y(), -z(),
                x(), w(), -z(), y(),
                y(), z(), w(), -x(),
                z(), -y(), x(), w();
            return m;
        }

        EulerAngles UnitQuaternion::to_euler()
        {
            return EulerAngles(*this);
        }

        Vector3d UnitQuaternion::unitX()
        {
            return Vector3d(
                w() * w() + x() * x() - y() * y() - z() * z(),
                2 * (x() * y() + w() * z()),
                2 * (x() * z() - w() * y()));
        }
        Vector3d UnitQuaternion::unitY()
        {
            return Vector3d(
                2 * (x() * y() - w() * z()),
                w() * w() + y() * y() - x() * x() - z() * z(),
                2 * (y() * z() + w() * x()));
        }
        Vector3d UnitQuaternion::unitZ()
        {
            return Vector3d(
                2 * (x() * z() + w() * y()),
                2 * (y() * z() - w() * x()),
                w() * w() + z() * z() - x() * x() - y() * y());
        }

        Vector4d UnitQuaternion::q_dot(Vector3d omega)
        {
            Vector4d omega_ext(0, omega.x(), omega.y(), omega.z());
            Vector4d res = 0.5 * to_prodmat() * omega_ext;
            return res;
        }

        UnitQuaternion UnitQuaternion::operator*(const UnitQuaternion &other) const
        {
            Quaterniond res = Quaterniond::operator*(other);
            return UnitQuaternion(res);
        }

        UnitQuaternion UnitQuaternion::operator*=(const UnitQuaternion &other)
        {
            Quaterniond::operator*=(other);
            return *this;
        }

        std::string UnitQuaternion::to_str()
        {
            std::ostringstream oss;
            oss << std::setprecision(3)
                << "UnitQuaternion("
                << w() << ", "
                << x() << ", "
                << y() << ", "
                << z()
                << ")";
            return oss.str();
        }

        EulerAngles::EulerAngles(double roll_p, double pitch_p, double yaw_p, bool in_degrees)
        {
            double multiplier{1.0};
            if (in_degrees)
            {
                multiplier *= deg_to_rad(1.0);
            }
            roll = roll_p * multiplier;
            pitch = pitch_p * multiplier;
            yaw = yaw_p * multiplier;
        }

        EulerAngles::EulerAngles(UnitQuaternion q)
        {
            double w = q.w();
            double x = q.x();
            double y = q.y();
            double z = q.z();
            roll = atan2(2 * (w * x + y * z), (w * w + z * z - x * x - y * y));
            if (roll > M_PI)
            {
                roll -= 2 * M_PI;
            }
            else if (roll <= -M_PI)
            {
                roll += 2 * M_PI;
            }
            pitch = asin(2 * (w * y - x * z));
            if (pitch > M_PI)
            {
                pitch -= 2 * M_PI;
            }
            else if (pitch <= -M_PI)
            {
                pitch += 2 * M_PI;
            }
            yaw = atan2(2 * (w * z + x * y), (w * w + x * x - y * y - z * z));
            if (yaw > M_PI)
            {
                yaw -= 2 * M_PI;
            }
            else if (yaw <= -M_PI)
            {
                yaw += 2 * M_PI;
            }
        }
        EulerAngles::EulerAngles(Matrix3d R)
        {
            auto q = UnitQuaternion(R);
            auto e = EulerAngles(q);
            roll = e.roll;
            pitch = e.pitch;
            yaw = e.yaw;
        }
        bool EulerAngles::operator==(const EulerAngles rhs) const
        {
            return (roll == rhs.roll) && (pitch == rhs.pitch) && (yaw == rhs.yaw);
        }
        bool EulerAngles::operator==(const Vector3d rhs) const
        {
            return (roll == rhs.x()) && (pitch == rhs.y()) && (yaw == rhs.z());
        }
        Vector3d EulerAngles::to_vector() { return Vector3d(roll, pitch, yaw); }
        UnitQuaternion EulerAngles::to_quaternion()
        {
            return UnitQuaternion(*this);
        }
        Matrix3d EulerAngles::R_roll()
        {
            Matrix3d m;
            double c_phi = cos(roll);
            double s_phi = sin(roll);
            m << 1, 0, 0, 0, c_phi, s_phi, 0, -s_phi, c_phi;
            return m;
        }
        Matrix3d EulerAngles::R_pitch()
        {
            Matrix3d m;
            double c_pitch = cos(pitch);
            double s_pitch = sin(pitch);
            m << c_pitch, 0, -s_pitch, 0, 1, 0, s_pitch, 0, c_pitch;
            return m;
        }
        Matrix3d EulerAngles::R_yaw()
        {
            Matrix3d m;
            double c_yaw = cos(yaw);
            double s_yaw = sin(yaw);
            m << c_yaw, s_yaw, 0, -s_yaw, c_yaw, 0, 0, 0, 1;
            return m;
        }
        Matrix3d EulerAngles::R_bi()
        {
            return R_ib().transpose();
        }
        Matrix3d EulerAngles::R_ib()
        {
            Matrix3d m = R_roll() * R_pitch() * R_yaw();
            return m;
        }
        Matrix3d EulerAngles::T_eb()
        {
            Matrix3d m;
            double c_roll = cos(roll);
            double s_roll = sin(roll);
            double c_pitch = cos(pitch);
            double s_pitch = sin(pitch);
            m << 1, 0, -s_pitch, 0, c_roll, s_roll * c_pitch, 0, -s_roll, c_roll * c_pitch;
            return m;
        }
        Matrix3d EulerAngles::T_be()
        {
            Matrix3d m;

            double c_roll = cos(roll);
            double s_roll = sin(roll);
            double c_pitch = cos(pitch);
            double t_pitch = tan(pitch);
            double sec_pitch = 1 / c_pitch;
            if (std::isnan(sec_pitch))
            {
                throw std::runtime_error("Euler angles in gimbal lock, can't calculate T_be.");
            }
            m << 1, s_roll * t_pitch, c_roll * t_pitch, 0, c_roll, -s_roll, 0, s_roll * sec_pitch, c_pitch * sec_pitch;
            return m;
        }

        std::string EulerAngles::to_str()
        {
            std::ostringstream oss;
            oss << std::setprecision(3)
                << "EulerAngles(roll="
                << roll << ", pitch="
                << pitch << ", yaw="
                << yaw
                << ")";
            return oss.str();
        }

        ////////////////////
        // Define Polynomial

        Polynomial::Polynomial()
        {
        }

        Polynomial::~Polynomial()
        {
        }

        //////////////////////
        // Define Polynomial1D

        // class constructor
        Polynomial1D::Polynomial1D(int maxOrder, double *coeffArray) : Polynomial()
        {
            int i;
            coeffNo = maxOrder;
            // Create and initialize polynomial coefficients container
            coeffs = (double *)malloc(sizeof(double) * (coeffNo + 1));
            for (i = 0; i <= coeffNo; i++)
            {
                coeffs[i] = coeffArray[i];
            }
        }

        // class destructor
        Polynomial1D::~Polynomial1D()
        {
            free(coeffs);
        }

        // polynomial evaluation
        double Polynomial1D::evaluate(double x)
        {
            int i;
            double sum = 0;
            for (i = 0; i <= coeffNo; i++)
            {
                sum += coeffs[i] * pow(x, i);
            }
            return sum;
        }

        //////////////////////
        // Define Polynomial2D

        // class constructor
        Polynomial2D::Polynomial2D(int maxOrder1, int maxOrder2, double *coeffArray) : Polynomial()
        {
            // Attention! maxOrder2 > maxOrder1. If not, swap the variables!
            int i;
            coeffNo1 = maxOrder1;
            coeffNo2 = maxOrder2;
            // Create and initialize polynomial coefficients container
            int arrayLen = (2 * maxOrder2 + 2 * maxOrder1 * maxOrder2 + maxOrder1 - maxOrder1 * maxOrder1 + 2) / 2;
            coeffs = (double *)malloc(sizeof(double) * arrayLen);
            for (i = 0; i < arrayLen; i++)
            {
                coeffs[i] = coeffArray[i];
            }
        }

        // class destructor
        Polynomial2D::~Polynomial2D()
        {
            free(coeffs);
        }

        // polynomial evaluation
        double Polynomial2D::evaluate(double x, double y)
        {
            int i, j, k = 0;
            double sum = 0;
            for (i = 0; i <= coeffNo1; i++)
            {
                for (j = 0; j <= coeffNo2; j++)
                {
                    if (i + j <= coeffNo2)
                    {
                        sum += coeffs[k] * pow(x, i) * pow(y, j);
                        k++;
                    }
                }
            }
            // std::cout << "2DPoly: " << x << " " << y << " " << sum << std::endl; // Sanity check output
            return sum;
        }

        /////////////////
        // Define Spline3
        // Cubic spline, 4 parameters per variable interval

        // class constructor
        Spline3::Spline3(int breaksNoIn, double *breaksIn, double *coeffsIn) : Polynomial()
        {
            int i;
            breaksNo = breaksNoIn;
            // Create and initialize breaks container
            breaks = (double *)malloc(sizeof(double) * (breaksNo + 1));
            for (i = 0; i <= breaksNo; i++)
            {
                breaks[i] = breaksIn[i];
            }
            // Create and initialize polynomial coefficients container
            coeffs = (double *)malloc(sizeof(double) * (breaksNo * 4));
            for (i = 0; i < (breaksNo * 4); i++)
            {
                coeffs[i] = coeffsIn[i];
            }
        }

        // class destructor
        Spline3::~Spline3()
        {
            free(breaks);
            free(coeffs);
        }

        // polynomial evaluation
        double Spline3::evaluate(double x)
        {
            int i;
            for (i = 0; i < breaksNo; i++)
            {
                if (x <= breaks[i + 1])
                    break;
            }
            if (i == breaksNo)
                i--;
            double delta = x - breaks[i];
            double value = coeffs[4 * i] * pow(delta, 3) + coeffs[4 * i + 1] * pow(delta, 2) + coeffs[4 * i + 2] * delta + coeffs[4 * i + 3];
            return value;
        }

        ///////////////
        // 3D Ellipsoid
        ///////////////

        Ellipsoid3D::Ellipsoid3D(const Ellipsoid3DCoefficients_t params)
        {
            update_coeffs(params);
        }

        void Ellipsoid3D::update_coeffs(const Ellipsoid3DCoefficients_t params)
        {
            coeffs = params; // Copy over parameters
        }

        double Ellipsoid3D::evaluate(double x, double y, double z)
        {
            double result{0};
            result += coeffs.A * x * x;
            result += coeffs.B * y * y;
            result += coeffs.C * z * z;
            result += 2 * coeffs.D * x * y;
            result += 2 * coeffs.E * x * z;
            result += 2 * coeffs.F * y * z;
            result += 2 * coeffs.G * x;
            result += 2 * coeffs.H * y;
            result += 2 * coeffs.I * z;
            result += coeffs.J;
            return result;
        }

        Vector3d Ellipsoid3D::get_center() // Get the ellipsoid center
        {
            Eigen::Matrix3d A;
            A << coeffs.A, coeffs.D, coeffs.E,
                coeffs.D, coeffs.B, coeffs.F,
                coeffs.E, coeffs.F, coeffs.C;
            A *= -1;

            Eigen::Vector3d b;
            b << coeffs.G, coeffs.H, coeffs.I;

            return A.colPivHouseholderQr().solve(b); // Taken from https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
        }

        Vector3d Ellipsoid3D::project_point(Vector3d point) // Project a point onto the ellipsoid
        {
            double eps = 1e-5; // Working accuracy

            Eigen::Vector3d center = get_center(); // Get ellipsoid center
            // std::cout << "***Projecting onto center: " << center.transpose() << std::endl;
            Eigen::Vector3d point_bar = point - center; // Build point shifted by the ellipsoid center
            // Evaluate the given point
            double point_value = evaluate(point.x(), point.y(), point.z());
            // If it is inside the ellipse force it outside
            while (point_value < 0)
            {
                // std::cout << "***Point inside ellipsoid, doubling norm\n";
                point_bar *= 2; // Double the shifted point vecor
                Eigen::Vector3d new_point = point_bar + center;
                point_value = evaluate(new_point.x(), new_point.y(), new_point.z()); // Re-evaluate the point
            }
            int count = 1;
            double lambda = 1;
            while (fabs(point_value) > eps) // While the projected point is not yet close to the ellipsoid surface
            {
                if (point_value > 0)
                {
                    lambda -= 1.0 / pow(2, count);
                    // std::cout << "***Point outside of FE, shrinking\n";
                } // If it is outside, move a division of 2 inwards
                else
                {
                    lambda += 1.0 / pow(2, count);
                    // std::cout << "***Point inside of FE, expanding\n";
                } // If it is inside, move a division of 2 outwards
                Eigen::Vector3d new_point = point_bar * lambda + center;
                // std::cout << "***New point: " << new_point.transpose() << std::endl;
                point_value = evaluate(new_point.x(), new_point.y(), new_point.z()); // Re-evaluate the point
                count++;                                                             // Increase step resolution
                if (count > 100)
                {
                    break;
                }
            }
            return point_bar * lambda + center;
        }

        ///////////////////////
        // Define discrTF class
        ///////////////////////

        // Constructor
        discrTF::discrTF(double *alphaIn, int alphaOrderIn, double *betaIn, int betaOrderIn)
        {
            int i;
            alphaOrder = alphaOrderIn;
            betaOrder = betaOrderIn;

            outputHist = (double *)malloc(sizeof(double) * alphaOrder);
            inputHist = (double *)malloc(sizeof(double) * alphaOrder);

            alpha = (double *)malloc(sizeof(double) * alphaOrder);
            beta = (double *)malloc(sizeof(double) * (betaOrder + 1));
            for (i = 0; i < alphaOrder; i++)
            {
                alpha[i] = alphaIn[i];
            }
            for (i = 0; i <= betaOrder; i++)
            {
                beta[i] = betaIn[i];
            }

            init(0, 0);
        }

        // Destructor
        discrTF::~discrTF()
        {
            free(alpha);
            free(beta);
            free(outputHist);
            free(inputHist);
        }

        // matrix reset
        void discrTF::init(double restInp, double restOut)
        {
            int i;
            for (i = 0; i < alphaOrder; i++)
            {
                outputHist[i] = restOut;
                inputHist[i] = restInp;
            }
        }

        // main step
        double discrTF::step(double input)
        {
            int i;
            double sum = 0;
            for (i = 0; i < alphaOrder; i++)
            { // evaluate past output contribution
                sum -= alpha[i] * outputHist[i];
            }
            for (i = 0; i <= betaOrder; i++)
            { // evaluate input contribution
                sum += beta[i] * inputHist[i];
            }
            for (i = 0; i < (alphaOrder - 1); i++)
            { // Slide history vectors back
                outputHist[i] = outputHist[i + 1];
                inputHist[i] = inputHist[i + 1];
            }
            outputHist[alphaOrder - 1] = sum;
            inputHist[alphaOrder - 1] = input;

            return sum;
        }

        //////////////////////
        // Inertial methods //
        //////////////////////

        Inertial::Inertial(double mass_p)
        {
            mass = mass_p;
            tensor.setIdentity();
        }

        Inertial::Inertial(double mass_p, std::vector<double> j_vec)
        { // Constructor with either a 9-vector or a 3-vector.
            mass = mass_p;

            if (j_vec.size() == 9)
            {
                tensor = Eigen::Map<Eigen::Matrix3d>(&j_vec[0], 3, 3);
            }
            else if (j_vec.size() == 3)
            {
                auto v = Eigen::Map<Eigen::Vector3d>(&j_vec[0], 3);
                tensor = v.asDiagonal();
            }
            else
            {
                throw std::runtime_error("Cannot initialize inertia tensor with " + std::to_string(j_vec.size()) + " elements");
            }
        }

        /////////////////////////////////////////
        // Check for NaN in various structures //
        /////////////////////////////////////////

        bool isnan(Vector3d vec)
        {
            if (vec.hasNaN())
            {
                return true;
            }
            return false;
        }

        bool isnan(Quaterniond q)
        {
            if (q.vec().hasNaN())
            {
                return true;
            }
            if (std::isnan(q.w()))
            {
                return true;
            }
            return false;
        }

        bool isnan_mtx(double *R, int n)
        {
            for (int i = 0; i < n; i++)
            {
                if (std::isnan(R[i]))
                {
                    return true;
                }
            }
            return false;
        }

        bool myisfinite(const Vector3d vec)
        {
            if (!vec.allFinite())
            {
                return false;
            }
            return true;
        }

        bool myisfinite(const Quaterniond q)
        {
            if (!q.vec().allFinite())
            {
                return false;
            }
            if (!std::isfinite(q.w()))
            {
                return false;
            }
            return true;
        }

        bool myisfnite_mtx(double *R, int n)
        {
            for (int i = 0; i < n; i++)
            {
                if (!std::isfinite(R[i]))
                {
                    return false;
                }
            }
            return true;
        }

        //////////////////////////////////////////////
        // Check if 3x3 matrix is positive definite //
        //////////////////////////////////////////////
        // Uses Sylvester's Criterion: https://en.wikipedia.org/wiki/Sylvester%27s_criterion
        int is_pos_def(double *R)
        {
            // Calculate determinant
            double det = R[0] * R[4] * R[8] - R[0] * R[5] * R[7] - R[1] * R[3] * R[8] + R[1] * R[5] * R[6] + R[2] * R[3] * R[7] - R[2] * R[4] * R[6];
            if (det == 0)
            {
                return -1; // Matrix is singular
            }

            // Matrix is not positive definite
            if (R[0] < 0)
            {
                return -2;
            }
            if ((R[0] * R[4] - R[1] * R[3]) < 0)
            {
                return -2;
            }
            if (det < 0)
            {
                return -2;
            }

            // Matrix is positive definite
            return 0;
        } // TODO: This would better return positive on success.

        int is_pos_def(Eigen::Matrix3d R)
        {
            // Calculate determinant
            double det =
                R(0, 0) * R(1, 1) * R(2, 2) - R(0, 0) * R(1, 2) * R(2, 1) - R(0, 1) * R(1, 0) * R(2, 2) + R(0, 1) * R(1, 2) * R(2, 0) + R(0, 2) * R(1, 0) * R(2, 1) - R(0, 2) * R(1, 1) * R(2, 0);
            if (det == 0)
            {
                return -1; // Matrix is singular
            }

            // Matrix is not positive definite
            if (R(0, 0) < 0)
            {
                return -2;
            }
            if ((R(0, 0) * R(1, 1) - R(0, 1) * R(1, 0)) < 0)
            {
                return -2;
            }
            if (det < 0)
            {
                return -2;
            }

            // Matrix is positive definite
            return 0;
        }

    } // namespace math_utils
} // namespace last_letter_lib
