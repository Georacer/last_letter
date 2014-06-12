#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <cstdio>

#define R_earth 6378137.0
#define f_earth 1.0/298.257223563
#define e_earth sqrt(2.0*f_earth - f_earth*f_earth)

extern "C" {
    // LU decomoposition of a general matrix
    void dgetrf_(int* M, int *N, double* A, int* lda, int* IPIV, int* INFO);

    // generate inverse of a matrix given its LU decomposition
    void dgetri_(int* N, double* A, int* lda, int* IPIV, double* WORK, int* lwork, int* INFO);
}


void
quat_normalize (geometry_msgs::Quaternion *q);
void
quat_normal_wcomplete (geometry_msgs::Quaternion *q);
void
quat_inverse (const geometry_msgs::Quaternion q, geometry_msgs::Quaternion *q_inv);
void
quat_equiv_wpos_get (geometry_msgs::Quaternion *q);
void
quat_equiv_wneg_get (geometry_msgs::Quaternion *q);
void
quat_product (const geometry_msgs::Quaternion a, const geometry_msgs::Quaternion b, geometry_msgs::Quaternion *c);

void
euler2quat (const geometry_msgs::Vector3 euler, geometry_msgs::Quaternion *q);
void
quat2euler (const geometry_msgs::Quaternion q, geometry_msgs::Vector3 *euler);
void
quat2rotmtx (const geometry_msgs::Quaternion q, double *rotmtx);
void
euler2rotmtx (const geometry_msgs::Vector3 euler,
		 double *rotmtx);

void
quat_vector3_rotate (geometry_msgs::Quaternion q, geometry_msgs::Vector3 v, geometry_msgs::Vector3 *res);

void
multi_mtx_mtx_3X3 (double *a, double *b, double *res);
void 
multi_mtx_mtx_3Xn(double *a, double *b, double *res,int n);
void
multi_mtxT_mtx_3X3 (double *a, double *b, double *res);
void 
multi_mtxT_mtx_3Xn(double *a, double *b, double *res,int n);

void
vector3_cross(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b, geometry_msgs::Vector3 *c);

geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3& a, geometry_msgs::Vector3& b);

geometry_msgs::Vector3 operator*(const double *mtx, geometry_msgs::Vector3& vec);
geometry_msgs::Vector3 operator/(const double *mtx, geometry_msgs::Vector3& vec); 

geometry_msgs::Vector3 operator*(const double a, geometry_msgs::Vector3& vec) ;

geometry_msgs::Vector3 operator-(geometry_msgs::Vector3& vec) ;

int
inverse(double* A, double* Ainv, int N);


void
WGS84_NM(double lat,double *NE, double *ME);
