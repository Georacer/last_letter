#include "mathutils/utls.hpp"
#include <cmath>

void
quat_normalize (geometry_msgs::Quaternion *q)
{
  double w = q->w, x = q->x, y = q->y, z = q->z;
  double norm = sqrt (w*w + x*x + y*y + z*z);
  q -> w /= norm;
  q -> x /= norm;
  q -> y /= norm;
  q -> z /= norm;
}

void
quat_normal_wcomplete (geometry_msgs::Quaternion *q)
{
  double x = q->x, y = q->y, z = q->z;
  q -> w = sqrt (1.0 - (x*x + y*y + z*z));
}

void
quat_equiv_wpos_get (geometry_msgs::Quaternion *q)
{
  if (q -> w < 0.0)
    {
      q->w = -(q->w);
      q->x = -(q->x); q->y = -(q->y); q->z = -(q->z);
    }
}

void
quat_equiv_wneg_get (geometry_msgs::Quaternion *q)
{
  if (q -> w > 0.0)
    {
      q->w = -(q->w);
      q->x = -(q->x); q->y = -(q->y); q->z = -(q->z);
    }
}

void
quat_inverse (const geometry_msgs::Quaternion q, geometry_msgs::Quaternion *q_inv)
{
  double w = q.w, x = q.x, y = q.y, z = q.z;
  double norm;
  norm = sqrt (w*w + x*x + y*y + z*z);
  q_inv -> w = +w / norm;
  q_inv -> x = -x / norm;
  q_inv -> y = -y / norm;
  q_inv -> z = -z / norm;
}

void
quat_product (const geometry_msgs::Quaternion a, const geometry_msgs::Quaternion b, geometry_msgs::Quaternion *c)
{
  c -> w = a.w*b.w - (a.x*b.x + a.y*b.y + a.z*b.z);
  c -> x = (a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y);
  c -> y = (a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x);
  c -> z = (a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w);
}



void
euler2quat (const geometry_msgs::Vector3 euler, geometry_msgs::Quaternion *q)
{
  double cpsi, spsi, ctheta, stheta, cphi, sphi;
  cpsi = cos (0.5 * euler.z); spsi = sin (0.5 * euler.z);
  ctheta = cos (0.5 * euler.y); stheta = sin (0.5 * euler.y);
  cphi = cos (0.5 * euler.x); sphi = sin (0.5 * euler.x);
  q -> w = cphi*ctheta*cpsi + sphi*stheta*spsi;
  q -> x = sphi*ctheta*cpsi - cphi*stheta*spsi;
  q -> y = cphi*stheta*cpsi + sphi*ctheta*spsi;
  q -> z = cphi*ctheta*spsi - sphi*stheta*cpsi;
}

void
quat2euler (const geometry_msgs::Quaternion q, geometry_msgs::Vector3 *euler)
/*{
  const double tol = 0.49999;
  const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
  const double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
  const double q01 = q0*q1, q23 = q2*q3;
  const double q12 = q1*q2, q03 = q0*q3;
  double test = q1*q3 - q0*q2;
  if (test < -tol)
    {
      euler->x = 0.0;
      euler->y = 0.5 * M_PI;
      euler->z = 2.0 * atan2 (q0, q3);
      return;
    }
  else if (test > +tol)
    {
      euler->x = 0.0;
      euler->y = -0.5 * M_PI;
      euler->z = 2.0 * atan2 (q0, q3);
      return;
    }
  else
    {
      euler->x = atan2 (2.0*(q01 + q23), q00 - q11 - q22 + q33);
      euler->y = asin (-2.0*test);
      euler->z = atan2 (2.0*(q12 + q03), q00 + q11 - q22 - q33);
      return;
    }
}*/

{
  const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
  const double q00 = q0*q0, q01 = q0*q1, q02 = q0*q2, q03 = q0*q3;
  const double q11 = q1*q1, q12 = q1*q2, q13 = q1*q3;
  const double q22 = q2*q2, q23 = q2*q3;
  const double q33 = q3*q3;
  euler->z = atan2 (2 * (q03 + q12), 1 - 2 * (q22 - q33));
  if (euler->z<0) {
   	euler->z+=2 * M_PI;
  }
  euler->y = asin (2 * (q02 - q13));
  euler->x = atan2 (2 * (q01 + q23), 1 - 2 * (q11 + q22));
  
  
  //euler->z = atan2 (2 * (q03 + q12), 2 * (q00 + q11) - 1);
  //euler->y = asin (-2 * (q13 - q02));
  //euler->x = atan2 (2 * (q23 + q01), 2 * (q00 + q33) -1);
  
  
  
}


void
quat2rotmtx (const geometry_msgs::Quaternion q, double *rotmtx)
{
  const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
  const double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
  const double q01 = q0*q1, q02 = q0*q2, q03 = q0*q3; 
  const double q12 = q1*q2, q13 = q1*q3;
  const double q23 = q2*q3;
  rotmtx [0] = q00 + q11 - q22 - q33;
  rotmtx [1] = 2.0*(q12 - q03);
  rotmtx [2] = 2.0*(q02 + q13);
  rotmtx [3] = 2.0*(q12 + q03);
  rotmtx [4] = q00 - q11 + q22 - q33;
  rotmtx [5] = 2.0*(q23 - q01);
  rotmtx [6] = 2.0*(q13 - q02);
  rotmtx [7] = 2.0*(q01 + q23);
  rotmtx [8] = q00 - q11 - q22 + q33;
  /*rotmtx [0] = 2 * (q00 + q11) -1;
  rotmtx [1] = 2 * (q12 + q03);
  rotmtx [2] = 2 * (q13 - q02);
  rotmtx [3] = 2 * (q12 - q03);
  rotmtx [4] = 2 * (q00 + q22) -1;
  rotmtx [5] = 2 * (q23 + q01);
  rotmtx [6] = 2 * (q13 + q02);
  rotmtx [7] = 2 * (q23 - q01);
  rotmtx [8] = 2 * (q00 + q33) -1;*/
}

void
euler2rotmtx (const geometry_msgs::Vector3 euler, double *rotmtx)
{
  double cpsi, spsi, ctheta, stheta, cphi, sphi;
  /**** Calculate trigonometric values. ****/
  cpsi = cos (euler.z); spsi = sin (euler.z);
  ctheta = cos (euler.y); stheta = sin (euler.y);
  cphi = cos (euler.x); sphi = sin (euler.x);
  /**** Calculate rotation matrix. ****/
  rotmtx [0] = cpsi * ctheta;
  rotmtx [1] = sphi * cpsi * stheta - cphi * spsi;
  rotmtx [2] = cphi * cpsi * stheta + sphi * spsi;
  rotmtx [3] = spsi * ctheta;
  rotmtx [4] = sphi * spsi * stheta + cphi * cpsi;
  rotmtx [5] = cphi * spsi * stheta - sphi * cpsi;
  rotmtx [6] = -stheta;
  rotmtx [7] = sphi * ctheta;
  rotmtx [8] = cphi * ctheta;
}

void
quat_vector3_rotate (geometry_msgs::Quaternion q, geometry_msgs::Vector3 v, geometry_msgs::Vector3 *res)
{
  geometry_msgs::Quaternion vq, q_inv, qa, qb;
  vq.w = 0.0; vq.x = v.x; vq.y = v.y; vq.z = v.z;
  quat_inverse (q, & q_inv);
  quat_product (q, vq, & qa);
  quat_product (qa, q_inv, & qb);
  res -> x = qb.x;
  res -> y = qb.y;
  res -> z = qb.z;
}


void 
multi_mtx_mtx_3X3(double *a, double *b, double *res)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < 3; ++ jjj) {
      res [iii * 3 + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * 3 + jjj] += a [iii * 3 + kkk] * b [kkk * 3 + jjj];
    }
  }
}

void 
multi_mtx_mtx_3Xn(double *a, double *b, double *res,int n)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < n; ++ jjj) {
      res [iii * n + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * n + jjj] += a [iii * 3 + kkk] * b [kkk*n + jjj];
    }
  }
}


void
multi_mtxT_mtx_3X3 (double *a, double *b, double *res)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < 3; ++ jjj) {
      res [iii * 3 + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * 3 + jjj] += a [kkk * 3 + iii] * b [kkk* 3 + jjj];
    }
  }
}

void 
multi_mtxT_mtx_3Xn(double *a, double *b, double *res,int n)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < n; ++ jjj) {
      res [iii * n + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * n + jjj] += a [kkk * 3 + iii] * b [kkk*n + jjj];
    }
  }
}


void
vector3_cross(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b, geometry_msgs::Vector3 *c)
{
	c->x = a.y*b.z - a.z*b.y;
	c->y = a.z*b.x - a.x*b.z;
	c->z = a.x*b.y - a.y*b.x;
}

geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3& a, geometry_msgs::Vector3& b) 
{
    geometry_msgs::Vector3 c;
    c.x = a.x+b.x;
    c.y = a.y+b.y;
    c.z = a.z+b.z;   

    return c;
}

geometry_msgs::Vector3 operator*(const double *mtx, geometry_msgs::Vector3& vec) 
{
    geometry_msgs::Vector3 c;
    c.x = mtx[0]*vec.x+mtx[1]*vec.y+mtx[2]*vec.z;
    c.y = mtx[3]*vec.x+mtx[4]*vec.y+mtx[5]*vec.z;
    c.z = mtx[6]*vec.x+mtx[7]*vec.y+mtx[8]*vec.z;

    return c;
}

geometry_msgs::Vector3 operator/(const double *mtx, geometry_msgs::Vector3& vec) 
{
    geometry_msgs::Vector3 c;
    c.x = mtx[0]*vec.x+mtx[3]*vec.y+mtx[6]*vec.z;
    c.y = mtx[1]*vec.x+mtx[4]*vec.y+mtx[7]*vec.z;
    c.z = mtx[2]*vec.x+mtx[5]*vec.y+mtx[8]*vec.z; 

    return c;
}

geometry_msgs::Vector3 operator-(geometry_msgs::Vector3& vec) 
{
    geometry_msgs::Vector3 c;
    c.x = -vec.x;
    c.y = -vec.y;
    c.z = -vec.z;

    return c;
}

geometry_msgs::Vector3 operator*(const double a, geometry_msgs::Vector3& vec) 
{
    geometry_msgs::Vector3 c;
    c.x = a*vec.x;
    c.y = a*vec.y;
    c.z = a*vec.z;

    return c;
}

int inverse(double* A, double* Ainv, int N)
{
    int *IPIV = new int[N+1];
    int LWORK = N*N;
    double *WORK = new double[LWORK];
    int INFO;

    memcpy(Ainv,A,LWORK*sizeof(double));


    dgetrf_(&N,&N,Ainv,&N,IPIV,&INFO);
    dgetri_(&N,Ainv,&N,IPIV,WORK,&LWORK,&INFO);

    delete IPIV;
    delete WORK;

    return INFO;
}

void WGS84_NM(double lat,double *NE, double *ME)
{
	double sfi =sin(lat);
	double e2 = e_earth*e_earth;

	double temp = 1.0-e2*sfi*sfi;

	*NE = R_earth / sqrt(temp);
	*ME = R_earth*(1.0-e2) / pow(temp,3.0/2.0);
}
