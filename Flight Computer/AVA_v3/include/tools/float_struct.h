/**
 * @file float_struct.h
 * @brief floating point algebra.
 */

#ifndef ALGEBRA_FLOAT_H
#define ALGEBRA_FLOAT_H

extern "C" {

#include <math.h>
#include <float.h>
#include <stdint.h>


/* this seems to be missing for some arch */
#ifndef M_SQRT2
#define M_SQRT2         1.41421356237309504880
#endif

struct FloatVect2 {
    float x;
    float y;
};

struct FloatVect3 {
    float x;
    float y;
    float z;
};

/**
 * @brief Rotation quaternion
 */
struct FloatQuat {
    float qi;
    float qx;
    float qy;
    float qz;
};

struct FloatMat33 {
    float m[3 * 3];
};

/**
 * @brief rotation matrix
 */
struct FloatRMat {
    float m[3 * 3];
};

/**
 * @brief euler angles
 * @details Units: radians */
struct FloatEulers {
    float phi; ///< in radians
    float theta; ///< in radians
    float psi; ///< in radians
};

/**
 * @brief angular rates
 * @details Units: rad/s */
struct FloatRates {
    float p; ///< in rad/s
    float q; ///< in rad/s
    float r; ///< in rad/s
};

/**
 * @brief vector in Latitude, Longitude and Altitude
 */
struct LlaCoor_f {
    float lat; ///< in radians
    float lon; ///< in radians
    float alt; ///< in meters 
};

/**
 * @brief vector in EarthCenteredEarthFixed coordinates
 * @details Origin at center of mass of the Earth. Z-axis is pointing north,
 * the x-axis intersects the sphere of the earth at 0° latitude (Equator)
 * and 0° longitude (Greenwich). Y-axis completes it to right-hand system.
 * Units: meters */
struct EcefCoor_f {
    float x; ///< in meters
    float y; ///< in meters
    float z; ///< in meters
};

/**
 * @brief vector in North East Down coordinates
 * Units: meters */
struct NedCoor_f {
    float x; ///< in meters
    float y; ///< in meters
    float z; ///< in meters
};

/* A Struct to hold actuator values */
struct Actuator {
    float throttle;
    float aileron;
    float elevator;
    float rudder;
    float flaps;
};


/* a = b */
#define ACTUATOR_COPY(_a, _b)         \
  {                                   \
    (_a).throttle = (_b).throttle;    \
    (_a).aileron  = (_b).aileron;     \
    (_a).elevator = (_b).elevator;    \
    (_a).rudder   = (_b).rudder;      \
    (_a).flaps    = (_b).flaps;       \
  }


/*
 * Returns the real part of the log of v in base of n
 */
static inline float float_log_n(float v, float n)
{
    if (fabsf(v) < 1e-4) { // avoid inf
    return - 1.0E+30;
    }
    if (fabsf(n) < 1e-4) { // avoid nan
    return 0;
    }
    return logf(fabsf(v)) / logf(n);
}

}

#define FLOAT_ANGLE_NORMALIZE(_a) {                 \
    while (_a >  M_PI) _a -= (2.*M_PI);             \
    while (_a < -M_PI) _a += (2.*M_PI);             \
    }


//
//
// Vector algebra
//
//

/*
 * Dimension 2 vectors
 */

/* a =  {x, y} */
#define VECT2_ASSIGN(_a, _x, _y) {    \
    (_a).x = (_x);        \
    (_a).y = (_y);        \
  }

/* a = b */
#define VECT2_COPY(_a, _b) {      \
    (_a).x = (_b).x;        \
    (_a).y = (_b).y;        \
  }

/* a += b */
#define VECT2_ADD(_a, _b) {     \
    (_a).x += (_b).x;       \
    (_a).y += (_b).y;       \
  }

/* a -= b */
#define VECT2_SUB(_a, _b) {     \
    (_a).x -= (_b).x;       \
    (_a).y -= (_b).y;       \
  }

/* c = a + b */
#define VECT2_SUM(_c, _a, _b) {     \
    (_c).x = (_a).x + (_b).x;     \
    (_c).y = (_a).y + (_b).y;     \
  }

/* c = a - b */
#define VECT2_DIFF(_c, _a, _b) {                \
    (_c).x = (_a).x - (_b).x;     \
    (_c).y = (_a).y - (_b).y;     \
  }

/* _vo = _vi * _s */
#define VECT2_SMUL(_vo, _vi, _s) {    \
    (_vo).x =  (_vi).x * (_s);      \
    (_vo).y =  (_vi).y * (_s);      \
  }

/* _vo =  _vi / _s */
#define VECT2_SDIV(_vo, _vi, _s) {    \
    (_vo).x =  (_vi).x / (_s);      \
    (_vo).y =  (_vi).y / (_s);      \
  }

/* _v = Bound(_v, _min, _max) */
#define VECT2_STRIM(_v, _min, _max) {         \
    (_v).x = (_v).x < _min ? _min : (_v).x > _max ? _max : (_v).x;  \
    (_v).y = (_v).y < _min ? _min : (_v).y > _max ? _max : (_v).y;  \
  }

/* _vo=v1*v2 */
#define VECT2_DOT_PRODUCT(_v1, _v2) ((_v1).x*(_v2).x + (_v1).y*(_v2).y)

#define VECT2_NORM2(_v) ((_v).x*(_v).x + (_v).y*(_v).y)

/*
 * Dimension 3 vectors
 */

/* a =  {x, y, z} */
#define VECT3_ASSIGN(_a, _x, _y, _z) {    \
    (_a).x = (_x);        \
    (_a).y = (_y);        \
    (_a).z = (_z);        \
  }


/* a =  {abs(x), abs(y), abs(z)} */
#define VECT3_ASSIGN_ABS(_a, _x, _y, _z) {      \
    (_a).x = ABS(_x);                              \
    (_a).y = ABS(_y);                              \
    (_a).z = ABS(_z);                              \
  }

/* a = b */
#define VECT3_COPY(_a, _b) {        \
    (_a).x = (_b).x;        \
    (_a).y = (_b).y;        \
    (_a).z = (_b).z;        \
  }

/* a += b */
#define VECT3_ADD(_a, _b) {     \
    (_a).x += (_b).x;       \
    (_a).y += (_b).y;       \
    (_a).z += (_b).z;       \
  }

/* a -= b */
#define VECT3_SUB(_a, _b) {     \
    (_a).x -= (_b).x;       \
    (_a).y -= (_b).y;       \
    (_a).z -= (_b).z;       \
  }

/* c = a + b */
#define VECT3_SUM(_c, _a, _b) {                 \
    (_c).x = (_a).x + (_b).x;     \
    (_c).y = (_a).y + (_b).y;     \
    (_c).z = (_a).z + (_b).z;     \
  }

/* a += b*s */
#define VECT3_ADD_SCALED(_a, _b, _s) {      \
    (_a).x += ((_b).x * (_s));        \
    (_a).y += ((_b).y * (_s));        \
    (_a).z += ((_b).z * (_s));        \
  }

/* c = a + _s * b */
#define VECT3_SUM_SCALED(_c, _a, _b, _s) {    \
    (_c).x = (_a).x + (_s)*(_b).x;      \
    (_c).y = (_a).y + (_s)*(_b).y;      \
    (_c).z = (_a).z + (_s)*(_b).z;      \
  }

/* c = a - b */
#define VECT3_DIFF(_c, _a, _b) {                \
    (_c).x = (_a).x - (_b).x;     \
    (_c).y = (_a).y - (_b).y;     \
    (_c).z = (_a).z - (_b).z;     \
  }

/* _vo = _vi * _s */
#define VECT3_SMUL(_vo, _vi, _s) {      \
    (_vo).x =  (_vi).x * (_s);        \
    (_vo).y =  (_vi).y * (_s);        \
    (_vo).z =  (_vi).z * (_s);        \
  }

/* _vo =  _vi / _s */
#define VECT3_SDIV(_vo, _vi, _s) {      \
    (_vo).x =  (_vi).x / (_s);        \
    (_vo).y =  (_vi).y / (_s);        \
    (_vo).z =  (_vi).z / (_s);        \
  }

/* _v = Bound(_v, _min, _max) */
#define VECT3_STRIM(_v, _min, _max) {         \
    (_v).x = (_v).x < _min ? _min : (_v).x > _max ? _max : (_v).x;  \
    (_v).y = (_v).y < _min ? _min : (_v).y > _max ? _max : (_v).y;  \
    (_v).z = (_v).z < _min ? _min : (_v).z > _max ? _max : (_v).z;  \
  }

/*  */
#define VECT3_EW_DIV(_vo, _va, _vb) {       \
    (_vo).x =  (_va).x / (_vb).x;       \
    (_vo).y =  (_va).y / (_vb).y;       \
    (_vo).z =  (_va).z / (_vb).z;       \
  }

/*  */
#define VECT3_EW_MUL(_vo, _va, _vb) {       \
    (_vo).x =  (_va).x * (_vb).x;       \
    (_vo).y =  (_va).y * (_vb).y;       \
    (_vo).z =  (_va).z * (_vb).z;       \
  }

/*  */
#define VECT3_BOUND_CUBE(_v, _min, _max) {        \
    if ((_v).x > (_max)) (_v).x = (_max); else if ((_v).x < (_min)) (_v).x = (_min); \
    if ((_v).y > (_max)) (_v).y = (_max); else if ((_v).y < (_min)) (_v).y = (_min); \
    if ((_v).z > (_max)) (_v).z = (_max); else if ((_v).z < (_min)) (_v).z = (_min); \
  }

/*  */
#define VECT3_BOUND_BOX(_v, _v_min, _v_max) {       \
    if ((_v).x > (_v_max).x) (_v).x = (_v_max).x; else if ((_v).x < (_v_min).x) (_v).x = (_v_min).x; \
    if ((_v).y > (_v_max).y) (_v).y = (_v_max).y; else if ((_v).y < (_v_min).y) (_v).y = (_v_min).y; \
    if ((_v).z > (_v_max).z) (_v).z = (_v_max).z; else if ((_v).z < (_v_min).z) (_v).z = (_v_min).z; \
  }

/*  */
#define VECT3_ABS(_vo, _vi) { \
    (_vo).x = ABS((_vi).x);   \
    (_vo).y = ABS((_vi).y);   \
    (_vo).z = ABS((_vi).z);   \
  }

#define VECT3_CROSS_PRODUCT(_vo, _v1, _v2) {        \
    (_vo).x = (_v1).y*(_v2).z - (_v1).z*(_v2).y;    \
    (_vo).y = (_v1).z*(_v2).x - (_v1).x*(_v2).z;    \
    (_vo).z = (_v1).x*(_v2).y - (_v1).y*(_v2).x;    \
  }

#define VECT3_DOT_PRODUCT(_v1, _v2) ((_v1).x*(_v2).x + (_v1).y*(_v2).y + (_v1).z*(_v2).z)

#define VECT3_NORM2(_v) ((_v).x*(_v).x + (_v).y*(_v).y + (_v).z*(_v).z)

#define VECT3_RATES_CROSS_VECT3(_vo, _r1, _v2) {    \
    (_vo).x = (_r1).q*(_v2).z - (_r1).r*(_v2).y;    \
    (_vo).y = (_r1).r*(_v2).x - (_r1).p*(_v2).z;    \
    (_vo).z = (_r1).p*(_v2).y - (_r1).q*(_v2).x;    \
  }


//
//
// Euler angles
//
//


#define EULERS_COPY(_a, _b) {       \
    (_a).phi   = (_b).phi;        \
    (_a).theta = (_b).theta;        \
    (_a).psi   = (_b).psi;        \
  }

#define EULERS_ASSIGN(_e, _phi, _theta, _psi) {   \
    (_e).phi   = (_phi);                            \
    (_e).theta = (_theta);                          \
    (_e).psi   = (_psi);                            \
  }

/* a += b */
#define EULERS_ADD(_a, _b) {        \
    (_a).phi   += (_b).phi;       \
    (_a).theta += (_b).theta;       \
    (_a).psi   += (_b).psi;       \
  }

/* a += b */
#define EULERS_SUB(_a, _b) {        \
    (_a).phi   -= (_b).phi;       \
    (_a).theta -= (_b).theta;       \
    (_a).psi   -= (_b).psi;       \
  }

/* c = a - b */
#define EULERS_DIFF(_c, _a, _b) {   \
    (_c).phi   = (_a).phi   - (_b).phi;   \
    (_c).theta = (_a).theta - (_b).theta; \
    (_c).psi   = (_a).psi   - (_b).psi;   \
  }


/* _vo =  _vi * _s */
#define EULERS_SMUL(_eo, _ei, _s) {       \
    (_eo).phi   =  (_ei).phi   * (_s);        \
    (_eo).theta =  (_ei).theta * (_s);        \
    (_eo).psi   =  (_ei).psi   * (_s);        \
  }

/* _vo =  _vi / _s */
#define EULERS_SDIV(_eo, _ei, _s) {       \
    (_eo).phi   =  (_ei).phi   / (_s);        \
    (_eo).theta =  (_ei).theta / (_s);        \
    (_eo).psi   =  (_ei).psi   / (_s);        \
  }

/* _v = Bound(_v, _min, _max) */
#define EULERS_BOUND_CUBE(_v, _min, _max) {                  \
    (_v).phi   = (_v).phi   < (_min) ? (_min) : (_v).phi   > (_max) ? (_max) : (_v).phi; \
    (_v).theta = (_v).theta < (_min) ? (_min) : (_v).theta > (_max) ? (_max) : (_v).theta; \
    (_v).psi   = (_v).psi   < (_min) ? (_min) : (_v).psi   > (_max) ? (_max) : (_v).psi;   \
  }

//
//
// Rates
//
//

/* ra =  {p, q, r} */
#define RATES_ASSIGN(_ra, _p, _q, _r) {   \
    (_ra).p = (_p);       \
    (_ra).q = (_q);       \
    (_ra).r = (_r);       \
  }

/* a = b */
#define RATES_COPY(_a, _b) {      \
    (_a).p = (_b).p;        \
    (_a).q = (_b).q;        \
    (_a).r = (_b).r;        \
  }

/* a += b */
#define RATES_ADD(_a, _b) {     \
    (_a).p += (_b).p;       \
    (_a).q += (_b).q;       \
    (_a).r += (_b).r;       \
  }

/* a -= b */
#define RATES_SUB(_a, _b) {     \
    (_a).p -= (_b).p;       \
    (_a).q -= (_b).q;       \
    (_a).r -= (_b).r;       \
  }

/* c = a + b */
#define RATES_SUM(_c, _a, _b) {     \
    (_c).p = (_a).p + (_b).p;     \
    (_c).q = (_a).q + (_b).q;     \
    (_c).r = (_a).r + (_b).r;     \
  }

/* c = a + _s * b */
#define RATES_SUM_SCALED(_c, _a, _b, _s) {    \
    (_c).p = (_a).p + (_s)*(_b).p;      \
    (_c).q = (_a).q + (_s)*(_b).q;      \
    (_c).r = (_a).r + (_s)*(_b).r;      \
  }

/* c = a - b */
#define RATES_DIFF(_c, _a, _b) {                \
    (_c).p = (_a).p - (_b).p;     \
    (_c).q = (_a).q - (_b).q;     \
    (_c).r = (_a).r - (_b).r;     \
  }

/* _ro =  _ri * _s */
#define RATES_SMUL(_ro, _ri, _s) {    \
    (_ro).p =  (_ri).p * (_s);      \
    (_ro).q =  (_ri).q * (_s);      \
    (_ro).r =  (_ri).r * (_s);      \
  }

/* _ro =  _ri / _s */
#define RATES_SDIV(_ro, _ri, _s) {    \
    (_ro).p =  (_ri).p / (_s) ;     \
    (_ro).q =  (_ri).q / (_s);      \
    (_ro).r =  (_ri).r / (_s);      \
  }

/* Element wise vector multiplication */
#define RATES_EWMULT_RSHIFT(c, a, b, _s) {  \
    (c).p = ((a).p * (b).p) >> (_s);    \
    (c).q = ((a).q * (b).q) >> (_s);    \
    (c).r = ((a).r * (b).r) >> (_s);    \
  }


/* _v = Bound(_v, _min, _max) */
#define RATES_BOUND_CUBE(_v, _min, _max) {        \
    (_v).p = (_v).p < (_min) ? (_min) : (_v).p > (_max) ? (_max) : (_v).p;  \
    (_v).q = (_v).q < (_min) ? (_min) : (_v).q > (_max) ? (_max) : (_v).q;  \
    (_v).r = (_v).r < (_min) ? (_min) : (_v).r > (_max) ? (_max) : (_v).r;  \
  }

#define RATES_BOUND_BOX(_v, _v_min, _v_max) {       \
    if ((_v).p > (_v_max).p) (_v).p = (_v_max).p; else if ((_v).p < (_v_min).p) (_v).p = (_v_min).p; \
    if ((_v).q > (_v_max).q) (_v).q = (_v_max).q; else if ((_v).q < (_v_min).q) (_v).q = (_v_min).q; \
    if ((_v).r > (_v_max).r) (_v).r = (_v_max).r; else if ((_v).r < (_v_min).r) (_v).r = (_v_min).r; \
  }

#define RATES_BOUND_BOX_ABS(_v, _v_max) {       \
    if ((_v).p > (_v_max).p) (_v).p = (_v_max).p; else if ((_v).p < -(_v_max).p) (_v).p = -(_v_max).p; \
    if ((_v).q > (_v_max).q) (_v).q = (_v_max).q; else if ((_v).q < -(_v_max).q) (_v).q = -(_v_max).q; \
    if ((_v).r > (_v_max).r) (_v).r = (_v_max).r; else if ((_v).r < -(_v_max).r) (_v).r = -(_v_max).r; \
  }

#define RATES_ADD_SCALED_VECT(_ro, _v, _s) {    \
    (_ro).p += (_v).x * (_s);                   \
    (_ro).q += (_v).y * (_s);                   \
    (_ro).r += (_v).z * (_s);                   \
  }

//
//
// Matrix
//
//


/*
 * 3x3 matrices
 */
/* accessor : row and col range from 0 to 2 */
#define MAT33_ELMT(_m, _row, _col) ((_m).m[(_row)*3+(_col)])

#define MAT33_COPY(_mat1,_mat2) {     \
    MAT33_ELMT((_mat1),0,0) = MAT33_ELMT((_mat2),0,0);  \
    MAT33_ELMT((_mat1),0,1) = MAT33_ELMT((_mat2),0,1);  \
    MAT33_ELMT((_mat1),0,2) = MAT33_ELMT((_mat2),0,2);  \
    MAT33_ELMT((_mat1),1,0) = MAT33_ELMT((_mat2),1,0);  \
    MAT33_ELMT((_mat1),1,1) = MAT33_ELMT((_mat2),1,1);  \
    MAT33_ELMT((_mat1),1,2) = MAT33_ELMT((_mat2),1,2);  \
    MAT33_ELMT((_mat1),2,0) = MAT33_ELMT((_mat2),2,0);  \
    MAT33_ELMT((_mat1),2,1) = MAT33_ELMT((_mat2),2,1);  \
    MAT33_ELMT((_mat1),2,2) = MAT33_ELMT((_mat2),2,2);  \
  }

#define MAT33_MULT_SCALAR(_mat1,_scalar) {     \
    MAT33_ELMT((_mat1),0,0) = MAT33_ELMT((_mat1),0,0)*_scalar;  \
    MAT33_ELMT((_mat1),0,1) = MAT33_ELMT((_mat1),0,1)*_scalar;  \
    MAT33_ELMT((_mat1),0,2) = MAT33_ELMT((_mat1),0,2)*_scalar;  \
    MAT33_ELMT((_mat1),1,0) = MAT33_ELMT((_mat1),1,0)*_scalar;  \
    MAT33_ELMT((_mat1),1,1) = MAT33_ELMT((_mat1),1,1)*_scalar;  \
    MAT33_ELMT((_mat1),1,2) = MAT33_ELMT((_mat1),1,2)*_scalar;  \
    MAT33_ELMT((_mat1),2,0) = MAT33_ELMT((_mat1),2,0)*_scalar;  \
    MAT33_ELMT((_mat1),2,1) = MAT33_ELMT((_mat1),2,1)*_scalar;  \
    MAT33_ELMT((_mat1),2,2) = MAT33_ELMT((_mat1),2,2)*_scalar;  \
  }

/* multiply _vin by _mat, store in _vout */
#define MAT33_VECT3_MUL(_vout, _mat, _vin) {    \
    (_vout).x = MAT33_ELMT((_mat), 0, 0) * (_vin).x + \
                MAT33_ELMT((_mat), 0, 1) * (_vin).y + \
                MAT33_ELMT((_mat), 0, 2) * (_vin).z;  \
    (_vout).y = MAT33_ELMT((_mat), 1, 0) * (_vin).x +   \
                MAT33_ELMT((_mat), 1, 1) * (_vin).y +   \
                MAT33_ELMT((_mat), 1, 2) * (_vin).z;  \
    (_vout).z = MAT33_ELMT((_mat), 2, 0) * (_vin).x + \
                MAT33_ELMT((_mat), 2, 1) * (_vin).y + \
                MAT33_ELMT((_mat), 2, 2) * (_vin).z;  \
  }

/* multiply _vin by transpose of _mat, store in _vout */
#define MAT33_VECT3_TRANSP_MUL(_vout, _mat, _vin) {     \
    (_vout).x = MAT33_ELMT((_mat), 0, 0) * (_vin).x + \
                MAT33_ELMT((_mat), 1, 0) * (_vin).y + \
                MAT33_ELMT((_mat), 2, 0) * (_vin).z;  \
    (_vout).y = MAT33_ELMT((_mat), 0, 1) * (_vin).x +   \
                MAT33_ELMT((_mat), 1, 1) * (_vin).y +   \
                MAT33_ELMT((_mat), 2, 1) * (_vin).z;  \
    (_vout).z = MAT33_ELMT((_mat), 0, 2) * (_vin).x +   \
                MAT33_ELMT((_mat), 1, 2) * (_vin).y + \
                MAT33_ELMT((_mat), 2, 2) * (_vin).z;  \
  }

/* invS = 1/det(S) com(S)' */
#define MAT33_INV(_minv, _m) {            \
    const float m00 = MAT33_ELMT((_m),1,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),1,2)*MAT33_ELMT((_m),2,1);    \
    const float m10 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),2,1);    \
    const float m20 = MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),1,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),1,1);    \
    const float m01 = MAT33_ELMT((_m),1,0)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),1,2)*MAT33_ELMT((_m),2,0);    \
    const float m11 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),2,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),2,0);    \
    const float m21 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),1,2) - MAT33_ELMT((_m),0,2)*MAT33_ELMT((_m),1,0);    \
    const float m02 = MAT33_ELMT((_m),1,0)*MAT33_ELMT((_m),2,1) - MAT33_ELMT((_m),1,1)*MAT33_ELMT((_m),2,0);    \
    const float m12 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),2,1) - MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),2,0);    \
    const float m22 = MAT33_ELMT((_m),0,0)*MAT33_ELMT((_m),1,1) - MAT33_ELMT((_m),0,1)*MAT33_ELMT((_m),1,0);    \
    const float det = MAT33_ELMT((_m),0,0)*m00 - MAT33_ELMT((_m),1,0)*m10 + MAT33_ELMT((_m),2,0)*m20; \
    if (fabs(det) > FLT_EPSILON) {          \
      MAT33_ELMT((_minv),0,0) =  m00 / det;           \
      MAT33_ELMT((_minv),1,0) = -m01 / det;           \
      MAT33_ELMT((_minv),2,0) =  m02 / det;           \
      MAT33_ELMT((_minv),0,1) = -m10 / det;           \
      MAT33_ELMT((_minv),1,1) =  m11 / det;           \
      MAT33_ELMT((_minv),2,1) = -m12 / det;           \
      MAT33_ELMT((_minv),0,2) =  m20 / det;           \
      MAT33_ELMT((_minv),1,2) = -m21 / det;           \
      MAT33_ELMT((_minv),2,2) =  m22 / det;           \
    }                 \
  }

/* set _row of _mat with _vin multiplied by scalar _s */
#define MAT33_ROW_VECT3_SMUL(_mat, _row, _vin, _s) {         \
    MAT33_ELMT((_mat), _row, 0) = (_vin).x * (_s);           \
    MAT33_ELMT((_mat), _row, 1) = (_vin).y * (_s);           \
    MAT33_ELMT((_mat), _row, 2) = (_vin).z * (_s);           \
  }

/* outer product of _v_a and _v_b, resulting in a 3x3 matrix */
#define VECT3_VECT3_TRANS_MUL(_mat, _v_a, _v_b) { \
    MAT33_ELMT((_mat),0,0) = (_v_a).x*(_v_b).x;   \
    MAT33_ELMT((_mat),0,1) = (_v_a).x*(_v_b).y;   \
    MAT33_ELMT((_mat),0,2) = (_v_a).x*(_v_b).z;   \
    MAT33_ELMT((_mat),1,0) = (_v_a).y*(_v_b).x;   \
    MAT33_ELMT((_mat),1,1) = (_v_a).y*(_v_b).y;   \
    MAT33_ELMT((_mat),1,2) = (_v_a).y*(_v_b).z;   \
    MAT33_ELMT((_mat),2,0) = (_v_a).z*(_v_b).x;   \
    MAT33_ELMT((_mat),2,1) = (_v_a).z*(_v_b).y;   \
    MAT33_ELMT((_mat),2,2) = (_v_a).z*(_v_b).z;   \
  }

/* elementwise subtraction of two 3x3 matrices */
#define MAT33_MAT33_DIFF(_mat1, _mat2, _mat3) {                                 \
    MAT33_ELMT((_mat1),0,0) = MAT33_ELMT((_mat2),0,0)-MAT33_ELMT((_mat3),0,0);  \
    MAT33_ELMT((_mat1),0,1) = MAT33_ELMT((_mat2),0,1)-MAT33_ELMT((_mat3),0,1);  \
    MAT33_ELMT((_mat1),0,2) = MAT33_ELMT((_mat2),0,2)-MAT33_ELMT((_mat3),0,2);  \
    MAT33_ELMT((_mat1),1,0) = MAT33_ELMT((_mat2),1,0)-MAT33_ELMT((_mat3),1,0);  \
    MAT33_ELMT((_mat1),1,1) = MAT33_ELMT((_mat2),1,1)-MAT33_ELMT((_mat3),1,1);  \
    MAT33_ELMT((_mat1),1,2) = MAT33_ELMT((_mat2),1,2)-MAT33_ELMT((_mat3),1,2);  \
    MAT33_ELMT((_mat1),2,0) = MAT33_ELMT((_mat2),2,0)-MAT33_ELMT((_mat3),2,0);  \
    MAT33_ELMT((_mat1),2,1) = MAT33_ELMT((_mat2),2,1)-MAT33_ELMT((_mat3),2,1);  \
    MAT33_ELMT((_mat1),2,2) = MAT33_ELMT((_mat2),2,2)-MAT33_ELMT((_mat3),2,2);  \
  }

/* elementwise addition of two 3x3 matrices */
#define MAT33_MAT33_SUM(_mat1, _mat2, _mat3) {                                  \
    MAT33_ELMT((_mat1),0,0) = MAT33_ELMT((_mat2),0,0)+MAT33_ELMT((_mat3),0,0);  \
    MAT33_ELMT((_mat1),0,1) = MAT33_ELMT((_mat2),0,1)+MAT33_ELMT((_mat3),0,1);  \
    MAT33_ELMT((_mat1),0,2) = MAT33_ELMT((_mat2),0,2)+MAT33_ELMT((_mat3),0,2);  \
    MAT33_ELMT((_mat1),1,0) = MAT33_ELMT((_mat2),1,0)+MAT33_ELMT((_mat3),1,0);  \
    MAT33_ELMT((_mat1),1,1) = MAT33_ELMT((_mat2),1,1)+MAT33_ELMT((_mat3),1,1);  \
    MAT33_ELMT((_mat1),1,2) = MAT33_ELMT((_mat2),1,2)+MAT33_ELMT((_mat3),1,2);  \
    MAT33_ELMT((_mat1),2,0) = MAT33_ELMT((_mat2),2,0)+MAT33_ELMT((_mat3),2,0);  \
    MAT33_ELMT((_mat1),2,1) = MAT33_ELMT((_mat2),2,1)+MAT33_ELMT((_mat3),2,1);  \
    MAT33_ELMT((_mat1),2,2) = MAT33_ELMT((_mat2),2,2)+MAT33_ELMT((_mat3),2,2);  \
  }

/* transpose of a 3x3 matrix */
#define MAT33_TRANS(_mat1,_mat2) {                      \
    MAT33_ELMT((_mat1),0,0) = MAT33_ELMT((_mat2),0,0);  \
    MAT33_ELMT((_mat1),0,1) = MAT33_ELMT((_mat2),1,0);  \
    MAT33_ELMT((_mat1),0,2) = MAT33_ELMT((_mat2),2,0);  \
    MAT33_ELMT((_mat1),1,0) = MAT33_ELMT((_mat2),0,1);  \
    MAT33_ELMT((_mat1),1,1) = MAT33_ELMT((_mat2),1,1);  \
    MAT33_ELMT((_mat1),1,2) = MAT33_ELMT((_mat2),2,1);  \
    MAT33_ELMT((_mat1),2,0) = MAT33_ELMT((_mat2),0,2);  \
    MAT33_ELMT((_mat1),2,1) = MAT33_ELMT((_mat2),1,2);  \
    MAT33_ELMT((_mat1),2,2) = MAT33_ELMT((_mat2),2,2);  \
  }


//
//
// Quaternion algebras
//
//

/* _q = [_i _x _y _z] */
#define QUAT_ASSIGN(_q, _i, _x, _y, _z) {   \
    (_q).qi = (_i);         \
    (_q).qx = (_x);         \
    (_q).qy = (_y);         \
    (_q).qz = (_z);         \
  }

/* _qc = _qa - _qb */
#define QUAT_DIFF(_qc, _qa, _qb) {      \
    (_qc).qi = (_qa).qi - (_qb).qi;     \
    (_qc).qx = (_qa).qx - (_qb).qx;     \
    (_qc).qy = (_qa).qy - (_qb).qy;     \
    (_qc).qz = (_qa).qz - (_qb).qz;     \
  }

/* _qo = _qi */
#define QUAT_COPY(_qo, _qi) {     \
    (_qo).qi = (_qi).qi;        \
    (_qo).qx = (_qi).qx;        \
    (_qo).qy = (_qi).qy;        \
    (_qo).qz = (_qi).qz;        \
  }

#define QUAT_EXPLEMENTARY(b,a) {    \
    (b).qi = -(a).qi;               \
    (b).qx = -(a).qx;               \
    (b).qy = -(a).qy;               \
    (b).qz = -(a).qz;               \
  }

/* _qo = _qi * _s */
#define QUAT_SMUL(_qo, _qi, _s) {     \
    (_qo).qi = (_qi).qi * (_s);       \
    (_qo).qx = (_qi).qx * (_s);       \
    (_qo).qy = (_qi).qy * (_s);       \
    (_qo).qz = (_qi).qz * (_s);       \
  }

/* _qo = _qo + _qi */
#define QUAT_ADD(_qo, _qi) {        \
    (_qo).qi += (_qi).qi;       \
    (_qo).qx += (_qi).qx;       \
    (_qo).qy += (_qi).qy;       \
    (_qo).qz += (_qi).qz;       \
  }

/* _qo = [qi -qx -qy -qz] */
#define QUAT_INVERT(_qo, _qi) {       \
    (_qo).qi =  (_qi).qi;       \
    (_qo).qx = -(_qi).qx;       \
    (_qo).qy = -(_qi).qy;       \
    (_qo).qz = -(_qi).qz;       \
  }

/* _vo=[ qx qy qz] */
#define QUAT_EXTRACT_Q(_vo, _qi) {  \
    (_vo).x=(_qi).qx;                 \
    (_vo).y=(_qi).qy;                 \
    (_vo).z=(_qi).qz;                 \
  }

/* _qo = _qo / _s */
#define QUAT_SDIV(_qo, _qi, _s) { \
    (_qo).qi = (_qi).qi / (_s); \
    (_qo).qx = (_qi).qx / (_s); \
    (_qo).qy = (_qi).qy / (_s); \
    (_qo).qz = (_qi).qz / (_s); \
  }

/* return = _qa * _qb */
#define QUAT_DOT_PRODUCT(_qa, _qb) ((_qa).qi * (_qb).qi + (_qa).qx * (_qb).qx + (_qa).qy * (_qb).qy + (_qa).qz * (_qb).qz)

//
//
// Rotation Matrices
//
//


/* accessor : row and col range from 0 to 2 */
#define RMAT_ELMT(_rm, _row, _col) MAT33_ELMT(_rm, _row, _col)

/* trace */
#define RMAT_TRACE(_rm) (RMAT_ELMT(_rm, 0, 0)+RMAT_ELMT(_rm, 1, 1)+RMAT_ELMT(_rm, 2, 2))


#define RMAT_DIFF(_c, _a, _b) {        \
    (_c).m[0] = (_a).m[0] - (_b).m[0];       \
    (_c).m[1] = (_a).m[1] - (_b).m[1];       \
    (_c).m[2] = (_a).m[2] - (_b).m[2];       \
    (_c).m[3] = (_a).m[3] - (_b).m[3];       \
    (_c).m[4] = (_a).m[4] - (_b).m[4];       \
    (_c).m[5] = (_a).m[5] - (_b).m[5];       \
    (_c).m[6] = (_a).m[6] - (_b).m[6];       \
    (_c).m[7] = (_a).m[7] - (_b).m[7];       \
    (_c).m[8] = (_a).m[8] - (_b).m[8];       \
  }

/* multiply _vin by _rmat, store in _vout */
#define RMAT_VECT3_MUL(_vout, _rmat, _vin) {     \
    (_vout).x = RMAT_ELMT((_rmat), 0, 0) * (_vin).x +  \
                RMAT_ELMT((_rmat), 0, 1) * (_vin).y +  \
                RMAT_ELMT((_rmat), 0, 2) * (_vin).z;   \
    (_vout).y = RMAT_ELMT((_rmat), 1, 0) * (_vin).x +    \
                RMAT_ELMT((_rmat), 1, 1) * (_vin).y +    \
                RMAT_ELMT((_rmat), 1, 2) * (_vin).z;   \
    (_vout).z = RMAT_ELMT((_rmat), 2, 0) * (_vin).x +  \
                RMAT_ELMT((_rmat), 2, 1) * (_vin).y +  \
                RMAT_ELMT((_rmat), 2, 2) * (_vin).z;   \
  }

#define RMAT_VECT3_TRANSP_MUL(_vout, _rmat, _vin) {     \
    (_vout).x = RMAT_ELMT((_rmat), 0, 0) * (_vin).x +  \
                RMAT_ELMT((_rmat), 1, 0) * (_vin).y +  \
                RMAT_ELMT((_rmat), 2, 0) * (_vin).z;   \
    (_vout).y = RMAT_ELMT((_rmat), 0, 1) * (_vin).x +    \
                RMAT_ELMT((_rmat), 1, 1) * (_vin).y +    \
                RMAT_ELMT((_rmat), 2, 1) * (_vin).z;   \
    (_vout).z = RMAT_ELMT((_rmat), 0, 2) * (_vin).x +  \
                RMAT_ELMT((_rmat), 1, 2) * (_vin).y +  \
                RMAT_ELMT((_rmat), 2, 2) * (_vin).z;   \
  }


#define RMAT_COPY(_o, _i) { memcpy(&(_o), &(_i), sizeof(_o));}



#define EULERS_FLOAT_OF_BFP(_ef, _ei) {     \
    (_ef).phi   = ANGLE_FLOAT_OF_BFP((_ei).phi);  \
    (_ef).theta = ANGLE_FLOAT_OF_BFP((_ei).theta);  \
    (_ef).psi   = ANGLE_FLOAT_OF_BFP((_ei).psi);  \
  }

#define EULERS_BFP_OF_REAL(_ei, _ef) {      \
    (_ei).phi   = ANGLE_BFP_OF_REAL((_ef).phi);   \
    (_ei).theta = ANGLE_BFP_OF_REAL((_ef).theta); \
    (_ei).psi   = ANGLE_BFP_OF_REAL((_ef).psi);   \
  }

#define RMAT_BFP_OF_REAL(_ei, _ef) {      \
    (_ei).m[0] = TRIG_BFP_OF_REAL((_ef).m[0]);    \
    (_ei).m[1] = TRIG_BFP_OF_REAL((_ef).m[1]);    \
    (_ei).m[2] = TRIG_BFP_OF_REAL((_ef).m[2]);    \
    (_ei).m[3] = TRIG_BFP_OF_REAL((_ef).m[3]);    \
    (_ei).m[4] = TRIG_BFP_OF_REAL((_ef).m[4]);    \
    (_ei).m[5] = TRIG_BFP_OF_REAL((_ef).m[5]);    \
    (_ei).m[6] = TRIG_BFP_OF_REAL((_ef).m[6]);    \
    (_ei).m[7] = TRIG_BFP_OF_REAL((_ef).m[7]);    \
    (_ei).m[8] = TRIG_BFP_OF_REAL((_ef).m[8]);    \
  }

#define RMAT_FLOAT_OF_BFP(_ef, _ei) {     \
    (_ef).m[0] = TRIG_FLOAT_OF_BFP((_ei).m[0]);   \
    (_ef).m[1] = TRIG_FLOAT_OF_BFP((_ei).m[1]);   \
    (_ef).m[2] = TRIG_FLOAT_OF_BFP((_ei).m[2]);   \
    (_ef).m[3] = TRIG_FLOAT_OF_BFP((_ei).m[3]);   \
    (_ef).m[4] = TRIG_FLOAT_OF_BFP((_ei).m[4]);   \
    (_ef).m[5] = TRIG_FLOAT_OF_BFP((_ei).m[5]);   \
    (_ef).m[6] = TRIG_FLOAT_OF_BFP((_ei).m[6]);   \
    (_ef).m[7] = TRIG_FLOAT_OF_BFP((_ei).m[7]);   \
    (_ef).m[8] = TRIG_FLOAT_OF_BFP((_ei).m[8]);   \
  }

#define QUAT_FLOAT_OF_BFP(_qf, _qi) {     \
    (_qf).qi = QUAT1_FLOAT_OF_BFP((_qi).qi);    \
    (_qf).qx = QUAT1_FLOAT_OF_BFP((_qi).qx);    \
    (_qf).qy = QUAT1_FLOAT_OF_BFP((_qi).qy);    \
    (_qf).qz = QUAT1_FLOAT_OF_BFP((_qi).qz);    \
  }

#define QUAT_BFP_OF_REAL(_qi, _qf) {      \
    (_qi).qi = QUAT1_BFP_OF_REAL((_qf).qi);   \
    (_qi).qx = QUAT1_BFP_OF_REAL((_qf).qx);   \
    (_qi).qy = QUAT1_BFP_OF_REAL((_qf).qy);   \
    (_qi).qz = QUAT1_BFP_OF_REAL((_qf).qz);   \
  }

#define RATES_FLOAT_OF_BFP(_rf, _ri) {      \
    (_rf).p = RATE_FLOAT_OF_BFP((_ri).p);   \
    (_rf).q = RATE_FLOAT_OF_BFP((_ri).q);   \
    (_rf).r = RATE_FLOAT_OF_BFP((_ri).r);   \
  }

#define RATES_BFP_OF_REAL(_ri, _rf) {     \
    (_ri).p = RATE_BFP_OF_REAL((_rf).p);    \
    (_ri).q = RATE_BFP_OF_REAL((_rf).q);    \
    (_ri).r = RATE_BFP_OF_REAL((_rf).r);    \
  }

#define POSITIONS_FLOAT_OF_BFP(_ef, _ei) {      \
    (_ef).x = POS_FLOAT_OF_BFP((_ei).x);    \
    (_ef).y = POS_FLOAT_OF_BFP((_ei).y);    \
    (_ef).z = POS_FLOAT_OF_BFP((_ei).z);    \
  }

#define POSITIONS_BFP_OF_REAL(_ef, _ei) { \
    (_ef).x = POS_BFP_OF_REAL((_ei).x);   \
    (_ef).y = POS_BFP_OF_REAL((_ei).y);   \
    (_ef).z = POS_BFP_OF_REAL((_ei).z);   \
  }

#define SPEEDS_FLOAT_OF_BFP(_ef, _ei) {     \
    (_ef).x = SPEED_FLOAT_OF_BFP((_ei).x);    \
    (_ef).y = SPEED_FLOAT_OF_BFP((_ei).y);    \
    (_ef).z = SPEED_FLOAT_OF_BFP((_ei).z);    \
  }

#define SPEEDS_BFP_OF_REAL(_ef, _ei) {      \
    (_ef).x = SPEED_BFP_OF_REAL((_ei).x);   \
    (_ef).y = SPEED_BFP_OF_REAL((_ei).y);   \
    (_ef).z = SPEED_BFP_OF_REAL((_ei).z);   \
  }

#define ACCELS_FLOAT_OF_BFP(_ef, _ei) {     \
    (_ef).x = ACCEL_FLOAT_OF_BFP((_ei).x);    \
    (_ef).y = ACCEL_FLOAT_OF_BFP((_ei).y);    \
    (_ef).z = ACCEL_FLOAT_OF_BFP((_ei).z);    \
  }

#define ACCELS_BFP_OF_REAL(_ef, _ei) {      \
    (_ef).x = ACCEL_BFP_OF_REAL((_ei).x);   \
    (_ef).y = ACCEL_BFP_OF_REAL((_ei).y);   \
    (_ef).z = ACCEL_BFP_OF_REAL((_ei).z);   \
  }

#define MAGS_FLOAT_OF_BFP(_ef, _ei) {     \
    (_ef).x = MAG_FLOAT_OF_BFP((_ei).x);    \
    (_ef).y = MAG_FLOAT_OF_BFP((_ei).y);    \
    (_ef).z = MAG_FLOAT_OF_BFP((_ei).z);    \
  }

#define MAGS_BFP_OF_REAL(_ef, _ei) {      \
    (_ef).x = MAG_BFP_OF_REAL((_ei).x);   \
    (_ef).y = MAG_BFP_OF_REAL((_ei).y);   \
    (_ef).z = MAG_BFP_OF_REAL((_ei).z);   \
  }


//
// Geodetic Define functions
//

#define ENU_OF_TO_NED(_po, _pi) {   \
    (_po).x =  (_pi).y;       \
    (_po).y =  (_pi).x;       \
    (_po).z = -(_pi).z;       \
  }

#define VECT2_ENU_OF_TO_NED(_po, _pi) {   \
    (_po).x =  (_pi).y;       \
    (_po).y =  (_pi).x;       \
  }

#define LLA_ASSIGN(_pos,_lat,_lon,_alt){  \
    (_pos).lat = (_lat);      \
    (_pos).lon = (_lon);      \
    (_pos).alt = (_alt);      \
  }

#define LLA_COPY(_pos1,_pos2){      \
    (_pos1).lat = (_pos2).lat;      \
    (_pos1).lon = (_pos2).lon;      \
    (_pos1).alt = (_pos2).alt;      \
  }

#define LTP_DEF_COPY(_def1,_def2){                              \
    LLA_COPY((_def1).lla, (_def2).lla);                         \
    VECT3_COPY((_def1).ecef, (_def2).ecef);                     \
    RMAT_COPY((_def1).ltp_of_ecef, (_def2).ltp_of_ecef);        \
    (_def1).hmsl = (_def2).hmsl;                                \
  }

#define UTM_COPY(_u1, _u2) {     \
    (_u1).north = (_u2).north;   \
    (_u1).east = (_u2).east;     \
    (_u1).alt = (_u2).alt;       \
    (_u1).zone = (_u2).zone;     \
  }

#define ENU_OF_UTM_DIFF(_pos, _utm1, _utm2) {   \
    (_pos).x = (_utm1).east - (_utm2).east;     \
    (_pos).y = (_utm1).north - (_utm2).north;   \
    (_pos).z = (_utm1).alt - (_utm2).alt;       \
  }

#define NED_OF_UTM_DIFF(_pos, _utm1, _utm2) {   \
    (_pos).x = (_utm1).north - (_utm2).north;   \
    (_pos).y = (_utm1).east - (_utm2).east;     \
    (_pos).z = -(_utm1).alt + (_utm2).alt;      \
  }

#define UTM_OF_ENU_ADD(_utm, _pos, _utm0) {    \
    (_utm).east = (_utm0).east + (_pos).x;     \
    (_utm).north = (_utm0).north + (_pos).y;   \
    (_utm).alt = (_utm0).alt + (_pos).z;       \
  }

#define UTM_OF_NED_ADD(_utm, _pos, _utm0) {    \
    (_utm).east = (_utm0).east + (_pos).y;     \
    (_utm).north = (_utm0).north + (_pos).x;   \
    (_utm).alt = (_utm0).alt - (_pos).z;       \
  }


#endif