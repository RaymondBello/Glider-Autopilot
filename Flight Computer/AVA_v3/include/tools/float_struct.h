/**
 * @file float_struct.h
 * @brief floating point algebra.
 */

#ifndef ALGEBRA_FLOAT_H
#define ALGEBRA_FLOAT_H

#ifdef __cplusplus
extern "C" {
#endif

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



#define FLOAT_ANGLE_NORMALIZE(_a) {                 \
    while (_a >  M_PI) _a -= (2.*M_PI);             \
    while (_a < -M_PI) _a += (2.*M_PI);             \
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

#ifdef __cplusplus
}
#endif

#endif