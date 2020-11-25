
#include "kalman_filter.h"

    // Kalman Filter constructor
    KalmanFilter::KalmanFilter(double U)
{
    SetKalmanFilter(U);
}

// Kalman filter function
void KalmanFilter::SetKalmanFilter(double U)
{
    // SOME MATH GOES HERE
    m_U = U;
}