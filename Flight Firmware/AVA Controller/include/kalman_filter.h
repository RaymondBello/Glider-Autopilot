#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter
{
private:
    double m_U;

    // Noise Covariance
    static constexpr double R = 40.0;
    // Measurement Map Scalar
    static constexpr double H = 1.0;

    double Q = 10;
    double P = 0;


public:
    KalmanFilter(double U);
    void SetKalmanFilter(double U);

    double getKalman() { return m_U; }
};
#endif