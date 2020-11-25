
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter
{
private:
    double m_U;

    // Noise Covariance
    static const double R = 40;   
    // Measurement Map Scalar
    static const double H = 1.00;
    

    static double Q = 10;
    static double P = 0;


public:
    KalmanFilter(double U);
    void SetKalmanFilter(double U);

    double getKalman() { return m_U; }
};

#endif