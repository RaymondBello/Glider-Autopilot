#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

double kalman_ax(double U);
double kalman_ax(double U)
{
    static const double R = 40; // Noise Covariance (its actually 10)
    static const double H = 1.00; // Measurement Map Scalar
    static double Q = 10;         // Initial estimated covariance
    static double P = 0;          // Inital error covariance (must be 0)
    static double U_hat = 0;      // Initial estimated state (assuming we dont know)
    static double K = 0;

    // Initialize
    K = P * H / (H * P * H + R);         // Update kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated value

    // Update error covariance
    P = (1 - K * H) * P + Q;

    //Return estimate of U - H * U_hat
    return U_hat;
}

double kalman_ay(double U);
double kalman_ay(double U)
{
    static const double R = 40;   // Noise Covariance (its actually 10)
    static const double H = 1.00; // Measurement Map Scalar
    static double Q = 10;         // Initial estimated covariance
    static double P = 0;          // Inital error covariance (must be 0)
    static double U_hat = 0;      // Initial estimated state (assuming we dont know)
    static double K = 0;

    // Initialize
    K = P * H / (H * P * H + R);         // Update kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated value

    // Update error covariance
    P = (1 - K * H) * P + Q;

    //Return estimate of U - H * U_hat
    return U_hat;
}

double kalman_az(double U);
double kalman_az(double U)
{
    static const double R = 40;   // Noise Covariance (its actually 10)
    static const double H = 1.00; // Measurement Map Scalar
    static double Q = 10;         // Initial estimated covariance
    static double P = 0;          // Inital error covariance (must be 0)
    static double U_hat = 0;      // Initial estimated state (assuming we dont know)
    static double K = 0;

    // Initialize
    K = P * H / (H * P * H + R);         // Update kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated value

    // Update error covariance
    P = (1 - K * H) * P + Q;

    //Return estimate of U - H * U_hat
    return U_hat;
}

double kalman_gx(double U);
double kalman_gx(double U)
{
    static const double R = 40;   // Noise Covariance (its actually 10)
    static const double H = 1.00; // Measurement Map Scalar
    static double Q = 10;         // Initial estimated covariance
    static double P = 0;          // Inital error covariance (must be 0)
    static double U_hat = 0;      // Initial estimated state (assuming we dont know)
    static double K = 0;

    // Initialize
    K = P * H / (H * P * H + R);         // Update kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated value

    // Update error covariance
    P = (1 - K * H) * P + Q;

<<<<<<< HEAD
    //Return estimate of U - H * U_hat
    return U_hat;
}

double kalman_gy(double U);
double kalman_gy(double U)
{
    static const double R = 40;   // Noise Covariance (its actually 10)
    static const double H = 1.00; // Measurement Map Scalar
    static double Q = 10;         // Initial estimated covariance
    static double P = 0;          // Inital error covariance (must be 0)
    static double U_hat = 0;      // Initial estimated state (assuming we dont know)
    static double K = 0;

    // Initialize
    K = P * H / (H * P * H + R);         // Update kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated value

    // Update error covariance
    P = (1 - K * H) * P + Q;

    //Return estimate of U - H * U_hat
    return U_hat;
}

double kalman_gz(double U);
double kalman_gz(double U)
{
    static const double R = 40;   // Noise Covariance (its actually 10)
    static const double H = 1.00; // Measurement Map Scalar
    static double Q = 10;         // Initial estimated covariance
    static double P = 0;          // Inital error covariance (must be 0)
    static double U_hat = 0;      // Initial estimated state (assuming we dont know)
    static double K = 0;

    // Initialize
    K = P * H / (H * P * H + R);         // Update kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated value

    // Update error covariance
    P = (1 - K * H) * P + Q;

    //Return estimate of U - H * U_hat
    return U_hat;
}

double kalman_mx(double U);
double kalman_mx(double U)
{
    static const double R = 40;   // Noise Covariance (its actually 10)
    static const double H = 1.00; // Measurement Map Scalar
    static double Q = 10;         // Initial estimated covariance
    static double P = 0;          // Inital error covariance (must be 0)
    static double U_hat = 0;      // Initial estimated state (assuming we dont know)
    static double K = 0;

    // Initialize
    K = P * H / (H * P * H + R);         // Update kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated value

    // Update error covariance
    P = (1 - K * H) * P + Q;

    //Return estimate of U - H * U_hat
    return U_hat;
}

double kalman_my(double U);
double kalman_my(double U)
{
    static const double R = 40;   // Noise Covariance (its actually 10)
    static const double H = 1.00; // Measurement Map Scalar
    static double Q = 10;         // Initial estimated covariance
    static double P = 0;          // Inital error covariance (must be 0)
    static double U_hat = 0;      // Initial estimated state (assuming we dont know)
    static double K = 0;

    // Initialize
    K = P * H / (H * P * H + R);         // Update kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated value

    // Update error covariance
    P = (1 - K * H) * P + Q;

    //Return estimate of U - H * U_hat
    return U_hat;
}

double kalman_mz(double U);
double kalman_mz(double U)
{
    static const double R = 40;   // Noise Covariance (its actually 10)
    static const double H = 1.00; // Measurement Map Scalar
    static double Q = 10;         // Initial estimated covariance
    static double P = 0;          // Inital error covariance (must be 0)
    static double U_hat = 0;      // Initial estimated state (assuming we dont know)
    static double K = 0;
=======
    // Noise Covariance
    static constexpr double R = 40.0;
    // Measurement Map Scalar
    static constexpr double H = 1.0;

    double Q = 10;
    double P = 0;
>>>>>>> 72ad10cb0944f7099dc6581d683584b89bb68fc1

    // Initialize
    K = P * H / (H * P * H + R);         // Update kalman gain
    U_hat = U_hat + K * (U - H * U_hat); // Update estimated value

    // Update error covariance
    P = (1 - K * H) * P + Q;

<<<<<<< HEAD
    //Return estimate of U - H * U_hat
    return U_hat;
}

=======
    double getKalman() { return m_U; }
};
>>>>>>> 72ad10cb0944f7099dc6581d683584b89bb68fc1
#endif