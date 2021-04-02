#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

double kalman_ax(double U);
double kalman_ay(double U);
double kalman_az(double U);
double kalman_gx(double U);
double kalman_gy(double U);
double kalman_gz(double U);
double kalman_mx(double U);
double kalman_my(double U);
double kalman_mz(double U);
double kalman_compass(double U);
double kalman_pitch(double U);
double kalman_roll(double U);

double kalman_ax(double U)
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

    //Return estimate of U - H * U_hat
    return U_hat;
}

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

double kalman_mz(double U)
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

double kalman_compass(double U)
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

double kalman_pitch(double U)
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

double kalman_roll(double U)
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

#endif