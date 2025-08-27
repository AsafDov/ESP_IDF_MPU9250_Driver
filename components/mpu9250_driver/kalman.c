#include "kalman.h"

void kalman_init(KalmanFilter *filter) {
    filter->angle = 0.0f;
    filter->bias = 0.0f;

    // Initialize covariance matrix - start with high uncertainty
    filter->P[0][0] = 1.0f;
    filter->P[0][1] = 0.0f;
    filter->P[1][0] = 0.0f;
    filter->P[1][1] = 1.0f;

    // --- Tuning Parameters ---
    // Start with these and adjust them to your needs
    filter->Q_angle = 0.02f; // High Q -> less trust in gyro
    filter->Q_bias = 0.003f;
    filter->R_measure = 0.3f;  // High R -> less trust in accelerometer
}

// Call kalman_init(&roll_filter); etc. in your main init function.

void kalman_predict(KalmanFilter *filter, float new_rate, float dt) {
    // Predict the state
    filter->angle += (new_rate - filter->bias) * dt;

    // Update the covariance matrix
    filter->P[0][0] += dt * (dt * filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
    filter->P[0][1] -= dt * filter->P[1][1];
    filter->P[1][0] -= dt * filter->P[1][1];
    filter->P[1][1] += filter->Q_bias * dt;
}

void kalman_update(KalmanFilter *filter, float new_measurement){
    float innovation = new_measurement - filter->angle;
    float S = filter->P[0][0] + filter->R_measure;
    float K[2]; // Kalman Gain vector
    K[0] = filter->P[0][0] / S;
    K[1] = filter->P[1][0] / S;

    // Update the state estimate
    filter->angle += K[0] * innovation;
    filter->bias += K[1] * innovation;

    // Update the covariance matrix
    float P00_temp = filter->P[0][0];
    float P01_temp = filter->P[0][1];

    filter->P[0][0] -= K[0] * P00_temp;
    filter->P[0][1] -= K[0] * P01_temp;
    filter->P[1][0] -= K[1] * P00_temp;
    filter->P[1][1] -= K[1] * P01_temp;

    // The final, filtered angle is now in filter->angle
    float final_roll = filter->angle;
};