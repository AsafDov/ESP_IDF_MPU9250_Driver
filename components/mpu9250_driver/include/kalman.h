typedef struct {
    // The state vector: [angle, gyro_bias]
    float angle;
    float bias;

    // The covariance matrix
    float P[2][2];

    // The noise parameters (tuning)
    float Q_angle;  // Process noise for angle
    float Q_bias;   // Process noise for bias
    float R_measure; // Measurement noise
} KalmanFilter;

void kalman_init(KalmanFilter *filter);
void kalman_predict(KalmanFilter *filter, float new_rate, float dt);
void kalman_update(KalmanFilter *filter, float new_measurement);