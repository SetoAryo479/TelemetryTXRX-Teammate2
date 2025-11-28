#ifndef _KALMAN_H_
#define _KALMAN_H_

class Kalman {
public:
    // Konstruktor dengan nilai default yang wajar
    Kalman(float Q_angle_ = 0.001f, float Q_bias_ = 0.003f, float R_measure_ = 0.03f) {
        Q_angle = Q_angle_;
        Q_bias  = Q_bias_;
        R_measure = R_measure_;

        angle = 0.0f;
        bias = 0.0f;

        P[0][0] = 0.0f;
        P[0][1] = 0.0f;
        P[1][0] = 0.0f;
        P[1][1] = 0.0f;
    }

    void setAngle(float newAngle) {
        angle = newAngle;
    }

    // core Kalman update: newAngle = measurement from accel, newRate = gyro rate (deg/s), dt = seconds
    float getAngle(float newAngle, float newRate, float dt) {
        // Predict
        rate = newRate - bias;
        angle += dt * rate;

        P[0][0] += dt * ( dt*P[1][1] - P[0][1] - P[1][0] + Q_angle );
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Update
        float S = P[0][0] + R_measure;
        if (S == 0.0f) S = 1e-9f; // guard against div0

        float K0 = P[0][0] / S;
        float K1 = P[1][0] / S;

        float y = newAngle - angle;

        angle += K0 * y;
        bias  += K1 * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K0 * P00_temp;
        P[0][1] -= K0 * P01_temp;
        P[1][0] -= K1 * P00_temp;
        P[1][1] -= K1 * P01_temp;

        return angle;
    }

    // setters/getters untuk tuning runtime
    void setQangle(float v){ Q_angle = v; }
    void setQbias(float v){ Q_bias = v; }
    void setRmeasure(float v){ R_measure = v; }
    float getQangle(){ return Q_angle; }
    float getQbias(){ return Q_bias; }
    float getRmeasure(){ return R_measure; }

private:
    float Q_angle;
    float Q_bias;
    float R_measure;

    float angle;
    float bias;
    float rate;
    float P[2][2];
};

#endif
