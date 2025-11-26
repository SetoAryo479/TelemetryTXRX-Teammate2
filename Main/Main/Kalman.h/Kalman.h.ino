#ifndef _KALMAN_H_
#define _KALMAN_H_

class Kalman {
public:
    Kalman() {
        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;

        angle = 0.0;
        bias = 0.0;

        P[0][0] = 0.0;
        P[0][1] = 0.0;
        P[1][0] = 0.0;
        P[1][1] = 0.0;
    }

    // Set awal sudut
    void setAngle(float newAngle) {
        angle = newAngle;
    }

    // ambil nilai angle terkini (Kalman Filter utama)
    float getAngle(float newAngle, float newRate, float dt) {
        rate = newRate - bias;
        angle += dt * rate;

        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        float y = newAngle - angle;

        angle += K[0] * y;
        bias += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }

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
