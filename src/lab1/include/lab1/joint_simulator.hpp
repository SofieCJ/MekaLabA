
#ifndef LAB1_JOINT_SIMULATOR_HPP
#define LAB1_JOINT_SIMULATOR_HPP

class jointSimulator {
private:
    // Tilstandsvariabler
    double angle;               // theta (rad)
    double angular_velocity;    // theta_dot (rad/s)
    double voltage;             // inngangsspenning V (V)
    double noise;               // additiv støy (rad)

    double K; // forsterkning (rad/V)
    double T; // tidskonstant (s)

public:
    jointSimulator(double K_ = 230.0, double T_ = 0.15)
        : angle(0.0), angular_velocity(0.0), voltage(0.0), noise(0.0), K(K_), T(T_) {}

    void setVoltage(double v) { voltage = v; }
    void setNoise(double n) { noise = n; }

    void setParameters(double K_, double T_) {
        K = K_;
        T = T_;
    }

    [[nodiscard]] double getAngle() const { return angle; }
    [[nodiscard]] double getAngularVelocity() const { return angular_velocity; }

    void update(double dt) {
        if (dt <= 0.0) return;
        double x2_dot = (-angular_velocity + K * voltage) / T;
        angular_velocity += x2_dot * dt;
        angle += angular_velocity * dt;
        angle += noise;
    }
};

#endif //LAB1_JOINT_SIMULATOR_HPP