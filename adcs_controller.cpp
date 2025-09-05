#include "adcs_controller.h"

#include <cmath>
#include <cstring>
#include <algorithm>

ADCSController::ADCSController(float kp_detumble, float kp_point, float kd_point)
    : kp_detumble_(kp_detumble), kp_point_(kp_point), kd_point_(kd_point),
      ki_point_(0.01f), max_torque_(0.1f), first_mag_reading_(true) {
    for (int i = 0; i < 3; ++i) {
        pid_controllers_[i] = new PIDController(kp_point_, ki_point_, kd_point_);
        pid_controllers_[i]->setLimits(-max_torque_, max_torque_);
    }
    std::memset(previous_mag_, 0, sizeof(previous_mag_));
}

ADCSController::~ADCSController() {
    for (int i = 0; i < 3; ++i) {
        delete pid_controllers_[i];
    }
}

void ADCSController::computeControl(const float* g, const float* m, float s, float* w, float* mq, int cm) {
    std::memset(w, 0, 3 * sizeof(float));
    std::memset(mq, 0, 3 * sizeof(float));

    switch (cm) {
        case 0: safeMode(w, mq); break;
        case 1: detumbleControl(g, m, mq); break;
        case 2: pointingControl(g, w); break;
        case 3: scienceControl(g, w); break;
        default: safeMode(w, mq); break;
    }
}

void ADCSController::detumbleControl(const float* g, const float* m, float* mq) {
    if (first_mag_reading_) {
        std::memcpy(previous_mag_, m, 3 * sizeof(float));
        first_mag_reading_ = false;
        return;
    }

    float dt = 0.1f;
    for (int i = 0; i < 3; ++i) {
        float b_dot = (m[i] - previous_mag_[i]) / dt;
        mq[i] = kp_detumble_ * b_dot;
        mq[i] = std::max(-1.0f, std::min(1.0f, mq[i]));
    }

    std::memcpy(previous_mag_, m, 3 * sizeof(float));
}

void ADCSController::pointingControl(const float* g, float* w) {
    float dt = 0.1f;
    for (int i = 0; i < 3; ++i) {
        w[i] = pid_controllers_[i]->compute(0.0f, g[i], dt);
    }
}

void ADCSController::scienceControl(const float* g, float* w) {
    float dt = 0.1f;
    for (int i = 0; i < 3; ++i) {
        float t = pid_controllers_[i]->compute(0.0f, g[i], dt);
        w[i] = t * 0.5f;
        w[i] = std::max(-0.05f, std::min(0.05f, w[i]));
    }
}

void ADCSController::safeMode(float* w, float* mq) {
    std::memset(w, 0, 3 * sizeof(float));
    std::memset(mq, 0, 3 * sizeof(float));
}

void ADCSController::setGains(float kp_d, float kp_p, float kd_p) {
    kp_detumble_ = kp_d;
    kp_point_ = kp_p;
    kd_point_ = kd_p;

    for (int i = 0; i < 3; ++i) {
        pid_controllers_[i]->setGains(kp_point_, ki_point_, kd_point_);
    }
}

void ADCSController::setMaxTorque(float mt) {
    max_torque_ = mt;
    for (int i = 0; i < 3; ++i) {
        pid_controllers_[i]->setLimits(-max_torque_, max_torque_);
    }
}
