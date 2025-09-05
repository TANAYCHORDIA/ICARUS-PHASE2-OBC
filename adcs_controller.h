#ifndef ADCS_CONTROLLER_H
#define ADCS_CONTROLLER_H
#include "pid_controller.h"

class ADCSController {
public:
    ADCSController(float kp_detumble, float kp_point, float kd_point);
    ~ADCSController();
    void computeControl(const float* gyro_rates, const float* magnetometer,
                      float sun_angle, float* wheel_torques,
                      float* magnetorquer, int control_mode);
    void setGains(float kp_detumble, float kp_point, float kd_point);
    void setMaxTorque(float max_torque);
private:
    void detumbleControl(const float* gyro_rates, const float* magnetometer,
                       float* magnetorquer);
    void pointingControl(const float* gyro_rates, float* wheel_torques);
    void scienceControl(const float* gyro_rates, float* wheel_torques);
    void safeMode(float* wheel_torques, float* magnetorquer);
    PIDController* pid_controllers_[3];
    float kp_detumble_;
    float kp_point_;
    float kd_point_;
    float ki_point_;
    float max_torque_;
    float previous_mag_[3];
    bool first_mag_reading_;
};
#endif // ADCS_CONTROLLER_H
