#include "PID.h"
#include "Arduino.h"

PID::PID(double kp, double ki, double kd, double max_error)
    : kp_(kp), ki_(ki), kd_(kd), max_error_(max_error), 
      last_error_(0), integral_sum_(0), last_output_(0) {}

double PID::Calculate(double setpoint, double input, double dt)
{
    if (dt <= 0.0) {
        return last_output_; 
    }
    double error = setpoint - input;
    double proportional = kp_ * error;  
    integral_sum_ += (error * dt);
    if (integral_sum_ > max_error_) {
        integral_sum_ = max_error_;
    } else if (integral_sum_ < -max_error_) {
        integral_sum_ = -max_error_;
    }
    
    double integral = ki_ * integral_sum_;
    double derivative = kd_ * (error - last_error_) / dt;
    double output = proportional + integral + derivative;
    last_error_ = error;
    last_output_ = output; 
    
    return output;
}