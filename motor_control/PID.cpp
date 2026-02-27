#include "PID.h"
#include "Arduino.h"

// Constructor implementation
PID::PID(double kp, double ki, double kd, double max_error)
    : kp_(kp), ki_(ki), kd_(kd), max_error_(max_error), 
      last_error_(0), integral_sum_(0), last_output_(0) {}

// Calculate method implementation
double PID::Calculate(double setpoint, double input, double dt)
{
    // Safety check: Prevent division by zero if dt is 0 or negative
    if (dt <= 0.0) {
        return last_output_; 
    }

    double error = setpoint - input;
    
    // Proportional term
    double proportional = kp_ * error;
    
    // Integral term with Anti-windup
    integral_sum_ += (error * dt);
    
    if (integral_sum_ > max_error_) {
        integral_sum_ = max_error_;
    } else if (integral_sum_ < -max_error_) {
        integral_sum_ = -max_error_;
    }
    
    double integral = ki_ * integral_sum_;
    
    // Derivative term
    double derivative = kd_ * (error - last_error_) / dt;
    
    // Total Output
    double output = proportional + integral + derivative;
  
    // Save state for the next cycle
    last_error_ = error;
    last_output_ = output; 
    
    return output;
}