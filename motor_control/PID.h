#ifndef PID_H
#define PID_H

class PID {
  public:
    PID(double kp, double ki, double kd, double max_error);
    
    double Calculate(double setpoint, double input, double dt);

  private:
    double kp_;
    double ki_;
    double kd_;
    double max_error_;
    
    double last_error_;
    double integral_sum_;
    double last_output_;
};

#endif