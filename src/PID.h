#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double best_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Coefficients increments
  */ 
  double Kp_inc;
  double Ki_inc;
  double Kd_inc;
  double inc;

  /*
  *Hyperparameters
  */
  int count;
  float tol;
  int settling_steps;
  int tuning_limit;

  /*
  *Flags
  */
  bool enable_twiddle;
  bool reset;
  bool p_add;
  bool i_add;
  bool d_add;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double inc, bool enable_twiddle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Twiddle algorithm for parameter tuning
  */
  void Twiddle(double &p, double &dp, double error, double &best_error, bool &add_flag);

};

#endif /* PID_H */
