#include "PID.h"
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double inc, bool enable_twiddle) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd; 
	this->inc = inc;
	this->enable_twiddle = enable_twiddle;

	Kp_inc = inc*Kp;
  	Ki_inc = inc*Ki;
  	Kd_inc = inc*Kd;

  	Kp += Kp_inc;
  	Ki += Ki_inc;
  	Kd += Kd_inc; 

	p_add = true;
	i_add = true;
	d_add = true;

	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	best_error = 0.0;

	count = 0;
	tol = inc*inc*(Kp + Ki + Kd);
	settling_steps = 5;
	tuning_limit = 1000;
 	reset = false;
}

void PID::UpdateError(double measured_error) {
	d_error = measured_error - p_error;
	p_error = measured_error;
	i_error += measured_error;	
	count += 1;
}

double PID::TotalError(){

	//twiddle at only certain time steps
	if((enable_twiddle == true) && (count%settling_steps == 0)){
			Twiddle(Kp, Kp_inc, p_error, best_error, p_add);
			Twiddle(Ki, Ki_inc, p_error, best_error, i_add);
			Twiddle(Kd, Kd_inc, p_error, best_error, d_add);
	}

	// reset twiddling after lap
 	if(count > tuning_limit){
 		Kp_inc = inc*Kp;
  			Ki_inc = inc*Ki;
  			Kd_inc = inc*Kd;
  			reset = true;
  			count = 0;
  		}

	return  Kp*p_error + Ki*i_error + Kd*d_error;
	
}

void PID::Twiddle(double &p, double &dp, double error, double &best_error, bool &add_flag){
	// read flag and increment/decrement parameter and step size

	if(add_flag){
		if(error < best_error) {
			best_error = error;
			dp *= 1.1;
			p += dp;	
		}else{
			p -= 2*dp;
			dp *= 0.9;
			add_flag = false;
		}
	}else{
		if(error < best_error) {
			best_error = error;
			dp *= 1.1;
			p -= dp;
		}else {
			p += 2*dp;
			dp *= 0.9;			
			add_flag = true;
		}
	}			

}	

