# PID Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project we would implement a PID controller to follow a desired trajectory around a test track in Udacity's simulator.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Reflection

**Describe the effect each of the P, I, D components had in your implementation.**

* The P component for the steering controller is responsible for giving a part of the control input that is proportional to the negative of the Cross Track Error. In the simulations it determined whether the car is able to turn properly at curves but it also lead to oscillations about the desired center position. For the throttle controller the P component mostly determines whether to increase or decrease current speed to reach desired speed. 

* The I component for the steering controller is responsible for giving a part of the control input that is proportional to the sum of the Cross Track Error over previous time steps. In the simulations it determined whether the car is able to maintain a zero mean position. For the throttle controller, the I component accelerated or deaccelerated the car to a small speed range around the the desired speed.

* The D component for the steering controller is responsible for giving a part of the control input that is proportional to the slope of the Cross Track Error. In the simulations it played a large role in reducing oscillations about mean position but also increased the inertia of the car at turns. For the throttle controller it was not used as it would hamper the braking of the car.

* For the throttle controller, the desired speed was set as adaptive to incorporate braking action - the desired speed was set as low when cross track error was high. This introduces tap braking which prevents the car from going offroad at turns and stabilizes oscillations somewhat. The "PID_constant.mp4" video shows the performance of the steering and throttle PID controllers for 2 laps for maximum desired speed of 60 MPH.


**Describe how the final hyperparameters were chosen.**

* The final hyperparameters were chosen by a combination of running a version of the twiddle algorithm and manual tuning. The twiddle algorithm functions like adaptive learning rate gradient descent for the three parameters P, I and D except that the increments don't depend on the actual gradient of a cost function w.r.t. the parameters. Instead the parameters are incremented/decremented at a certain time step by a predefined amount and after a certain settling time the impact on the error is estimated to evaluate whether the change in parameter lead to a lower error which is in turn followed by appropriate action. 

* The "PID_twiddle.mp4" video shows the above described twiddle algorithm based PID controllers in which the parameters are constantly adjusted until they converge to a certain value by the end of the lap. At the next lap the twiddle algorithm is restarted. This procedure yielded some rough values which were further tweaked a bit manually to yield the costant values of the P, I and D components for both the steering and throttle controllers as shown in the first video. Please refer to the left terminal window in the videos to see current values of the parameters.


**Videos**

* [Twiddle](https://youtu.be/gkix5AbVKHM)
* [Constant](https://youtu.be/UxB48SMWs6s)
