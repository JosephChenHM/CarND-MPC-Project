# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Driving Simulator
### Driving Video
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Gc68NO1NEm8/sddefault.jpg )](https://www.youtube.com/watch?v=Gc68NO1NEm8)

## The Vehicle Model
The vehicle model used here is a kinematic bicycle model that neglecting the complex dynamical effect such as vehicle inertia, tires friction, and tire slip angle. The model equations are as follow:
![](https://i.imgur.com/eFPFrOd.png)
* `Lf`: Distance between mass center to vehicle front wheels
* `Xt , Yt`: Vehicle position (x,y)
* `psi`: Vehicle orientation
* `V`: Velocity
* `cte`: Cross-track error
* `e_psi`: Orientation error
## Timestep Length and Elapsed Duration (N & dt)
The prediction horizon is define as `T = N * dt`. If the prediction horizon is too short, the MPC will be sensitive. It reacts fast but easily goes off the track. Increase prediction horizon help solve this problem; however, it means that we have to solve bigger MPC problem. This will increase latency for the car.

I first choose my first parameter set `T=1, N=20, dt=0.1`. It predicts 2 sec trajectory for the future. However,  I found it really unstable for my simulator. This is because my laptop is quite old and it caused long latency that my car really unstable. After several trial and error, I finalize my parameter set `T=0.8, N=8, dt=0.1`. This could drive the car around 69mph in my simulator.

## Polynomial Fitting and MPC Preprocessing
The reference waypoints are provided by Udacity simulator in Map coordinate system. In order to visualize reference trajectory of the car, we transformed it to car coordinate system.

We then used a 3rd-degree polynomial function to fit the transformed waypoints. It returns polynomial coefficients that we could evaluate our MPC error, such as `cte` and `e_psi`.

## Model Predictive Control with Latency

In order to deal with latency, we need to redefine our current state. The current vehicle state will be 100ms (predefined in this project) further than actual vehicle state since the actuator latency. The current state needs to propagating forward as actuator exactly effect the car. We then send this state to MPC to find out optimal control input. The propagating state can be found at (line `112` in `src/main.cpp`)

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).