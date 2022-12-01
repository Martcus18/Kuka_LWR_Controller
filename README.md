# Observer based residuals for KUKA LWR-4

The controller allows to pilot the robot in position and the collision detection and isolation framework uses the observer-based residuals.

## Compiling procedure

For compiling use the standard procedure:

    - mkdir build

    - cd build

    - cmake ..

    - make 


## Simulation

The simulations are implemented in C++ considering both fault and collision scenarios, the robot is commanded with a feedback linearitazion control scheme.

1. _main_sim.cpp_ 
2. _main_sim_noise.cpp_ regards the simulation in which a Gaussian noise with zero mean has been added to the measurements of joint positions and motor torques

## Experiment

The code running on the real KUKA LWR 4+ is implemented in C++.

1. _main_pos_reduced_observer.cpp_ regards the implementation of the approximated residual when a reduced-order observer is used for the estimation of the joint velocities
2. _main_pos_full_observer.cpp_ regards the implementation of the approximated residual when a full-order observer is used for the estimation of the joint velocities

In order to connect to the robot and run the code, use the following procedure:

    - sudo ./main_pos_reduced_observer
or

    - sudo ./main_pos_full_observer
