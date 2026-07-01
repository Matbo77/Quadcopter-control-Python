# Quadcopter-control-Python

Simulation and control of a quadcopter UAV, visualization of its trajectory and tracking performance in Python.

<a href= "https://img.shields.io/badge/github-repo-blue?logo=github"> <img src="https://img.shields.io/badge/github-repo-blue?logo=github" alt="GitHub Badge"/></a>
 ![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
 <!-- <a href= "https://img.shields.io/badge/CasADi-orange"> <img src="https://img.shields.io/badge/CasADi-orange" alt="CasADi Badge"/></a> -->
 
---
## 📌 Overview
This project aims to develop a simple simulation model of a quadcopter UAV with its graphic visualization in Python. Moreover, it implements different controllers for tracking several types of 3D reference trajectories. It also includes visualization of the flight data, and controller performances. 



## System dynamic description

The quadrotor is modeled using the full 6-DOF Newton–Euler equations (second law of Newton and Euler’s rotation equations), including nonlinear couplings, gyroscopic moments, and cross-inertia effects. 

$$
 \begin{array}{lll}
    \begin{bmatrix} \ddot{x} \\ 
    \ddot{y} \\ 
    \ddot{z} \end{bmatrix} &=& \begin{bmatrix} 0 \\
    0 \\
    -g \end{bmatrix} + \frac{1}{m}R^{-1}(\phi,\theta,\psi) \begin{bmatrix} 0 \\
    0 \\ 
    u_1 \end{bmatrix}  - \frac{1}{m}k_D \begin{bmatrix} \dot{x} \\
    \dot{y} \\
    \dot{z} \end{bmatrix} \\
    \begin{bmatrix} \dot{\omega}_x \\ 
    \dot{\omega}_y \\
    \dot{\omega}_z \end{bmatrix} &=&    \begin{bmatrix} \frac{1}{I_{xx}} u_2 \\
    \frac{1}{I_{yy}} u_3 \\
    \frac{1}{I_{zz}} u_4 \end{bmatrix} -  \begin{bmatrix} \frac{I_{yy}-I_{zz}}{I_{xx}} \omega_y\omega_z \\
    \frac{I_{zz}-I_{xx}}{I_{yy}} \omega_x\omega_z \\ 
    \frac{I_{xx}-I_{yy}}{I_{zz}} \omega_x\omega_y \end{bmatrix} \\
      \begin{bmatrix}  \dot{\phi} \\
      \dot{\theta} \\
      \dot{\psi} \end{bmatrix}  &=& J(\phi,\theta)
    \begin{bmatrix} \omega_x \\
    \omega_y \\
    \omega_z \end{bmatrix}
    \end{array} 
$$

with $u = [u_1,u_2,u_3,u_4]^\top = [T_B,\tau_{\phi},\tau_{\theta},\tau_{\psi}]^\top$ the torque input, $(\phi,\theta,\psi)$ the Euler attitudes (roll,pitch,yaw), $(\omega_x,\omega_y,\omega_z)$ the angular velocities in the body frame, $R(\phi,\theta,\psi)$ the rotation matrix from from the inertial world frame to the body frame, $m$ the quadcopter mass, $k_D$ the drag coefficient, $I_{xx},I_{yy},I_{zz}$ the inertia matrix diagonal coefficients, $J(\phi,\theta)$ the angular velocity transformation matrix which convert attitude and angular speed of the quadcopter from body frame to inertial frame.

## Control loop scheme
To simplify we can consider command in torque that will be converted into a command in rotation speed of each motor:

<img alt="Control" src="pictures/quadcopter_control_loop.png" width="70%" height="70%"> </img>

### 📊 Controllers Implemented:
- Feedback linearization
- Sliding Mode Control (SMC) with saturation

## 📈 Visualize Results


- Tracking helicoidal trajectory using SMC:
  
<img alt="Traj heli" src="pictures/quadcopter_trajectory_helicoidale_SMC.gif" width="60%" height="60%"> </img>

- Tracking Bernoulli lemniscate type trajectory using SMC:
  
<img alt="Traj lemniscate" src="pictures/quadcopter_trajectory_lemniscate_SMC.gif" width="60%" height="60%"> </img>


<!--  <img alt="Traj lemniscate" src="pictures/Lemniscate_traj_SMC.png" width="60%" height="60%"> </img>-->


## How to use it

1. Run the main script `quadcopter_main.py` on your favorite Python interpreter.

## 🤝 Contributing

Contributions are welcome!

Future improvements could include:
- MPC implementation
- Better controller tuning
- More detailled Readme
- Monte Carlo simulation
---

## 📚 References  
- ["Model Based Control of Quadcopters", Spring Semester 2015-2016, MT Master Project EPFL, Martí POMÉS ARNAU](https://upcommons.upc.edu/server/api/core/bitstreams/b03e2a1f-d047-45d4-8b43-941f029e6729/content)  
