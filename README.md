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
\begin{bmatrix}
 \ddot{x} \\ 
 \ddot{y} \\   
 \ddot{z}
\end{bmatrix} = \frac{1}{D(\theta)} \begin{bmatrix}
 lF(t) - l\psi\dot{x} - ml^2\dot{\theta}^2\sin(\theta) - mgl\sin(\theta)\cos(\theta) -\phi\dot{\theta}\cos(\theta)  \\  
 -(M + m)g\sin(\theta) - \frac{M + m}{ml}\phi\dot{\theta} + \cos(\theta)F(t) - \psi\dot{x}\cos(\theta) - ml\dot{\theta}^2\sin(\theta)\cos(\theta)  \\ 
 0
\end{bmatrix} 
$$

## Control loop scheme

<img alt="Control" src="pictures/quadcopter_control_loop.png" width="70%" height="70%"> </img>

### 📊 Controllers Implemented:
- Feedback linearization
- Sliding Mode Control (SMC)

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
