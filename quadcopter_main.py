import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import norm

import controler_dynamic_functions as fun
from plot_perf_comp import *
from gen_traj_3D import *

# Constants and parameters
g = 9.81  # gravity (m/s^2)
L = 0.23  # arm length (m)
I = np.diag([7.5e-3, 7.5e-3, 1.3e-2])  # Inertia matrix (kg·m²)
m = 0.65  # mass (kg)
k_D = 0.1  # Aerodynamic thrust drag coefficient
k_r = 0.1  # Aerodynamic moment drag coefficient
k_f = 3.13e-5  # Thrust coefficient
k_M = 7.7e-5  # Moment coefficient
nu = 4  # number of motors

# Steady-state motor speed squared to hover
wsteady_sq = m * g / (4 * k_f)

# Data dictionary
data = {
    "m": m,
    "g": g,
    "L": L,
    "k_f": k_f,
    "k_M": k_M,
    "k_D": k_D,
    "I": I,
}

# Simulation time
start_time = 0
end_time = 14 #15  # 10
dt = 0.01  # 0.01 # time step (s)  # sensitive to the sampling time
Te = dt
t = np.arange(start_time, end_time, dt)
Nsim = len(t)  # number of time steps


# Initialize state vectors
X = np.zeros((3, Nsim))  # position [x, y, z]
X0 = [0, 0, 0]
X[:, 0] = X0  # initial position
Xdot = np.zeros((3, Nsim))  # velocity [vx, vy, vz]
Theta = np.zeros((3, Nsim))  # orientation [phi, theta, psi]
Theta0 = [0, 0, 0]
Theta[:, 0] = Theta0 
Thetadot = np.zeros((3, Nsim))  # angular velocity [dphi/dt, dtheta/dt, dpsi/dt]
omega_B = np.zeros((3, Nsim))  # body frame angular velocity
omegadot_B = np.zeros((3, Nsim))  # body frame angular acceleration
acc = np.zeros((3, Nsim))  # linear acceleration Xddot 

# Global state vector: [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]
Xglob = np.vstack([X, Xdot, Theta, omega_B]) #  , Thetadot # omegadot_B
n = Xglob.shape[0]

Xglob_Euler = Xglob.copy()
Xglob_RK2 = Xglob.copy()
Xglob_RK4 = Xglob.copy()

# Control parameters (2nd order dynamic placement)

# first attempt/mode RMSE 1.19
xi_x, xi_y, xi_z = 1, 1, 1
t5x, t5y, t5z = 1.0, 1.0, 0.6
w_x, w_y, w_z = 4.75 / t5x, 4.75 / t5y, 4.75 / t5z

xi_phi, xi_theta, xi_psi = 1, 1, 1
t5phi, t5theta, t5psi = 0.1, 0.1, 0.5
w_phi, w_theta, w_psi = 4.75 / t5phi, 4.75 / t5theta, 4.75 / t5psi

# Second mode 1.66
xi_x, xi_y, xi_z = 1, 1, 1
t5x, t5y, t5z = 1.2, 1.2, 0.8
w_x, w_y, w_z = 4.75 / t5x, 4.75 / t5y, 4.75 / t5z

xi_phi, xi_theta, xi_psi = 1, 1, 1
t5phi, t5theta, t5psi = 0.35, 0.35, 0.7
w_phi, w_theta, w_psi = 4.75 / t5phi, 4.75 / t5theta, 4.75 / t5psi

# Artificial saturation on virtual angle reference
max_phi_ref = 35 * np.pi / 180  # 35
max_theta_ref = 35 * np.pi / 180 # 35

# Performance dictionary
perf = {
    "xi_x": xi_x, "w_x": w_x,
    "xi_y": xi_y, "w_y": w_y,
    "xi_z": xi_z, "w_z": w_z,
    "xi_phi": xi_phi, "w_phi": w_phi,
    "xi_theta": xi_theta, "w_theta": w_theta,
    "xi_psi": xi_psi, "w_psi": w_psi,
    "max_phi":max_phi_ref, "max_theta": max_theta_ref
}


# # time ot reach the sliding surface + time to slide to 0 (so t5z/2 to reach the same perf with SMC)
me_z, me_y, me_x = 3/(t5z/2), 3/(t5y/2), 3/(t5x/2)
me_phi, me_theta, me_psi = 3/(t5phi/2), 3/(t5theta/2), 3/(t5psi/2)

perf_SMC = { "me_z": me_z, "me_y": me_y , "me_x": me_x,
            "me_phi": me_phi, "me_theta": me_theta , "me_psi": me_psi,
            "max_phi":max_phi_ref, "max_theta": max_theta_ref }


# Reference trajectory

Xref,diff_traj = traj_step(Nsim,100,Xglob[:,0:1],Te,1)
#Xref,diff_traj = traj_helicoidale(Nsim,Te)
Xref,diff_traj = traj_helicoidal_custom(X0,1.5,3,Nsim,Te)
#Xref,diff_traj = traj_test_vz(X0, Nsim,Te) #trajectory with ups and down to test z-axis
#Xref,diff_traj = traj_test_explicit_derivative(X0,Nsim,t,Te,g) # traj with explicit velocity, acceleration
Xref,diff_traj =  traj_lemniscate_Bernoulli(2,X0[0],X0[1],X0[2],Nsim,Te,0)
#Xref,diff_traj = traj_helicoidal_variation(-X0[0],X0[1],2,Nsim,Te)
#Xref,diff_traj = traj_affine(Nsim,Te)
Xref_locplan = Xref.copy()

#plot_traj_z(t,Xref,[])

# Control inputs
u = np.zeros((nu, Nsim - 1))  # [atheta, aphi, apsi, F]
u_1 = np.zeros(Nsim - 1)  # thrust
wi_sq = np.zeros((nu, Nsim- 1))  # squared motor speeds


# Max acceleration with saturation angle and max thrust
# max_ax = 
# max_ay =

u_max = 1000 #?
max_vx_th = u_max/k_D*max_theta_ref
max_vy_th = u_max/k_D*max_phi_ref

# Max descent (-z) velocity 
min_vz_th = -g*m/k_D

u_min= 0.3*m*g # to avoid negative motor RP
min_az = -g + u_min/m   # -6.8
min_diff_dz = min_az*Te

min_dz = min_vz_th*Te

T_Beq = m*g
# max_vx = T_Beq/k_D
# max_dx = max_vx*Te

# max_vy =
# max_dy =

s_e_z = np.zeros(Nsim)
s_e_z[0] = (Xdot[2, 0]-Xref_locplan[5,0]) + me_z*(X[2,0]-Xref_locplan[2,0])
#Xref_replanner = replanner_test(Xref,min_vz,Nsim,Te)
#Xref = Xref_replanner


showQuad = True  # False
#visualize_quadcopter_pos_attitude(X0,Theta0,showQuad)


# Main simulation loop
for k in range(0,Nsim - 1):

    #print("---- Time step",k)  #  ,"/",N

    #Xref_locplan[:,k+1:k+2] = local_planner(Xglob,Xref,diff_traj,min_az,k,Te) #use local planner
    Xref_locplan = Xref #without local planner


    # Yaw reference : keep the initial or constant
    psi_ref_k = 0 #pi/4 # Theta[2,k]

    # Yaw reference: orientation toward center (0,0)
    #psi_ref_k = np.arctan2(X[1, k], X[0, k])

    # Yaw reference: orientation toward target
    #psi_ref_k = np.arctan2(Xref[1, k]- X[1, k], Xref[0, k]-X[0, k] )
    #psi_ref_k = np.arctan2(Xref_locplan[1, k+1]- Xglob[1, k], Xref_locplan[0, k+1]-Xglob[0, k]) - 0*pi/4
    
    Xref[8, k+1] = psi_ref_k
    Xref_locplan[8, k+1] = psi_ref_k




    # 2nd order dynamic (with feedback linearization)
    #u[:, k:k+1], wi_sq[:, k:k+1] = fun.second_order_dynamic_controller(Xglob,Thetadot,Xref_locplan,Xref,t,k,data,perf)

    # MPC ( TO DO )


    # Sliding Mode Control
    u[:, k:k+1], wi_sq[:, k:k+1] = fun.SMC_dynamic_controller(Xglob,Thetadot,Xref_locplan,Xref,t,k,data,perf_SMC,1)
    


    # System dynamic
    # Compute accelerations
    #omega_B[:, k] = fun.thetadot2omega(Thetadot[:, k], Theta[:, k])
    #acc[:, k+1:k+2] = fun.quad_acceleration(wi_sq[:, k], Theta[:, k:k+1], Xdot[:, k:k+1], data)
    #omegadot_B[:, k+1:k+2] = fun.quad_angular_acceleration(wi_sq[:, k], omega_B[:, k:k+1], data)
    #Thetadot[:, k+1:k+2] = fun.omega2thetadot(omega_B[:, k:k+1], Theta[:, k:k+1])

    
    # Euler integration (slow)
    # omega_B[:, k + 1] = omega_B[:, k] + dt * omegadot_B[:, k+1]
    # Theta[:, k + 1] = Theta[:, k] + dt * Thetadot[:, k + 1]
    # Xdot[:, k + 1] = Xdot[:, k] + dt * acc[:, k+1]
    # X[:, k + 1] = X[:, k] + dt * Xdot[:, k]
    # Theta[:, k + 1] = fun.normalize_angle(Theta[:, k + 1])
    # Xglob[:, k + 1] = np.concatenate([X[:, k + 1].flatten(),Xdot[:, k + 1].flatten(),Theta[:, k + 1].flatten(), omega_B[:, k + 1].flatten()])  # ,Thetadot[:, k + 1].flatten()


    # Numerical integration of system dynamic 
    #Xglob_Euler[:, k+1:k+2] = fun.Euler_explicit(k,Xglob_Euler[:, k:k+1],wi_sq[:, k],np.zeros((nu,1)),Te,n,data)
    #Xglob_Euler[6:9, k + 1] = fun.normalize_angle(Xglob_Euler[6:9, k + 1])
    #Xglob_RK2[:, k+1: k+2] = fun.RK2(k,Xglob_RK2[:, k:k+1],wi_sq[:, k],np.zeros((nu,1)),Te,n,data)  # Xglob[:, k + 1]
    #Xglob_RK2[6:9, k + 1] = fun.normalize_angle(Xglob_RK2[6:9, k + 1])
    Xglob_RK4[:, k+1: k+2],acc[:, k+1:k+2] = fun.RK4(k,Xglob_RK4[:, k:k+1],wi_sq[:, k],np.zeros((nu,1)),Te,n,data)  # Xglob[:, k + 1]
    Xglob_RK4[6:9, k + 1] = fun.normalize_angle(Xglob_RK4[6:9, k + 1])


    Xglob[:, k+1:k+2] = Xglob_RK4[:, k+1: k+2]
    X[:, k+1:k+2] = Xglob[0:3, k+1:k+2]
    Xdot[:, k+1:k+2] = Xglob[3:6, k+1:k+2]
    Thetadot[:, k+1:k+2] = fun.omega2thetadot(omega_B[:, k:k+1], Theta[:, k:k+1])
    Theta[:, k+1:k+2] = Xglob[6:9, k+1:k+2]
    omega_B[:, k+1:k+2] = Xglob[9:12, k+1:k+2]
    
    s_e_z[k+1] = (Xdot[2, k+1]-Xref_locplan[5,k+1]) + me_z*(X[2, k+1]-Xref_locplan[2,k+1])

# Compute RMSE for position tracking
pos_RMSE = np.mean(np.sqrt(np.sum((X - Xref[:3, :])**2, axis=0)))
final_pos_RMSE = np.mean(np.sqrt(np.sum((X[:,Nsim-20:] - Xref[:3, Nsim-20:])**2, axis=0)))

# Extract final states
x, y, z = X[0, :], X[1, :], X[2, :]
phi, theta, psi = Theta[0, :], Theta[1, :], Theta[2, :]
min_wi_sq = np.min(wi_sq)
# min_wi_sq = min(min(wi_sq))

#print(f"Squared motor speed steady: {wsteady_sq:.4f}") 
print(f"Position RMSE: {pos_RMSE:.4f}")  # 5.3782  #15s
print(f"Final Position RMSE: {final_pos_RMSE:.4f}")  # 5.3782  #15s
print(f"Minimum squared motor rotation speed [rot/s]: {min_wi_sq/(2*np.pi):.4f}")  #convert from rad/s to rotations/s rps

# # Visualization UAV trajecotry and tracking performance

# plt.figure(figsize=(10, 8))
# plt.plot(x_ref, y_ref, label="Reference Trajectory", linestyle="--")
# plt.plot(x, y, label="Actual Trajectory", linestyle="-")
# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")
# plt.title("Quadcopter Trajectory Tracking")
# plt.legend()
# plt.grid()
# plt.show()

#plot_traj_z(t,Xref,Xglob,Xref_locplan)
#plt.show()


animate_quadcopter(t, X, Xref, Theta, dt,save_gif=False)  # Xref_locplan # False # True



plot_command_quadcopter(t,Nsim,u,wi_sq)

plt.figure(figsize=(8, 6))
plt.plot(t,Xref[2,:], label="Zref", linestyle="-",marker='o')
plt.plot(t,Xref_locplan[2,:], label="Zref_locplan", linestyle="-",marker='o')
#plt.plot(t,Xglob[2,:], label="Zreal", linestyle="--",marker='o')
plt.xlabel("Time [s]")
plt.ylabel("Z ref [m]")
plt.title("Z reference vs local planner")
plt.legend()
plt.grid()

plt.figure(figsize=(8, 6))
plt.plot(t,s_e_z, label="s_e_z", linestyle="-",marker='o')
plt.xlabel("Time [s]")
plt.ylabel("Sliding surface (m/s)")
plt.title(r"Z sliding surface $s_{e_z}$")
plt.legend()
plt.grid()

plot_perf_quadcopter(t,Nsim,X,Xdot,acc,Theta,Thetadot,omega_B,Xref)


fig2 = plt.figure(figsize=(10, 8))
ax = fig2.add_subplot(111, projection = '3d')
ax.plot(Xref[0,:], Xref[1,:],Xref[2,:], label="Reference Trajectory", linestyle="--")
ax.plot(x,y,z, marker = '+', label="Actual Trajectory", linestyle="-")
#ax.plot(Xglob_RK2[0,:],Xglob_RK2[1,:],Xglob_RK2[2,:], marker = '.', label="Trajectory RK2", linestyle="--")
#ax.plot(Xglob_RK4[0,:],Xglob_RK4[1,:],Xglob_RK4[2,:], marker = '*', label="Trajectory RK4", linestyle="-.")
ax.legend()
ax.grid(True)
#plt.ioff()
#plt.ion()
title = 'Trajectory quadcopter' #+ 
ax.set_title(title)
ax.set(xlabel='Position x [m]', ylabel='Position y [m]', zlabel='Position z [m]')
plt.show(block = True)

#plt.pause(20) 
#plt.close("all")




#norm_error_Euler = norm(Xglob_RK4 - Xglob_Euler,axis=0)
#norm_error_RK2 = norm(Xglob_RK4 - Xglob_RK2,axis=0)
#mean_norm_error_Euler = np.mean(norm_error_Euler)
#mean_norm_error_RK2 = np.mean(norm_error_RK2)
#print(f"mean_norm_error_integration Euler: {mean_norm_error_Euler:.4f}")
#print(f"mean_norm_error_integration RK2: {mean_norm_error_RK2:.4f}")  
#plot_difference_integration(t,Xglob_RK4 - Xglob_Euler,"Euler")
#plot_difference_integration(t,Xglob_RK4 - Xglob_RK2,"RK2")


#Suggestion improvements

# use quaternions !!!

# add wind perturbation

# add an observer that estimate the state

# add measurement noise

# pb orientation switch/discontinuity with atan2 pi/-pi

# clean function definition and explanations