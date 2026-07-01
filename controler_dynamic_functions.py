
import numpy as np
import matplotlib.pyplot as plt
import math


from casadi import *
from casadi.tools import *

# Helper functions (placeholders - implement these)
def thetadot2omega(Thetadot, Theta):
    # Convert Euler angle rates (dphi, dtheta, dpsi) to body frame angular velocity
    phi, theta, psi = Theta
    #dphi, dtheta, dpsi = Thetadot

    # omega_x = dphi - dpsi * np.sin(theta)
    # omega_y = dtheta * np.cos(phi) + dpsi * np.cos(theta) * np.sin(phi)
    # omega_z = -dtheta * np.sin(phi) + dpsi * np.cos(theta) * np.cos(phi)
    
    J_inv = np.array([[1, 0, -np.sin(theta)], [0, np.cos(phi), np.sin(phi)*np.cos(theta)],  [0, -np.sin(phi), np.cos(phi)*np.cos(theta)]])
    omega_B = J_inv @ Thetadot

    return omega_B      #np.array([omega_x, omega_y, omega_z])

def omega2thetadot(omega_B, Theta):
    """ Convert body frame angular velocity to inertial Euler angle rates """
    #phi, theta, psi = Theta
    phi = Theta[0,0]
    theta = Theta[1,0]
    psi = Theta[2,0]
    
    #omega_x, omega_y, omega_z = omega_B
    
    # dphi = omega_x + omega_y * np.sin(phi) * np.tan(theta) + omega_z * np.cos(phi) * np.tan(theta)
    # dtheta = omega_y * np.cos(phi) - omega_z * np.sin(phi)
    # dpsi = (omega_y * np.sin(phi) + omega_z * np.cos(phi)) / np.cos(theta)
    J = np.array([[1,  np.tan(theta)*np.sin(phi), np.tan(theta)*np.cos(phi)],[0, np.cos(phi), -np.sin(phi)],[ 0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

    Thetadot = J @ omega_B
    return Thetadot    # np.array([dphi, dtheta, dpsi])

def input_Theta_from_position_ref(Xglob, Xref, u_1, t, perf, data):
    # Convert position reference to angular reference (phi, theta)
    # u_1 : thrust

    m = data["m"]
    g = data["g"]
    L = data["L"]
    k_f = data["k_f"]
    k_M = data["k_M"]
    k_D = data["k_D"]

    xi_x = perf["xi_x"]
    w_x = perf["w_x"]
    xi_y = perf["xi_y"]
    w_y = perf["w_y"]

    x = Xglob[0]
    y = Xglob[1]
    dx = Xglob[3]
    dy = Xglob[4]
    psi = Xglob[8]

    x_ref = Xref[0]
    y_ref = Xref[1]
    dx_ref = Xref[3]
    dy_ref = Xref[4]



    M = np.array([[np.sin(psi), np.cos(psi)], [-np.cos(psi), np.sin(psi)]])
    inv_M = np.array([[np.sin(psi), -np.cos(psi)],[np.cos(psi), np.sin(psi)]])  # np.linalg.inv(M) 

    #M = np.array([[np.sin(psi), -np.cos(psi)], [np.cos(psi), np.sin(psi)]]) # new
    #inv_M = np.array([[np.sin(psi), np.cos(psi)],[-np.cos(psi), np.sin(psi)]])


    Theta_ref = inv_M @ (m/u_1*np.array([[-2*xi_x*w_x*(dx-dx_ref) - w_x**2*(x-x_ref)], [-2*xi_y*w_y*(dy-dy_ref) - w_y**2*(y-y_ref)]]) + k_D/u_1 * np.array([[dx], [dy]]))
    # better for tracking with dx_ref, dy_ref

    #Theta_ref = inv_M @ (m/u_1*np.array([[-2*xi_x*w_x*dx - w_x**2*(x-x_ref), -2*xi_y*w_y*dy - w_y**2*(y-y_ref)]]).T + k_D/u_1 * np.array([[dx, dy]]).T)

    phi_ref = Theta_ref[0,0]
    theta_ref = Theta_ref[1,0]

    return phi_ref, theta_ref

def torque_2nd_order_track_ref(Xglob, Thetadot, Xref, u_1, k, perf, data):
    # Compute control torques for 2nd order tracking
    # give the torque vector u for a tracking reference problem with
    # some 2nd order dynamic performance

    m = data["m"]
    g = data["g"]
    #L = data["L"]
    k_f = data["k_f"]
    k_M = data["k_M"]
    k_D = data["k_D"]

    I = data["I"]

    Ixx = I[0,0]
    Iyy = I[1,1]
    Izz = I[2,2]

    z = Xglob[2] # at time k
    z_ref = Xref[2]
    dz = Xglob[5]
    dz_ref = Xref[5]

    # angle
    phi = Xglob[6]
    phi_ref = Xref[6]
    theta = Xglob[7]
    theta_ref = Xref[7]
    psi = Xglob[8]
    psi_ref = Xref[8]

    #diff_theta = normalize_angle(theta,theta_ref)
    #diff_phi = normalize_angle(phi,phi_ref)
    diff_psi = psi - psi_ref
    #diff_psi = mod(psi - psi_ref + pi,2*pi) - pi;  % normalize angle between [-pi,pi]
    diff_psi = normalize_angle(diff_psi)

    # rotation speed
    dphi = Thetadot[0]    # Xglob[9]
    dtheta = Thetadot[1] # Xglob[10]
    dpsi = Thetadot[2]  #Xglob[11]

    #perf
    xi_z = perf["xi_z"]
    w_z = perf["w_z"]
    xi_phi = perf["xi_phi"]
    xi_theta = perf["xi_theta"]
    xi_psi = perf["xi_psi"]
    w_phi = perf["w_phi"]
    w_theta = perf["w_theta"]
    w_psi = perf["w_psi"]

    #wsteady_sq = m*g/(4*k_f);

    # u1 allready computed
    # more sophisticated
    #u_1 = 1/(np.cos(phi)*np.cos(theta))*(m*g + k_D*dz - m*(2*xi_z*w_z*dz + w_z**2*(z - z_ref)))

    #u_1_test = (1 / (np.cos(Xglob[7]) * np.cos(Xglob[6]))) * (m * g + k_D * Xglob[5] - m * (2 * xi_z * w_z * Xglob[5] + w_z**2 * (Xglob[2] - Xref[2])))

    #u_1 = m*g + k_D*dz - m*(2*xi_z*w_z*(dz- dz_ref) + w_z**2*(z - z_ref))
    #u_1_test = m * g + k_D * Xglob[5] - m * (2 * xi_z * w_z * Xglob[5] + w_z**2 * (Xglob[2] - Xref[2]))
    #u_1 = max(0,u_1); #saturation

    u_2 = Ixx*((Iyy-Izz)/Ixx*dtheta*dpsi - 2*xi_phi*w_phi*dphi - w_phi**2*(phi - phi_ref)) 
    u_3 = Iyy*((Izz-Ixx)/Iyy*dphi*dpsi - 2*xi_theta*w_theta*dtheta - w_theta**2*(theta - theta_ref)) 
    u_4 = Izz*((Ixx-Iyy)/Izz*dphi*dtheta - 2*xi_psi*w_psi*dpsi - w_psi**2*(diff_psi))


    u = np.array([[u_1, u_2, u_3, u_4]]).T

    return u

def torque2motor_speed(u, k, data):
    # Convert torques to motor rotation velocity (squared)
    # w_sq = [w_1**2 ; w_2**2 ; w_3**2 ; w_4**2] % square motors rotation speed
    # expressed in [rad/s]
    # u : torque input vector

    m = data["m"]
    g = data["g"]
    L = data["L"]
    k_f = data["k_f"]
    k_M = data["k_M"]

    u_1 = u[0]
    u_2 = u[1]
    u_3 = u[2]
    u_4 = u[3]
    
    w_sq = np.array([[1/(4*k_f)*u_1 - 1/(2*L*k_f)*u_3 + 1/(4*k_M)*u_4 , 
            1/(4*k_f)*u_1 + 1/(2*L*k_f)*u_2 - 1/(4*k_M)*u_4 , 
            1/(4*k_f)*u_1 + 1/(2*L*k_f)*u_3 + 1/(4*k_M)*u_4 , 
            1/(4*k_f)*u_1 - 1/(2*L*k_f)*u_2 - 1/(4*k_M)*u_4]]).T
    
    #w_sq = np.zeros((4,1))

    # verify w_sq >= 0
    ##w_sq = np.array([max(0, w) for w in w_sq ])
    #w_sq = np.maximum(w_sq , 0)

    # if min(w_sq)<0:
    #     print("Negative w:",min(w_sq),"at",k)
    
    return w_sq

def quad_acceleration(wi_sq : np.ndarray, Theta, Xdot, data):
    # Compute linear acceleration
    # wi_sq : omega_i_square -> inputs in [rad/s]
    m = data["m"]
    g = data["g"]
    #L = data["m"]
    k_f = data["k_f"]
    #k_M = data["k_M"]
    k_D = data["k_D"]

    gravity = np.array([[0, 0, -g]]).T

    R_B2I = rot_mat_B2I(Theta)  #R_inv
    #R_I2B = rot_mat_I2B(Theta)

    T_B = thrust(wi_sq, k_f) # thrust in the quadcopter body frame (along z)

    T_I = R_B2I @ T_B # T_I thrust in inertial reference frame

    Fd = -k_D * Xdot # fluid friction in inertial reference frame

    acc = gravity + 1 / m * T_I + 1 / m * Fd

    return acc

def  rot_mat_I2B(Theta):
    """ Rotation matrix from inertial frame to quadcopter bodyframe
     """

    R_I2B = rot_mat_B2I(Theta).T  # inverse matrix := transpose (orthogonal)
    

    return R_I2B

def rot_mat_B2I(Theta):
    """ Rotation matrix from bodyframe to quadcopter inertial frame 
    with Theta a column vector of Euler angles Z-Y-X """
    # R_ZYX = R(z,-psi) @ R(y,-theta) @ R(x,-phi)

    phi = Theta[0,0]
    theta = Theta[1,0]
    psi = Theta[2,0]

    cphi = np.cos(phi)
    sphi = np.sin(phi)

    ct = np.cos(theta)
    st = np.sin(theta)

    cpsi = np.cos(psi)
    spsi = np.sin(psi)


    #R_B2I = rot_mat_I2B(-Theta)   # old
    # R_B2I = rot_mat_I2B(Theta).T  # inverse matrix := transpose (orthogonal)

    R_B2I = np.array([[ct*cpsi, cpsi*st*sphi - spsi*cphi, cpsi*st*cphi + spsi*sphi],
                       [ct*spsi, spsi*st*sphi + cpsi*cphi, spsi*st*cphi - cpsi*sphi ],
                       [-st, ct*sphi, ct*cphi]])

    # R_B2I = np.array([[ct*cpsi, cpsi*st*sphi + spsi*cphi,  spsi*sphi - cpsi*st*cphi],
    #                   [-ct*spsi, cpsi*cphi - spsi*st*sphi, spsi*st*cphi + cpsi*sphi ],
    #                   [st, -ct*sphi, ct*cphi]])

    return R_B2I

def thrust(wi_sq : np.ndarray, k_f):
    # Compute thrust given current inputs and thrust coefficient.

    # wi_sq input are values for ${\omega_i}**2$

    T_B = np.array([[0, 0, k_f * np.sum(wi_sq)]]).T #oriented toward z in the quadcopter body frame 

    return T_B

def quad_angular_acceleration(wi_sq : np.ndarray, omega_B, data):
    # Compute angular acceleration
    # omegadot = [omegadot_x, omegadot_y, omegadot_z]

    #m = data["m"]
    #g = data["g"]
    L = data["L"]
    k_f = data["k_f"]
    k_M = data["k_M"]
    #k_D = data["k_D"]

    I = data["I"]

    Ixx = I[0,0]
    Iyy = I[1,1]
    Izz = I[2,2]
    
    # rotor speed rotation wi_sq
    tau = torques(wi_sq, L, k_M, k_f)
    #tau = [tau_phi, tau_theta, tau_psi]

    #omegadot = inv(I) * (tau - cross(omega, I * omega)); 
    #other way to write the formula

    omegadot = np.array([[1/Ixx*tau[0] - (Iyy-Izz)/Ixx*omega_B[1,0]*omega_B[2,0]],  
                         [1/Iyy*tau[1] - (Izz-Ixx)/Iyy*omega_B[0,0]*omega_B[2,0]],
                        [1/Izz*tau[2] - (Ixx-Iyy)/Izz*omega_B[0,0]*omega_B[1,0]]])

    return omegadot


def torques(wi_sq: np.ndarray, L, k_M, k_f):
    """ Compute torques, given current inputs, length, drag coefficient, and thrust coefficient."""
    # wi_sq are values for ${\omega_i}**2$

    tau = np.array([
        L * k_f * (wi_sq[1] - wi_sq[3]),  
        L * k_f * (wi_sq[2] - wi_sq[0]),  
        k_M * (wi_sq[0] - wi_sq[1] + wi_sq[2] - wi_sq[3])])

    #k_M * (wi_sq[0] + wi_sq[2] - (wi_sq[1] + wi_sq[3]))

    # tau = [tau_phi, tau_theta, tau_psi]

    return tau


def normalize_angle(angle):
    """ Return angle between [-pi,pi] """
    return  ((angle + math.pi) % (2*math.pi)) - math.pi # numpy vectorization compatible
    #return math.fmod(angle + math.pi,2*math.pi) - math.pi 


def f_dyn(t,X_state,wi_sq,data):
    """ Compute in one single function the quadcopter dynamic : state derivative"""
    # dynamic_quadcopter

    # Unpack the state variables
    X = X_state[0:3]
    Xdot = X_state[3:6]
    Theta = X_state[6:9]
    #Thetadot = X_state[9:12]   
    omega_B = X_state[9:12] 


    #omega_B = thetadot2omega(Thetadot, Theta)

    acc = quad_acceleration(wi_sq, Theta, Xdot, data)
    omegadot_B = quad_angular_acceleration(wi_sq, omega_B, data)
    Thetadot = omega2thetadot(omega_B,Theta)

    # Thetaddot,

    dot_X_state = np.concatenate((Xdot, acc, Thetadot, omegadot_B))

    return dot_X_state

def Euler_explicit(t,Xk,uk,wk,Te,n,data):
    """ Return a numerical approximation of the next time step discrete state given the continuous dynamic
    using Euler explicit method """

    dXk = f_dyn(t,Xk,uk,data)

    #Xk_1 = Xk + Te*dXk
    #Xk_1 = [Xk[i] + Te*dXk[i] for i in range(n) ] # list 

    Xk_1 = Xk + Te*dXk # numpy


    return Xk_1


def RK2(t,Xk,uk,wk,Te,n,data):

    k_1 = f_dyn(t,Xk,uk,data)
    #Xk_12 = [Xk[i] + Te/2*k_1[i] for i in range(n) ]
    Xk_12 = Xk + Te/2*k_1

    k_2 = f_dyn(t,Xk_12,uk,data)

    
    #Xk_1 = [Xk[i] + Te*k_2[i] for i in range(n) ]
    Xk_1 = Xk + Te*k_2

    return Xk_1

def RK4(t,Xk,uk,wk,Te,n,data):
    """ Runge-Kutta order 4"""
    # Xk : state at k
    # uk : command at k
    # Te : sampling period
    # Xk_1 : state at k+1 
    k_1 = f_dyn(t,Xk,uk,data)
    #Xk_12 = [Xk[i] + Te/2*k_1[i] for i in range(n) ] # len(Xk)
    Xk_12 = Xk + Te/2*k_1
    k_2 = f_dyn(t,Xk_12,uk,data)
    #Xk_23 = [Xk[i] + Te/2*k_2[i] for i in range(n) ]
    Xk_23 = Xk + Te/2*k_2    
    k_3 = f_dyn(t,Xk_23,uk,data)
    #Xk_34 = [Xk[i] + Te*k_3[i] for i in range(n) ]
    Xk_34 = Xk + Te*k_3
    k_4 = f_dyn(t,Xk_34,uk,data)

    #Xk_1 = [Xk[i] + Te/6*(k_1[i] + 2*k_2[i] + 2*k_3[i] + k_4[i]) for i in range(n) ]
    Xk_1 = Xk + Te/6*(k_1 + 2*k_2 + 2*k_3 + k_4)

    acc = k_2[3:6,:] # linear acceleration
    return Xk_1, acc


def RK4_casadi(t, Xk, Uk, Wk, Te, n):
    # k1
    k1 = f_dyn_casadi(t, Xk, Uk, Wk)
    Xk_12 = Xk + (Te / 2) * k1

    # k2
    k2 = f_dyn_casadi(t, Xk_12, Uk, Wk)
    Xk_23 = Xk + (Te / 2) * k2

    # k3
    k3 = f_dyn_casadi(t, Xk_23, Uk, Wk)
    Xk_34 = Xk + Te * k3

    # k4
    k4 = f_dyn_casadi(t, Xk_34, Uk, Wk)

    # Final result
    Xk_1 = Xk + (Te / 6) * (k1 + 2*k2 + 2*k3 + k4)
    return Xk_1


def f_dyn_casadi(t, Xk, Uk, Wk):
    # special f_dyn compatible with CasADi

    # TO update !!!!
    v = Uk[0] + Wk[0]
    w = Uk[1] + Wk[1]
    theta = Xk[2]

    # Compute the derivatives with CasADi functions
    dx = v * cos(theta)
    dy = v * sin(theta)
    dtheta = w

    return vertcat(dx, dy, dtheta)


def rho_affine_saturation(x,xmin,xmax,rho_min,rho_max):
    """ Linear saturation function usefull for SMC"""
    #np.sign(phi_ref) * min(abs(phi_ref), max_phi_ref)
    #s_x = min(max(x,xmin),xmax)
    #= min(max(s_x,ymin),ymax) 
    if x>=0:

        rho_s_x = rho_max*x/xmax

        if x>=xmax: 
            rho_s_x = rho_max

    if x<=0:

        rho_s_x = rho_min*x/xmin 

        if x<=xmin: 
            rho_s_x = rho_min
        
    return rho_s_x

#def sigmoid

def second_order_dynamic_controller(Xglob,Thetadot,Xref_locplan,Xref,t,k,data,perf):
    """ quadcopter position and attitude controller second order dynamic """
    
    m = data["m"]
    g = data["g"]
    #L = data["m"]
    k_f = data["k_f"]
    #k_M = data["k_M"]
    k_D = data["k_D"]

    # perf
    xi_z = perf["xi_z"]
    w_z = perf["w_z"]
    max_theta_ref = perf["max_theta"]
    max_phi_ref = perf["max_phi"]


    # Thrust command (z-axis)
    #u_1 = m*g + k_D*Xglob[5, k] - m * (2 * xi_z * w_z * (Xglob[5, k] - Xref[5,k]) + w_z**2 * (Xglob[2, k] - Xref[2,k])) # z_ref[k]

    # sophisticated
    #u_1 = 1/(np.cos(Xglob[7, k])*np.cos(Xglob[6, k])) * ( m*g + k_D*Xglob[5, k] - m * ( 2*xi_z*w_z*(Xglob[5, k]) + w_z**2 *(Xglob[2, k]-Xref_locplan[2,k+1])))
    u_1 = (1/(np.cos(Xglob[7, k])*np.cos(Xglob[6, k]))) * ( m*g + k_D*Xglob[5, k] - m * ( 2*xi_z*w_z*(Xglob[5, k]) + w_z**2 *(Xglob[2, k]-Xref_locplan[2,k+1])))

    #  -Xref_locplan[9,k+1] +  # -Xref_locplan[5,k+1]

    #u_1 = max(0, u_1[k])  # Thrust can only be positive, and there are issues when negative so ensure u_1 > 0
    u_1lim = 0.3*m*g # 1e-4
    if u_1 <= u_1lim:    
        #print("pb : u_1 =",u_1," at",t[k])
        
        #Descent case : minimum thrust
        u_1 = u_1lim
        
        #Xref[6, k+1] = Xref[6, k] 
        #Xref[7, k+1] = Xref[7, k]
        #Xref[8, k+1] = Xref[8, k]

        #u[:, k:k+1] = np.array([[u_1[k], 0, 0, 0]]).T

    #else:
    # Convert position reference to angular reference
    phi_ref, theta_ref = input_Theta_from_position_ref(Xglob[:, k], Xref_locplan[:, k+1], u_1, t[k], perf, data)

    # Saturate angular references
    theta_ref_sat = np.sign(theta_ref) * min(abs(theta_ref), max_theta_ref)
    phi_ref_sat = np.sign(phi_ref) * min(abs(phi_ref), max_phi_ref)

    # Update reference vector
    Xref[6, k+1] = phi_ref_sat
    Xref[7, k+1] = theta_ref_sat
    Xref_locplan[6, k+1] = phi_ref_sat
    Xref_locplan[7, k+1] = theta_ref_sat


    # Compute control torques and motor speeds
    u = torque_2nd_order_track_ref(Xglob[:, k], Thetadot[:, k], Xref_locplan[:, k+1], u_1, k, perf, data) # u_1 computed a second time

    wi_sq = torque2motor_speed(u, k, data)

    return u, wi_sq


def SMC_dynamic_controller(Xglob,Thetadot,Xref_locplan,Xref,t,k,data,perf,saturation):
    """ quadcopter position and attitude sliding mode controller SMC """

    m = data["m"]
    g = data["g"]
    #L = data["m"]
    k_f = data["k_f"]
    #k_M = data["k_M"]
    k_D = data["k_D"]

    # perf
    me_z = perf["me_z"]
    me_y = perf["me_y"]
    me_x = perf["me_x"]
    me_phi = perf["me_phi"]
    me_theta = perf["me_theta"]
    me_psi = perf["me_psi"]
    max_theta_ref = perf["max_theta"]
    max_phi_ref = perf["max_phi"]

    #rho_z defined the max acceleration
    rho_z = me_z**2*0.6 / 3 #me_z*0.5 / 0.8 #g  # me_z*0.5 / 0.8  me_z**2*0.5 / 3  0.2/L
    rho_y = me_y**2*0.6 / 3  #
    rho_x = me_x**2*0.6 / 3  # me_x**2*0.5 / 3
    rho_phi = me_phi**2*max_phi_ref / 3 #me_phi*max_phi_ref / 0.35 #15 #   # me_phi*max_phi_ref / 0.35
    rho_theta = me_theta**2*max_theta_ref / 3 # me_theta*max_theta_ref / 0.35 # 15 # me_theta*max_theta_ref / 0.35
    rho_psi = me_psi**2*90*pi/180 / 3 # me_psi*45*pi/180 / 0.7 #  me_psi*45*pi/180 / 0.7   30*pi/180
    epsilon_x = me_x*0.2
    epsilon_y = me_y*0.2

    I = data["I"]
    Ixx = I[0,0]
    Iyy = I[1,1]
    Izz = I[2,2]

    x = Xglob[0, k]
    y = Xglob[1, k]
    z = Xglob[2, k]
    dx = Xglob[3, k]
    dy = Xglob[4, k]
    dz = Xglob[5, k]
    psi = Xglob[8, k]

    # x_ref = Xref[0, k]
    # y_ref = Xref[1, k]
    # dx_ref = Xref[3, k]
    # dy_ref = Xref[4, k]
    
    e_z = z-Xref_locplan[2,k]
    e_dz = dz -Xref_locplan[5,k]
    ddz_ref = Xref_locplan[9,k]


    e_x = x-Xref_locplan[0,k]
    e_dx = dx-Xref_locplan[3,k]
    e_y = y-Xref_locplan[1,k]
    e_dy = dy-Xref_locplan[4,k]

    s_e_z =  e_dz + me_z*e_z
    s_e_x =  e_dx + me_x*e_x
    s_e_y =  e_dy + me_y*e_y


    dphi = Thetadot[0,k] 
    dtheta = Thetadot[1,k] 
    dpsi = Thetadot[2,k] 

    psi_ref = Xref_locplan[8, k+1]
    e_dpsi =  Thetadot[2,k]      
    e_psi = psi- psi_ref  
    s_e_psi = e_dpsi + me_psi*e_psi


    # Thrust command (z-axis)
    #u_1 = m*g + k_D*Xglob[5, k] - m * (2 * xi_z * w_z * (Xglob[5, k] - Xref[5,k]) + w_z**2 * (Xglob[2, k] - Xref[2,k])) # z_ref[k]

    # sophisticated
    #u_1[k] = 1/(np.cos(Xglob[7, k])*np.cos(Xglob[6, k])) * ( m*g + k_D*Xglob[5, k] - m * ( 2*xi_z*w_z*(Xglob[5, k]) + w_z**2 *(Xglob[2, k]-Xref_locplan[2,k+1])))

    #plot_saturation(-me_z*0.3,me_z*0.3,-rho_z,0.6*g)   # -3.0*g,rho_z

    if saturation==0:
        u_1 = (1/(np.cos(Xglob[7, k])*np.cos(Xglob[6, k]))) * ( -m*rho_z*np.sign(s_e_z) +  m*g + k_D*dz + m *( ddz_ref - me_z*e_dz)) #sign  +ddz_ref
    else:
        u_1 = (1/(np.cos(Xglob[7, k])*np.cos(Xglob[6, k]))) * ( -m*rho_affine_saturation(s_e_z,-me_z*0.3,me_z*0.3,-rho_z,0.6*g) +  m*g + k_D*dz + m *( ddz_ref - me_z*e_dz)) #sat #+  ddz_ref
        #u_1 = (1/(np.cos(Xglob[7, k])*np.cos(Xglob[6, k]))) * ( -m*rho_affine_saturation(s_e_z,-me_z*0.8,me_z*0.8,-rho_z,rho_z) +  m*g + k_D*dz + m *( ddz_ref - me_z*e_dz))

    #if k>347:
        #print("Time",k)


    M = np.array([[np.sin(psi), np.cos(psi)], [-np.cos(psi), np.sin(psi)]])
    inv_M = np.array([[np.sin(psi), -np.cos(psi)],[np.cos(psi), np.sin(psi)]]) # np.linalg.inv(M) 

    u_1lim = 0.32*m*g  # 0.3*m*g # 0.4/L
    if u_1<=u_1lim:
        #print("u_1<=u_1lim at ",k)
        u_1 = u_1lim
    

    if saturation==0:
        Theta_ref = inv_M @ (m/u_1*np.array([[-rho_x*np.sign(s_e_x) - me_x*e_dx], [-rho_y*np.sign(s_e_y) - me_y*e_dy]]) + k_D/u_1 * np.array([[dx], [dy]]))
    else:
        Theta_ref = inv_M @ (m/u_1*np.array([[-rho_affine_saturation(s_e_x,-epsilon_x,epsilon_x,-rho_x,rho_x) + k_D/u_1*dx - me_x*e_dx], [-rho_affine_saturation(s_e_y,-epsilon_y,epsilon_y,-rho_y,rho_y) + k_D/u_1*dy - me_y*e_dy]]))
    
    # + ddx_ref + ddy_ref

    phi_ref = Theta_ref[0,0]
    theta_ref = Theta_ref[1,0]

    # Saturate angular references
    phi_ref_sat = np.sign(phi_ref) * min(abs(phi_ref), max_phi_ref)
    theta_ref_sat = np.sign(theta_ref) * min(abs(theta_ref), max_theta_ref)

    # Update reference vector
    Xref[6, k+1] = phi_ref_sat
    Xref[7, k+1] = theta_ref_sat
    Xref_locplan[6, k+1] = phi_ref_sat
    Xref_locplan[7, k+1] = theta_ref_sat


    e_phi = Xglob[6, k]- Xref_locplan[6, k+1]
    e_theta = Xglob[7, k]- Xref_locplan[7, k+1]

    e_dphi = Thetadot[0,k] 
    e_dtheta = Thetadot[1,k] 

    s_e_phi = e_dphi + me_phi*e_phi
    s_e_theta = e_dtheta + me_theta*e_theta
    
    if saturation==0:
        u_2 = (-rho_phi*np.sign(s_e_phi) - me_phi*e_dphi)*Ixx + (Iyy-Izz)*dtheta*dpsi
        u_3 = (-rho_theta*np.sign(s_e_theta) - me_phi*e_dtheta)*Iyy + (Izz-Ixx)*dphi*dpsi
        u_4 = (-rho_psi*np.sign(s_e_psi) - me_psi*e_dpsi)*Izz + (Ixx-Iyy)*dtheta*dphi
    else:
        u_2 = (-rho_affine_saturation(s_e_phi,-0.8*me_phi*max_phi_ref,0.8*me_phi*max_phi_ref, -rho_phi,rho_phi) - me_phi*e_dphi)*Ixx + (Iyy-Izz)*dtheta*dpsi
        u_3 = (-rho_affine_saturation(s_e_theta,-0.8*me_theta*max_theta_ref,0.8*me_theta*max_theta_ref, -rho_theta,rho_theta) - me_theta*e_dtheta)*Iyy + (Izz-Ixx)*dphi*dpsi
        u_4 = (-rho_affine_saturation(s_e_psi,-me_psi*math.pi,me_psi*math.pi, -rho_psi,rho_psi) - me_psi*e_dpsi)*Izz + (Ixx-Iyy)*dtheta*dphi

    #plot_saturation(-me_phi*max_phi_ref,me_phi*max_phi_ref, -rho_phi,rho_phi)

    u = np.array([[u_1, u_2, u_3, u_4]]).T

    #u = torque_2nd_order_track_ref(Xglob[:, k], Thetadot[:, k], Xref_locplan[:, k+1], u_1, k, perf, data) # u_1 computed a second time

    wi_sq = torque2motor_speed(u, k, data)

    return u, wi_sq


def plot_saturation(epsilon_min,epsilon_max,ymin,ymax):


    s_e = np.linspace(1.5*epsilon_min,1.5*epsilon_max,100)
    rho_sat_s_e = []

    plt.figure()
    plt.grid(True)
    plt.title(r"$\rho$ Saturation")  #Suivi en ref de position
    plt.xlabel("s_e: sliding surface")
    for elt in s_e:
        rho_sat_s_e.append(rho_affine_saturation(elt,epsilon_min,epsilon_max,ymin,ymax)) 
    
    plt.plot(s_e,rho_sat_s_e)
    plt.show()
    #rho_affine_saturation(s_e,epsilon_min,epsilon_max,ymin,ymax)
    #rho_affine_saturation(s_e_z,-me_z*0.8,me_z*0.8,-rho_z,2.0*g)        
    return 