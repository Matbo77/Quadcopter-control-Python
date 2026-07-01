
import numpy as np
import math




def traj_step(Nsim,initial_delay,X0,Te,value):
    """ Generate traj step in every direction from X0 """
    """ thus positive and negative steps""" 
    differentiable_traj = 0

    # Global reference vector
    Xref = X0 * np.ones((12, Nsim))

    Nstep = (Nsim - initial_delay)//6

    X_steps = value*np.array([[0.3,0,0], [0,0.3,0], [0,0,0.5]])
    # np.eye() np.-eye()

    index_start = initial_delay 
    index_stop = initial_delay
    for k in range(3):
         
        index_start = index_stop 
        index_stop = index_start + Nstep
        Xref[0:3,index_start:index_stop] = Xref[0:3,index_start:index_stop] + X_steps[k:k+1,:].T

        index_start = index_stop 
        index_stop = index_start + Nstep
        Xref[:,index_start:index_stop] = X0 * np.ones((12, Nstep))


    #vx_ref = np.append(np.diff(Xref[0,:]),0)/Te
    #vy_ref = np.append(np.diff(Xref[1,:]),0)/Te
    #vz_ref = np.append(np.diff(Xref[2,:]),0)/Te
    
    # optionnal
    #Xref[3, :] = vx_ref
    #Xref[4, :] = vy_ref
    #Xref[5, :] = vz_ref


    #print(Xref.shape)
    return Xref,differentiable_traj



def traj_helicoidale(N,Te):
    """ Generate 3D position traj + desired speed"""
    differentiable_traj = 0
    theta_circle = np.linspace(0, 2 * np.pi, N)
    x_ref = 4 * np.cos(theta_circle) - 4 
    y_ref = 4 * np.sin(theta_circle) + 0
    #z_ref = np.concatenate([2 * np.ones(N // 3), 5 * np.ones(N // 3),5 * np.ones(N - 2 * (N // 3))])

    z_ref = np.concatenate([2 * np.ones(N // 4), 1.5 * np.ones(N // 4), 1 * np.ones(N // 4), 0 * np.ones(N - 3 * (N // 4))])  #descent

    z_ref = np.concatenate([2 * np.ones(N // 4), 1.9 * np.ones(N // 4), 1.8 * np.ones(N // 4), 1.9 * np.ones(N - 3 * (N // 4))]) #slow ups and downs
    #z_ref = np.linspace(0, 4, N)
    
    vx_ref = np.append(np.diff(x_ref),0)/Te
    vy_ref = np.append(np.diff(y_ref),0)/Te
    vz_ref = np.append(0,np.diff(z_ref))/Te

    phi_ref = np.zeros(N)
    theta_ref = np.zeros(N)
    psi_ref = np.zeros(N)

    # Global reference vector
    Xref = np.zeros((12, N))
    Xref[0, :] = x_ref
    Xref[1, :] = y_ref
    Xref[2, :] = z_ref
    Xref[3, :] = vx_ref
    Xref[4, :] = vy_ref
    Xref[5, :] = vz_ref
    Xref[8, :] = psi_ref  # yaw

    az_ref = np.append(np.diff(vz_ref),0)/Te

    #additionnal (if velocity differentiable)
    #Xref[9, :] = az_ref


    return Xref,differentiable_traj

def traj_affine(N,Te):
    differentiable_traj = 1
    theta_circle = np.linspace(0, 2 * np.pi, N)
    x_ref = 2 * theta_circle + 1
    y_ref = 1 * theta_circle + 0.5
    #z_ref = np.concatenate([2 * np.ones(N // 3), 3 * np.ones(N // 3), 4 * np.ones(N - 2 * (N // 3))])

    z_ref = 2 * theta_circle + 2

    phi_ref = np.zeros(N)
    theta_ref = np.zeros(N)
    #psi_ref = np.zeros(N)

    # Global reference vector
    Xk_ref = np.zeros((12, N))
    Xk_ref[0, :] = x_ref
    Xk_ref[1, :] = y_ref
    Xk_ref[2, :] = z_ref
    #Xk_ref[8, :] = psi_ref  # yaw

    vx_ref = np.append(np.diff(x_ref),0)/Te
    vy_ref = np.append(np.diff(y_ref),0)/Te
    vz_ref = np.append(np.diff(z_ref),0)/Te

    # optionnal
    Xk_ref[3, :] = vx_ref
    Xk_ref[4, :] = vy_ref
    Xk_ref[5, :] = vz_ref

    return Xk_ref,differentiable_traj



def traj_lemniscate_Bernoulli(r,x,y,z,Nsim,Te,constant_z):
    """ Return a ref describing a lemniscate"""
    #""" Famous eight shape trajectory"""
    differentiable_traj = 1
    phi = np.linspace(-math.pi,math.pi,Nsim)

    Xk_ref = np.zeros((12, Nsim))
    Xk_ref[0, :] = x + r*np.sin(phi)/(1+np.cos(phi)**2)
    Xk_ref[1, :] = y + r*np.sin(phi)*np.cos(phi)/(1+np.cos(phi)**2)

    if constant_z:
        z_ref = z*np.ones((1,Nsim))
        #Xref = np.array([x + a*np.sin(phi)/(1+np.cos(phi)**2), y + a*np.sin(phi)*np.cos(phi)/(1+np.cos(phi)**2)]).T

    else:
        z_ref = np.concatenate((np.linspace(z,z+1.0,np.floor(Nsim//2)),np.linspace(z+1.0,z,np.ceil(Nsim//2))), axis=0)
        #z_ref = np.linspace(2,0,N)
        #Xref = np.array([, , z_ref]).T
    
    Xk_ref[2, :] = z_ref

    vx_ref = np.append(np.diff(Xk_ref[0, :]),0)/Te
    vy_ref = np.append(np.diff(Xk_ref[1, :]),0)/Te
    vz_ref = np.append(np.diff(z_ref),0)/Te

    # optionnal
    Xk_ref[3, :] = vx_ref
    Xk_ref[4, :] = vy_ref
    Xk_ref[5, :] = vz_ref

    return Xk_ref,differentiable_traj

def traj_helicoidal_custom(X,r,z_plus,Nsim,Te):
    """ 3D trajectory, helicoidal up to z_plus, with a radius r around the center (x,y) """

    differentiable_traj = 1
    theta = np.linspace(-2*math.pi,2*math.pi,Nsim)
    dtheta = (theta[1] - theta[0])/Te

    z_ref = np.linspace(X[2], z_plus, Nsim)
    dz_ref = (z_ref[1] - z_ref[0])/Te

    Xk_ref = np.zeros((12, Nsim))
    Xk_ref[0, :] = X[0] - r + r*np.cos(theta)  # - r
    Xk_ref[1, :] = X[1] + r*np.sin(theta)
    Xk_ref[2, :] = z_ref

    # vx_ref1 = np.append(np.diff(Xk_ref[0, :]),0)/Te
    # vy_ref1 = np.append(np.diff(Xk_ref[1, :]),0)/Te
    # vz_ref1 = np.append(np.diff(z_ref),0)/Te

    # explicit derivative
    vx_ref = -dtheta* r*np.sin(theta) 
    vy_ref = dtheta* r*np.cos(theta) 
    vz_ref = dz_ref * np.ones(Nsim)

    #az_ref = 0 

    Xk_ref[3, :] = vx_ref
    Xk_ref[4, :] = vy_ref
    Xk_ref[5, :] = vz_ref


    return Xk_ref,differentiable_traj


def traj_helicoidal_variation(x,y,r,Nsim,Te):

    differentiable_traj = 1
    theta = np.linspace(0, 8 * np.pi, Nsim)
    #dtheta = theta[1] - theta[0]
    z_ref = np.linspace(0, 3, Nsim)
    r =  0.25*(z_ref-2)**2 + 0.5 # variable radius

    Xk_ref = np.zeros((12, Nsim))
    Xk_ref[0, :] = x + r * np.cos(theta)
    Xk_ref[1, :] = y + r * np.sin(theta)
    Xk_ref[2, :] = z_ref



    vx_ref = np.append(np.diff(Xk_ref[0, :]),0)/Te
    vy_ref = np.append(np.diff(Xk_ref[1, :]),0)/Te
    vz_ref = np.append(np.diff(z_ref),0)/Te

    # optionnal
    Xk_ref[3, :] = vx_ref
    Xk_ref[4, :] = vy_ref
    Xk_ref[5, :] = vz_ref


    return Xk_ref,differentiable_traj

def traj_test_explicit_derivative(X0,N,t,Te,g):

    differentiable_traj = 1

    # Global reference vector
    Xk_ref = np.zeros((12, N))

    x_ref = X0[0]* np.ones(N)
    y_ref = X0[1]* np.ones(N)

    #y_ref = 1 * alpha + 0.5
    #z_ref = np.concatenate([2 * np.ones(N // 3), 3 * np.ones(N // 3), 4 * np.ones(N - 2 * (N // 3))])

    #constant velocity
    vz_cst = 2
    z_ref = vz_cst * t + X0[2]
    vz_ref = vz_cst* np.ones(N)
    az_ref = np.zeros(N)

    # constant acceleration
    az_cst = 0.4*g
    z_ref = 0.5*az_cst * t**2 + X0[2]
    vz_ref = az_cst*t
    az_ref = az_cst* np.ones(N)

    #     
    theta = np.linspace(0,2*math.pi,N)
    dtheta = (theta[1] - theta[0])/Te
    ddtheta = 0

    r = 2.0
    x_ref = X0[0]* np.ones(N)
    y_ref = X0[1] - r + r* np.cos(theta)
    z_ref = X0[2] + r* np.sin(theta)  # +2

    vx_ref = np.zeros(N)
    vy_ref = -dtheta* r*np.sin(theta) 
    vz_ref = dtheta* r*np.cos(theta)

    #ax_ref
    #ay_ref
    az_ref = ddtheta* r*np.cos(theta) - dtheta**2 * r*np.sin(theta)


    Xk_ref[0, :] = x_ref
    Xk_ref[1, :] = y_ref
    Xk_ref[2, :] = z_ref
    Xk_ref[3, :] = vx_ref
    Xk_ref[4, :] = vy_ref
    Xk_ref[5, :] = vz_ref
    
    # optionnal
    Xk_ref[9,:] = az_ref

    return Xk_ref,differentiable_traj



def traj_test_vz(X0,N,Te):
    """ Generate 3D position traj + desired speed"""
    
    differentiable_traj = 0
    theta_circle = np.linspace(0, 2 * np.pi, N)
    x_ref = 2 * np.cos(theta_circle) - 2 
    y_ref = 2 * np.sin(theta_circle) + 0

    #x_ref = X0[0]* np.ones(N)
    #y_ref = X0[1]* np.ones(N)
    #z_ref = np.concatenate([2 * np.ones(N // 3), 5 * np.ones(N // 3),5 * np.ones(N - 2 * (N // 3))])

    # z_ref = np.concatenate([3 * np.ones(N // 5), 0 * np.ones(N // 5), 2.5 * np.ones(N // 5),
    #                         -0.5 * np.ones(N // 5), -2.5 * np.ones(N - 4 * (N // 5))]) #descent
    
    
    z_ref = np.concatenate([X0[2] * np.ones(N // 5), -1 * np.ones(N // 5), 2.5 * np.ones(N // 5),
                            0 * np.ones(N // 5), -1.0 * np.ones(N - 4 * (N // 5))]) 
    
    #z_ref = np.linspace(0, 4, N)
    
    vx_ref = np.append(np.diff(x_ref),0)/Te
    vy_ref = np.append(np.diff(y_ref),0)/Te
    vz_ref = np.append(np.diff(z_ref),0)/Te

    az_ref = np.append(np.diff(vz_ref),0)/Te


    phi_ref = np.zeros(N)
    theta_ref = np.zeros(N)
    psi_ref = np.zeros(N)

    # Global reference vector
    Xref = np.zeros((12, N))
    Xref[0, :] = x_ref
    Xref[1, :] = y_ref
    Xref[2, :] = z_ref
    Xref[3, :] = vx_ref
    Xref[4, :] = vy_ref
    #Xref[5, :] = vz_ref
    Xref[8, :] = psi_ref  # yaw
    
    # optionnal #if differentiable
    #Xref[9,:] = az_ref


    return Xref,differentiable_traj


def replanner_test(Xref,min_vz,Nsim,Te):
    """ Correct a trajectory to make it feasible for our quadcopter controller scheme"""

    # Correction on z-axis
    Xref_corrected = Xref.copy()
    i_z_correction = np.where(Xref[5,:]<min_vz)[0]   #Xref[5,:],
    vz_values = Xref[5,i_z_correction]
    

    for i in range(len(i_z_correction)):
        vz_value = vz_values[i]
        i_z = i_z_correction[i]
        n_steps = int(np.ceil(vz_value/min_vz)) # number of steps at minimum z velocity

        if i_z>0:
            if i_z+1+n_steps <= Nsim+1:
                #Xref[5,i_z:i_z+1+n_steps] = np.linspace(Xref[5,i_z-1],vz_value,n_steps)

                #correct velocity with respect to the minimum one for n_steps
                Xref_corrected[5,i_z:i_z+n_steps] =  vz_value/n_steps*np.ones(n_steps)

                for k in range(n_steps): # correct ref position accordingly 
                    Xref_corrected[2,i_z+k+1] =  Xref_corrected[2,i_z+k] +  Xref_corrected[5,i_z+k]*Te

            else:
                print("i_z+1+n_steps > Nsim+1") # to handle
        else:
            print("i_z>0")


    return Xref_corrected


def local_planner(Xglob,Xref,diff_traj,min_az,k,Te):
    """ Correct the trajectory to make it feasible at next time step for our quadcopter controller scheme 
    given its position at the current time step k"""

    # min_az stands for the minimal reachable acceleration on z for the quadcopter

    # min_vz
    z, vz = Xglob[2,k], Xglob[5,k]
    z_ref,vz_ref = Xref[2,k+1], Xref[5,k+1]
    Xref_local_planner = Xref[:,k+1:k+2].copy()

    # 

    # Correction on z-axis
    vz_goal = (z_ref - z)/Te
    
    az_goal = (vz_goal - vz)/Te

    #az_goal = (z_ref - 2 * z + Xglob[2, k-1]) / (Te**2)

    factor = 1000  # 100
    max_az = 10*10 # 10*g

    a_z = np.clip(az_goal, factor*min_az, factor*max_az)

    if diff_traj==1:
            Xref_local_planner[9,0] = factor*a_z  #acceleration
            Xref_local_planner[5,0] = vz + a_z*Te  #velocity  

    Xref_local_planner[2,0] = z + (vz + 0.5*a_z*Te)*Te

    """
    if az_goal < factor*min_az:

        #print("At ",k,", az_goal:",az_goal," / ",factor,"min_az:",factor*min_az)
        #Xref_local_planner[2,0] = z + factor*min_vz*Te
        # Xref_local_planner[5,0] = factor*min_vz
        # Xref_local_planner[9,0] = (factor*min_vz- vz)/Te

        if diff_traj==1:
            Xref_local_planner[9,0] = factor*min_az  #acceleration
        
        Xref_local_planner[5,0] = vz + factor*min_az*Te  #velocity     
        Xref_local_planner[2,0] = z + (vz + 0.5*factor*min_az*Te)*Te  # position   #vz*Te  Xref_local_planner[5,0]

        #Xref_local_planner[5,0] = vz_ref + factor*min_az*Te  #velocity     
        #Xref_local_planner[2,0] = z_ref + (vz_ref + factor*min_az*Te)*Te  # position   #vz*Te  Xref_local_planner[5,0]

    elif az_goal > factor*max_az:

        if diff_traj==1:
            Xref_local_planner[9,0] = factor*max_az  #acceleration
        
        Xref_local_planner[5,0] = vz + factor*max_az*Te  #velocity
        Xref_local_planner[2,0] = z + (vz + 0.5*factor*max_az*Te)*Te  # position   #vz*Te  Xref_local_planner[5,0]

        #Xref_local_planner[5,0] = vz_ref + factor*max_az*Te  #velocity
        #Xref_local_planner[2,0] = z_ref + (vz_ref + factor*max_az*Te)*Te  # position   #vz*Te  Xref_local_planner[5,0]
    """

    #else vz_goal > max_vz:


    """ 
    max_dx = 0.6
    max_dy = 0.6

    dx_goal = Xref[0,k+1] - Xglob[0,k]
    dy_goal = Xref[1,k+1] - Xglob[1,k]
    """
    
    # Xref_local_planner[0,0] = Xglob[0,k] + np.sign(dx_goal)*min(abs(dx_goal),max_dx)
    # Xref_local_planner[1,0] = Xglob[1,k] + np.sign(dy_goal)*min(abs(dy_goal),max_dy)
    

    return Xref_local_planner


    #def local_planner2(Xglob,Xref,diff_traj,min_az,max_az,k,Te):

