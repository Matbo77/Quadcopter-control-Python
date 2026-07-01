from math import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import norm


from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D

from controler_dynamic_functions import rot_mat_B2I, rot_mat_I2B

from io import BytesIO
from PIL import Image

def plot_command_quadcopter(t,N,u,wi_sq):

    # plot command  (feasability)

    plt.figure()
    plt.subplot(4, 1, 1)
    plt.plot(t[:N-1], u[0, :N-1])
    plt.grid(True)
    plt.legend(["u_1: thrust"])
    plt.title("Control input u") # Iterative closest point
    plt.ylabel("$u_1$ (N)")

    plt.subplot(4, 1, 2)
    plt.plot(t[:N-1], u[1, :N-1])
    plt.grid(True)
    plt.legend([r'$u_2: \tau_{\phi}$'])
    plt.ylabel("$u_2$ (Nm)")

    plt.subplot(4, 1, 3)
    plt.plot(t[:N-1], u[2, :N-1])
    plt.grid(True)
    plt.legend([r'$u_3: \tau_{\theta}$'])
    plt.xlabel("Time (s)")
    plt.ylabel("$u_3$ (Nm)")

    plt.subplot(4, 1, 4)
    plt.plot(t[:N-1], u[3, :N-1])
    plt.grid(True)
    plt.legend([r'$u_4: \tau_{\psi}$'])
    plt.xlabel("Time (s)")
    plt.ylabel("$u_4$ (Nm)")

    # plot rpm motors %wi_sq
    # 180/np.pi * wi_sq[0, :N-1] #convert ot degree

    # w_i square
    plt.figure()
    plt.plot(t[:N-1], wi_sq[0, :N-1]/(2*np.pi) )  #convert from rad/s to rotations/s rps
    plt.plot(t[:N-1], wi_sq[1, :N-1]/(2*np.pi))
    plt.plot(t[:N-1], wi_sq[2, :N-1]/(2*np.pi))
    plt.plot(t[:N-1], wi_sq[3, :N-1]/(2*np.pi))
    plt.grid(True)
    plt.legend(["$w_1$", "$w_2$", "$w_3$", "$w_4$"])
    plt.title("Commands square rotation speed w")
    plt.ylabel(r'$w_i^2$(rps)')
    plt.xlabel("Time (s)")

    # w_i
    # plt.figure()
    # plt.plot(t[:N-1], 180/np.pi * np.sqrt(wi_sq[0, :N-1]))
    # plt.plot(t[:N-1], 180/np.pi * np.sqrt(wi_sq[1, :N-1]))
    # plt.plot(t[:N-1], 180/np.pi * np.sqrt(wi_sq[2, :N-1]))
    # plt.plot(t[:N-1], 180/np.pi * np.sqrt(wi_sq[3, :N-1]))
    # plt.grid(True)
    # plt.legend(["$w_1$", "$w_2$", "$w_3$", "$w_4$"])
    # plt.title("Commands rotation speed w")
    # plt.ylabel(r'$w_i$ (rpm)')
    # plt.xlabel("Time (s)")

    # plot artificial angles and saturations

    # \omega

    return


def plot_perf_quadcopter(t,N,X,Xdot,acc,Theta,Thetadot,omega_B,Xref):

    # plot perf tracking attitude and position

    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(t,X[0,0:N])
    plt.plot(t,Xref[0,:])
    plt.grid(True)
    plt.legend(["$x$","$x_{ref}$"])
    plt.title("Tracking position")  #Suivi en ref de position
    plt.ylabel("pos x (m)")

    plt.subplot(3,1,2)

    plt.plot(t,X[1,0:N])
    plt.plot(t,Xref[1,:])
    plt.grid(True)
    plt.legend(["$y$","$y_{ref}$"])
    plt.ylabel("pos y (m)")

    plt.subplot(3,1,3)
    plt.plot(t,X[2,0:N])
    plt.plot(t,Xref[2,:])
    plt.grid(True)
    plt.legend(["$z$","$z_{ref}$"])
    plt.xlabel("Time (s)")
    plt.ylabel("pos z (m)")



    # plot angles

    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(t,180/pi*Theta[0,0:N])
    plt.plot(t,180/pi*Xref[6,:])
    plt.grid(True)
    plt.legend([r"$\phi$",r"$\phi_{ref}$"])
    plt.title("Tracking attitude")   # Suivi angulaire
    plt.ylabel("roll angle (deg)")

    plt.subplot(3,1,2)

    plt.plot(t,180/pi*Theta[1,0:N])
    plt.plot(t,180/pi*Xref[7,:])
    plt.grid(True)
    plt.legend([r"$\theta$",r"$\theta_{ref}$"])
    plt.ylabel("pitch angle (deg)")

    plt.subplot(3,1,3)

    plt.plot(t,180/pi*Theta[2,0:N])

    plt.plot(t,180/pi*Xref[8,:])

    plt.grid(True)
    plt.legend([r"$\phi$",r"$\phi_{ref}$"])
    plt.xlabel("Time (s)")
    plt.ylabel("yaw angle (deg)")


    if True: #speed curve

        """plt.figure()
        plt.subplot(3,1,1)
        plt.plot(t,Thetadot[0,0:N])
        plt.plot(t,omega_B[0,0:N])
        plt.grid(True)
        plt.legend([r"$\dot{\phi}$",r"$\omega_x$"])
        plt.title("Comparison angular velocity")  #Suivi en ref de position
        plt.ylabel("Angular velocity (rad/s)")

        plt.subplot(3,1,2)

        plt.plot(t,Thetadot[1,0:N])
        plt.plot(t,omega_B[1,0:N])
        plt.grid(True)
        plt.legend([r"$\dot{\theta}$",r"$\omega_y$"])
        plt.ylabel("Angular velocity (rad/s)")

        plt.subplot(3,1,3)
        plt.plot(t,Thetadot[2,0:N])
        plt.plot(t,omega_B[2,0:N])
        plt.grid(True)
        plt.legend([r"$\dot{\psi}$",r"$\omega_z$"])
        plt.xlabel("Time (s)")
        plt.ylabel("Angular velocity (rad/s)") """


        plt.figure()
        plt.subplot(3,1,1)
        plt.plot(t,Xdot[0,0:N])
        plt.plot(t,Xref[3,:])
        plt.grid(True)
        plt.legend(["$v_x$","$v_{x,ref}$"])
        plt.title("Tracking velocity")  #Suivi en ref de position
        plt.ylabel("Velocity x (m/s)")

        plt.subplot(3,1,2)

        plt.plot(t,Xdot[1,0:N])
        plt.plot(t,Xref[4,:])
        plt.grid(True)
        plt.legend(["$v_y$","$v_{y,ref}$"])
        plt.ylabel("Velocity y (m/s)")

        plt.subplot(3,1,3)
        plt.plot(t,Xdot[2,0:N])
        plt.plot(t,Xref[5,:])
        plt.grid(True)
        plt.legend(["$v_z$","$v_{z,ref}$"])
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity z (m/s)")
        plt.show()

        #acceleration curve

        plt.figure()
        #plt.subplot(3,1,1)
        plt.plot(t,acc[2,0:N])
        plt.plot(t,Xref[9,:])
        plt.grid(True)
        plt.legend(["$a_z$","$a_{z,ref}$"])
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration z ($m/s^2$)")
        plt.show()
        plt.title("Tracking acceleration")  



    return

#def plot3D_anime_body_frame(): 
# Create the 3D figure

# --- Fonction principale d'animation ---
def animate_quadcopter(
    t, X, Xref, Theta, dt, showQuad=True,
    save_gif=False,
    gif_filename="quadcopter_trajectory_SMC.gif",
    gif_duration=6.0  # Durée en secondes pour le GIF
):
    """
    Affiche une animation 3D du drone avec contrôle de vitesse et sauvegarde en GIF.

    Args:
        t (array): Temps (1D, longueur N).
        X (ndarray): Positions réelles [3, N] (x, y, z).
        Xref (ndarray): Positions de référence [3, N].
        Theta (ndarray): Angles d'Euler [3, N] (phi, theta, psi en radians).
        dt (float): Pas de temps.
        save_gif (bool): Si True, sauvegarde en GIF.
        gif_filename (str): Nom du fichier GIF.
        gif_duration (float): Durée en secondes pour le GIF.
    """
    N = len(t)
    x, y, z = X[0], X[1], X[2]
    phi,theta,psi = Theta[0], Theta[1], Theta[2]
    xref, yref, zref = Xref[0], Xref[1], Xref[2]

    r_rotors = 0.1
    L = 0.3 # lenth arm quadcopter size in plot


    # List to stock frames
    frames = []

    # --- Configuration de la figure 3D ---
    fig = plt.figure(figsize=(8, 7))
    mngr = plt.get_current_fig_manager() 
    mngr.window.geometry("+300+50") # set figure position
    ax = fig.add_subplot(111, projection="3d")
    ax.grid(True)
    ax.set_box_aspect([1, 1, 1])  # Équivalent à `axis equal`
    ax.view_init(elev=20, azim=80)  # Équivalent à `view(80, 20)`
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    # --- Panneau de contrôle (slider + bouton Stop) ---
    ax_color = "lightgoldenrodyellow"
    panel_ax = plt.axes([0.01, 0.0, 0.2, 0.2], facecolor=ax_color)
    panel_ax.axis("off")

    # Slider pour la vitesse
    draw_speed = 1
    # # plt.axes([left, bottom, width, height],
    slider_ax = plt.axes([0.06, 0.25, 0.12, 0.2], facecolor=ax_color)
    speed_slider = Slider(slider_ax, "Speed", 1.0, 20.0, valinit=draw_speed)

    # Bouton Stop
    button_ax = plt.axes([0.06, 0.25, 0.12, 0.05], facecolor=ax_color)
    stop_button = Button(button_ax, "Stop Plot")

    # --- Annotations (temps, angles, position, distance) ---
    time_annot = ax.text2D(0.88, 0.7, "", transform=ax.transAxes, fontsize=10)
    angle_annot = ax.text2D(0.85, 0.4, "", transform=ax.transAxes, fontsize=10)
    pos_annot = ax.text2D(0.85, 0.2, "", transform=ax.transAxes, fontsize=10)
    dist_annot = ax.text2D(0.85, 0.0, "", transform=ax.transAxes, fontsize=10)


    # --- Référentiel inertiel (fixe) ---
    ax.plot3D(0, 0, 0, "ko")  # Origine
    ax.quiver3D(0, 0, 0, 0.5, 0, 0, color="g", linewidth=2)  # Axes x (green)
    ax.quiver3D(0, 0, 0, 0, 0.5, 0, color="b", linewidth=2)  # Axes y (blue)
    ax.quiver3D(0, 0, 0, 0, 0, 0.5, color="m", linewidth=2)  # Axes z (magenta

    theta_circle = np.linspace(0, 2 * np.pi, 20)
    circle_points_local = np.array([
        r_rotors * np.cos(theta_circle),
        r_rotors * np.sin(theta_circle),
        np.zeros_like(theta_circle)
    ])


    # --- Variables d'état ---
    stop_animation = False
    current_frame = 0
    dynamic_artists = []  # Stocke les éléments dynamiques à supprimer

    # --- Callback pour le bouton Stop ---
    def stop_plot(event):
        nonlocal stop_animation
        stop_animation = True

    stop_button.on_clicked(stop_plot)

    # --- Fonction de mise à jour de l'animation ---
    def update(frame):
        nonlocal current_frame, draw_speed, stop_animation, dynamic_artists

        if stop_animation:
            return dynamic_artists

        # Mise à jour de la vitesse à partir du slider
        draw_speed = int(speed_slider.val)
        current_frame += draw_speed
        if current_frame >= N:
            current_frame = N - 1
        i = current_frame

        # Suppression des éléments dynamiques précédents
        for artist in dynamic_artists:
            if artist:
                artist.remove()
        dynamic_artists = []

        # --- Mise à jour des annotations ---
        time_annot.set_text(f"Time:\n {t[i]:.2f} s")
        # angle_annot.set_text(
        #     f"Angle:\n $phi$ = {np.degrees(Theta[0, i]):.2f}°\n "
        #     f"$theta$ = {np.degrees(Theta[1, i]):.2f}°\n "
        #     f"$psi$ = {np.degrees(Theta[2, i]):.2f}°"
        # )
        # pos_annot.set_text(
        #     f"Pos:\n x = {x[i]:.2f} m\n "
        #     f"y = {y[i]:.2f} m\n "
        #     f"z = {z[i]:.2f} m"
        # )
        # dist_annot.set_text(
        #     f"Dist ref:\n dx = {x[i] - xref[i]:.2f} m\n "
        #     f"dy = {y[i] - yref[i]:.2f} m\n "
        #     f"dz = {z[i] - zref[i]:.2f} m"
        # )
        # dynamic_artists.extend([time_annot, angle_annot, pos_annot, dist_annot])


        # --- Trajectory ---
        trajectory, = ax.plot3D(x[:i+1], y[:i+1], z[:i+1], "r-")
        dynamic_artists.append(trajectory)

        trajectory_ref, = ax.plot3D(xref[:i+1], yref[:i+1], zref[:i+1], "b--")
        dynamic_artists.append(trajectory_ref)

        # quadcopter
        dynamic_artists = add_plot_quadcopter(x[i],y[i],z[i],phi[i],theta[i],psi[i],ax,dynamic_artists,circle_points_local,L,showQuad)
        

        # --- Ligne de distance à la référence ---
        dist_coeff = np.linspace(0, 1, 10)
        dist_x = x[i] + dist_coeff * (xref[i] - x[i])
        dist_y = y[i] + dist_coeff * (yref[i] - y[i])
        dist_z = z[i] + dist_coeff * (zref[i] - z[i])
        dist_line, = ax.plot3D(dist_x, dist_y, dist_z, "b--", linewidth=1)
        dynamic_artists.append(dist_line)

        # --- Point de référence ---
        ref_point, = ax.plot3D(xref[i], yref[i], zref[i], "b*")
        dynamic_artists.append(ref_point)

        # --- Ajustement dynamique des limites des axes ---
        margin = 1.0
        all_x = np.concatenate([x[:i+1], xref[:i+1]])
        all_y = np.concatenate([y[:i+1], yref[:i+1]])
        all_z = np.concatenate([z[:i+1], zref[:i+1]])
        ax.set_xlim([all_x.min() - margin, all_x.max() + margin])
        ax.set_ylim([all_y.min() - margin, all_y.max() + margin])
        ax.set_zlim([all_z.min() - margin, all_z.max() + margin])

        frames.append(_copy_frame(fig))
        #print(frames[0].shape)

        return dynamic_artists

    def _copy_frame(fig):
        """Copie la frame actuelle de l'axe sous forme d'image."""
        with BytesIO() as buff:
            fig.savefig(buff, format='raw')
            buff.seek(0)
            data = np.frombuffer(buff.getvalue(), dtype=np.uint8)
        w, h = fig.canvas.get_width_height()
        im = data.reshape((int(h), int(w), -1))
        return im

    fps_gif = 20
    gif_frames_speed = N/fps_gif/gif_duration

    if save_gif:
        speed_slider.set_val(gif_frames_speed)

    # --- Création de l'animation ---
    ani = FuncAnimation(
        fig,
        update,
        frames=N,
        interval=int(1/fps_gif),  # Délai entre les frames (ms)
        blit=False,
        repeat=False #save_gif   # False if not save_gif
        #repeat_delay=0
    )
    plt.show()

    # --- Sauvegarde en GIF (si activé) ---
    """if save_gif:

        #gif_frames = min(N, int(gif_duration / (N*dt)))
        #gif_frames = N/gif_duration
        gif_frames_speed = N/fps_gif/gif_duration
        speed_slider.set_val(gif_frames_speed) 
        writer = PillowWriter(fps=fps_gif)
        ani.save(gif_filename, writer=writer, dpi=100, loop=0, savefig_kwargs={"transparent": True})"""

    if save_gif:
        frames_pil = [Image.fromarray(frame) for frame in frames]
        # Sauvegarder le premier frame pour initialiser le GIF
        frames_pil[0].save(
        gif_filename,
        save_all=True,
        append_images=frames_pil[1:],
        duration=int(1/fps_gif),  # Durée de chaque frame en ms
        loop=0,  # 0 = boucle infinie
        disposal=2   # transparency=0,
    )

 #_save_as_gif(frames, gif_filename)

    return ani

def visualize_quadcopter_pos_attitude(X0,Theta0,showQuad):
    """ Visualize in 3D the quadcopter pos and attitudes : roll, pitch, yaw """ 

    r_rotors = 0.08 
    L = 0.23 # lenth arm quadcopter

    x, y, z = X0[0], X0[1], X0[2]
    phi, theta, psi = Theta0[0], Theta0[1], Theta0[2]

    # --- Configuration de la figure 3D ---
    fig = plt.figure(figsize=(9, 7))
    mngr = plt.get_current_fig_manager() 
    mngr.window.geometry("+300+50") # set figure position
    ax = fig.add_subplot(111, projection="3d")
    #fig.subplots_adjust(left=0.05, right=0.95, top=0.85, bottom=0.05)
    ax.grid(True)
    #ax.set_box_aspect([1, 1, 1])  # Équivalent à `axis equal`
    axis_limit = 1.5
    ax.set_xlim(-axis_limit, axis_limit)
    ax.set_ylim(-axis_limit, axis_limit)
    ax.set_zlim(-axis_limit, axis_limit)
    ax.view_init(elev=20, azim=80)  # Équivalent à `view(80, 20)`
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    # --- Référentiel inertiel (fixe / base) ---
    ax.plot3D(0, 0, 0, "ko")  # Origine
    ax.quiver3D(0, 0, 0, 0.5, 0, 0, color="g", linewidth=2)  # Axes x (green)
    ax.quiver3D(0, 0, 0, 0, 0.5, 0, color="b", linewidth=2)  # Axes y (blue)
    ax.quiver3D(0, 0, 0, 0, 0, 0.5, color="m", linewidth=2)  # Axes z (magenta)


    # --- Panneau de contrôle (slider + bouton Stop) ---
    ax_color = "lightgoldenrodyellow"
    ax_color2 = "blue"
    panel_ax = plt.axes([0.01, 0.0, 0.15, 0.3], facecolor=ax_color)
    panel_ax.axis("off")

    left_offset = 0.05
    height_slider = 0.15

    # Slider pour les positions
    # # plt.axes([left, bottom, width, height],
    slider_ax = plt.axes([left_offset, 0.2, 0.12, height_slider])
    pos_x_slider = Slider(slider_ax, "x", -axis_limit, axis_limit, valinit=x)

    slider_ay = plt.axes([left_offset, 0.3, 0.12, height_slider])
    pos_y_slider = Slider(slider_ay, "y", -axis_limit, axis_limit, valinit=y)

    slider_az = plt.axes([left_offset, 0.4, 0.12, height_slider])
    pos_z_slider = Slider(slider_az, "z", -axis_limit, axis_limit, valinit=z)

    # angles
    slider_phi = plt.axes([left_offset, 0.7, 0.12, height_slider])
    phi_slider = Slider(slider_phi, r"$\phi$", -pi, pi, valinit=phi)

    slider_theta = plt.axes([left_offset, 0.6, 0.12, height_slider])
    theta_slider = Slider(slider_theta, r"$\theta$", -pi, pi, valinit=theta)

    slider_psi = plt.axes([left_offset, 0.5, 0.12, height_slider])
    psi_slider = Slider(slider_psi, r"$\psi$", -pi, pi, valinit=psi)

    # Bouton Stop
    button_ax = plt.axes([left_offset, 0.15, 0.12, 0.05])
    stop_button = Button(button_ax, "Close Plot")

    # --- Annotations (temps, angles, position, distance) ---
    #angle_annot = ax.text2D(0.85, 0.4, "", transform=ax.transAxes, fontsize=10)
    #pos_annot = ax.text2D(0.85, 0.2, "", transform=ax.transAxes, fontsize=10)

    theta_circle = np.linspace(0, 2 * np.pi, 20)
    circle_points_local = np.array([
        r_rotors * np.cos(theta_circle),
        r_rotors * np.sin(theta_circle),
        np.zeros_like(theta_circle)
    ])

    # State variables
    stop_animation = False
    current_frame = 0
    draw_speed = 1
    dynamic_artists = []  # Stocke les éléments dynamiques à supprimer

    # --- Callback pour le bouton close plot ---
    def close_plot(event):
         nonlocal stop_animation
         stop_animation = True

    stop_button.on_clicked(close_plot)

    # --- Fonction de mise à jour de l'animation ---
    def update(frame):
        nonlocal current_frame, draw_speed, stop_animation, dynamic_artists

        if stop_animation:
            plt.close()
            return dynamic_artists


        # Suppression des éléments dynamiques précédents
        for artist in dynamic_artists:
            if artist:
                artist.remove()
        dynamic_artists = []

        # Update pos and angles from slider
        x = pos_x_slider.val
        y = pos_y_slider.val
        z = pos_z_slider.val
        phi = phi_slider.val
        theta = theta_slider.val
        psi = psi_slider.val

        dynamic_artists = add_plot_quadcopter(x,y,z,phi,theta,psi,ax,dynamic_artists,circle_points_local,L,showQuad)
        

    # --- Création de l'animation ---
    ani = FuncAnimation(
        fig,
        update,
        frames=None, # N
        interval=50,  # Délai entre les frames (ms)
        blit=False,
        repeat=True,      
        cache_frame_data=False  # Évite les fuites mémoire
    )

    # # --- Sauvegarde en GIF (si activé) ---
    # if save_gif:
    #     gif_frames = min(N, int(gif_duration / dt))
    #     writer = PillowWriter(fps=15)
    #     ani.save(gif_filename, writer=writer, dpi=100)

    plt.show()  # block="True"
    return ani


def add_plot_quadcopter(x,y,z,phi,theta,psi,ax,dynamic_artists,circle_points_local,L,showQuad):

    #L = 0.23 # lenth arm quadcopter 

    # --- Position du drone ---
    drone_pos, = ax.plot3D(x, y, z, "ro")
    dynamic_artists.append(drone_pos)

    # --- Rotation matrix ---
    R_B2I = rot_mat_B2I(np.array([[phi,theta,psi]]).T)

    # --- Référentiel du drone (one quiver for each axes) ---
    length_quad = 0.5
    line_width = 2.0 #1.5
    for vec, color, lw in zip(
        [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])],
        ["g", "b", "m"],
        [line_width]*3
    ):
        arrow = length_quad * R_B2I @ vec
        q = ax.quiver3D(x, y, z, *arrow, color=color, linestyle="--", linewidth=lw)  #  linestyle="--"
        dynamic_artists.append(q)

    if showQuad:
        
        # first quadcopter shape
        shape_rotors1 = [np.array([1, 0, 0]), np.array([-1, 0, 0]), np.array([0, 1, 0]), np.array([0, -1, 0])]
        color_rotors = ["r", "k", "r", "k"]
        for vec, color in zip(
            shape_rotors1,
            color_rotors
        ):
            arrow = L * R_B2I @ vec
            #q = ax.quiver3D(x, y, z, *arrow, color=color, linewidth=1)
            x_line = x + arrow[0]
            y_line = y + arrow[1]
            z_line = z + arrow[2]
            q, = ax.plot([x,x_line], [y,y_line], [z,z_line], color=color, linewidth=1.5)
            dynamic_artists.append(q)

            # rotors

            # 3D
            circle_points_global = (R_B2I @ circle_points_local) + np.array([[x_line,y_line,z_line]]).T
            circle, = ax.plot(
            circle_points_global[0, :],
            circle_points_global[1, :],
            circle_points_global[2, :],
            "k-", linewidth=1.5)

            dynamic_artists.append(circle)


    return dynamic_artists



def plot_difference_integration(t,error,name):
    
    

    #plt.plot(t,error[0,:], label="Error norm", linestyle="-")


    plt.figure(figsize=(10, 6)) # fig = plt.figure()

    plt.subplot(6,1,1)
    plt.plot(t,error[0,:])
    plt.grid(True)
    plt.legend([r"Error pos x"])
    title_plot = "Comparison Error " + name + " and RK4"
    plt.title(title_plot)  #Suivi en ref de position
    plt.ylabel('Error pos [m]')
    #plt.xlabel('Time [s]')

    plt.subplot(6,1,2)
    plt.plot(t,error[1,:])
    plt.grid(True)
    plt.legend([r"Error pos y"])
    plt.ylabel('Error pos [m]')

    plt.subplot(6,1,3)
    plt.plot(t,error[2,:])
    plt.grid(True)
    plt.legend([r"Error pos z"])
    plt.xlabel("Time (s)")
    plt.ylabel('Error pos [m]')

    plt.subplot(6,1,4)
    plt.plot(t,error[6,:])
    plt.grid(True)
    plt.legend([r"Error angle $\phi$"])
    #plt.title("Comparison Error")  #Suivi en ref de position
    plt.ylabel('Error angle [rad]')
    #plt.xlabel('Time [s]')

    plt.subplot(6,1,5)
    plt.plot(t,error[7,:])
    plt.grid(True)
    plt.legend([r"Error angle $\theta$"])
    plt.ylabel('Error angle [rad]')

    plt.subplot(6,1,6)
    plt.plot(t,error[8,:])
    plt.grid(True)
    plt.legend([r"Error angle $\psi$"])
    plt.xlabel("Time (s)")
    plt.ylabel('Error angle [rad]')

    
    plt.show(block = True)


    return 


def plot_traj_z(t,Xref,X,Xref_locplan):
    title_str = "Reference on $z$"
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(t,Xref[2,:],'-o',label=r"$z_{ref}$",markersize=3)
    if len(X):
        plt.plot(t,X[2,:],'-o',label=r"$z$",markersize=3)
        title_str ="Tracking the $z$ reference"
    if len(Xref_locplan):
        plt.plot(t,Xref_locplan[2,:],'-o',label=r"$z_{loc plan}$",markersize=3)
    plt.grid(True)
    plt.legend()
    plt.title(title_str)  #Suivi en ref de position
    plt.ylabel("Pos $z (m)$")

    plt.subplot(2,1,2)

    plt.plot(t,Xref[5,:],'-o',label=r"$v_{z,ref}$",markersize=3)
    if len(X):
        plt.plot(t,X[5,:],'-o',label=r"$v_{z}$",markersize=3)
    if len(Xref_locplan):
        plt.plot(t,Xref_locplan[5,:],'-o',label=r"$v_{z,loc plan}$",markersize=3)
    plt.grid(True)
    plt.legend()
    plt.ylabel("Velocity $z (m/s)$")

    '''plt.subplot(3,1,3)
    plt.plot(t,Xref[9,:],'-o',markersize=8)
    plt.grid(True)
    plt.legend(["$a_{z,ref}$"])
    plt.xlabel("Time (s)")
    plt.ylabel("Acc $z (m/s^2)$")'''

    return