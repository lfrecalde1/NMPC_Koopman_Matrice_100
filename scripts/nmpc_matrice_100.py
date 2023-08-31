
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from aerial_system_model_complete import export_uav_model
import scipy.linalg
import numpy as np
import time
import matplotlib.pyplot as plt
from casadi import Function
from casadi import MX
from fancy_plots import fancy_plots_2, fancy_plots_1, fancy_plots_3, plot_states, plot_states_velocity, plot_states_reference, plot_control, plot_states_reference_angular
from fancy_plots  import fancy_plots_4, plot_control_full
import scipy.io
from nmpc import  rbf, lift_Fun_angular, lift_Fun_linear, lift_Fun, f_angular_system, create_ocp_solver_description
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from scipy.spatial.transform import Rotation as R
#from c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

## Load angular parameters
Identification = scipy.io.loadmat('matrices_complete.mat') 
cent_a = Identification['cent_a']
cent_l = Identification['cent_l']
cent_lz = Identification['cent_lz']
C = Identification['C']
A_aux = Identification['A'] 
B_aux = Identification['B'] 

# Global variables Odometry Drone
x_real = 0.0
y_real = 0.0
z_real = 0.0
vx_real = 0.0
vy_real = 0.0
vz_real = 0.0

# Angular velocities
qx_real = 0.0005
qy_real = 0.0
qz_real = 0.0
qw_real = 1.0
wx_real = 0.0
wy_real = 0.0
wz_real = 0.0

# Auxiliar variables
t_s = 0.02
mass = 3.80
gravity = 9.81
def init_system(control_pub, ref_drone, velocity_z):
    # Function to Initialize the system
    vx_c = np.array([[0.0], [0.0], [0.0]])
    vy_c = np.array([[0.0], [0.0], [0.0]])
    vz_c = np.array([[0.0], [0.0], [0.0]])
    for k in range(0, 250):
        tic = time.time()
        # Get system velocites Respect to W frame
        velocities = get_system_velocity_sensor()

        # PID control associated to the desired velocities
        u_global, vz_c = controller_z(mass, gravity, [0, 0, velocity_z], velocities[0:3], vz_c)
        u_ref_pitch, vx_c = controller_attitude_pitch([0, 0, velocity_z], velocities[0:3], t_s, vx_c)
        u_ref_roll, vy_c = controller_attitude_roll([0, 0, velocity_z], velocities[0:3], t_s, vy_c)
        ref_drone = get_reference([u_global, u_ref_roll, u_ref_pitch, 0], ref_drone)

        # Send control action to the aerial robotic system
        send_reference(ref_drone, control_pub)

        # Loop_rate.sleep()
        while (time.time() - tic <= t_s):
                None
        toc = time.time() - tic 
    
    return None

def pid(sp, real, memories, kp, ki, kd, t_sample):
    error = np.tanh(sp - real)
    error_1 = memories[1, 0]
    error_2 = memories[2, 0]
    u_1 = memories[0, 0]
    p = kp * (error - error_1)
    i = ki * error * t_sample
    d = kd * (error - 2 * error_1 + error_2) / t_sample
    u = u_1 + p + i + d

    # Update memories
    memories[0, 0] = u
    memories[2, 0] = error_1
    memories[1, 0] = error
    return u, memories

def controller_attitude_pitch(qdp, qp, ts, v_c):
    # Rotational matrix
    quat = np.array([qx_real, qy_real, qz_real, qw_real], dtype=np.double)
    rot = R.from_quat(quat)
    euler = rot.as_euler('xyz', degrees=False)

    # Euler angles
    psi = euler[2]

    # Create rotational matrix z
    R_z = R.from_euler('z', psi, degrees=False)
    R_z_data = R_z.as_matrix()

    # Create vector linear velocity world frame
    qp = np.array([[qp[0]], [qp[1]], [qp[2]]], dtype=np.double)

    # Velocity with respect to the body frame only rotation z
    velocity = np.linalg.inv(R_z_data)@qp
    
    # Velocity with respect to the body frame complete rotation matrix
    rot_matrix = rot.as_matrix()
    rot_inv = np.linalg.inv(rot_matrix)
    v_body = rot_inv@qp

    # Desired velocity and real velocity proyection
    xpd = qdp[0]
    #xp = np.cos(theta)*v_body[0]
    xp = velocity[0]

    # PID controller for lateral velocity y
    pitch_d, v_c = pid(xpd, xp, v_c, 0.15, 0.0, 0.001, ts)
    return pitch_d, v_c

def controller_attitude_roll(qdp, qp, ts, v_c):
    # Rotational matrix
    quat = np.array([qx_real, qy_real, qz_real, qw_real], dtype=np.double)
    rot = R.from_quat(quat)
    euler = rot.as_euler('xyz', degrees=False)

    # Euler Angles
    psi = euler[2]

    # Create rotational matrix z
    R_z = R.from_euler('z', psi, degrees=False)
    R_z_data = R_z.as_matrix()
    
    # Create vector of the velocities respect to the frame W
    qp = np.array([[qp[0]], [qp[1]], [qp[2]]], dtype=np.double)

    # Linear velocity projection over B frame using on Z rotation
    velocity = np.linalg.inv(R_z_data)@qp

    # Linear velocity projection over B frame using complete rotation matrix
    rot_matrix = rot.as_matrix()
    rot_inv = np.linalg.inv(rot_matrix)
    v_body = rot_inv@qp

    # Desired Velocity and velocty body proyection
    ypd = qdp[1]
    #yp = np.cos(theta)*v_body[1]
    yp = velocity[1]

    roll_d, v_c = pid(ypd, yp, v_c, 0.15, 0.0, 0.001, ts)
    return -roll_d, v_c

def controller_z(mass, gravity, qdp, qp, v_c):
    # Control Function only z velocity
    # Control Gains
    Kp = 15*np.eye(3, 3)

    # Control error
    error = qdp - qp
    error_vector = error.reshape((3,1))

    # Split values
    zpd = qdp[2]
    zp = qp[2]

    # Control Law
    #aux_control = Kp@error_vector
    aux_control, v_c = pid(zpd, zp, v_c, 15, 0.05, 0.001, t_s)

    # Gravity + compensation velocity
    control_value = mass*gravity + aux_control
    
    return control_value, v_c


## Reference system
def get_reference(ref, ref_msg):
        ref_msg.twist.linear.x = 0
        ref_msg.twist.linear.y = 0
        ref_msg.twist.linear.z = ref[0]

        ref_msg.twist.angular.x = ref[1]
        ref_msg.twist.angular.y = ref[2]
        ref_msg.twist.angular.z = ref[3]
        return ref_msg

def send_reference(ref_msg, ref_pu):
    ref_pu.publish(ref_msg)
    return None

def odometry_call_back(odom_msg):
    global x_real, y_real, z_real, qx_real, qy_real, qz_real, qw_real, vx_real, vy_real, vz_real, wx_real, wy_real, wz_real
    # Read desired linear velocities from node
    time_message = odom_msg.header.stamp
    x_real = odom_msg.pose.pose.position.x 
    y_real = odom_msg.pose.pose.position.y
    z_real = odom_msg.pose.pose.position.z
    vx_real = odom_msg.twist.twist.linear.x
    vy_real = odom_msg.twist.twist.linear.y
    vz_real = odom_msg.twist.twist.linear.z


    qx_real = odom_msg.pose.pose.orientation.x
    qy_real = odom_msg.pose.pose.orientation.y
    qz_real = odom_msg.pose.pose.orientation.z
    qw_real = odom_msg.pose.pose.orientation.w

    wx_real = odom_msg.twist.twist.angular.x
    wy_real = odom_msg.twist.twist.angular.y
    wz_real = odom_msg.twist.twist.angular.z
    return None

def get_system_states_sensor():
    # Thhis Funtion get the system pose including the euler angles
    quat = np.array([qx_real, qy_real, qz_real, qw_real], dtype=np.double)
    rot = R.from_quat(quat)
    euler = rot.as_euler('xyz', degrees=False)
    x = np.array([x_real, y_real, z_real, qw_real, qx_real, qy_real, qz_real, euler[0], euler[1], euler[2]], dtype=np.double)
    return x

# Get system velocities
def get_system_velocity_sensor():
    # Funntion to geet the   ssystem velocities respect to w frame
    x = np.array([vx_real, vy_real, vz_real, wx_real, wy_real, wz_real], dtype=np.double)
    return x

def main(control_pub):
    # Initial Values System
    # Read Matlab Data
    # Simulation time parameter
    global t_s
    tf = 10
    t = np.arange(0, tf+t_s, t_s, dtype=np.double)

    # Prediction Time
    t_prediction= 1;

    # Nodes inside MPC
    N = np.arange(0, t_prediction + t_s, t_s)
    N_prediction = N.shape[0]


    # Sample time vector
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = t_s*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Vector Initial conditions
    x = np.zeros((9, t.shape[0]+1 - N_prediction), dtype = np.double)

    # Initial Control values
    u_control = np.zeros((4, t.shape[0] - N_prediction), dtype = np.double)
    ref_drone = TwistStamped()


    # Desired Values
    xref = np.zeros((9, t.shape[0]), dtype = np.double)
    # Linear Velocities
    xref[0,:] = 2*np.cos(0.4*t)*np.sin(0.5*t)+0.3*np.cos(0.7*t)*np.cos(0.3*t)
    xref[1,:] =  2.5*np.sin(0.4*t)*np.sin(0.5*t)+0*np.cos(0.7*t)*np.cos(0.3*t)
    xref[2,:] =  2 + 1.5*np.cos(0.4*t)*np.sin(0.5*t)+0.3*np.cos(0.7*t)*np.cos(0.3*t)
    # Angular Velocities
    xref[3,:] = 0.0
    xref[4,:] =  0.0
    xref[5,:] = 0.0
    xref[6,:] = 0.0
    xref[7,:] =  0.0
    xref[8,:] = 0.2*np.cos(0.4*t)*np.sin(0.5*t)+0.2*np.cos(0.7*t)*np.cos(0.3*t)

    # Xlift space desired values
    xref_lift = np.zeros((28, t.shape[0]), dtype = np.double)
    for k in range(0, xref.shape[1]):
        xref_lift[0:24, k] = lift_Fun(xref[:, k], cent_a, cent_l, cent_lz)

    ## Complete states of the system
    h = np.zeros((10, t.shape[0]+1 - N_prediction), dtype = np.double)
    hp = np.zeros((6, t.shape[0]+1 - N_prediction), dtype = np.double)


    init_system(control_pub, ref_drone, velocity_z=2.0)
    init_system(control_pub, ref_drone, velocity_z=0.0)
    # Set initial Conditions
    h[:, 0] = get_system_states_sensor()
    hp[:, 0] = get_system_velocity_sensor()

    # Definition of euler angles

    # Get Euler dot throughout

    ## Initial Condition
    x[0:3, 0] = hp[0:3, 0]
    x[3:6, 0] = h[7:10, 0]
    x[6:9, 0] = hp[3:6, 0]

    # Create Model of the system
    f_complete, model = export_uav_model()

    # Initial Condition system
    x_lift = lift_Fun(x[:, 0], cent_a, cent_l, cent_lz)

    # Limits Control values
    z_max = 100
    phi_max = 0.13
    theta_max = 0.13
    psi_p_max = 1.0

    phi_min = -phi_max
    theta_min = -theta_max
    psi_p_min = -psi_p_max
    z_min = 30

    ### Optimization problem definition
    ocp = create_ocp_solver_description(x_lift, N_prediction, t_prediction, z_max, z_min, phi_max, phi_min, theta_max, theta_min, psi_p_max, psi_p_min)
    ## Optimization Problem

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)


    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    ## Initial States Acados
    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", 0.0 * np.ones(x_lift.shape))
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))

    ## Simulation System
    t_k = 0
    aux_falla = 0
    for k in range(0, t.shape[0]- N_prediction):

        # Get Computational Time
        tic = time.time()
        x_lift = lift_Fun(x[:, k], cent_a, cent_l, cent_lz)
        acados_ocp_solver.set(0, "lbx", x_lift)
        acados_ocp_solver.set(0, "ubx", x_lift)

        # Update yref
        for j in range(N_prediction):
            yref = xref_lift[:,k+j]
            acados_ocp_solver.set(j, "yref", yref)
        yref_N = xref_lift[:,k+N_prediction]
        acados_ocp_solver.set(N_prediction, "yref", yref_N[0:24])
        # Get Computational Time
        status = acados_ocp_solver.solve()

        # Check any failure in the system
        if (status != 0) and (k!=0):
            aux_falla = aux_falla + 1
            u_control[:, k] = u_control[:, k -1]
            if aux_falla >7:
                init_system(velocity_publisher, ref_drone, velocity_z = 0.0)
                print("Falla del sistema")
                break
            else:
                None
        else:
            aux_falla = 0
            u_control[:, k] = acados_ocp_solver.get(0, "u")

        # Get Control Signal
        ref_drone = get_reference(u_control[:, k], ref_drone)
        send_reference(ref_drone, control_pub)

        # System Evolution
        #x[:, k+1] = f_angular_system(x_lift, u_control[:, k], f_complete, C)
        # Loop_rate.sleep()
        while (time.time() - tic <= t_s):
                None
        toc_solver = time.time()- tic
        # Save Data
        h[:, k+1] = get_system_states_sensor()
        hp[:, k+1] = get_system_velocity_sensor()
        # Definition of euler angles

        ## Initial Condition
        x[0:3, k+1] = hp[0:3, k+1]
        x[3:6, k+1] = h[7:10, k+1]
        x[6:9, k+1] = hp[3:6, k+1]
        t_k = t_k + toc_solver
        # Set zero Values
        delta_t[:, k] = toc_solver
        print(toc_solver)

    init_system(control_pub, ref_drone, velocity_z = 0.0)

    # System Figures
    plt.imshow(A_aux)
    plt.colorbar()
    #plt.show()

    
    plt.imshow(B_aux)
    plt.colorbar()
    #plt.show()

    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_states_reference(fig13, ax13, ax23, ax33, x[0:3,:], xref[0:3,:], t, "Reference_linear")
    
    fig14, ax14, ax24, ax34 = fancy_plots_3()
    plot_states_reference_angular(fig14, ax14, ax24, ax34, x[6:9,:], xref[6:9,:], t, "Reference_angular")

    fig15, ax15, ax25, ax35, ax45= fancy_plots_4()
    plot_control_full(fig15, ax15, ax25, ax35, ax45, u_control, t, "Control_actions")

    ### Time Plot
    fig3, ax13 = fancy_plots_1()
    #### Axis definition necesary to fancy plots
    ax13.set_xlim((t[0], t[-1]))

    time_1, = ax13.plot(t[0:delta_t.shape[1]],delta_t[0,:],
                    color='#00429d', lw=2, ls="-")
    tsam1, = ax13.plot(t[0:t_sample.shape[1]],t_sample[0,:],
                    color='#9e4941', lw=2, ls="-.")

    ax13.set_ylabel(r"$[s]$", rotation='vertical')
    ax13.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)
    ax13.legend([time_1,tsam1],
            [r'$t_{compute}$',r'$t_{sample}$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
    ax13.grid(color='#949494', linestyle='-.', linewidth=0.5)

    fig3.savefig("time.eps")
    fig3.savefig("time.png")


    ## Systems Results
    print(f'Mean iteration time with MLP Model: {1000*np.mean(delta_t):.1f}ms -- {1/np.mean(delta_t):.0f}Hz)')

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("NMPC_controller",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic = "/dji_sdk/odometry"
        odometry_subscriber = rospy.Subscriber(odomety_topic, Odometry, odometry_call_back)

        # Subscribe Info
        velocity_topic = "/m100/velocityControl"
        velocity_publisher = rospy.Publisher(velocity_topic, TwistStamped, queue_size = 10)

        # Main System
        main(velocity_publisher)

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        ref_drone = TwistStamped()
        init_system(velocity_publisher, ref_drone, velocity_z = 0.0)
        pass
    else:
        print("Complete Execution")
        ref_drone = TwistStamped()
        init_system(velocity_publisher, ref_drone, velocity_z = 0.0)
        pass