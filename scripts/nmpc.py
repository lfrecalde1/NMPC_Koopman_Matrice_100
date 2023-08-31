import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from aerial_system_model_complete import export_uav_model
import scipy.linalg
def rbf(X, C, rbf_type, eps=None, k=None):
    X = X.reshape(1, 1)
    rbf_type = rbf_type.lower()
    
    if eps is None:
        eps = 1
    if k is None:
        k = 1
    
    Cbig = C
    Y = np.zeros((C.shape[1], X.shape[1]))
    
    for i in range(0, Cbig.shape[1]):
        C = Cbig[:, i].reshape(-1, 1)
        C = np.tile(C, (1, X.shape[1]))
        r_squared = np.sum((X - C) ** 2, axis=0)
        
        if rbf_type == 'thinplate':
            y = r_squared * np.log(np.sqrt(r_squared)) + 0.5 * np.sqrt(r_squared)
            y[np.isnan(y)] = 0
        elif rbf_type == 'gauss':
            y = np.exp(-eps ** 2 * r_squared)
        elif rbf_type == 'invquad':
            y = 1 / (1 + eps ** 2 * r_squared)
        elif rbf_type == 'invmultquad':
            y = 1 / np.sqrt(1 + eps ** 2 * r_squared)
        elif rbf_type == 'polyharmonic':
            y = r_squared ** (k / 2) * np.log(np.sqrt(r_squared))
            y[np.isnan(y)] = 0
        else:
            raise ValueError('RBF type not recognized')
        
        Y[i, :] = y
    Y = Y.reshape(Y.shape[0], )
    return Y


def lift_Fun_angular(x, cent_a):
    x_lift = []
    for k in x: x_lift.append(k)
    x_lift.append(np.sin(x[0])*np.tan(x[1])*x[4])
    x_lift.append(np.cos(x[0])*np.tan(x[1])*x[5])
    x_lift.append(np.cos(x[0])*x[4])
    x_lift.append(np.sin(x[0])*x[5])
    x_lift.append((np.sin(x[0])/np.cos(x[1]))*x[4])
    x_lift.append((np.cos(x[0])/np.cos(x[1]))*x[5])
    x_lift.append(x[3]*x[4])
    x_lift.append(x[3]*x[5])
    x_lift.append(x[4]*x[5])
    x_lift = np.array(x_lift)
    return x_lift

def lift_Fun_linear(x, cent_l, cent_lz):
    x_lift = []
    for k in x: x_lift.append(k)
    for k in rbf(x[0], cent_l, "gauss"): x_lift.append(k)
    for k in rbf(x[1], cent_l, "gauss"): x_lift.append(k)
    for k in rbf(x[2], cent_lz, "gauss"): x_lift.append(k)
    x_lift = np.array(x_lift)
    return x_lift

def lift_Fun(x, cent_a, cent_l, cent_lz):
    a_lift = lift_Fun_angular(x[3:9], cent_a)
    v_lift = lift_Fun_linear(x[0:3], cent_l, cent_lz)

    complete = np.hstack((v_lift, a_lift))
    return complete


def f_angular_system(x, u, f_system, C):
    x_lift_k = f_system(x, u)
    y_output = C@x_lift_k
    out = np.array(y_output[:,0]).reshape((9,))
    return out

def create_ocp_solver_description(x0, N_horizon, t_horizon, z_max, z_min, phi_max, phi_min, theta_max, theta_min, psi_p_max, psi_p_min) -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
    f_system, model = export_uav_model()

    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost
    Q_mat = np.zeros((nx, nx), dtype=np.double)
    Q_mat[0, 0] = 5
    Q_mat[1, 1] = 5
    Q_mat[2, 2] = 5

    Q_mat[9, 9] = 0.1
    Q_mat[10, 10] = 0.1
    Q_mat[11, 11] = 0.0

    Q_mat[12, 12] = 1
    Q_mat[13, 13] = 1
    Q_mat[14, 14] = 5
    R_mat = 2 * np.diag([(1/z_max), (10/phi_max), (10/theta_max), (1/psi_p_max)])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    ocp.constraints.lbu = np.array([z_min, phi_min, theta_min, psi_p_min])
    ocp.constraints.ubu = np.array([z_max, phi_max, theta_max, psi_p_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    ocp.constraints.x0 = x0

    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "EXACT"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = t_horizon

    return ocp