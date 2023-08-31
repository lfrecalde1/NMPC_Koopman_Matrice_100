from acados_template import AcadosModel
from casadi import MX, SX, vertcat, sin, cos
from casadi import Function
import numpy as np
import scipy.io
# Load Identification matrices
Identification = scipy.io.loadmat('matrices_complete.mat') 
A_aux = Identification['A'] 
B_aux = Identification['B'] 
G_aux = Identification['G'] 
nx = A_aux.shape[0]
nu = B_aux.shape[1]

def create_matrix(A, data):
    for i in range(0, data.shape[0]):
        for j in range(0, data.shape[1]):
            A[i, j] = data[i, j]
    return A
        

def Rot_zyx(x):
    phi = x[9, 0]
    theta = x[10, 0]
    psi = x[11, 0]

    # Rot Matrix axis X
    RotX = MX.zeros(3, 3)
    RotX[0, 0] = 1.0
    RotX[1, 1] = cos(phi)
    RotX[1, 2] = -sin(phi)
    RotX[2, 1] = sin(phi)
    RotX[2, 2] = cos(phi)

    # Rot Matrix axis Y
    RotY = MX.zeros(3, 3)
    RotY[0, 0] = cos(theta)
    RotY[0, 2] = sin(theta)
    RotY[1, 1] = 1.0
    RotY[2, 0] = -sin(theta)
    RotY[2, 2] = cos(theta)

    RotZ = MX.zeros(3, 3)
    RotZ[0, 0] = cos(psi)
    RotZ[0, 1] = -sin(psi)
    RotZ[1, 0] = sin(psi)
    RotZ[1, 1] = cos(psi)
    RotZ[2, 2] = 1.0

    R = RotZ@RotY@RotX
    return R
def export_uav_model():

    model_name = 'angular_drone'

    # Model MatriceS
    A_a = MX.zeros(nx, nx)
    A_a = create_matrix(A_a, A_aux)

    B_a = MX.zeros(nx, nu)
    B_a = create_matrix(B_a, B_aux)

    G_a = MX.zeros(nx, 1)
    G_a = create_matrix(G_a, G_aux)

    ## Definition Symbolic Variables states

    x = MX.sym('x', nx, 1)

    R = Rot_zyx(x)

    R_t = MX.zeros(6, 6)
    R_t[0, 0] = R[0, 0]
    R_t[0, 1] = R[0, 1]
    R_t[0, 2] = R[0, 2]

    R_t[1, 0] = R[1, 0]
    R_t[1, 1] = R[1, 1]
    R_t[1, 2] = R[1, 2]

    R_t[2, 0] = R[2, 0]
    R_t[2, 1] = R[2, 1]
    R_t[2, 2] = R[2, 2]

    R_t[3, 3] = 1
    R_t[4, 4] = 1
    R_t[5, 5] = 1

    ## Definition of control variables

    u = MX.sym('u', 4, 1)
    u_aux = vertcat(0, 0, u[0, 0], u[1, 0], u[2, 0], u[3, 0])


    ## System Evolution
    x_k = A_a@x + B_a@R_t@u_aux + G_a
    # populate structure
    model = AcadosModel()
    model.x = x
    model.u = u
    model.disc_dyn_expr = x_k
    model.name = model_name
    f_system = Function('system',[x, u], [x_k])
    return f_system, model