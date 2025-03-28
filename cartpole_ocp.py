"""
cartpole_ocp.py

一个示例：从 config.py 中读取 Q, R, P, Horizon, Ts 等，来搭建 OCP。
"""

import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import config  # 引用 config.py
import scipy.linalg

# 导入 CartPole 模型
from cartpole_model import export_cartpole_ode_model

def create_ocp_solver(x0):
    ocp = AcadosOcp()

    # 读取 config 里的各种参数
    Nx = config.Num_State
    Nu = config.Num_Input
    N  = config.Horizon
    tf = N * config.Ts   # 总时域 tf = N_horizon * Ts

    # 设置 OCP 参数
    ocp.solver_options.N_horizon = N  # 设置预测步数
    ocp.solver_options.tf = tf       # 设置总时域


    # 加载 CartPole 模型
    model = export_cartpole_ode_model()
    ocp.model = model
    ocp.model.x = model.x
    ocp.model.u = model.u

    # 成本函数设置
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)
    ocp.cost.W = scipy.linalg.block_diag(config.Q, config.R)

    ocp.cost.yref = np.zeros(Nx + Nu)  # (6维)

    # 终端成本
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.W_e = config.P

    ocp.cost.yref_e = np.zeros(Nx)     # (5维)

    # 约束条件
    ocp.constraints.x0 = x0
    ocp.constraints.lbu = np.array([-config.Fmax])
    ocp.constraints.ubu = np.array([+config.Fmax])
    ocp.constraints.idxbu = np.array([0])  # 控制量u只有1维

    # 求解器设置
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'IRK'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.nlp_solver_max_iter = 2000
    ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    ocp.solver_options.print_level = 0

    # 构造 OCP 求解器
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_cartpole.json")

    acados_integrator = AcadosSimSolver(ocp, json_file = "acados_ocp_cartpole.json")

    return ocp, acados_solver, acados_integrator

def simulate_closed_loop(x0, N_sim=50):
    ocp, ocp_solver, integrator = create_ocp_solver(x0)
   
    nx = ocp.model.x.size()[0]  # Should be 5
    nu = ocp.model.u.size()[0]  # Should be 1

    # 初始状态
    simX = np.zeros((N_sim+1, nx))
    simU = np.zeros((N_sim, nu))
    simX[0, :] = x0  # 初始化状态为传入的 x0

    # 闭环仿真
    for i in range(N_sim):
        u_guess, x_guess = config.GenerateRandomInitialGuess()
        x_ini_guess = config.generate_initial_guess(x_guess, u_guess)
        # 更新求解器的初始猜测
        ocp_solver.set(0, "u", u_guess)  # 设置控制输入初始猜测
        ocp_solver.set(0, "x", x_ini_guess)  # 设置状态初始猜测

        u_opt = ocp_solver.solve_for_x0(x0_bar = simX[i, :])

        simU[i,:] = u_opt

        # 更新状态
        x_next = integrator.simulate(x=simX[i,:], u=u_opt)
        simX[i+1,:] = x_next

    t = np.linspace(0, N_sim*config.Ts, N_sim+1)
    return t, simX, simU
