"""
cartpole_model.py

定义 CartPole + 虚拟状态 (theta_stat) 的动力学模型，
供 ACADOS OCP 使用。
"""

import numpy as np
import casadi as ca
from acados_template import AcadosModel

def export_cartpole_ode_model():
    """
    状态: x = [ pos, xdot, theta, theta_dot, theta_stat ]
    控制: u = [ F ]

    动力学方程参考: 老师给的 dynamic_update_virtual_Casadi
    """
    # 常量定义（与老师给的类似）
    M_CART = 2.0
    M_POLE = 1.0
    L_POLE = 1.0
    G = 9.81
    MPLP = M_POLE * L_POLE      # m*l
    MPG  = M_POLE * G          # m*g
    M_TOTAL = M_CART + M_POLE
    MTG = M_TOTAL * G
    MTLP = M_TOTAL * G
    # MTLP = M_TOTAL * L_POLE
    PI_UNDER_2 = 2.0 / np.pi

    # CasADi 符号
    x_sym = ca.SX.sym('x', 5)  # [pos, vel, theta, theta_dot, theta_stat]
    u_sym = ca.SX.sym('u', 1)  # [F]

    # 便于命名
    x_pos      = x_sym[0]
    x_vel      = x_sym[1]
    theta      = x_sym[2]
    theta_vel  = x_sym[3]
    theta_stat = x_sym[4]
    F = u_sym[0]

    # 动力学
    #  xdot(0) = x_vel
    xdot_pos = x_vel
    #  xdot(1) = ...
    xdot_vel = (
        MPLP * (-ca.sin(theta)) * (theta_vel**2)
        + MPG  * ca.sin(theta)*ca.cos(theta)
        + F
    ) / (M_TOTAL - M_POLE*ca.cos(theta))**2

    #  xdot(2) = theta_vel
    xdot_theta = theta_vel

    #  xdot(3) = ...
    xdot_theta_vel = (
       -MPLP * ca.sin(theta)*ca.cos(theta)*(theta_vel**2)
       - MTG * ca.sin(theta)
       - ca.cos(theta)*F
    ) / (MTLP - MPLP*(ca.cos(theta)**2))

    #  xdot(4) = - (2/pi)*(theta - pi)*theta_vel
    xdot_theta_stat = - PI_UNDER_2 * (theta - np.pi) * theta_vel

    # 显式形式
    f_expl = ca.vertcat(
        xdot_pos,
        xdot_vel,
        xdot_theta,
        xdot_theta_vel,
        xdot_theta_stat
    )

    # 隐式形式: xdot - f_expl = 0
    xdot_sym = ca.SX.sym('xdot', 5)
    f_impl = xdot_sym - f_expl

    # 封装到 AcadosModel
    model = AcadosModel()
    model.name = "cartpole_5states"
    model.x    = x_sym
    model.xdot = xdot_sym
    model.u    = u_sym
    model.p    = []
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl

    return model
