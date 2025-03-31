"""
config.py

集中管理 MPC 相关设置（Q, R, P, Horizon, Ts 等）
以及初始状态采样与随机初始猜测生成函数等。
"""

import numpy as np

###############################################################################
# 1) 基本 MPC 参数
###############################################################################
Horizon = 64          # 预测步数
Ts = 0.01             # 采样时间
Num_State = 5         # 状态维数(如 cartpole + 虚拟态) 
Num_Input = 1         # 控制量维数
Fmax = 6000

# Q 矩阵
Q = np.diag([0.01,    # x 
             0.01,    # xdot
             0.0,     # theta
             0.01,    # thetadot
             1000.0]) # theta_stat

# R 标量
R = 0.001

# P 矩阵 (终端加权)
P = np.array([
    [0.01, 0,    0,    0,    0   ],
    [0,    0.1,  0,    0,    0   ],
    [0,    0,    0.0,  0,    0   ],
    [0,    0,    0,    0.1,  0   ],
    [0,    0,    0,    0,    1000]
], dtype=float)

###############################################################################
# 2) 初始状态采样设定
###############################################################################
def GenerateX0Samples():
    """
    生成初始状态 x0，pos 在 [-3, 3] 区间取 10 个样本，theta 在 [1.8, 4.4] 取 15 个样本。
    """
    pos_candidates   = np.linspace(-3.0, 3.0, 10)
    theta_candidates = np.linspace(1.8,  4.4, 15)

    X0_list = []
    for p in pos_candidates:
        for th in theta_candidates:
            x0 = np.array([
                p,       # pos
                0.0,     # xdot
                th,      # theta
                0.0,     # thetadot
                - (1 / np.pi) * (th - np.pi) ** 2 + np.pi      # theta_stat
            ])
            X0_list.append(x0)
    return X0_list

###############################################################################
# 3) 初始猜测生成
###############################################################################
def GenerateRandomInitialGuess(min_random=-6000.0, max_random=6000.0):
    """
    生成一个随机的 (u_ini_guess, x_ini_guess)
    其中 u_ini_guess 在 [min_random, max_random] 里均匀随机取,范围我不清楚，问！
    """
    u_ini_guess = np.random.uniform(min_random, max_random, 1)[0]
    if u_ini_guess >= 0:
        x_ini_guess =  np.full(5, 5)
    else:
        x_ini_guess = np.full(5, -5)
    return u_ini_guess, x_ini_guess

# def GenerateRandomInitialGuess(min_random=-6000.0, max_random=6000.0):
#     """
#     生成一个随机的 (u_ini_guess, x_ini_guess)
#     其中 u_ini_guess 在 [min_random, max_random] 里均匀随机取,范围我不清楚，问！
#     """
#     u_ini_guess = np.random.uniform(min_random, max_random, 1)[0]
#     if u_ini_guess >= 0:
#         x_ini_guess =  np.zeros(5)
#         x_ini_guess[0] = 10
#     else:
#         x_ini_guess = np.zeros(5)
#         x_ini_guess[0] = -10
#         x_ini_guess[0] = 2*np.pi
#     return u_ini_guess, x_ini_guess
