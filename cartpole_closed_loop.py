"""
cartpole_closed_loop.py
我没有写main.py
此即为主入口脚本：读取/设置模型参数(在 config.py)，构造并求解 OCP，然后进行闭环仿真 + 可视化。
运行方式: python cartpole_closed_loop.py
"""


import config
from cartpole_ocp import create_ocp_solver, simulate_closed_loop  
from cartpole_utils import plot_cartpole_trajectories, animate_cartpole
import time
import matplotlib.pyplot as plt
import numpy as np

def main():

    # 2) 生成初始状态样本
    X0_samples = config.GenerateX0Samples()
    print(f"共生成 {len(X0_samples)} 个 x0 样本。")

    x0_theta = []
    x0 = X0_samples[0]
    for i in range(7, 150, 15):
        x0_theta.append(X0_samples[i])
    # 4) 闭环仿真
    N_sim = 50  # 修改为50次模拟
    all_simX = np.zeros((51,5,10))
    all_simU = np.zeros((50,1,10))
    i = 0
    for initial_state in x0_theta:
        starttime = time.time()
        t, simX, simU = simulate_closed_loop(initial_state, N_sim=N_sim)
        endtime = time.time()
        all_simX[:,:,i] = simX
        all_simU[:,:,i] = simU
        i = i+1
        elapsed_time = endtime - starttime
        print(f"Simulation for initial state {initial_state} took {elapsed_time:.4f} seconds.")


        # 5) 动画
        # animate_cartpole(t, simX, L=1.0, interval=50)

        # 6) 绘制曲线
        # plot_cartpole_trajectories(t, simX, simU)

    theta_values = all_simX[:, 2, :]  # shape: (51, 10)，提取所有步骤的theta，10个样本

    # 计算每个步骤的最大值、最小值和中位数
    theta_max = np.max(theta_values, axis=1)  # shape: (51,)
    theta_min = np.min(theta_values, axis=1)  # shape: (51,)
    theta_median = np.median(theta_values, axis=1)  # shape: (51,)

    # 绘制 theta 的范围图
    plt.figure(figsize=(12, 6))

    # 填充最大值和最小值之间的区域，表示范围
    plt.fill_between(range(51), theta_min, theta_max, color='lightblue', label='Range', alpha=0.5)

    # 绘制中位数线
    plt.plot(range(51), theta_median, color='blue', label='Median', linewidth=2)

    # 设置图形标题和标签
    plt.title('Theta (State 3) Range and Median Over Time')
    plt.xlabel('Step')
    plt.ylabel('Theta Value')

    # 显示图例
    plt.legend()

    # 显示图形
    plt.show()   
    # steps = 50
    # num_x0 = 10  # 10个x0

    # # 1. 将数据重新整理为适合绘制Boxplot的格式
    # data_for_boxplot = []

    # for step in range(steps):
    #     # 提取每个步长对应的10个控制输入值
    #     data_for_boxplot.append(all_simU[step, 0, :])

    # # 将数据转化为numpy数组，以便 matplotlib 进行绘制
    # data_for_boxplot = np.array(data_for_boxplot).T  # 转置，确保每一列为一个控制输入的样本

    # # 2. 使用matplotlib绘制Boxplot
    # plt.figure(figsize=(12, 6))

    # # 使用matplotlib的boxplot绘制
    # plt.boxplot(data_for_boxplot, widths=0.6)

    # # 3. 设置标签和标题
    # plt.title('Control Input in Dataset')
    # plt.xlabel('Step')
    # plt.ylabel('Control Input (ctrl)')

    # # 设置x轴标签
    # xticks = [i*10 + 5 for i in range(steps // 10)]  # 每10个步长在x轴上显示一个标签
    # plt.xticks(xticks, [str(i) for i in range(0, steps, 10)])

    # # 显示图形
    # plt.show()
        
if __name__ == "__main__":
    main()