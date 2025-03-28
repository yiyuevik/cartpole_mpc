"""
cartpole_closed_loop.py
我没有写main.py
此即为主入口脚本：读取/设置模型参数(在 config.py)，构造并求解 OCP，然后进行闭环仿真 + 可视化。
运行方式: python cartpole_closed_loop.py
"""


import config
from cartpole_ocp import create_ocp_solver, simulate_closed_loop  
from cartpole_utils import plot_cartpole_trajectories, animate_cartpole

def main():

    # 2) 生成初始状态样本
    X0_samples = config.GenerateX0Samples()
    print(f"共生成 {len(X0_samples)} 个 x0 样本。")

    x0 = X0_samples[0]

    # 4) 闭环仿真
    N_sim = 50  # 修改为50次模拟
    t, simX, simU = simulate_closed_loop(x0, N_sim=N_sim)
    
    # 5) 绘制曲线
    plot_cartpole_trajectories(t, simX, simU)

    # 6) 动画
    animate_cartpole(t, simX, L=1.0, interval=50)
if __name__ == "__main__":
    main()