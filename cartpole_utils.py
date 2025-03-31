"""
cartpole_utils.py

我先放了一些辅助函数：如绘制状态/控制量曲线，以及简单动画等
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def plot_cartpole_trajectories(t, simX, simU=None):
    fig, axs = plt.subplots(6, 1, figsize=(6,8), sharex=True)

    labels = ['x', 'xdot', 'theta', 'theta_dot', 'theta_stat']
    print("最后的theta：",simX[-1,2])
    for i in range(5):
        axs[i].plot(t, simX[:, i], label=labels[i])
        axs[i].grid(True)
        axs[i].legend()

    if simU is not None:
        axs[5].plot(t[:-1], simU[:,0], label='Force')
        axs[5].grid(True)
        axs[5].legend()

    axs[-1].set_xlabel("time (s)")
    plt.suptitle("CartPole closed-loop trajectories")

    plt.tight_layout()
    # plt.show()

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def animate_cartpole(t, X, L=1.0, interval=50):
    """
    Animates the CartPole system.

    Args:
    - t: Time vector.
    - X: State matrix (each row is a state at a time step, with columns for x, x_dot, theta, and theta_dot).
    - L: Length of the pendulum (default is 1.0).
    - interval: Time interval between frames in milliseconds (default is 50).
    """
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.set_xlim([-10, 10])  # x-axis limits for cart movement
    ax.set_ylim([-1, 2])  # y-axis limits for cart and pendulum movement
    ax.set_aspect('equal')
    ax.set_title('CartPole Animation')

    # Cart dimensions
    cart_width = 0.3
    cart_height = 0.2

    # Create cart (black rectangle)
    cart = plt.Rectangle((0, 0), cart_width, cart_height, color='black', animated=True)
    ax.add_patch(cart)

    # Create line for the pole
    line, = ax.plot([], [], lw=2, color='blue')

    # Update function for animation
    def update(frame):
        # Get the current state from X
        x_cart = X[frame, 0]  # x position of the cart
        theta = X[frame, 2]  # theta

        # Update the cart position
        cart.set_xy((x_cart - cart_width/2, 0))

        # Calculate the position of the pole
        pole_top_x = x_cart
        pole_top_y = cart_height
        x_end = pole_top_x - L * np.sin(theta)  
        y_end = pole_top_y + L * np.cos(theta) 

        # Update the ple line
        line.set_data([pole_top_x, x_end], [pole_top_y, y_end])

        return cart, line

    # Create the animation
    ani = FuncAnimation(fig, update, frames=len(t), blit=True, interval=interval)
    plt.show()

