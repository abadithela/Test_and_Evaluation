# plot_trajectory.m
# by Apurva Badithela

#  Functions that plots the trajectory of the system (robot car) and an
#  adversarial environment component (patrol car) in response to external
#  signals: park and move_patrol
#  Inputs: Each element of cell array SIG contains the trace of signal or -1
#  if the signal is not used in the simulation. SIG = {park_trace,
#  move_patrol_trace, ...}
#  Interpolating between frames to show path of the car

import numpy as np
import scipy as sp
import networkx as nx
import scipy.linalg as la
import matplotlib.pyplot as plt
import math
import matplotlib.animation as animation

def plot_trajectory(SYS_TRAJ, ENV_TRAJ, FUEL_TRAJ):
    # Plotting tools
    alw = 0.75    # AxesLineWidth
    fsz = 12      # Fontsize
    lw = 2        # LineWidth
    msz = 6       # MarkerSize
    # Gridworld size: M-by-N gridworld
    M = 4
    N = 4
    # Plotting car trajectory
    fuel = FUEL_TRAJ.copy()
    Cr = np.zeros(len(ENV_TRAJ))
    for ii in range(0,len(ENV_TRAJ)):
        if(ENV_TRAJ[ii] <= 2):
            Cr[ii] = ENV_TRAJ[ii] + 1
        elif(ENV_TRAJ[ii] <= 4):
            Cr[ii] = ENV_TRAJ[ii] + 3
        elif(ENV_TRAJ[ii] <= 6):
            Cr[ii] = ENV_TRAJ[ii] + 5
        else:
            Cr[ii] = ENV_TRAJ[ii] + 7
    
    Xr = SYS_TRAJ
    
    fig, ax = base_plot()
    points_sys, = ax.plot([], [], marker='o', color='blue', markersize=2*msz, markerfacecolor='blue')
    points_env, = ax.plot([], [], marker='o', color='red',  markersize=2*msz, markerfacecolor='red')
    fuel_text = ax.text(0.8, 0.9, '', transform=ax.transAxes)
    len_TRAJ = len(SYS_TRAJ)    
        
    jmax = 10
    
    MAX_FRAMES = jmax*len_TRAJ
    
    def animate(frame_idx):
        ii = frame_idx//jmax
        jj = frame_idx%jmax
        
#        robot_x = Xr[ii]%4
#        if robot_x == 0:
#            robot_x = 4
#        
#        patrol_x = Cr[ii]%4
#        if patrol_x == 0:
#            patrol_x = 4
#        
#        robot_y = 4 - math.floor((Xr[ii]-1)/4)
#        patrol_y = 4 - math.floor((Cr[ii]-1)/4)
        
        robot_x, robot_y, patrol_x, patrol_y = grid_position(Xr[ii], Cr[ii])
        # In the first iteration, the old_robot_pos is the same as
        # curr_robot_pos
        if ii == 0: 
            old_robot_x = robot_x
            old_robot_y = robot_y
            old_patrol_x = patrol_x
            old_patrol_y = patrol_y
        else:
            old_robot_x, old_robot_y, old_patrol_x, old_patrol_y = grid_position(Xr[ii-1], Cr[ii-1])
#            old_robot_x = Xr[ii-1]%4
#            if old_robot_x == 0:
#                old_robot_x = 4
#            old_robot_y = 4 - math.floor((Xr[ii-1]-1)/4)
#            old_patrol_x = Cr[ii-1]%4
#            if old_patrol_x == 0:
#                old_patrol_x = 4
#            old_patrol_y = 4 - math.floor((Cr[ii-1]-1)/4)
            
        int_robot_x = np.linspace(old_robot_x, robot_x, jmax)
        int_robot_y = np.linspace(old_robot_y, robot_y, jmax)
        int_patrol_x = np.linspace(old_patrol_x, patrol_x, jmax)
        int_patrol_y = np.linspace(old_patrol_y, patrol_y, jmax)
        
        points_sys.set_data(int_robot_x[jj],int_robot_y[jj])
        points_env.set_data(int_patrol_x[jj],int_patrol_y[jj])
        fuel_text.set_text("Fuel: "+str(fuel[ii]))
        return [points_sys, points_env, fuel_text]
    
    # Takes in raw grid cell location numbers for system and environment and returns location coordinates for system and environment
    def grid_position(sys_raw, env_raw):
        sys_x = sys_raw%N
        if sys_x == 0:
            sys_x = N
        
        env_x = env_raw%N
        if env_x == 0:
            env_x = N
        
        sys_y = M - math.floor((sys_raw-1)/M)
        env_y = M - math.floor((env_raw-1)/M)
        return sys_x, sys_y, env_x, env_y
    
    ani = animation.FuncAnimation(fig, animate, frames=MAX_FRAMES, interval = 100, blit=True)
    ani.save("gridworld_example.avi")
    plt.show()
    return ani


def base_plot():
    msz = 6       # MarkerSize
    goal_cell_x = 1
    goal_cell_y = 4
    home_cell_x = 4
    home_cell_y = 1
    refuel_cell_x = 4
    refuel_cell_y = 3

    grid_loc_x = [1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4]
    grid_loc_y = [1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4]
    text_loc = {'c13', 'c9', 'c5', 'c1', 'c14', 'c10', 'c6', 'c2', 'c15', 'c11', 'c7', 'c3', 'c16', 'c12', 'c8', 'c4'}
    
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    plt.plot(grid_loc_x, grid_loc_y, 'o', markersize=msz, markerfacecolor='black')
    # set(gcf,'Visible', 'off') # Keep it from popping up
    # plt.hold()
    plt.plot(goal_cell_x, goal_cell_y, 'o', markersize=msz, markerfacecolor='blue')
    plt.text(goal_cell_x + 0.1, goal_cell_y + 0.2, "Goal")
    plt.plot(home_cell_x, home_cell_y, 'o', markersize=msz, markerfacecolor='blue')
    plt.text(home_cell_x + 0.1, home_cell_y + 0.2, "Home")
    plt.plot(refuel_cell_x, refuel_cell_y, 'o', markersize=msz, markerfacecolor='blue')
    plt.text(refuel_cell_x + 0.1, refuel_cell_y + 0.2, "Refuel")
    ax.set_xlim(0,5)
    ax.set_ylim(0,5)
    plt.xticks(np.arange(5))
    plt.yticks(np.arange(5))
    
    return fig, ax