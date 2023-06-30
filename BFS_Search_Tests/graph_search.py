# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import scipy as sp
import numpy as np
from synth_graph import synth_graph
import networkx as nx
import random
from TE2_v2 import TE_ctrl_init2
from breadth_first_search import BFS
import csv
from plot_trajectory import plot_trajectory
import pdb 
from collections import OrderedDict as od
# from synth_graph import synth_graph

# Grid: 4-by-4
M = 4
N = 4
nS = M*N
nE = 2*M

# Synthesizing graph for the system and environment
G, A = synth_graph(M, N)

# Dictionary to return state:
def get_coordinates(state_num):
    state_dict = od()
    reverse_state_dict = dict()
    for sys in range(1,nS+1):
        for env in range(1,nE+1):
            state = nE*(sys-1) + env - 1
            state_dict[sys, env] = state
            reverse_state_dict[state]= (sys, env)
    return reverse_state_dict[state_num]

# Get one of the shortest paths chosen at random:
def rand_shortest_path(P):
    short_path = []
    if P:
        min_sz = len(P[0])
        for ii in range(0,len(P)):
            sz = len(P[ii])
            if(sz > min_sz):
                nmin_sz = ii-1
                break
            else:
                nmin_sz = ii
        path_index = random.randint(0, nmin_sz)
        short_path = P[path_index].copy()
    return short_path

# Converting obstacle index (1-8) to 4-by-4 grid index location (1-16)
def convert_env(env0):
    if(env0 <= 2):
        obst_pos = env0 + 1
    elif(env0 <= 4):
        obst_pos = env0 + 3
    elif(env0 <= 6):
        obst_pos = env0 + 5
    else:
        obst_pos = env0 + 7
    return obst_pos

sys0 = 16
env0 = convert_env(1)
max_fuel = 10
init_sys = sys0
init_env = 1
fname= "wctest_init"+str(env0)+"_fuel" + str(max_fuel) + "_data.txt"
fname1 = "wctest_init"+str(env0)+"_fuel" + str(max_fuel) +"paths1" + "_data.txt"
fname2 = "wctest_init"+str(env0)+"_fuel" + str(max_fuel) +"paths2" + "_data.txt"

# Initialize
fuel = max_fuel
PARK = True
goal = 1
refuel = 8

# Keeping log of all trajectories
SYS_TRAJ = [init_sys]
ENV_TRAJ = [init_env]
FUEL_TRAJ = [fuel]
PARK_TRAJ = [PARK]
SIM_TIME = []
Paths1 = []
Paths2 = []
sim_time = 100

# Initialize system controller
sys_control = TE_ctrl_init2()
print(sys_control.state, sim_time, init_sys, init_env, PARK, fuel)
obst_pos1 = convert_env(init_env)
u = TE_ctrl_init2.move(sys_control, PARK, obst_pos1)
init_sys = u["Xr"]
fuel = u["fuel"]

# Finding obstacle transitions:
while sim_time > 0:
    # Initial Step:
    print(f"System in cell {init_sys} with fuel {fuel}, and the obstacle in state {init_env}")
    # Environment takes an action
    if(init_env == 1):
            env1 = 2
            env2 = 3
    elif(init_env == 2):
            env1 = 4
            env2 = 1
    elif(init_env == 3):
            env1 = 1
            env2 = 5
    elif(init_env == 4):
            env1 = 2
            env2 = 6
    elif(init_env == 5):
            env1 = 3
            env2 = 7
    elif(init_env == 6):
            env1 = 4
            env2 = 8
    elif(init_env == 7):
            env1 = 8
            env2 = 5
    elif(init_env == 8):
            env1 = 7
            env2 = 6
    
    # Finding the shortest paths
    short_dist1, P1 = BFS(G, A, nS, nE, init_sys, env1, goal)
    short_path1 = rand_shortest_path(P1)
        
    short_dist2, P2 = BFS(G, A, nS, nE, init_sys, env2, goal)
    short_path2 = rand_shortest_path(P2)
    
    n1 = len(P1)
    n2 = len(P2)
    
    # If fuel is not enough to get to goal, we find the shortest path to go to refueling
    if(short_dist1 > fuel):
        short_dist1_refuel, P1 = BFS(G, A, nS, nE, init_sys, env1, refuel)
        short_path1 = rand_shortest_path(P1)
        if short_path1 == []:
            print("No P1 refuel")
        else:
            n1 = len(P1)
    
    if(short_dist2 > fuel):
        short_dist2_refuel, P2 = BFS(G, A, nS, nE, init_sys, env2, refuel)
        short_path2 = rand_shortest_path(P2)
        if short_path2 == []:
            print("No P2 refuel")
        else:
            n2 = len(P2)

    # Finding environment reaction to system transition:
    if(init_sys == goal):
        park_switch = random.randint(0, 1)
        if(park_switch):
            PARK = not PARK
            if(PARK):
                goal = 1
                init_sys = sys0
            else:
                goal = sys0
                init_sys = 1
#       else:
#           old_init_sys = init_sys
#           init_sys = old_init_sys    
    
    elif(n1 > 0 and n2 == 0):
        init_env = env1
        if short_path1:
            obst_pos1 = convert_env(init_env)
            u = TE_ctrl_init2.move(sys_control, PARK, obst_pos1)
            init_sys = u["Xr"]
            fuel = u["fuel"]
                    
    elif(n2 > 0 and n1 == 0):
        init_env = env2
        if short_path2:
            obst_pos2 = convert_env(init_env)
            u = TE_ctrl_init2.move(sys_control, PARK, obst_pos2)
            init_sys = u["Xr"]
            fuel = u["fuel"] # After system takes a step.
    elif(n1==0 and n2 ==0):
        print("Car cannot move forward")
        break
    else:
        if(n1 < n2):
            init_env = env1
            if short_path1:
                try:
                    obst_pos1 = convert_env(init_env)
                    u = TE_ctrl_init2.move(sys_control, PARK, obst_pos1)
                    init_sys = u["Xr"]
                    fuel = u["fuel"]
                except:
                    pdb.set_trace()    
        
        elif(n1 >= n2):
            init_env = env2
            if short_path2:
                try: 
                    obst_pos2 = convert_env(init_env)
                    u = TE_ctrl_init2.move(sys_control, PARK, obst_pos2)
                    init_sys = u["Xr"]
                    fuel = u["fuel"] # After system takes a step.
                except:
                    print("Error no control action for system")
                    pdb.set_trace()
                
    
    # Save trajectory
    SYS_TRAJ.append(init_sys)
    ENV_TRAJ.append(init_env)
    FUEL_TRAJ.append(fuel)
    PARK_TRAJ.append(PARK)
    Paths1.append(P1)
    Paths2.append(P2)
    
    # Stop game if you run out of fuel.
    if(fuel < 0):
        print("Fuel < 0")
        break
    sim_time = sim_time-1
    print(sys_control.state, sim_time, init_sys, init_env, PARK, fuel)
    
    #%% Write to a file:
file = open(fname, "w+")
file.write(', '.join(str(s) for s in SYS_TRAJ))
file.write("\n")
file.write(', '.join(str(e) for e in ENV_TRAJ))
file.write("\n")
file.write(', '.join(str(f) for f in FUEL_TRAJ))
file.write("\n")
file.write(', '.join(str(p) for p in PARK_TRAJ))
file.write("\n")
file.close()

file = open(fname1, "w+")
file.close()

file = open(fname2, "w+")
file.close()

#%%  Plot trajectory:
animation = plot_trajectory(SYS_TRAJ, ENV_TRAJ, FUEL_TRAJ)
print("Completed. See animation")