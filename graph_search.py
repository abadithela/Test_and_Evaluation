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
# from synth_graph import synth_graph

# Grid: 4-by-4
M = 4
N = 4
nS = M*N
nE = 2*M

# Synthesizing graph for the system and environment
G, A = synth_graph(M, N)

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

# Finding obstacle transitions:
while sim_time > 0:
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
    short_path1 = []
    if P1:
        min_sz = len(P1[0])
        for ii in range(0,len(P1)):
            sz = len(P1[ii])
            if(sz > min_sz):
                nmin_sz = ii-1
                break
            else:
                nmin_sz = ii
        path_index = random.randint(0, nmin_sz)
        short_path1 = P1[path_index].copy()
        
    short_dist2, P2 = BFS(G, A, nS, nE, init_sys, env2, goal)
    short_path2 = []
    if P2:
        min_sz = len(P2[0])
        for ii in range(0,len(P2)):
            sz = len(P2[ii])
            if(sz > min_sz):
                nmin_sz = ii-1
                break
            else:
                nmin_sz = ii
        path_index = random.randint(0, nmin_sz)
        short_path2 = P2[path_index].copy()
        
    n1 = len(P1)
    n2 = len(P2)
    
    # If fuel is not enough to get to goal, we find the shortest path to go to refueling
    if(short_dist1 > fuel):
        short_dist1_refuel, P1_refuel = BFS(G, A, nS, nE, init_sys, env1, refuel)
        if P1_refuel:
            min_sz_refuel = len(P1_refuel[0])
            for ii in range(0,len(P1_refuel)):
                sz = len(P1_refuel[ii])
                if(sz > min_sz_refuel):
                    nmin_sz_refuel = ii-1
                    break
                else:
                    nmin_sz_refuel = ii
            path_index_refuel = random.randint(0, nmin_sz)
            n1 = len(P1_refuel)
            P1 = P1_refuel
            short_path1 = P1_refuel[path_index_refuel].copy()
        else:
            print("No P1 refuel")
    
    if(short_dist2 > fuel):
        short_dist2_refuel, P2_refuel = BFS(G, A, nS, nE, init_sys, env2, refuel)
       
        if P2_refuel:
            min_sz_refuel = len(P2_refuel[0])
            for ii in range(0,len(P2_refuel)):
                sz = len(P2_refuel[ii])
                if(sz > min_sz_refuel):
                    nmin_sz_refuel = ii-1
                    break
                else:
                    nmin_sz_refuel = ii   
            path_index_refuel = random.randint(0, nmin_sz)
            n2 = len(P2_refuel)
            P2 = P2_refuel
            short_path2 = P2_refuel[path_index_refuel].copy()
        else:
            print("No P2 refuel")
            
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
                obst_pos1 = convert_env(init_env)
                u = TE_ctrl_init2.move(sys_control, PARK, obst_pos1)
                init_sys = u["Xr"]
                fuel = u["fuel"]
                    
        elif(n1 >= n2):
            init_env = env2
            if short_path2:
                obst_pos2 = convert_env(init_env)
                u = TE_ctrl_init2.move(sys_control, PARK, obst_pos2)
                init_sys = u["Xr"]
                fuel = u["fuel"] # After system takes a step.
                
    
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
