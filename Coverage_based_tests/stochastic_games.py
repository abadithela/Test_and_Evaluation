# Vertices of players: V1, V2, and Vp(half-player)
# Player 1: Circle
# Player 2: Diamond
# Stochastic player: Square
# Finite set of actions for environment and system: A1, A2 and finite set of actions
# of the probabilistic player, Ap.
# Transition map from V1 to V2 given by T1
# Transition map from V2 to Vp given by T2
# Probabilistic transition map from Vp to V1 given by Tdelta
# Phi is the specification

import numpy as np
from random import randrange
import pandas as pd
# import networkx as nx
# File for system controller:
from strategy_simple_runner_blocker import runner

##% Defining variables for test strategy synthesis:
# Filenames for stored system controllers and related variables:
fname = "strategy_simple_runner_blocker.py"
class_name = "runner"
sys_var = "Xr" # This is the system variable that goes in the runner function

# Max. number of test-cases in the test suite:
Ntests = 3

# class GG:
#     def __init__(self, V1, V2, Vp, A1, A2, Ap, T1, T2, Tp, L):
#         self.V = (V1, V2, Vp)
#         self.A = (A1, A2, Ap)
#         self.T = (T1, T2, Tp)
#         self.label = L
#         if not Vp:
#             self.graph = nx.complete_bipartite_graph(len(V1), len(V2))
#         else:
#             self.graph = nx.complete_multipartite_graph(len(V1), len(V2), len(Vp))
#         self.build_DFA()

#     def build_DFA(self):
#         pass


# def trans_automata(V1, V2, Vp, x0, A1, A2, Ap, T1, T2, Tp, L, phi):
#     TA = GG(V1, V2, Vp, A1, A2, Ap, T1, T2, Tp, L)
#     return TA

# V1: env action vertices
# V2: sys action vertices
# Unsafe set:
def unsafe(collision_indices):
    U = []
    for ii in collision_indices:
        s = state(ii,ii)
        U.extend(["v1_"+str(s)])
        U.extend(["v2_"+str(s)])
    return U
# Generate transition system:
def trans_sys(V1, V2, Vp, T1, T2, Tp):
    if not Vp:
        GVp = []
    GEdges=[]
    # First all environment action transitions:
    for env_state in range(1,6):
        for end_env_state in T1[env_state-1]:
            for sys_state in range(1,6):
                start_state = state(sys_state, env_state)
                start_vtx = "v1_"+str(start_state)
                end_state = state(sys_state, end_env_state)
                end_vtx = "v2_"+str(end_state)
                edge = [start_vtx, end_vtx, "ae"]
                GEdges.append(edge)
    # Now, all system action transitions:
    for sys_state in range(1,6):
        for end_sys_state in T2[sys_state-1]:
            for env_state in range(1,6):
                start_state = state(sys_state, env_state)
                start_vtx = "v2_"+str(start_state)
                end_state = state(end_sys_state, env_state)
                end_vtx = "v1_"+str(end_state)
                edge = [start_vtx, end_vtx, "as"]
                GEdges.append(edge)
    return GVp, GEdges

# Generate winning sets for 2 player games:
def state(sys_loc, env_loc):
    return 5*(sys_loc) + env_loc

# Given number of system and environment transitions:
def vertices(Ns, Ne):
    Vp =[]
    V1 = []
    V2 = []
    for xs in range(1,Ns+1):
        for xe in range(1,Ne+1):
            s = state(xs, xe)
            V1.extend(["v1_"+str(s)]) # Environment action vertices
            V2.extend(["v2_"+str(s)]) # System action vertices
    return V1, V2, Vp
    
# Main function to synthesize winning sets:
# Returns winning set for n steps
# N: Maximum number of iterations for fixed-point iterations
# Win_ii_n: The entire winning set upto the nth fixed point iteration
# Pre_ii_n: Consists only of the Pre set of states computed at the nth iteration

def synt_winning_set(GVp, GEdges, U, W0, N):
    W = [W0] # Winning set with 0 iterations
    Pre_cur = W0.copy()
    Win_cur = W0.copy()
    for ii in range(1,N+1):
        Pre_ii = pre(GVp, GEdges, Pre_cur, U, 1, 0, 1)
        Pre_ii_1 = Pre_ii[0].copy()
        Pre_ii_2 = Pre_ii[1].copy()
        Win_cur_1 = Win_cur[0].copy()
        Win_cur_2 = Win_cur[1].copy()
        if Pre_ii_1: # If it is not empty
            Win_cur_1.extend(Pre_ii_1)
            Win_cur_1 = list(dict.fromkeys(Win_cur_1)) # Removes duplicates
        if Pre_ii_2: # If it is not empty
            Win_cur_2.extend(Pre_ii_2)
            Win_cur_2 = list(dict.fromkeys(Win_cur_2)) # Removes duplicates
        Win_ii_1 = Win_cur_1.copy()
        Win_ii_2 = Win_cur_2.copy()
        Win_ii = [Win_ii_1, Win_ii_2]
        W.append(Win_ii)
        Pre_cur = Pre_ii.copy()
        Win_cur = Win_ii.copy()
    return W

# ToDo: Fix the notation of W0 here...
# Defining Predecessor operator for synthesizing winning sets: 
# Assume: Player 1 is the environment and Player 2 is the System
# Winning sets would only contain environment action states
# Pre(S):= {x \in V2| \forall   }
# U: Unsafe set of states
# Qualifier notations: there_exists: 0 and forall: 1
def pre(GVp, GEdges, W0, U, qual1, qual2, qual3):
    if not GVp: # 2-player game winning set
        # Simple backward reachability:
        env_W0 = W0[0].copy() # First row of W0 has env action nodes in winning set
        sys_W0 = W0[1].copy() # Second row of W0 has sys action nodes in winning set
        Win1 = [] # Winning set containing environment action states
        Win2 = [] # Winning set containing system action states
        Win = [] # Winning set containing env winning actions in the first row and sys winning actions in the second row

        # Backward reachability for winning set with environment action state
        for env_win in env_W0:
            end_node = [row[1] for row in GEdges]
            env_win_idx = [ii for ii, x in enumerate(end_node) if x==env_win]
            start_node = [row[0] for row in GEdges] # Extracting the first column in G.Edges
            env_nbr = [start_node[ii] for ii in env_win_idx]
            if env_nbr: # If list is not empty
                for env_nbr_elem in env_nbr:
                    if env_nbr_elem not in U:  # Not in unsafe set
                        Win2.append(env_nbr_elem)

        # Backward reachability for winning set with system action state. All environment actions must lead to a winning state
        for sys_win in sys_W0:
            end_node = [row[1] for row in GEdges]
            potential_sys_win_idx = [ii for ii, x in enumerate(end_node) if x==sys_win]
            start_node = [row[0] for row in GEdges] # Extracting the first column in G.Edges                
            potential_sys_nbr = [start_node[ii] for ii in potential_sys_win_idx]
            sys_nbr = []
            for potential_nbr in potential_sys_nbr:
                if potential_nbr not in U:
                    potential_nbr_idx = [ii for ii, x in enumerate(start_node) if x==potential_nbr]
                    potential_nbr_end_node = [end_node[ii] for ii in potential_nbr_idx]
                    if set(potential_nbr_end_node) <= set(sys_W0):
                        sys_nbr.extend([potential_nbr])
         
            Win1.extend(sys_nbr) 
        Win1 = list(dict.fromkeys(Win1)) # Removes duplicates
        Win2 = list(dict.fromkeys(Win2)) # Removes duplicates
        Win.append(Win1)
        Win.append(Win2)

    else: # Find sure, almost-sure and positive winning sets
        Win=[]
    return Win

def get_state(state):
    if state%5 == 0:
        env_state = 5
        sys_state = state/5 - 1
    else:
        env_state = state%5
        sys_state = state//5
    return sys_state, env_state

### Main run of the file:
# Runner blocker example
Ns = 5
Ne = 5
V1, V2, Vp = vertices(Ns,Ne)
# Transition system
T1 = [[1], [3], [2,4], [3], [5]] #Environment transitions
T2 = [[1,2,3,4], [1,2,3,5], [1,2,3,4,5], [1,3,4,5], [2,3,4,5]] # System transitions
Tp = []

# Graph transition
GVp, GEdges = trans_sys(V1, V2, Vp, T1, T2, Tp)
collision_indices = [2,3,4]

# Unsafe set
U = unsafe(collision_indices)

# Initial Winning set:
sys_W0 = [] # Winning states from system action states
env_W0 = [] # Winning states from environment action states
for env_state in [2,3,4]:
    for sys_state in [5]:
        w0 = state(sys_state, env_state)
        sys_W0.extend(["v2_"+str(w0)])
        env_W0.extend(["v1_"+str(w0)])
W0 = [env_W0, sys_W0]

N = 5 # Can reach winning set in atmost 5 steps
W = synt_winning_set(GVp, GEdges, U, W0, N)
print(W)

# Reachability: 
# If this is a winning set, it is a winning set only ... 
# Create copies of vertices to store their winning set information:
W_V1 = [[v1, ''] for v1 in V1] # Env. action states
W_V2 = [[v2, ''] for v2 in V2] # Sys. action states
W_Vp = [[vp, ''] for vp in Vp] # Prob. action states

# Retrieve states:
sys_win = []
env_win = []
for ii in range(0,N):
    W_ii = W[ii].copy()
    env_action_states = W_ii[0].copy()
    sys_action_states = W_ii[1].copy()
    sys_win_ii = []
    env_win_ii = []
    for ee in env_action_states:
        ee_idx = V1.index(ee)
        if not W_V1[ee_idx][1]:      # If there is not a winning set assignment yet, make an assignment
            W_V1[ee_idx][1] = ii
        s = int(ee[3:])
        [sys_st, env_st] = get_state(s)
        sys_win_ii.append([env_st, sys_st])
    for ss in sys_action_states:
        ss_idx = V2.index(ss)
        if not W_V2[ss_idx][1]:
            W_V2[ss_idx][1] = ii
        s = int(ss[3:])
        [sys_st, env_st] = get_state(s)
        env_win_ii.append([env_st, sys_st])
    sys_win.append(sys_win_ii)
    env_win.append(env_win_ii)

print("System action winning states")

print(sys_win[0])
print(sys_win[1])    
print(sys_win[2])
print(sys_win[3])
print(sys_win[4])

print("Environment action winning states")

print(env_win[0])
print(env_win[1])
print(env_win[2])
print(env_win[3])
print(env_win[4])

# Winning sets : [W0_sys]
# ADDITION on 3/17/20:
# Edge information for test agent
# GEdges are the edges on the game graph
# N: No. of winning sets
# env_win: Env action states in winning sets
# sys_win: Sys action states in winning sets
# Returns edge information for all edges from an environment action state
# Edge weight of 0 for transition leaving winning set, one for staying in the same winning set, and 2 for moving into the next winning set
# ToDo: Make the data sructures more like dictionaries so that they can be easily read. Learn using pandas

def edge_info(GEdges, N, sys_win, env_win, W_V1, W_V2):
    env_edge_info = [GEdges[ii] for ii in range(len(GEdges)) if GEdges[ii][2]=='ae'] # Env. action states
    sys_edge_info = [GEdges[ii] for ii in range(len(GEdges)) if GEdges[ii][2]=='as'] # Sys. action states
    V1 = [row[0] for row in W_V1]
    V2 = [row[0] for row in W_V2]
    for env_edge in env_edge_info:
        # Finding start and end nodes of the edge:
        # Add if successor vertex is in winning sets
        start = env_edge[0]
        succ = env_edge[1]
        # Add number of transitions.
        # Initially the number of transitions is 0
        env_edge.append(0)
        start_idx = V1.index(start)
        succ_idx = V2.index(succ)
        env_win_set = W_V1[start_idx][1]
        sys_win_set = W_V2[succ_idx][1]
        # If the successor remains in the same winning set, the env edge information records this as 1
        if sys_win_set and env_win_set:
            if(env_win_set > sys_win_set): # Better because you're in the smaller attractor
                env_edge.append(2)
            elif(env_win_set == sys_win_set):
                env_edge.append(1)
        elif sys_win_set and not env_win_set:
            env_edge.append(2)
        else:
            env_edge.append(0) # Action leading outside winning set

    for sys_edge in sys_edge_info:
        # Finding start and end nodes of the edge:
        # Add if successor vertex is in winning sets
        start = sys_edge[0]
        succ = sys_edge[1]
        # Add number of transitions.
        # Initially the number of transitions is 0
        sys_edge.append(0)
        start_idx = V2.index(start)
        succ_idx = V1.index(succ)
        env_win_set = W_V1[succ_idx][1]
        sys_win_set = W_V2[start_idx][1]
        # If the successor remains in the same winning set, the env edge information records this as 1
        if sys_win_set and env_win_set:
            if(sys_win_set > env_win_set): # Better because you're in the smaller attractor
                sys_edge.append(2)
            elif(env_win_set == sys_win_set):
                sys_edge.append(1)
        elif env_win_set and not sys_win_set:
            sys_edge.append(2)
        else:
            sys_edge.append(0) # Action leading outside winning set

    return env_edge_info, sys_edge_info

# Finding edge information to then pass into synth_test_strategies:
env_edge_info, sys_edge_info = edge_info(GEdges, N, sys_win, env_win, W_V1, W_V2)

# Return edges where v is the starting vertex. The vertex input is in the form of [e,s]
# Player is either the system or the environment
def edge(v, edge_info, player):
    v_edges = []
    st = set_state(v, player)
    v_edges = [edges for edges in edge_info if edges[0]==st]
    v_edges_idx = [ii for ii in range(len(edge_info)) if edge_info[ii][0]==st]
    return v_edges, v_edges_idx

# Input v: vertex containing system and environment information
#        player: 'e' or 's' for environment or system
# Output: Returns vertex

def set_state(v, player):
    x = state(v[1], v[0]) # Feed in the system location and environment location
    if(player == 'e'):
        st = "v1_"+str(x)
    elif(player == 's'):
        st = "v2_"+str(x)
    else:
        print("Error in set_state: Input either 's' (system) or 'e' (environment) for the player variable.")
        st = []
    return st

# Randomly choosing a successor vertex from a given set of vertices
def random_succ(succ_vts):
    r = []
    if succ_vts:
        l = len(succ_vts)
        r = succ_vts[randrange(l)]
    else:
        print("Error in random_succ: The input list is empty.")
    return r

# Function to synthesize a single test strategy:
# Environment goes first and then the system acts
# Test run is a state-action sequence: [e1, s1, e2, ...]
# Wf: Final winning set
sys_control = runner()
def synth_test_strategy(env_edge_info, sys_edge_info, e0, env_win, sys_win):
    test_run = [e0]
    start = e0
    player = 'e'
    if(player == 'e'):
        Wf = env_win[0]
        W = env_win[N-1]
    else:
        Wf = sys_win[0]
        W = sys_win[N-1]
        
    while start not in Wf: # Final winning set
        if (player=='e'):
            edge_information = env_edge_info.copy()
            succ_edge_information = sys_edge_info.copy()
            succ_player = 's'
        
            edges, edges_idx = edge(start, edge_information, player)
            succ = [edge[1] for edge in edges] # List of all system successor vertices
            succ_win = [edge[4] for edge in edges]
            # First check if e0 is in the maximal winning set:
            if start not in W:
                succ_not_win = [succ[ii] for ii in range(len(succ_win)) if not succ_win[ii]] # All successor vertices not in winning set
                fin = finish(succ_not_win, succ, edges_idx, player)
                test_run.append(fin)
                break
            # Once you're outside the winning set, there's no way back in
            # ToDo: Make the following modular in functions...
            else:
                assert all(succ_win) # All successor vertices are winning vertices
                succ_1 = [succ[ii] for ii in len(succ) if edges[ii][4] == 1]
                succ_2 = [succ[ii] for ii in len(succ) if edges[ii][4] == 2]
                succ_1_idx = [edges_idx[ii] for ii in len(succ) if edges[ii][4] == 1]
                succ_2_idx = [edges_idx[ii] for ii in len(succ) if edges[ii][4] == 2]
                # Unvisited nodes
                unvisit_succ_1 = [succ[ii] for ii in len(succ) if (edges[ii][4] == 1 and edges[ii][3] == 0)]
                unvisit_succ_2 = [succ[ii] for ii in len(succ) if (edges[ii][4] == 2 and edges[ii][3] == 0)]
                # Indices of unvisited nodes:
                unvisit_succ_1_idx = [edges_idx[ii] for ii in len(succ) if (edges[ii][4] == 1 and edges[ii][3] == 0)]
                unvisit_succ_2_idx = [edges_idx[ii] for ii in len(succ) if (edges[ii][4] == 2 and edges[ii][3] == 0)]

                # Successor nodes in same winning set with some winning actions not taken
                action_succ_1 = []
                action_succ_2 = []
                action_succ_1_idx = []
                action_succ_2_idx = []
                for ii in range(len(succ_1)):
                    s1 = succ_1[ii]
                    s1_idx = succ_1_idx[ii]
                    s1_edges, s1_edge_idx = edges(s1, succ_edge_information, succ_player)
                    # Have the winning actions been taken?
                    untested_winning_transitions1 = [t for t in s1_edges if (s1_edges[3]==0 and s1_edges[4])]
                    if untested_winning_transitions1:
                        if not action_succ_1:
                            action_succ_1 = [s1]
                            action_succ_1_idx = s1_idx
                        else:
                            action_succ_1.append[s1]
                            action_succ_1_idx.append[s1_idx]
                # Successor nodes in attractor winning set with some winning actions not taken:
                for ii in range(len(succ_2)):
                    s2 = succ_2[ii]
                    s2_idx = succ_2_idx[ii]
                    s2_edges, s2_edge_idx = edge(s2, succ_edge_information, succ_player)
                    # Have the winning actions been taken?
                    untested_winning_transitions2 = [t for t in s2_edges if (s2_edges[3]==0 and s2_edges[4])]
                    if untested_winning_transitions2:
                        if not action_succ_2:
                            action_succ_2 = [s2]
                            action_succ_2_idx = s2_idx
                        else:
                            action_succ_2.append[s2]
                            action_succ_2_idx.append[s2_idx]

                # First, go through all unvisited vertices in the same winning set, before proceeding to the attractor set at the next level
                if unvisit_succ_1:
                    fin = finish(unvisit_succ_1, succ, edges_idx, player)
                elif unvisit_succ_2:
                    fin = finish(unvisit_succ_2, succ, edges_idx, player)
                elif action_succ_1:
                    fin = finish(action_succ_1, succ, edges_idx, player)
                elif action_succ_2:
                    fin = finish(action_succ_2, succ, edges_idx, player)
                else:
                    fin = finish(succ, succ, edges_idx, player)
            # Then keep a loop until you reach the winning set
                test_run.append(fin)
            player = 's'
            start = fin

        # Here, we use the correct-by-construction controller to respond to the environment changes
        elif (player == 's'):
            edge_information = sys_edge_info.copy()
            succ_edge_information = env_edge_info.copy()
            succ_player = 'e'
            # Correct-by-construction controller for system:
            fin = system_controller(start)
            test_run.append(fin)
            player = 'e'
            start = fin
        else:
            print("Player must be 'e' or 's'")
    return test_run                                              

# Function to synthesize a test suite:
def test_suite(N, Ntests, env_win, sys_win):
    # Wf = [env_win[0], sys_win[0]]
    # W = [env_win[N-1], sys_win[N-1]]
    e0 = [2,1]
    test_suite = []
    for ii in range(Ntests):
        test = synth_test_strategy(env_edge_info, sys_edge_info, e0, env_win, sys_win)
        if test_suite:
            test_suite.append(test)
        else:
            test_suite = [test]
    return test_suite

# Function to find the end_vertex randomly from a list of vertices vts, succ is the list of all successors, edge_idx is the index of all edges in the main player_edge_info list
# Function also takes care of updating the main env_Edge_info or sys_edge_info list
def finish(vts, succ, edges_idx, player):
    rand_fin = random_succ(vts) # Choosing final state
    rand_fin_idx = edges_idx[succ.index(rand_fin)]# Finding final state winning index in either sys_edge_info/env_edge_info

    # Increasing number of times successor has been visited
    if player == 'e':
        env_edge_info[rand_fin_idx][3] += 1
    elif player == 's':
        sys_edge_info[rand_fin_idx][3] += 1
    rand_fin_sys, rand_fin_env = get_state(int(rand_fin[3:]))
    fin = [rand_fin_env, rand_fin_sys]
    return fin

# A function to take in the correct-by-construction controller for the system:
def system_controller(start):
    sys_pos, env_pos = get_state(start)
    u = runner.move(sys_control, env_pos)
    sys_pos = u[sys_var]
    finish = [env_pos, sys_pos]
    return finish

# Function for testing node coverage:
def node_coverage():
    pass

# Test Suite
TS = test_suite(N, Ntests, env_win, sys_win)
print(TS)