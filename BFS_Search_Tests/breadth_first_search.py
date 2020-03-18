import numpy as np
import scipy as sp
import networkx as nx
import scipy.linalg as la
import matplotlib.pyplot as plt

def BFS(G, A, nS, nE, init_sys, init_env, goal_state):
    start = int(8*(init_sys-1) + init_env) - 1
    goal = 8*(goal_state-1)*np.ones(8) + np.arange(0,8,1) # Zero indexed for Python
    nnodes = nS*nE
    # Because of zero-indexing, the first-node will be not be assigned.
    node_color = ["" for x in range(nnodes)]
    node_dist = np.zeros(nnodes)
    parent = ["" for x in range(nnodes)]
    P = [] # List of all paths
    p = []
    if(init_sys == goal_state):
        short_dist = 0
    else:
        short_dist = 100
    
    for ii in range(0,nnodes):
        if(not(ii==start)):
            node_color[ii]='w'
            node_dist[ii] = float("inf")
            parent[ii] = '0'

    node_color[start] = 'g'
    node_dist[start] = 0
    parent[start] = '0'
    Q = [start]
    Qp = [[start]] # Queue of paths    
    Qp_count = 0
    
    while Q:
        # Dequeue the first node
        u = Q.pop(0)
        adj_u_tuple = np.nonzero(A[u,:]) # IDs of all nodes adjacent to u
        adj_u = adj_u_tuple[0]
        for ii in range(0, len(adj_u)):
            v = adj_u[ii]
            # We only want to find system paths
            if(A[u][v] == 1):
                if(node_color[v] == 'w'):  # white
                    node_color[v] = 'g'    # gray
                    node_dist[v] = node_dist[u] + 1
                    parent[v] = u
                    Q.append(v)
                    if(v in goal): # This is the shortest distance to GOAL
                        short_dist = node_dist[v]
        node_color[u] = 'b' # black

    # Listing all paths
    while Qp:
        Qp_count += 1 # Keeping track of the number of paths in Qp
        curr_path = Qp.pop(0) # Grab the first path
        curr_vtx = curr_path[-1]
        if(curr_vtx in goal):
            P.append(curr_path)
        adj_vtx_tuple = np.nonzero(A[curr_vtx,:])
        adj_vtx = adj_vtx_tuple[0]
        for ii in range(0,len(adj_vtx)):
            v = adj_vtx[ii]
            if(A[curr_vtx][v] == 1): # Only paths that the system can take.
                if(v not in curr_path):
                    p = curr_path.copy()
                    p.append(v)                    
                    Qp.append(p)        
           
    return short_dist, P
