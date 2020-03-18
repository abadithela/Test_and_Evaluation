# Construct an undirected graph with adjacency edges
import numpy as np
import scipy as sp
import networkx as nx
import scipy.linalg as la
import matplotlib.pyplot as plt

def synth_graph(M,N):
    nE = 2*N
    nS = M*N
    iE = np.array([1, 2, 4, 6, 8, 7, 5, 3])-1
    jE = np.array([2, 4, 6, 8, 7, 5, 3, 1])-1
    vE = 2*np.ones(8)
    Ap = sp.sparse.csc_matrix((vE, (iE, jE)), shape=(nE, nE)).toarray() # Matrix in compressed column format
    Ap = Ap + Ap.transpose()
    Apr = [Ap]*nS # Repeat patrol states nS times

    # Make a block diagonal matrix
    A = la.block_diag(*Apr)
    iS = np.array([1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 16, 16, 2, 3, 6, 7, 10, 11, 14, 15])-1
    jS = np.array([2, 5, 1, 3, 6, 2, 4, 7, 3, 8, 1, 6, 9, 2, 5, 7, 10, 3, 6, 8, 11, 4, 7, 12, 5, 10, 13, 6, 9, 11, 14, 7, 10, 12, 15, 8, 11, 16, 9, 14, 13, 10, 15, 14, 11, 16, 15, 12, 2, 3, 6, 7, 10, 11, 14, 15])-1

    for idx in range(0,len(iS)):
    #     rr = range(8*(iS[idx]-1),  8*(iS[idx])-1)
    #     cc = range(8*(jS[idx]-1),  8*(jS[idx])-1)
        rr = range(8*iS[idx],  8*(iS[idx]+1))
        cc = range(8*jS[idx],  8*(jS[idx]+1))

        for jdx in range(0,8):
            A[rr[jdx]][cc[jdx]] = 1

        # Collision avoidance (CA)-- robot and system cannot occupy the same
        # gridcell
    col_avoid = np.array([[2, 1],[3, 2], [6, 3], [7, 4], [10, 5], [11, 6], [14, 7], [15, 8]])-1 # -1 for zero-indexing
    for idx in range(0, len(col_avoid)):
        ii = 8*(col_avoid[idx][0])+col_avoid[idx][1] # Zero-indexing and trying to get indexing right
        A[ii] = np.zeros(nS*nE)
        A[:,ii] = (np.zeros(nS*nE)).T

    plt.spy(A)
    G = nx.from_numpy_matrix(A)
    return G, A
    
