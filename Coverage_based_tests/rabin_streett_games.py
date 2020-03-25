# Stochastic Rabin and Streett Games
# Apurva Badithela
# Reference: The Complexity of Stochastic Rabin and Streett Games
# Input: Game G with condition (E,F), D = S\(E U F)
# Set: X,Y,Z, \bar(X), \bar(Y), \bar(Z)
# \bar(X) <-- S; \bar(Z) <-- S; \bar(Y) <-- \phi
# (E,F) are Buchi acceptance condition
# Game graph G

import numpy as numpy

def Almost(G, phi, D_list, E_list, F_list, X_list, Y_list, Z_list, bar_X_list, bar_Y_list, bar_Z_list):
    D = set(D_list)
    E = set(E_list)
    F = set(F_list)
    S = D.union(E,F)
    X = set(X_list)
    Y = set(Y_list)
    Z = set(Z_list)
    bar_X = set(bar_X_list)
    bar_Y = set(bar_Y_list)
    bar_Z = set(bar_Z_list)
    while True:
        X = bar_X.copy()
        while True:
            Y = bar_Y.copy()
            while True:
                Z = bar_Z.copy()
                Pre1_X = set(Pre1(X_list))
                Pre2_XY = set(Pre2(X_list, Y_list))
                Pre3_XYZ = set(Pre3(X_list, Y_list, Z_list))
                A1 = F.intersect(Pre1_X)
                A2 = E.intersect(Pre2_XY)
                A3 = D.intersect(Pre3_XYZ)
                bar_Z = A1.union(A2, A3)
                if (Z == bar_Z):
                    break
            bar_Y = Z.copy()
            bar_Z = S.copy()
            if(Y == bar_Y):
                break
        bar_X = Y.copy()
        bar_Y = set()
        if(X == bar_X):
            break
    return X

#  S: All states in game graph
# phi: Specification
# X: Set of states to which we need to find predecessor
def Pre1(X):
    
    return Pre1_X

def Pre2(X, Y):
    pass

def Pre3(X, Y, Z):
    pass
