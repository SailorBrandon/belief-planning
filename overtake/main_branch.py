import sys
from Init_MPC import initBranchMPC
from MPC_branch import BranchMPC, BranchMPCParams, BranchMPCProx
from highway_branch_dyn import *
import numpy as np
import pdb
import Highway_env_branch
from Highway_env_branch import merge_geometry
from utils import Branch_constants, MPCParams
def sim_overtake():
    # ======================================================================================================================
    # ============================================= Initialize parameters  =================================================
    # ======================================================================================================================
    N = 8                                # number of time steps for each branch
    n = 4;   d = 2                       # State and Input dimension
    x0 = np.array([0, 1.8, 0, 0])        # Initial condition (only for initializing the MPC, not the actual initial state of the sim)
    am = 6.0
    rm = 0.3
    dt = 0.1
    NB = 2                               # number of branching, 2 means a tree with 1-m-m^2 branches at each level.

    N_lane = 4


    # Initialize controller parameters
    xRef = np.array([0.5,1.8,15,0])
    cons =  Branch_constants(s1=2,s2=3,c2=0.5,tran_diag=0.3,alpha=1,R=1.2,am=am,rm=rm,J_c=20,s_c=1,ylb = 0.,yub = 7.2,L=4, W=2.5,col_alpha=5,Kpsi = 0.1)
    backupcons = [lambda x:backup_maintain(x,cons),lambda x:backup_brake(x,cons),lambda x:backup_lc(x,xRef)]
    model = PredictiveModel(n, d, N, backupcons, dt, cons)

    mpcParam = initBranchMPC(n,d,N,NB,xRef,am,rm,N_lane,cons.W)
    mpc = BranchMPCProx(mpcParam, model)

    Highway_env_branch.sim_overtake(mpc,N_lane)

if __name__== "__main__":
  sim_overtake()
