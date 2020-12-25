# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 19:37:11 2020

@author: Suriya Arulselvan
"""
import numpy as np
import modern_robotics as mr

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    thetalist = np.array(thetalist0).copy()
    jointVectorMatrix = np.stack((thetalist0))
    i = 0
    maxiterations = 20
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist,thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    
    print("Iteration 0:\n")
    print("joint vector:", thetalist, "\n")
    print("SE(3) end-effector config:", mr.FKinBody(M, Blist, thetalist), "\n")
    print("error twist V_b:", Vb, "\n")
    print("angular error magnitude ||omega_b||:", np.linalg.norm([Vb[0],Vb[1],Vb[2]]), "\n")
    print("linear error magnitude ||v_b||:", np.linalg.norm([Vb[3],Vb[4],Vb[5]]), "\n\n")    
    
    while err and i < maxiterations:
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        i = i + 1
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist,thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
        
        jointVectorMatrix = np.vstack((jointVectorMatrix, thetalist))        
        print("Iteration",i,":\n")
        print("joint vector:", thetalist, "\n")
        print("SE(3) end-effector config:", mr.FKinBody(M, Blist, thetalist), "\n")
        print("error twist V_b:", Vb, "\n")
        print("angular error magnitude ||omega_b||:", np.linalg.norm([Vb[0],Vb[1],Vb[2]]), "\n")
        print("linear error magnitude ||v_b||:", np.linalg.norm([Vb[3],Vb[4],Vb[5]]), "\n\n")    
        np.savetxt("iterates.csv", jointVectorMatrix, delimiter = "," )
    return (thetalist, not err)

#Desired configuration
Tsd = np.array([[0,1,0,-0.5],[0,0,-1,0.1],[-1,0,0,0.1],[0,0,0,1]])

eomg = 0.001
ev = 0.0001

#Dimenstions of robot
L1 = 0.425
L2 = 0.392
W1 = 0.109
W2 = 0.082
H1 = 0.089
H2 = 0.095

#B vectors
B1 = np.array([0,1,0,(W1+W2), 0, (L1+L2)])
B2 = np.array([0,0,1,H2,(-L1-L2),0])
B3 = np.array([0,0,1,H2,-L2,0])
B4 = np.array([0,0,1,H2,0,0])
B5 = np.array([0,-1,0,-W2,0,0])
B6 = np.array([0,0,1,0,0,0])

Blist = np.transpose(np.stack((B1,B2,B3,B4,B5,B6)))
M = np.array([[-1,0,0,(L1+L2)], [0,0,1,(W1+W2)], [0,1,0,(H1-H2)], [0,0,0,1]])

#Initial Guess
thetalist0 = np.array([-3,1,4,-2.5,0.72,1.72])

#Call the main function
IKinBodyIterates(Blist, M, Tsd, thetalist0, eomg, ev)