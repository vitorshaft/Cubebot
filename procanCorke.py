import numpy as np
import math
import roboticstoolbox as rtb
from sympy import Matrix, nsimplify
from spatialmath.base import *
from spatialmath import SE3
import spatialmath.base.symbolic as sym

'''
DH = [[0, 0, -0.1160, 0],    #i1 - base
    [0, 90*math.pi/180, 0, 0.0294],   #i2 - i1
    [180*math.pi/180, 180*math.pi/180, 0.23, 0], #i3 - i2
    [-90*math.pi/180, 90*math.pi/180, 0.08173, -0.00286], #i4 - i3 (multiplicar Z, depois X)
    [180*math.pi/180, 90*math.pi/180, 0.0019, -0.0904],  #i5 - i4
    [-90*math.pi/180, -90*math.pi/180, 0.0438, 0],     #i6 - i5
    #[-180*math.pi/180, -90*math.pi/180, 0.0438, 0],
    [0., 0., 0., 0.]]
'''

pi = math.pi
#d*Rz*a*Rx
robot = rtb.robot.DHRobot(
    [
        #rtb.robot.RevoluteDH(a=-0.116),
        rtb.robot.RevoluteDH(d=0.0294, alpha=pi/2),
        rtb.robot.RevoluteDH(a=0.23),# alpha=pi),
        rtb.robot.RevoluteDH(d=-0.00286, a=0.08173, alpha=-pi/2),
        rtb.robot.RevoluteDH(d=-0.0904, a=0.0019, alpha=pi/2),
        rtb.robot.RevoluteDH(a=0.0438, alpha=-pi/2),#,
        rtb.robot.RevoluteDH()
    ], name="PROCaN")

print(robot)

q = sym.symbol('q_:6')#_:6')
T = robot.fkine(q)
Ts = T.simplify()
#Ts = nsimplify(Ts, tolerance=1e-4)
M = Matrix(Ts.A)
#print(M)
from sympy import lambdify
T_func = lambdify(q, M, modules='numpy')
#T_func(0,0,pi,-pi/2,pi,-pi/2)
g = sym.symbol('g')
robot.gravity = [0, 0, g]
qd = sym.symbol('q_:6')#d_:6')
qd
qdd = sym.symbol('qdd_:6')#_:6')
qdd