from scipy.spatial.transform import Rotation as R
import sympy as sp
from sympy import *
import math
import numpy as np
#from equacoes_procan import Braco

q1 = sp.Symbol('q1')
q2 = sp.Symbol('q2')
q3 = sp.Symbol('q3')
q4 = sp.Symbol('q4')
q5 = sp.Symbol('q5')
q6 = sp.Symbol('q6')

t3s = [Matrix([
 [cos(q1), -sin(q1), 0, -0.116],
 [sin(q1),  cos(q1), 0,      0],
 [      0,        0, 1,      0],
 [      0,        0, 0,      1]]),
 Matrix([
 [cos(q2), -6.12323399573677e-17*sin(q2),          1.0*sin(q2),      0],
 [sin(q2),  6.12323399573677e-17*cos(q2),         -1.0*cos(q2),      0],
 [      0,                           1.0, 6.12323399573677e-17, 0.0294],
 [      0,                             0,                    0,      1]]),
 Matrix([
 [         -1.0*cos(q3), -sin(q3), 1.22464679914735e-16*cos(q3), 0.23],
 [         -1.0*sin(q3),  cos(q3), 1.22464679914735e-16*sin(q3),    0],
 [-1.22464679914735e-16,        0,                         -1.0,    0],
 [                    0,        0,                            0,    1]]),
 Matrix([
 [cos(q4 - 1.57), -6.12323399573677e-17*sin(q4 - 1.57),   1.0*sin(q4 - 1.57),  0.08173],
 [sin(q4 - 1.57),  6.12323399573677e-17*cos(q4 - 1.57),  -1.0*cos(q4 - 1.57),  0.01863],
 [             0,                                  1.0, 6.12323399573677e-17, -0.00286],
 [             0,                                    0,                    0,        1]]),
 Matrix([
 [6.12323399573677e-17*cos(q5 + 1.57), -sin(q5 + 1.57),   1.0*cos(q5 + 1.57),  0.0205],
 [6.12323399573677e-17*sin(q5 + 1.57),  cos(q5 + 1.57),   1.0*sin(q5 + 1.57),       0],
 [                               -1.0,               0, 6.12323399573677e-17, -0.0904],
 [                                  0,               0,                    0,       1]]),
 Matrix([
 [6.12323399573677e-17*cos(q6 - 1.57), -sin(q6 - 1.57),  -1.0*cos(q6 - 1.57), 0.0438],
 [6.12323399573677e-17*sin(q6 - 1.57),  cos(q6 - 1.57),  -1.0*sin(q6 - 1.57),      0],
 [                                1.0,               0, 6.12323399573677e-17,      0],
 [                                  0,               0,                    0,      1]])]

CIS = [[0.1160, 0, 0, 0, 0, -q1],    #i1 - base
    [0, 0.0294, 0, -90*math.pi/180, 0, -q2],   #i2 - i1
    [0.23, 0, 0, 0, 180*math.pi/180, q3], #i3 - i2
    [0.01863,0.00286,0.08173, -90*math.pi/180, 0, q4+round(90*math.pi/180,2)], #i4 - i3
    [0.0904, 0.0205, 0, 0, 90*math.pi/180, q5+round(-90*math.pi/180,2)],  #i5 - i4
    [0, 0.0019, 0.0438, 0, -90*math.pi/180, q6+round(90*math.pi/180,2)]] #i6 - i5

def cInvS(lista):
    x,y,z,X,Y,Z = lista[0],lista[1],lista[2],lista[3],lista[4],lista[5]
    Rx, Ry, Rz = sp.eye(4),sp.eye(4),sp.eye(4)
    st = sp.sin(Z)
    ct = sp.cos(Z)
    #rotacao em X
    Rx[1] = [0,sp.cos(X),sp.sin(X),0]
    Rx[2] = [0,-sp.sin(X),sp.cos(X),0]
    #rotacao em Y
    Ry[0] = [sp.cos(Y),0,sp.sin(Y),0]
    Ry[2] = [-sp.sin(Y),0,sp.cos(Y),0]
    #rotacao em Z
    Rz[0] = [sp.cos(Z),sp.sin(Z),0,0]
    Rz[1] = [-sp.sin(Z),sp.cos(Z),0,0]
    #translacao
    T = sp.eye(4)
    T[3] = [x,y,z,1]
    #MULTIPLICA NA ORDEM INVERSA (X, Y E DEPOIS Z)
    R = Rx@Ry@Rz
    #T = np.around(T@R, decimals=4)
    T = T@R
    return(T)

T3S = t3s[0]@t3s[1]@t3s[2]@t3s[3]@t3s[4]@t3s[5]

tinvs = []

for inv in CIS:
    tinvs.append(cInvS(inv))

#CINEMATICA INVERSA:

#T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56 , logo:
#T06 @ T65 @ T54 @ T43 = T01 @ T12 @ T23:

T06 = T3S
T65 = cInvS(CIS[5])
T54 = cInvS(CIS[4])
T43 = cInvS(CIS[3])
T32 = cInvS(CIS[2])
T21 = cInvS(CIS[1])
T10 = cInvS(CIS[0])

T01 = t3s[0]
T12 = t3s[1]
T23 = t3s[2]
T34 = t3s[3]
T45 = t3s[4]

esq = T06@T65   #@T54@T43
esq = nsimplify(esq, tolerance=1e-5,rational=True) #esq[10] parece ser o menor:

#dir = T01@T12@T23
dir = T01@T12@T23@T34@T45
dir = nsimplify(dir, tolerance=1e-5,rational=True)  #dir[10] = 0
