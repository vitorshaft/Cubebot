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

a3 = sp.Symbol('a3')
d4 = sp.Symbol('d4')
pi2 = sp.Symbol('pi/2')
mpi2 = sp.Symbol('-pi/2')

#DH: [theta,alpha,a,d]

DHlivro = [[q1, 0, 0, 0],
    [q2, mpi2, 0, 0],
    [q3, 0, a3, 0],
    [q4, mpi2, 0, d4],
    [q5, pi2, 0, 0],
    [q6, mpi2, 0, 0],
    [0., 0., 0., 0.]]

DH2 = [[q1, 0, 0.1160, 0],
    [q2, -math.pi/2, 0.0006, 0.0170],
    [q3, 180*math.pi/180, 0.2285, 0.0016],
    [q4, -math.pi/2, 0.0768, -0.0058],
    [q5, 0*math.pi/180, 0.0084, 0.0987],
    [q6, 90.0*math.pi/2, 0.0091, -0.0114],
    [0., 0., 0., 0.]]

'''
T3DS = [[-0.1160, 0, 0, 0, 0, q1],    #i1 - base
    [0, 0, 0.0294, 90*math.pi/180, 0, q2],   #i2 - i1
    [0.23, 0, 0, 0, -180*math.pi/180, q3], #i3 - i2
    [0.08173, 0.01863, -0.00286, 90*math.pi/180, 0, q4+round(-90*math.pi/180,2)], #i4 - i3
    [0, 0, -0.1012, 0, 0, q5],  #i5 - i4
    [0.0119, 0, -0.0142, -90*math.pi/180, 0, q6+round(180*math.pi/180,2)]] #i6 - i5

T3D = [[-0.1160, 0, 0, 0, 0, 0],    #i1 - base
    [0, 0, 0.0294, 90*math.pi/180, 0, 0],   #i2 - i1
    [0.23, 0, 0, 0, -180*math.pi/180, 0], #i3 - i2
    [0.08173, 0.01863, -0.00286, 90*math.pi/180, 0, -90*math.pi/180], #i4 - i3
    [0, 0, -0.1012, 0, 0, 0],  #i5 - i4
    [0.0119, 0, -0.0142, -90*math.pi/180, 0, 180*math.pi/180]] #i6 - i5

CIS = [[0.1160, 0, 0, 0, 0, -q1],    #i1 - base
    [0, 0.0294, 0, -90*math.pi/180, 0, -q2],   #i2 - i1
    [0.23, 0, 0, 0, 180*math.pi/180, q3], #i3 - i2
    [0.01863,0.00286,0.08173, -90*math.pi/180, 0, q4+round(90*math.pi/180,2)], #i4 - i3
    [0, 0, 0.1012, 0, 0, -q5],  #i5 - i4
    [0.0119, -0.0142, 0, 90*math.pi/180, 0, q6-round(180*math.pi/180,2)]] #i6 - i5

\begin{pmatrix}\cos \left(90º\right)&-\sin \left(90º\right)&0\\ \:\:\sin \left(90º\right)&\cos \left(90º\right)&0\\ \:\:0&0&1\end{pmatrix}\times \begin{pmatrix}\cos \left(-90º\right)&0&-\sin \left(-90º\right)\\ \:\:0&1&0\\ \:\:\sin \left(-90º\right)&0&\cos \left(-90º\right)\end{pmatrix}

Between '/J3' and '/J4':
    d=0.0029
    theta=89.3
    a=0.0186
    alpha=90.0

Between '/J4' and '/J5':
    d=-0.1721
    theta=180.0
    a=0.0227
    alpha=90.0

Between '/J5' and '/J6':
    d=-0.0000
    theta=-89.3
    a=0.0019
    alpha=-90.0

'''

T3DS = [[-0.1160, 0, 0, 0, 0, q1],    #i1 - base
    [0, 0, 0.0294, 90*math.pi/180, 0, q2],   #i2 - i1
    [0.23, 0, 0, 0, -180*math.pi/180, q3], #i3 - i2 (alpha = 180°)
    [0.08173, 0.01863, -0.00286, 90*math.pi/180, 0, q4+round(-90*math.pi/180,2)], #i4 - i3 (a = 0.083826, th = 12,84°)
    [0.0205, 0, -0.0904, 0, -90*math.pi/180, q5+round(90*math.pi/180,2)],  #i5 - i4
    [0.0438, 0, 0, 0, 90*math.pi/180, q6+round(-90*math.pi/180,2)]] #i6 - i5 (a = 0.04375, th = -87,52°)

#DH: [theta,alpha,a,d]

DH = [[0, 0, -0.1160, 0],    #i1 - base
    [0, 90*math.pi/180, 0, 0.0294],   #i2 - i1
    [180*math.pi/180, 180*math.pi/180, 0.23, 0], #i3 - i2
    [-90*math.pi/180, 90*math.pi/180, 0.08173, -0.00286], #i4 - i3 (multiplicar Z, depois X)
    [180*math.pi/180, 90*math.pi/180, 0.0019, -0.0904],  #i5 - i4
    [-90*math.pi/180, -90*math.pi/180, 0.0438, 0],     #i6 - i5
    #[-180*math.pi/180, -90*math.pi/180, 0.0438, 0],
    [0., 0., 0., 0.]]

T3D = [[-0.1160, 0, 0, 0, 0, 0],    #i1 - base
    [0, 0, 0.0294, 90*math.pi/180, 0, 0],   #i2 - i1
    [0.23, 0, 0, 180*math.pi/180, 0, 180*math.pi/180], #i3 - i2
    [0.08173, 0, -0.00286, 0, -90*math.pi/180, -90*math.pi/180], #i4 - i3
    [0.0019, 0, -0.0904, 90*math.pi/180, 0, 180*math.pi/180],  #i5 - i4
    [0.0438, 0, 0, 0, 90*math.pi/180, 90*math.pi/180]] #i6 - i5

CIS = [[0.1160, 0, 0, 0, 0, -q1],    #i1 - base
    [0, 0.0294, 0, -90*math.pi/180, 0, -q2],   #i2 - i1
    [0.23, 0, 0, 0, 180*math.pi/180, q3], #i3 - i2
    [0.01863,0.00286,0.08173, -90*math.pi/180, 0, q4+round(90*math.pi/180,2)], #i4 - i3
    [0.0904, 0.0205, 0, 0, 90*math.pi/180, q5+round(-90*math.pi/180,2)],  #i5 - i4
    [0, 0.0019, 0.0438, 0, -90*math.pi/180, q6+round(90*math.pi/180,2)]] #i6 - i5

def springer(lista):
    theta,alpha,a,d = lista[0],lista[1],lista[2],lista[3]

    T = np.eye(4)
    T[0] = [np.cos(theta), -np.sin(theta),0,a]
    T[1] = [np.sin(theta)*np.cos(alpha),np.cos(theta)*np.cos(alpha),-np.sin(alpha),-np.sin(alpha)*d]
    T[2] = [np.sin(theta)*np.sin(alpha),np.cos(theta)*np.sin(alpha),np.cos(alpha),np.cos(alpha)*d]

    return T

def springer3D(lista):
    a,y,d,alpha,gamma,theta = lista[0],lista[1],lista[2],lista[3],lista[4],lista[5]
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    sg = np.sin(gamma)
    cg = np.cos(gamma)
    st = np.sin(theta)
    ct = np.cos(theta)
    T = np.eye(4)
    T[0] = [cg*ct,-cg*st,-sg,a]#(d*sg)+a]
    T[1] = [(ca*st)-(sa*sg*ct),(sa*sg*st)+(ca*ct),-sa*cg,y]#(y*ca)-(d*sa*cg)]
    T[2] = [(ca*sg*ct)-(sa*st),(-ca*sg*st)-(sa*ct),ca*cg,d]#(d*ca*cg)-(y*sa)]

    return T

def transf(teta, alfa, a, d):
    X = np.eye(4)
    Z = np.eye(4)
    #X[0] = [1, 0, 0, a]
    X[1] = [0, np.cos(alfa), -np.sin(alfa), 0]
    X[2] = [0, np.sin(alfa), np.cos(alfa), 0]
    Z[0] = [np.cos(teta), -np.sin(teta), 0, 0]
    Z[1] = [np.sin(teta),np.cos(teta), 0, 0]
    #Z[2] = [0, 0, 1, d]
    '''
    D = np.eye(4)
    D = np.insert(D,3,[a,0,d,1],axis=1)
    D = np.delete(D,4,1)
    R = X@Z
    T = D@R
    '''
    Tx,Tz = np.eye(4),np.eye(4)
    Tx = np.insert(Tx,3,[a,0,0,1],axis=1)
    Tx = np.delete(Tx,4,1)
    Tz = np.insert(Tz,3,[0,0,d,1],axis=1)
    Tz = np.delete(Tz,4,1)
    T = X@Tx@Z@Tz
    T = np.around(T, decimals=4)
    return T

def transfZX(teta, alfa, a, d):
    X = np.eye(4)
    Z = np.eye(4)
    #X[0] = [1, 0, 0, a]
    X[1] = [0, np.cos(alfa), -np.sin(alfa), 0]
    X[2] = [0, np.sin(alfa), np.cos(alfa), 0]
    Z[0] = [np.cos(teta), -np.sin(teta), 0, 0]
    Z[1] = [np.sin(teta),np.cos(teta), 0, 0]
    #Z[2] = [0, 0, 1, d]
    D = np.eye(4)
    D = np.insert(D,3,[a,0,d,1],axis=1)
    D = np.delete(D,4,1)
    R = Z@X
    T = D@R
    T = np.around(T, decimals=4)
    return T

def tSymb(teta, alfa, a, d):
    X = sp.eye(4)
    Z = sp.eye(4)
    
    X[1] = [0, sp.cos(alfa), sp.sin(alfa), 0]
    X[2] = [0, -sp.sin(alfa), sp.cos(alfa), 0]
    X[3] = [a, 0, 0, 1]
    Z[0] = [sp.cos(teta), sp.sin(teta), 0, 0]
    Z[1] = [-sp.sin(teta),sp.cos(teta), 0, 0]
    Z[3] = [0, 0, d, 1]
    T = X @ Z
    return [T,Z,X]

def tf3D(lista):
    #print(lista)
    x,y,z,X,Y,Z = lista[0],lista[1],lista[2],lista[3],lista[4],lista[5]
    Rx, Ry, Rz = np.eye(4),np.eye(4),np.eye(4)
    #rotacao em X
    #Rx[0] = [1,0,0,x]
    Rx[1] = [0,np.cos(X),-np.sin(X),0]
    Rx[2] = [0,np.sin(X),np.cos(X),0]
    #rotacao em Y
    Ry[0] = [np.cos(Y),0,-np.sin(Y),0]
    #Ry[1] = [0,1,0,y]
    Ry[2] = [np.sin(Y),0,np.cos(Y),0]
    #rotacao em Z
    Rz[0] = [np.cos(Z), -np.sin(Z), 0, 0]
    Rz[1] = [np.sin(Z),np.cos(Z), 0, 0]
    #Rz[2] = [0,0,1,z]
    
    #translacao
    T = np.eye(4)
    T = np.insert(T,3,[x,y,z,1],axis=1)
    D = np.delete(T,4,1)
    
    #D = np.array([[x],[y],[z],[1]])
    R = Rx@Ry@Rz
    #R = Rz@Ry@Rx
    T = np.around(D@R, decimals=4)
    return(T)

def t3Dsimb(lista):
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
    R = Rz@Ry@Rx
    #T = np.around(T@R, decimals=4)
    T = T@R
    return(T)

#CINEMATICA INVERSA SIMBOLICA

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

tee = []
t6 = []
st6 = []
t3 = []
st3 = []
t3s = []

#CINEMATICA INVERSA

tinvs = []

for s in DH:
    t6.append(transf(s[0], s[1], s[2], s[3]))

'''
for s in DH[:3]:
    t6.append(transf(s[0], s[1], s[2], s[3]))
s = DH[3]
t6.append(transfZX(s[0], s[1], s[2], s[3]))
for u in DH[4:]:
    t6.append(transf(u[0],u[1],u[2],u[3]))
'''
T6 = t6[0]@t6[1]@t6[2]@t6[3]@t6[4]@t6[5]

for st in DH:
    st6.append(springer(st))

ST6 = st6[0]@st6[1]@st6[2]@st6[3]@st6[4]@st6[5]


for d in T3D:
    t3.append(tf3D(d))

for st in T3D:
    st3.append(springer3D(st))

ST3 = st3[0]@st3[1]@st3[2]@st3[3]@st3[4]@st3[5]

for j in T3DS:
    t3s.append(t3Dsimb(j))

T01 = t3Dsimb(T3DS[0])
t16 = []

for rel in T3DS[1:]:
    t16.append(t3Dsimb(rel))

for inv in CIS:
    tinvs.append(cInvS(inv))

T3 = t3[0]@t3[1]@t3[2]@t3[3]@t3[4]@t3[5]
T3S = t3s[0]@t3s[1]@t3s[2]@t3s[3]@t3s[4]@t3s[5]

T16 = t16[0]@t16[1]@t16[2]@t16[3]@t16[4]
T16 = nsimplify(T16, tolerance=1e-5, rational=True)

#simplificando a matriz homogenea
for r in range(len(T3S)):
    T3S[r] = simplify(T3S[r])

Pa = np.array([[2],[2],[2],[10]])
Pa = Pa/10

pos = T3@Pa
pos = np.around(pos, decimals=3)

psimb = T3S@Pa
#psimb = np.around(psimb, decimals=3)
#psimb = sp.N(psimb, 4)  #reduz os valores do sympy a 3 casa decimais
psimb = nsimplify(psimb,tolerance=1e-5,rational=True)
T06s = nsimplify(T3S,tolerance=1e-5,rational=True)
#psimb = psimb.evalf(4)

#print(pos)
#print(psimb)

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

'''
((-43*sin(q3 - q4)*sin(q5 + q6)/53998 - sin(q5 + q6)*cos(q3 - q4) + 60*cos(q3 - q4)*cos(q5 + q6)/37673)*sin(q6 - 157/50) 
+ (43*sin(q3 - q4)*cos(q5 + q6)/53998 + 60*sin(q5 + q6)*cos(q3 - q4)/37673 + cos(q3 - q4)*cos(q5 + q6))*cos(q6 - 157/50))*sin(q5) 
+ ((-43*sin(q3 - q4)*sin(q5 + q6)/53998 - sin(q5 + q6)*cos(q3 - q4) + 60*cos(q3 - q4)*cos(q5 + q6)/37673)*cos(q6 - 157/50) 
- (43*sin(q3 - q4)*cos(q5 + q6)/53998 + 60*sin(q5 + q6)*cos(q3 - q4)/37673 + cos(q3 - q4)*cos(q5 + q6))*sin(q6 - 157/50))*cos(q5)
'''

#dir = T01@T12@T23
dir = T01@T12@T23@T34@T45
dir = nsimplify(dir, tolerance=1e-5,rational=True)  #dir[10] = 0

'''
rodar ROS
roscore
outra janela
roslaunch gazebo_ros emptyworld.launch
ou
rosrun rviz rviz
'''