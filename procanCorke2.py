import roboticstoolbox as rtb
from roboticstoolbox import ET, ETS, ERobot
from sympy import Matrix, nsimplify
from spatialmath.base import *
import spatialmath.base.symbolic as sym
import math

pi = math.pi

l0 = ET.tz(0.0294)*ET.Rz()
l1 = ET.Rx(pi/2)*ET.Rz()
l2 = ET.tx(0.23)*ET.Rz()
l3 = ET.tx(0.08173)*ET.Ry(-pi/2)*ET.Rz()
l4 = ET.tx(-0.0019)*ET.tz(-0.0904)*ET.Rx(pi/2)*ET.Rz()
l5 = ET.tx(0.0438)*ET.Ry(pi/2)*ET.Rz()

Link = rtb.robot.Link
link0 = Link(l0, name='link1', m=0.029, r=[-8.2*1e-3,-1.6*1e-4,-7.02*1e-3], I=[2.05*1e-3,1.58*1e-3,7.1*1e-4], Jm=0.0002)
link1 = Link(l1, name='link2', parent=link0, m=1.108, r=[1.64*1e-2,-2.96*1e-3,-1.12*1e-6], I=[2.45*1e-3,2.27*1e-3,4.55*1e-4], Jm=0.0002)
link2 = Link(l2, name='link3', parent=link1, m=0.0389, r=[0.0,7.76*1e-3,0.0], I=[1.28*1e-3,1.09*1e-3,7.06*1e-4], Jm=0.0002)
link3 = Link(l3, name='link4', parent=link2, m=0.0389, r=[7.76*1e-3,0.0,0.0], I=[1.28*1e-3,1.09*1e-3,7.06*1e-4], Jm= 3.3*1e-5)
link4 = Link(l4, name='link5', parent=link3, m=0.1404, r=[6.54*1e-3,0.0,-2.08*1e-4], I=[6.05*1e-4,4.5*1e-4,1.96*1e-4], Jm= 3.3*1e-5)
link5 = Link(l5, name='link6', parent=link4, m=0.3118, r=[-1.045*1e-2,1.37*1e-4,-9.97*1e-5], I=[5.056*1e-3,3.1*1e-3,1.96*1e-3], Jm= 3.3*1e-5)

procan = ERobot([link0,link1,link2,link3,link4,link5],name='PROCaN',manufacturer='Vitor Domingues',gravity=[0,0,0])

print(procan)
qr = [0,0,pi,1.57,pi,1.57]
q30 = [0.5,0.5,0.5,0.5,0.5,0.5]
#procan.plot(q30)
q = sym.symbol('ϴ_:6')#_:6')
#q = sym.symbol('φ,ϴ,ψ,α,β,γ')
T = procan.fkine(q)

import numpy as np
vector = np.vectorize(np.int_)
puma = rtb.models.DH.Puma560()
#procan.gravity = [0,0,0]
qd = sym.symbol('ϴd_:6')
qdd = sym.symbol('ϴdd_:6')
z = np.array([[0],[0],[0],[0],[0],[0]])
#z = vector(z)
#qp = np.array([[0.5],[0.5],[0.5],[0.5],[0.5],[0.5]])
qp = np.array([0.5,0.5,0.5,0.5,0.5,0.5])
#qp = vector(qp)
qpp = qp

tau = procan.rne(qp,0.5+np.zeros((6,)),0.5+np.zeros((6,)))
#tauS = procan.rne(q,qd,qdd,symbolic=True,gravity=np.array([0,0,0]))
#%time tau = procan.rne_python(q, qd, qdd)
#pc.robot.accel(qp,0.5 * np.ones(6), np.zeros(6))
#pc.robot.inertia(qp)

qr = np.array([[0,0,pi,1.57,pi,1.57]])
pIn = procan.inertia(qr)
print(pIn)
'''
ERobot: PROCaN (by Vitor Domingues), 6 joints (RRRRRR), dynamics
┌─────┬────────┬───────┬────────┬──────────────────────────────────────────────┐
│link │  link  │ joint │ parent │             ETS: parent to link              │
├─────┼────────┼───────┼────────┼──────────────────────────────────────────────┤
│   0 │ link1  │     0 │ BASE   │ tz(0.0294) ⊕ Rz(q0)                          │
│   1 │ link2  │     1 │ link1  │ Rx(90°) ⊕ Rz(q1)                             │
│   2 │ link3  │     2 │ link2  │ tx(0.23) ⊕ Rz(q2)                            │
│   3 │ link4  │     3 │ link3  │ tx(0.08173) ⊕ Ry(-90°) ⊕ Rz(q3)              │
│   4 │ link5  │     4 │ link4  │ tx(-0.0019) ⊕ tz(-0.0904) ⊕ Rx(90°) ⊕ Rz(q4) │
│   5 │ @link6 │     5 │ link5  │ tx(0.0438) ⊕ Ry(90°) ⊕ Rz(q5)                │
└─────┴────────┴───────┴────────┴──────────────────────────────────────────────┘

erro em:  0
erro em:  1
[[[0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]]

 [[0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]
  [0. 0. 0. 0. 0. 0.]]]


  
'''