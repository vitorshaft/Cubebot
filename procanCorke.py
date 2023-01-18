import numpy as np
import math
import roboticstoolbox as rtb
from sympy import Matrix, nsimplify
from spatialmath.base import *
from spatialmath import SE3
import spatialmath.base.symbolic as sym
from roboticstoolbox import ET, ETS, ERobot

pi = math.pi

#DESCRIÇÃO DO PROCAN NO MODELO EROBOT

base = SE3(0.116,0,0)
lb = ET.tx(-0.116)
l0 = ET.tz(0.0294)*ET.Rz()
l1 = ET.Rx(pi/2)*ET.Rz()
l2 = ET.tx(0.23)*ET.Rz()
l3 = ET.tx(0.08173)*ET.Ry(-pi/2)*ET.Rz()
l4 = ET.tx(-0.0019)*ET.tz(-0.0904)*ET.Rx(pi/2)*ET.Rz()
l5 = ET.tx(0.0438)*ET.Ry(pi/2)*ET.Rz()

Link = rtb.robot.Link
Base = Link(lb, name='lBase', m=0.20)
link0 = Link(l0, name='link1', parent=Base, m=0.029, r=[-8.2*1e-3,-1.6*1e-4,-7.02*1e-3], I=[2.05*1e-3,1.58*1e-3,7.1*1e-4], Jm=0.0002)
link1 = Link(l1, name='link2', parent=link0, m=1.108, r=[1.64*1e-2,-2.96*1e-3,-1.12*1e-6], I=[2.45*1e-3,2.27*1e-3,4.55*1e-4], Jm=0.0002)
link2 = Link(l2, name='link3', parent=link1, m=0.0389, r=[0.0,7.76*1e-3,0.0], I=[1.28*1e-3,1.09*1e-3,7.06*1e-4], Jm=0.0002)
link3 = Link(l3, name='link4', parent=link2, m=0.0389, r=[7.76*1e-3,0.0,0.0], I=[1.28*1e-3,1.09*1e-3,7.06*1e-4], Jm= 3.3*1e-5)
link4 = Link(l4, name='link5', parent=link3, m=0.1404, r=[6.54*1e-3,0.0,-2.08*1e-4], I=[6.05*1e-4,4.5*1e-4,1.96*1e-4], Jm= 3.3*1e-5)
link5 = Link(l5, name='link6', parent=link4, m=0.3118, r=[-1.045*1e-2,1.37*1e-4,-9.97*1e-5], I=[5.056*1e-3,3.1*1e-3,1.96*1e-3], Jm= 3.3*1e-5)

procan = ERobot([Base,link0,link1,link2,link3,link4,link5],name='PROCaN',manufacturer='Vitor Domingues',base=base, gravity=[0,0,0])

procan.addconfiguration_attr("q30", [0.5,0.5,0.5,0.5,0.5,0.5])


'''
DH = [[0, 0, -0.1160, 0],    #i1 - base
    [0, 90*math.pi/180, 0, 0.0294],   #i2 - i1
    [180*math.pi/180, 180*math.pi/180, 0.23, 0], #i3 - i2
    [-90*math.pi/180, 90*math.pi/180, 0.08173, -0.00286], #i4 - i3 (multiplicar Z, depois X)
    [180*math.pi/180, 90*math.pi/180, 0.0019, -0.0904],  #i5 - i4
    [-90*math.pi/180, -90*math.pi/180, 0.0438, 0],     #i6 - i5
    #[-180*math.pi/180, -90*math.pi/180, 0.0438, 0],
    [0., 0., 0., 0.]]


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
    ], name="PROCaN", manufacturer="Vitor Domingues", base = SE3(-0.116,0,0))
'''
print(procan)

def simbolico():
    q = sym.symbol('q_:6')#_:6')
    T = procan.fkine(q)
    Ts = T.simplify()
    #Ts = nsimplify(Ts, tolerance=1e-4)
    M = Matrix(Ts.A)
    #print(M)
    from sympy import lambdify
    T_func = lambdify(q, M, modules='numpy')
    #T_func(0,0,pi,-pi/2,pi,-pi/2)
    g = sym.symbol('g')
    procan.gravity = [0, 0, g]
    qd = sym.symbol('q_:6')#d_:6')
    qd
    qdd = sym.symbol('qdd_:6')#_:6')

def fk(q):
    return procan.fkine(q)[0]

def ik(p):
    return procan.ikine_LMS(p)

def jac(q):
    return procan.jacobe(q)

def tau(q,qd,qdd,g):
    return procan.rne(q,qd,qdd,g)