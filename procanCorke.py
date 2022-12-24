import numpy as np
import math
import roboticstoolbox as rtb
from sympy import Matrix
from spatialmath.base import *
from spatialmath import SE3
import spatialmath.base.symbolic as sym

pi = math.pi

robot = rtb.robot.DHRobot(
    [
        rtb.robot.RevoluteDH(a=-0.116),
        rtb.robot.RevoluteDH(d=0.0294, alpha=pi/2),
        rtb.robot.RevoluteDH(a=0.23, alpha=pi),
        rtb.robot.RevoluteDH(d=-0.00286, a=0.08173, alpha=pi/2),
        rtb.robot.RevoluteDH(d=-0.0904, a=0.0019, alpha=pi/2),
        rtb.robot.RevoluteDH(a=0.0438, alpha=-pi/2),
    ], name="Puma560")

print(robot)

q = sym.symbol('q_:6')
T = robot.fkine(q)
Ts = T.simplify()
M = Matrix(Ts.A)
print(M)