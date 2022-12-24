"""
Inverse Kinematics for an n-link arm in 3D
Author: Takayuki Murooka (takayuki5168)
"""
import math
from NLinkArm3d import NLinkArm
import random


def random_val(min_val, max_val):
    return min_val + random.random() * (max_val - min_val)

DH = [[math.pi / 2, -math.pi / 2, .0198, 0.1143],
        [-11.3*math.pi/180, -math.pi/2, 0.0294, 0.0005],
        [-176.5*math.pi/180, 178*math.pi/180, -0.0061, 0.2285],
        [-math.pi/2, -math.pi/2, -0.0063, 0.0271],
        [1.5*math.pi/180, 4.3*math.pi/180, 0.8031, 0.0668],
        [-132.2*math.pi/180, 94.2*math.pi/180, -0.9939, 0.1018],
        [0., 0., 0., 0.]]
#[theta,alpha,a,d] >>>>> INVERTER!!!
DH2 = [[math.pi/2, 0, 0.1160, 0],
    [-11.3*math.pi/180, -math.pi/2, 0.0006, 0.0170],
    [-176.5*math.pi/180, 180*math.pi/180, 0.2285, 0.0016],
    [-math.pi/2, -math.pi/2, 0.0768, -0.0058],
    [1.5*math.pi/180, 0*math.pi/180, 0.0084, 0.0987],
    [-132.2*math.pi/180, 90.0*math.pi/180, 0.0091, -0.0114],
    [0., 0., 0., 0.]]

DH3 = [[0, 0, 0, 0.0440],   #J1 a base
    [math.pi/2, 0, 0.0003, 0.0880], #J2 a J1
    [90*math.pi/180, 0, 0.2101, 0.0],
    [0.1*math.pi/180, math.pi/2, 0.03, 0.0006],
    [168.8*math.pi/180, 90*math.pi/180, 0.0009, 0.2214],
    [-178.8*math.pi/180, 90.0*math.pi/180, 0.0005, -0.0001],
    [0., 0., 0., 0.]]

def main():
    print("Start solving Inverse Kinematics 10 times")
    # init NLinkArm with Denavit-Hartenberg parameters of PR2
    #                       
    n_link_arm = NLinkArm(DH3)
    # execute IK 10 times
    #goal = [0.17315,0.0696,0.4066]
    goal = [-0.32492,0.0060364,0.01]
    for _ in range(4):
        angulos = n_link_arm.inverse_kinematics([goal[0],
                                       goal[1],
                                       goal[2],
                                       random_val(-0.5, 0.5),
                                       random_val(-0.5, 0.5),
                                       random_val(-0.5, 0.5)], plot=True)


if __name__ == "__main__":
    main()
