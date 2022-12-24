import sympy as sp
import math
import numpy as np

class Elo:
    def __init__(self, dh):
        self.dh = dh

    def Ti(self):
        q = self.dh[0]
        alpha = self.dh[1]
        a = self.dh[2]
        d = self.dh[3]
        
        st = sp.sin(q)
        ct = sp.cos(q)
        sa = sp.sin(alpha)
        ca = sp.cos(alpha)

        trans = sp.Matrix([[ct, -st * ca, st * sa, a * ct],
                          [st, ct * ca, -ct * sa, a * st],
                          [0, sa, ca, d],
                          [0, 0, 0, 1]])
        return trans
    
    @staticmethod
    def J_basica(de, para):
        pos_de = sp.Matrix([de[0,3], de[1,3],de[2,3]])
        z_de = sp.Matrix([de[0,2],de[1,2],de[2,2]])

        J_basica = np.hstack((np.cross(z_de, para-pos_de), z_de))

        return J_basica

class Braco:
    def __init__(self,listaDH):
        self.lista_elos = []
        for i in range(len(listaDH)):
            self.lista_elos.append(Elo(listaDH[i]))
    #sem simplificações
    def Ti(self):
        trans = sp.eye(4)
        for i in range(len(self.lista_elos)):
            trans = np.dot(trans, self.lista_elos[i].Ti())
        return trans

    def cinematica_direta(self, plot = False):
        trans = self.Ti()

        x = trans[0,3]
        y = trans[1,3]
        z = trans[2,3]
        alpha, beta, gamma = self.angulos_euler()

        if plot:
            x_list = []
            y_list = []
            z_list = []
            trans = sp.eye(4)

            x_list.append(trans[0,3])
            y_list.append(trans[1,3])
            z_list.append(trans[2,3])

            for i in range(len(self.lista_elos)):
                trans = np.dot(trans, self.lista_elos[i].Ti())

    def angulos_euler(self):
        trans = self.Ti()

        alpha = math.atan2(trans[1][2], trans[0][2])
