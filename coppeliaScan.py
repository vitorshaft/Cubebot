from time import sleep
import numpy as np
import cv2
import procanCorke as pc
from spatialmath import SE3

try:
    import sim
except:
    print ('****************************************************************')
    print ('"sim.py" e "remoteApi" deverão estar na mesma pasta, verifique! ')
    print ('****************************************************************')
 
import msgpack
import math

clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Conecta ao CoppeliaSim

def tImagem(x_im, y_im, z_real):
    c_u = 159.10219838656386
    c_v = 119.15530866673073
    imAlpha = 0.9995444024861037
    f = 432.501742465656
    x_real = (z_real*(x_im-c_u))/(f*imAlpha)
    y_real = (z_real*(y_im-c_v))/(f)
    return([x_real,y_real,z_real])

def ik(lista):
    Pa = SE3(lista[0],lista[1],lista[2])
    return pc.ik(Pa)

def plotar(conf,loc):
    T = pc.fk(conf)
    P = SE3(-loc[2],loc[1],loc[0])
    #print(P)
    TP = T*P
    print(TP[0])
    juntas = pc.ik(TP)
    return(TP,juntas)

destino = []


cascata = cv2.CascadeClassifier('cascade.xml')
cam = True
linhas = 240
cols = 320
x_medio = int(cols / 2)         #calcula x do meio da tela
y_medio = int(linhas/2)         #calcula y do meio da tela
centro = int(cols / 2)          #mesma coisa, mas essa variavel nao vai mudar
centroY = int(linhas/2)         
posicao = 90 # degrees          #valor inicial do motor do robo
xm_atual = 160
gol = False
pCube = []

qTraj = []
df = []
#while (sim.simxGetConnectionId(clientID) != -1 and cam == True):
#configurações da camera
print('Vision Sensor object handling')
res, v1 = sim.simxGetObjectHandle(clientID, 'CamP1', sim.simx_opmode_oneshot_wait)
print('Getting first image')
err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_streaming)

jq = [24, 27, 30, 33, 36, 38]
laser = 41
def escanear(cam):
    cascata = cv2.CascadeClassifier('cascade.xml')
    #cam = True
    linhas = 240
    cols = 320
    x_medio = int(cols / 2)         #calcula x do meio da tela
    y_medio = int(linhas/2)         #calcula y do meio da tela
    centro = int(cols / 2)          #mesma coisa, mas essa variavel nao vai mudar
    centroY = int(linhas/2)         
    posicao = 90 # degrees          #valor inicial do motor do robo
    xm_atual = 160
    gol = False
    pCube = []

    qTraj = []
    df = []
    print('Vision Sensor object handling')
    res, v1 = sim.simxGetObjectHandle(clientID, 'CamP1', sim.simx_opmode_oneshot_wait)
    print('Getting first image')
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_streaming)

    jq = [24, 27, 30, 33, 36, 38]
    laser = 41
    while(cam == True):
        
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_buffer)
        
        if err == sim.simx_return_ok:
            #print("image OK!!!")
            
            img = np.array(image,dtype=np.uint8)
            
            img.resize([resolution[1],resolution[0],3])
            
            menor = 10
            maior = 50
            low_cinza = np.array([menor,menor,menor])    #menor RGB possivel
            high_cinza = np.array([maior,maior,maior])   #maior RGB possivel
            cinza_mask = cv2.inRange(img,low_cinza,high_cinza)    #cria intervalo de cores (mascara)
            contornos, _ = cv2.findContours(cinza_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #procura contornos da cor detectada
            
            contornos = sorted(contornos, key=lambda x:cv2.contourArea(x), reverse=True)    #inverte a ordem dos contornos            
            
            cinza = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # detecta as faces
            faces = cascata.detectMultiScale(cinza, 1.1, 5,None,None)#[320,240])   #(frame, fator de escala, min vizinhos, tam min, tam max,)
            
            # desenha retangulo em volta de cada face
            for (x, y, w, h) in faces:
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                x_medio = int((x + x + w) / 2)
                y_medio = int((y + y + h) / 2)
                break
            
            cv2.line(img, (x_medio, 0), (x_medio, 240), (0, 255, 0), 1)
            cv2.line(img, (0, y_medio), (320, y_medio), (0,255,0), 1)
            #cv2.line(cinza_mask, (x_medio, 0), (x_medio, 240), (255, 255, 255), 1)
            #cv2.line(cinza_mask, (0, y_medio), (320, y_medio), (255,255,255), 1)
            cv2.line(cinza, (x_medio, 0), (x_medio, 240), (255, 255, 255), 1)
            cv2.line(cinza, (0, y_medio), (320, y_medio), (255,255,255), 1)
            #cv2.imshow('image',image)
            cv2.imshow('POV',img)
            cv2.imshow('P/B',cinza)
            #cv2.imshow('P/B',cinza_mask)
            
            q = [sim.simxGetJointPosition(clientID,jq[0],sim.simx_opmode_oneshot)[1],
                    sim.simxGetJointPosition(clientID,jq[1],sim.simx_opmode_oneshot)[1],
                    sim.simxGetJointPosition(clientID,jq[2],sim.simx_opmode_oneshot)[1],
                    sim.simxGetJointPosition(clientID,jq[3],sim.simx_opmode_oneshot)[1],
                    sim.simxGetJointPosition(clientID,jq[4],sim.simx_opmode_oneshot)[1],
                    sim.simxGetJointPosition(clientID,jq[5],sim.simx_opmode_oneshot)[1]]
            qTraj.append(q)
            xOK = False
            yOK = False
            dist = sim.simxReadProximitySensor(clientID,laser,sim.simx_opmode_streaming)[2][2]
            
            x6,y6,z6 = (2*(160-x_medio)),(2*(120-y_medio)),(1000*dist)
            PC = 0
            if(dist<1 and dist > 0):
                p_c = tImagem(x6,y6,z6)
                #print(p_c)
                PC = plotar(q,p_c)
                pCube.append(PC[0])
                destino.append(PC[1])
                return(PC)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                #print(destino[-1])
                cam = False
            
        elif err == sim.simx_return_novalue_flag:
            print("no image yet")
            pass
        else:
            print( err)
            