# sim.py, simConst.py, and the remote API library available
#python 3.7
#https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm

'''
1. Executar padrão de varredura:
    J3 e J5 a 90°
    J4 e J6 varrem de 0° a 180
    Se J5<180 incrementa 1°
2. Computar Cinemática Inversa do objeto
3. Computar Cinemática Diferencial para obter
velocidade de juntas a partir da vel. linear para as 3 1ªs juntas
4. Deslocar EF para posição de captura
5. Computar vetor erro de posição linear
6. Mover manipulador a partir do vetor com ganho proporcional
ao inverso da massa de cada elo
7. Executar movimento de captura: J5 = 180°
8. Recolher manipulador com J5 fixo em 180°
'''
import time
import cv2
import numpy as np
import procanCorke as pc
from spatialmath import SE3
import matplotlib.pyplot as plt

try:
    import sim
except:
    print ('****************************************************************')
    print ('"sim.py" e "remoteApi" deverão estar na mesma pasta, verifique! ')
    print ('****************************************************************')
 
import msgpack
import math


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
    #print(TP[0])
    juntas = pc.ik(TP)
    return(TP,juntas)

def movimento(q):
        Config=q

        Dados_de_Movimento={"id":"SeqMov","tipo":"mov","Config":Config,"CoordVel":CoordVel,"Velmax":Velmax,"Acelmax":Acelmax}
        
        Empacotamento_dados_de_movimento=msgpack.packb(Dados_de_Movimento)
        sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Dados_Movimento',[],[],[],Empacotamento_dados_de_movimento,sim.simx_opmode_oneshot)

        # Executa a sequência de movimento
        sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Movimento',[],[],[],'SeqMov',sim.simx_opmode_oneshot)
        
        # Aguarda até que a sequência de movimento acima termine a execução
        espera_de_execução_movimento('SeqMov')

destino = []

# carrega o classificador em cascata
cascata = cv2.CascadeClassifier('cascade.xml')
delta_omega = 0

print ('Programa Iniciado')
sim.simxFinish(-1) # Fecha todas as conexões
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Conecta ao CoppeliaSim
if clientID!=-1:
    print ('Conectado com o remote API-Legacy server')

    client_Id_Executado='Sem leitura'
    Manipulador= 'Cubebot'  
    Nome_do_Sinal='P_Arm_Id_Executado'

    def espera_de_execução_movimento(id):
        global client_Id_Executado
        global Nome_do_Sinal
        while client_Id_Executado!=id:
            retCode,s=sim.simxGetStringSignal(clientID,Nome_do_Sinal,sim.simx_opmode_buffer)
            if retCode==sim.simx_return_ok:
                if type(s)==bytearray:
                    s=s.decode('ascii') 
                client_Id_Executado=s

    # Inscreve o Nome_do_Sinal recebido do servidor(em bytes) para chamada_de_retorno (cliente) 
    sim.simxGetStringSignal(clientID,Nome_do_Sinal,sim.simx_opmode_streaming)

    # Set-up some movement variables:
    vel = [ 1.90777254,  1.91676066,  4.21490854,  8.32765953,  7.61692325,
       13.05404777]
    Vel=13.0#5*math.pi/180
    Acel=600*math.pi/180
    Velmax= vel #[Vel,Vel,Vel,Vel,Vel,Vel]
    Acelmax=[Acel/10,Acel,3/2*Acel,Acel,Acel,Acel]
    CoordVel=[0,0,0,0,0,0]

    #configurações da camera
    print('Vision Sensor object handling')
    res, v1 = sim.simxGetObjectHandle(clientID, 'CamP1', sim.simx_opmode_oneshot_wait)
    print('Getting first image')
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_streaming)

    # Inicia a simulação
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    # Espera de leitura
    '''
    1. Executar padrão de varredura:
    J3 e J5 a 90°
    J4 e J6 varrem de 0° a 180
    Se J5<180 incrementa 1°
    '''
    espera_de_execução_movimento('Ler')
    pi = math.pi
    j1 = 0*math.pi/180
    j2 = 0*math.pi/180
    j3 = 90*math.pi/180
    j4 = 0*math.pi/180
    j5 = 70*math.pi/180
    j6 = -180*math.pi/180
    #jq = [23,26,29,32,35,37]
    #jq = [22, 25, 28, 31, 34, 36]
    jq = [24, 27, 30, 33, 36, 38]
    laser = 41
    inercial = 45
    # Envia a primeira sequência de movimento
    #Config=[30*math.pi/180,30*math.pi/180,-30*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180]
    Config=[j1,j2,j3,j4,j5,j6]
    #Config=[pi-0.1468398 ,  pi-2.29526532, pi-0.61810689, 0,0,0]
    #Config = [ 0.21176678,  pi-0.52539497, pi-2.1626808 ,  0,0,0]
    Dados_de_Movimento={"id":"SeqMov","tipo":"mov","Config":Config,"CoordVel":CoordVel,"Velmax":Velmax,"Acelmax":Acelmax}
    
    Empacotamento_dados_de_movimento=msgpack.packb(Dados_de_Movimento)
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Dados_Movimento',[],[],[],Empacotamento_dados_de_movimento,sim.simx_opmode_oneshot)

    # Executa a sequência de movimento
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Movimento',[],[],[],'SeqMov',sim.simx_opmode_oneshot)
    espera_de_execução_movimento('SeqMov')

    #resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_buffer)
    #linhas, cols, _ = image.shape   #obtem numero de linhas e colunas da imagem
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
    while(cam == True):
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_buffer)
        
        J0_atual = sim.simxGetJointPosition(clientID,jq[0],sim.simx_opmode_oneshot)[1]
        J1_atual = sim.simxGetJointPosition(clientID,jq[1],sim.simx_opmode_oneshot)[1]
        J2_atual = sim.simxGetJointPosition(clientID,jq[2],sim.simx_opmode_oneshot)[1]
        J3_atual = sim.simxGetJointPosition(clientID,jq[3],sim.simx_opmode_oneshot)[1]
        J4_atual = sim.simxGetJointPosition(clientID,jq[4],sim.simx_opmode_oneshot)[1]
        J5_atual = sim.simxGetJointPosition(clientID,jq[5],sim.simx_opmode_oneshot)[1]
        
        '''
        for item in range(314):
            sim.simxSetJointTargetPosition(clientID,jq[1],J1_atual+item/100,sim.simx_opmode_oneshot)
            time.sleep(0.1)
        for item in range(150):
            sim.simxSetJointTargetPosition(clientID,jq[2],J2_atual+(item/100),sim.simx_opmode_oneshot)
            time.sleep(0.1)
        '''
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
            faces = cascata.detectMultiScale(cinza, 1.06, 5,None,None)#[320,240])   #(frame, fator de escala, min vizinhos, tam min, tam max,)
            
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
            #print(q)
            qTraj.append(q)
            xOK = False
            yOK = False
            dist = sim.simxReadProximitySensor(clientID,laser,sim.simx_opmode_streaming)[2][2]
            
            omega_base = sim.simxGetObjectOrientation(clientID,20,inercial,sim.simx_opmode_streaming)[1]
            delta_omega = delta_omega-omega_base[1]
            '''
            # estabelecer relação de controle e matriz de ganho - OK
            # publicar coordenadas em uma memória assíncrona (ex: matriz global) para usar como sinal de erro
            #   ENQUANTO A ORIENTAÇÃO DO EE NÃO FOR ALTERADA, NÃO HAVERÁ VELOCIDADE NAS 3 ULTIMAS JUNTAS
            #   
            '''
            x6,y6,z6 = (2*(160-x_medio)),(2*(120-y_medio)),(1000*dist)

            def controle(lista):    #RETORNANDO UM VALOR MUITO ALTO EM RADIANOS
                dy, dz, dx = lista
                qd = np.array(q)
                J_inv = np.linalg.inv(pc.jac(qd))
                #from spatialmath import SE3
                #dTheta = J_inv*SE3(dx,dy,dz)
                v = np.array([dx/1000,dy/1000,dz/1000,0,0,0])
                #print(v)
                dTheta = J_inv @ v
                print(dTheta)   #ta retornando 3 valores ao inves de 6 dos 6 DOF
                return(dTheta)
                #return J_inv #dTheta
            
            #print(controle([dist,x_medio,y_medio]))

            #if(dist > 0.1 and dist < 1.0):
            p_c = tImagem(x6,y6,z6)
            print(p_c)
            ganho = controle(p_c)/100
            qd = q
            qd[1] = qd[1]+ganho[1]#(0.2*0.36)
            qd[2] = qd[2]+ganho[2]#(0.1*0.32)
            
                #movimento(q)
            '''
                if(y_medio > 130):
                    qd[4] = qd[4]+0.01#+ganho[4]#(0.1*0.32)
                    #movimento(q)
                elif(y_medio < 110):
                    qd[4] = qd[4]-0.01#-ganho[4]#(0.1*0.32)
                    #movimento(q)
                #print(ganho[1],ganho[2],ganho[4])
                '''
            #print(qd[1],qd[2],qd[4])
            #print(ganho)
            movimento(qd)
            #print(omega_base[1])    #printa rotacao em ÿ em relacao ao referencial inercial
            print(y_medio)
            print(dist)

            #if (dist != 0):
                #print(dist)
            #print([x_medio,y_medio,1000*dist])
            #135,116,249.4227
            #136,91,241.3288
            
            #p_c = tImagem(2*(160-x_medio),2*(120-y_medio),1000*dist)
            #I = sim.getShapeInertia(20) #printa matriz de inercia da base
            #print(I)
            #print(dist)
            if(dist<1 and dist > 0):
                p_c = tImagem(x6,y6,z6)
                #print(p_c)
                PC = plotar(q,p_c)
                pCube.append(PC[0])
                destino.append(PC[1])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                #print(destino[-1])
                cam = False
            
        elif err == sim.simx_return_novalue_flag:
            print("no image yet")
            pass
        else:
          print( err)
          #break
        atual = sim.simxGetJointPosition(clientID,jq[0],sim.simx_opmode_oneshot)
        atualJ2 = sim.simxGetJointPosition(clientID,jq[2],sim.simx_opmode_oneshot)
        atualJ6 = sim.simxGetJointPosition(clientID,jq[5],sim.simx_opmode_oneshot)[1]
        #print(atual)

        estendido = sim.simxGetJointPosition(clientID,jq[1],sim.simx_opmode_oneshot)[1]>=(0.1)
    #espera_de_execução_movimento('SeqMov')
    '''
    vel = [ 1.90777254,  1.91676066,  4.21490854,  8.32765953,  7.61692325,
       13.05404777]
    for i,hand in enumerate(jq):
        sim.simxSetJointTargetVelocity(clientID, hand, vel[i], sim.simx_opmode_oneshot)
    Config=[]
    for i,j in enumerate(destino[-1]):
        if (j<0):
            if(i==0 or i == 2):
                j = pi+j
            else:
                j = -1*j
            Config.append(j)
        else:
            if(i==0 or i == 2):
                j = pi+j
            Config.append(j)
    Config[4] +=pi/2
    pc.plot(Config)
    #Config=[pi-0.1468398 ,  pi-2.29526532, pi-0.61810689, 0,0,0]
    #Config = [ 0.21176678,  pi-0.52539497, pi-2.1626808 ,  0,0,0]
    Dados_de_Movimento={"id":"SeqMov","tipo":"mov","Config":Config,"CoordVel":CoordVel,"Velmax":Velmax,"Acelmax":Acelmax}
    
    Empacotamento_dados_de_movimento=msgpack.packb(Dados_de_Movimento)
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Dados_Movimento',[],[],[],Empacotamento_dados_de_movimento,sim.simx_opmode_oneshot)

    # Executa a sequência de movimento
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Movimento',[],[],[],'SeqMov',sim.simx_opmode_oneshot)
    
    # Aguarda até que a sequência de movimento acima termine a execução
    espera_de_execução_movimento('SeqMov')
    '''
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
    sim.simxGetStringSignal(clientID,Nome_do_Sinal,sim.simx_opmode_discontinue)
    sim.simxGetPingTime(clientID)
    
    # Encerra a conexão com o Coppelia
    sim.simxFinish(clientID)
else:
    print ('Falhou a conexão com o remote API server')
cv2.destroyAllWindows()
#a = input()
print ('Programa finalizado')

