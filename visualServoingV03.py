# sim.py, simConst.py, and the remote API library available
#python 3.7
#https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm
import time
import cv2
import numpy as np

try:
    import sim
except:
    print ('****************************************************************')
    print ('"sim.py" e "remoteApi" deverão estar na mesma pasta, verifique! ')
    print ('****************************************************************')
 
import msgpack
import math

#variaveis globais
graus = math.pi/180
vel=5*math.pi/180
acel=5*math.pi/180
velMax=[vel,vel,vel,vel,vel,vel]
acelMax=[acel,acel,acel,acel,acel,acel]
coordVel=[0,0,0,0,0,0]
handles = [23, 26, 29, 32, 35, 38]
jAtual = []
config = []
for j in range(6):
    config.append(90*graus)
config[3]=0
config[4]=0
target = [
    90*graus,
    30*graus,
    70*graus,
    0,
    0,
    90*graus
]
procan = [1.9998165,1.916378,0.135191,6.28-2.757191,6.28-4.796837,0.476764]
clientIdExecutado='Sem leitura'
Manipulador= 'Cubebot'  
nomeSinal='P_Arm_Id_Executado'
goal = False

#config da camera
linhas = 240
cols = 320
x_medio = int(cols / 2)         #calcula x do meio da tela
y_medio = int(linhas/2)         #calcula y do meio da tela
centro = int(cols / 2)          #mesma coisa, mas essa variavel nao vai mudar
centroY = int(linhas/2)         
posicao = 90 # degrees          #valor inicial do motor do robo
xm_atual = 160

def posicao(conf):
    adquireSinal('Ler') #Sinal de coleta de informaçoes do script no CoppeliaSim
    Dados_de_Movimento={"id":"SeqMov","tipo":"mov","Config":conf,"CoordVel":coordVel,"Velmax":velMax,"Acelmax":acelMax}
    
    packMov=msgpack.packb(Dados_de_Movimento)
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Dados_Movimento',[],[],[],packMov,sim.simx_opmode_oneshot)

    # Executa a sequência de movimento
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Movimento',[],[],[],'SeqMov',sim.simx_opmode_oneshot)
    adquireSinal('SeqMov')

def adquireSinal(id):
    global clientIdExecutado
    global nomeSinal
    while clientIdExecutado != id:
        retCode, s = sim.simxGetStringSignal(clientID,nomeSinal,sim.simx_opmode_buffer)
        if retCode == sim.simx_return_ok:
            s = s.decode('ascii')
        clientIdExecutado = s

def movJ(indices,posicoes):
    while (sim.simxGetConnectionId(clientID) != -1):
        atuais = []
        atuais = config
        for j in range(len(indices)):
            atuais[j] = sim.simxGetJointPosition(clientID,indices[j],sim.simx_opmode_oneshot)[1]
        print('Atuais: ',atuais)
        off = []
        for item in range(len(atuais)):
            off.append(posicoes[item]-atuais[item])
        while atuais != posicoes:
            for i in range(len(indices)):
                sim.simxSetJointTargetPosition(clientID,indices[i],atuais[i]+(off[i]/100),sim.simx_opmode_oneshot)
            for j in range(len(indices)):
                atuais[j] = sim.simxGetJointPosition(clientID,indices[j],sim.simx_opmode_oneshot)[1]
            #print('Atuais: ',atuais)
    


def aprox_final(xm_atual,ym_atual,z_atual,J_atual):
        if (xm_atual < 140):
            Kp = (140-xm_atual)/140
            sim.simxSetJointTargetVelocity(clientID,23,Kp*vel,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,23,J_atual[0]-0.01,sim.simx_opmode_oneshot)
            xOK = False
        elif (xm_atual > 180):
            Kp = (180-xm_atual)/320
            sim.simxSetJointTargetVelocity(clientID,23,Kp*vel,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,23,J_atual[0]+0.01,sim.simx_opmode_oneshot)
            xOK = False
        else:
            xOK = True

        if (ym_atual < 100):
            Kp = (100-xm_atual)/100
            sim.simxSetJointTargetVelocity(clientID,26,Kp*vel,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,26,J_atual[1]-0.01,sim.simx_opmode_oneshot)
            yOK = False
        elif (ym_atual > 140):
            Kp = (140-xm_atual)/240
            sim.simxSetJointTargetVelocity(clientID,26,Kp*vel,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,26,J_atual[1]+0.01,sim.simx_opmode_oneshot)
            yOK = False
        else:
            yOK = True
            #sim.simxSetJointTargetPosition(clientID,38,4.5,sim.simx_opmode_oneshot)
        
        if (z_atual < 0.05):
            Kp = (0.05-z_atual)/0.2
            sim.simxSetJointTargetVelocity(clientID,29,1,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID,38,1,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,29,J_atual[2]-0.01,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,38,J_atual[5]-0.01,sim.simx_opmode_oneshot)
            zOK = False
        
        elif (z_atual > 0.1):
            Kp = (0.1-z_atual)/0.2
            sim.simxSetJointTargetVelocity(clientID,29,1,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID,38,1,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,29,J_atual[2]+0.01,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,38,J_atual[5]+0.01,sim.simx_opmode_oneshot)
            zOK = False
        
        else:
            zOK = True
        return(xOK*yOK*zOK)

def capturar(atual):
    for a in range(450-int(100*atual)):
        sim.simxSetJointTargetPosition(clientID,38,(atual/100)+0.01,sim.simx_opmode_oneshot)

# carrega o classificador em cascata
cascata = cv2.CascadeClassifier('cascade.xml')

sim.simxFinish(-1)  #fecha qualquer conexão pendente
#inicia conexão com o cliente (CoppeliaSim) e salva o ID
#simxStart(string connectionAddress,number connectionPort,bool waitUntilConnected,bool doNotReconnectOnceDisconnected,number timeOutInMs,number commThreadCycleInMs)
clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5)
#se conseguir conexão...
if clientID != -1:
    print('Conectado com sucesso')
    # Inscreve o Nome_do_Sinal recebido do servidor(em bytes) para chamada_de_retorno (cliente) 
    sim.simxGetStringSignal(clientID,nomeSinal,sim.simx_opmode_streaming)
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
    print('Simulacao iniciada!')
    #adquireSinal('Ler') #Sinal de coleta de informaçoes do script no CoppeliaSim
    
    #posicao(config)
    #posicao(target)
    #posicao(procan)
    #adquireSinal('SeqMov')
    movJ(handles,target)
    
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
    sim.simxGetStringSignal(clientID,nomeSinal,sim.simx_opmode_discontinue)
    sim.simxGetPingTime(clientID)
    
    # Encerra a conexão com o Coppelia
    sim.simxFinish(clientID)
else:
    print ('Falhou a conexão com o remote API server')
cv2.destroyAllWindows()
print ('Programa finalizado')