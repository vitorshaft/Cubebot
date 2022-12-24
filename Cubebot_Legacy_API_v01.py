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
    Vel=5*math.pi/180
    Acel=5*math.pi/180
    Velmax=[Vel,Vel,Vel,Vel,Vel,Vel]
    Acelmax=[Acel,Acel,Acel,Acel,Acel,Acel]
    CoordVel=[0,0,0,0,0,0]

    #configurações da camera
    print('Vision Sensor object handling')
    res, v1 = sim.simxGetObjectHandle(clientID, 'CamP1', sim.simx_opmode_oneshot_wait)
    print('Getting first image')
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_streaming)

    # Inicia a simulação
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    # Espera de leitura
    espera_de_execução_movimento('Ler')
    
    j1 = 0
    j2 = 90*math.pi/180
    j3 = 90*math.pi/180
    j4 = 0
    j5 = 0
    j6 = 0

    # Envia a primeira sequência de movimento
    #Config=[30*math.pi/180,30*math.pi/180,-30*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180]
    Config=[0*math.pi/180,90*math.pi/180,90*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180]
    Dados_de_Movimento={"id":"SeqMov","tipo":"mov","Config":Config,"CoordVel":CoordVel,"Velmax":Velmax,"Acelmax":Acelmax}
    
    Empacotamento_dados_de_movimento=msgpack.packb(Dados_de_Movimento)
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Dados_Movimento',[],[],[],Empacotamento_dados_de_movimento,sim.simx_opmode_oneshot)

    # Executa a sequência de movimento
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Movimento',[],[],[],'SeqMov',sim.simx_opmode_oneshot)
    
    #resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_buffer)
    #linhas, cols, _ = image.shape   #obtem numero de linhas e colunas da imagem
    linhas = 320
    cols = 240
    x_medio = int(cols / 2)         #calcula x do meio da tela
    y_medio = int(linhas/2)         #calcula y do meio da tela
    centro = int(cols / 2)          #mesma coisa, mas essa variavel nao vai mudar
    centroY = int(linhas/2)         
    posicao = 90 # degrees          #valor inicial do motor do robo

    while (sim.simxGetConnectionId(clientID) != -1):
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_buffer)
        
        if err == sim.simx_return_ok:
            print("image OK!!!")
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[1],resolution[0],3])
            menor = 10
            maior = 35
            low_cinza = np.array([menor,menor,menor])    #menor RGB possivel
            high_cinza = np.array([maior,maior,maior])   #maior RGB possivel
            cinza_mask = cv2.inRange(img,low_cinza,high_cinza)    #cria intervalo de cores (mascara)
            contornos, _ = cv2.findContours(cinza_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #procura contornos da cor detectada
            
            contornos = sorted(contornos, key=lambda x:cv2.contourArea(x), reverse=True)    #inverte a ordem dos contornos            
            for cnt in contornos:
                (x, y, w, h) = cv2.boundingRect(cnt)
                if (((x_medio**2)-(int((x + x + w) / 2))**2)>16):
                    x_medio = int((x + x + w) / 2)
                    y_medio = int((y + y + h) / 2)
                break
            
            cv2.line(img, (x_medio, 0), (x_medio, 320), (0, 255, 0), 1)
            cv2.line(img, (0, y_medio), (240, y_medio), (0,255,0), 1)
            cv2.line(cinza_mask, (x_medio, 0), (x_medio, 320), (255, 255, 255), 1)
            cv2.line(cinza_mask, (0, y_medio), (240, y_medio), (255,255,255), 1)
            #cv2.imshow('image',image)
            cv2.imshow('POV',img)
            cv2.imshow('P/B',cinza_mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == sim.simx_return_novalue_flag:
            print("no image yet")
            pass
        else:
          print( err)
          break

    # Aguarda até que a sequência de movimento acima termine a execução
    espera_de_execução_movimento('SeqMov')
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
    sim.simxGetStringSignal(clientID,Nome_do_Sinal,sim.simx_opmode_discontinue)
    sim.simxGetPingTime(clientID)
    
    # Encerra a conexão com o Coppelia
    sim.simxFinish(clientID)
else:
    print ('Falhou a conexão com o remote API server')
cv2.destroyAllWindows()
print ('Programa finalizado')

