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

# carrega o classificador em cascata
cascata = cv2.CascadeClassifier('cascade.xml')

print ('Programa Iniciado')
sim.simxFinish(-1) # Fecha todas as conexões
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Conecta ao CoppeliaSim
if clientID!=-1:
    print ('Conectado com o remote API-Legacy server')

    client_Id_Executado='Sem leitura'
    Manipulador= 'Cubebot'  
    Nome_do_Sinal='P_Arm_Id_Executado'
    goal = False

    # Set-up some movement variables:
    Vel=5*math.pi/180
    Acel=5*math.pi/180
    Velmax=[Vel,Vel,Vel,Vel,Vel,Vel]
    Acelmax=[Acel,Acel,Acel,Acel,Acel,Acel]
    CoordVel=[0,0,0,0,0,0]
    J_atual = []
    def espera_de_execução_movimento(id):
        global client_Id_Executado
        global Nome_do_Sinal
        while client_Id_Executado!=id:
            retCode,s=sim.simxGetStringSignal(clientID,Nome_do_Sinal,sim.simx_opmode_buffer)
            if retCode==sim.simx_return_ok:
                if type(s)==bytearray:
                    s=s.decode('ascii') 
                client_Id_Executado=s

    def aprox_final(xm_atual,ym_atual,z_atual,J_atual):
        if (xm_atual < 140):
            Kp = (140-xm_atual)/140
            sim.simxSetJointTargetVelocity(clientID,23,Kp*Vel,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,23,J_atual[0]-0.01,sim.simx_opmode_oneshot)
            xOK = False
        elif (xm_atual > 180):
            Kp = (180-xm_atual)/320
            sim.simxSetJointTargetVelocity(clientID,23,Kp*Vel,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,23,J_atual[0]+0.01,sim.simx_opmode_oneshot)
            xOK = False
        else:
            xOK = True
        #xm_atual=x_medio

        if (ym_atual < 100):
            Kp = (100-xm_atual)/100
            sim.simxSetJointTargetVelocity(clientID,26,Kp*Vel,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetPosition(clientID,26,J_atual[1]-0.01,sim.simx_opmode_oneshot)
            yOK = False
        elif (ym_atual > 140):
            Kp = (140-xm_atual)/240
            sim.simxSetJointTargetVelocity(clientID,26,Kp*Vel,sim.simx_opmode_oneshot)
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

    # Inscreve o Nome_do_Sinal recebido do servidor(em bytes) para chamada_de_retorno (cliente) 
    sim.simxGetStringSignal(clientID,Nome_do_Sinal,sim.simx_opmode_streaming)
    #configurações da camera
    print('Vision Sensor object handling')
    res, v1 = sim.simxGetObjectHandle(clientID, 'CamP1', sim.simx_opmode_oneshot_wait)
    print('Getting first image')
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_streaming)

    # Inicia a simulação
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    # Espera de leitura
    espera_de_execução_movimento('Ler')
    
    j1 = 0*math.pi/180
    j2 = 45*math.pi/180
    j3 = 30*math.pi/180
    j4 = 0
    j5 = 170*math.pi/180
    j6 = 0*math.pi/180

    # Envia a primeira sequência de movimento
    #Config=[30*math.pi/180,30*math.pi/180,-30*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180]
    Config=[j1,j2,j3,j4,j5,j6]
    Dados_de_Movimento={"id":"SeqMov","tipo":"mov","Config":Config,"CoordVel":CoordVel,"Velmax":Velmax,"Acelmax":Acelmax}
    
    Empacotamento_dados_de_movimento=msgpack.packb(Dados_de_Movimento)
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Dados_Movimento',[],[],[],Empacotamento_dados_de_movimento,sim.simx_opmode_oneshot)

    # Executa a sequência de movimento
    sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Movimento',[],[],[],'SeqMov',sim.simx_opmode_oneshot)
    
    #resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_buffer)
    #linhas, cols, _ = image.shape   #obtem numero de linhas e colunas da imagem
    linhas = 240
    cols = 320
    x_medio = int(cols / 2)         #calcula x do meio da tela
    y_medio = int(linhas/2)         #calcula y do meio da tela
    centro = int(cols / 2)          #mesma coisa, mas essa variavel nao vai mudar
    centroY = int(linhas/2)         
    posicao = 90 # degrees          #valor inicial do motor do robo
    xm_atual = 160

    #{J1, J2, J3, J4, J5, J6}
    #{23, 26, 29, 32, 35, 38}
    J0_atual = sim.simxGetJointPosition(clientID,23,sim.simx_opmode_oneshot)[1]
    J2_atual = sim.simxGetJointPosition(clientID,29,sim.simx_opmode_oneshot)[1]
    '''
    for item in range(314):
        sim.simxSetJointTargetPosition(clientID,23,(math.pi/2)+(item/100),sim.simx_opmode_oneshot)
        time.sleep(0.1)
    for item in range(150):
        sim.simxSetJointTargetPosition(clientID,29,(math.pi/2)+(item/100),sim.simx_opmode_oneshot)
        time.sleep(0.1)
    '''
    
    while (sim.simxGetConnectionId(clientID) != -1):
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_buffer)
        
        if err == sim.simx_return_ok:
            #print("image OK!!!")
            '''
            for item in range(157):
                sim.simxSetJointTargetPosition(clientID,38,item/100,sim.simx_opmode_oneshot)
                time.sleep(0.1)
            
            for i in range(314):
                sim.simxSetJointTargetPosition(clientID,32,i/100,sim.simx_opmode_oneshot)
                time.sleep(0.1)

            '''
            img = np.array(image,dtype=np.uint8)
            
            img.resize([resolution[1],resolution[0],3])
           
            cinza = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            # detecta as faces
            faces = cascata.detectMultiScale(cinza, 1.1, 25,None,[50,50])
            
            # desenha retangulo em volta de cada face
            for (x, y, w, h) in faces:
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                x_medio = int((x + x + w) / 2)
                y_medio = int((y + y + h) / 2)
                break
            
            cv2.line(img, (x_medio, 0), (x_medio, 240), (0, 255, 0), 1)
            cv2.line(img, (0, y_medio), (320, y_medio), (0,255,0), 1)
            
            cv2.line(cinza, (x_medio, 0), (x_medio, 240), (255, 255, 255), 1)
            cv2.line(cinza, (0, y_medio), (320, y_medio), (255,255,255), 1)
            
            cv2.imshow('POV',img)
            cv2.imshow('P/B',cinza)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == sim.simx_return_novalue_flag:
            print("no image yet")
            pass
        else:
          print( err)
          break

        
        indices = [23, 26, 29, 32, 35, 38]
        for j in indices:
            J_atual.append(sim.simxGetJointPosition(clientID,j,sim.simx_opmode_oneshot)[1])
        '''
        atual = sim.simxGetJointPosition(clientID,23,sim.simx_opmode_oneshot)[1]
        atualJ2 = sim.simxGetJointPosition(clientID,29,sim.simx_opmode_oneshot)[1]
        atualJ6 = sim.simxGetJointPosition(clientID,38,sim.simx_opmode_oneshot)[1]
        '''
        estendido = sim.simxGetJointPosition(clientID,26,sim.simx_opmode_oneshot)[1]>=(j2)
        dist = sim.simxReadProximitySensor(clientID,41,sim.simx_opmode_streaming)[2][2]
        if dist <= 0:
            dist = 0.08
        #print(J_atual,x_medio,y_medio,dist)
        if(estendido):
            #print(J_atual)
            #goal = aprox_final(x_medio,y_medio,dist,J_atual)
            xm_atual = x_medio
            ym_atual = y_medio
            z_atual = dist
            if (xm_atual < 140):
                Kp = (140-xm_atual)/140
                sim.simxSetJointTargetVelocity(clientID,23,Kp*Vel,sim.simx_opmode_oneshot)
                sim.simxSetJointTargetPosition(clientID,23,J_atual[0]-0.01,sim.simx_opmode_oneshot)
                xOK = False
            elif (xm_atual > 180):
                Kp = (180-xm_atual)/320
                sim.simxSetJointTargetVelocity(clientID,23,Kp*Vel,sim.simx_opmode_oneshot)
                sim.simxSetJointTargetPosition(clientID,23,J_atual[0]+0.01,sim.simx_opmode_oneshot)
                xOK = False
            else:
                xOK = True
            #xm_atual=x_medio

            if (ym_atual < 100):
                Kp = (100-xm_atual)/100
                sim.simxSetJointTargetVelocity(clientID,26,Kp*Vel,sim.simx_opmode_oneshot)
                sim.simxSetJointTargetPosition(clientID,26,J_atual[1]-0.01,sim.simx_opmode_oneshot)
                yOK = False
            elif (ym_atual > 140):
                Kp = (140-xm_atual)/240
                sim.simxSetJointTargetVelocity(clientID,26,Kp*Vel,sim.simx_opmode_oneshot)
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
        #if goal:
            #capturar(J_atual[5])
        
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

