from time import sleep
import numpy as np
import coppeliaScan as cs
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
    #Vel=13.0#5*math.pi/180
    vel = [ 1.90777254,  1.91676066,  4.21490854,  8.32765953,  7.61692325,
       13.05404777]
    #10% da vel max
    for i, item in enumerate(vel):
        vel[i]= item/10

    Acel=600*math.pi/180
    Velmax= vel  #[Vel,Vel,Vel,Vel,Vel,Vel]
    Acelmax=[Acel/10,Acel,3/2*Acel,Acel,Acel,Acel]
    CoordVel=[0,0,0,0,0,0]

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
    j5 = 90*math.pi/180
    j6 = 0*math.pi/180
    #jq = [23,26,29,32,35,37]
    #jq = [22, 25, 28, 31, 34, 36]
    hj1 = sim.simxGetObjectHandle(clientID,'/',sim.simx_opmode_oneshot)
    print(hj1)
    jq = [24, 27, 30, 33, 36, 38]
    laser = 41

    def movimento(q):
        Config=q

        Dados_de_Movimento={"id":"SeqMov","tipo":"mov","Config":Config,"CoordVel":CoordVel,"Velmax":Velmax,"Acelmax":Acelmax}
        
        Empacotamento_dados_de_movimento=msgpack.packb(Dados_de_Movimento)
        sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Dados_Movimento',[],[],[],Empacotamento_dados_de_movimento,sim.simx_opmode_oneshot)

        # Executa a sequência de movimento
        sim.simxCallScriptFunction(clientID,Manipulador,sim.sim_scripttype_childscript,'legacy_API_Movimento',[],[],[],'SeqMov',sim.simx_opmode_oneshot)
        
        # Aguarda até que a sequência de movimento acima termine a execução
        espera_de_execução_movimento('SeqMov')

    # Envia a primeira sequência de movimento
    #Config=[30*math.pi/180,30*math.pi/180,-30*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180]
    
    def Scan(j5):
        
        inicial = [j1,j2,j3,j4,j5,j6]
        movimento(inicial)
        sleep(3)
        #pc = cs.escanear(True)
        scan = [j1,j2,j3,j4,j5,-math.pi]
        movimento(scan)
        sleep(3)
        
        #scan[4]=scan[4]+0.1
        movimento(inicial)
        sleep(3)
        
    def Scan2(j5):
        
        inicial = [j1,j2,j3,j4,j5,-math.pi]
        movimento(inicial)
        sleep(3)
        #pc = cs.escanear(True)
        scan = [j1,j2,j3,math.pi,j5,-math.pi]
        movimento(scan)
        sleep(3)
        
        #scan[4]=scan[4]+0.1
        movimento(inicial)
        sleep(3)
    
    for item in range(10):
        cs.escanear(True)
        j5 = j5-item/10
        Scan(j5)
        sim.simxSetJointTargetPosition(clientID,jq[4],j5,sim.simx_opmode_streaming)
        sleep(4)
    
    j5 = 90*math.pi/180
    q90=[j1,j2,j3,j4,j5,j6]
    movimento(q90)
    sleep(3)
        
    for item in range(10):
        
        j5 = j5-item/10
        Scan2(j5)
        sim.simxSetJointTargetPosition(clientID,jq[4],j5,sim.simx_opmode_streaming)
        sleep(4)
    
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
    sim.simxGetStringSignal(clientID,Nome_do_Sinal,sim.simx_opmode_discontinue)
    sim.simxGetPingTime(clientID)
    
    # Encerra a conexão com o Coppelia
    sim.simxFinish(clientID)
else:
    print ('Falhou a conexão com o remote API server')

a = input()
print ('Programa finalizado')