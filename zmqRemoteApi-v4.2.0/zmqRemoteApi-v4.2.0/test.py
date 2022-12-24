from time import sleep
from zmqRemoteApi import *
client = RemoteAPIClient()
sim = client.getobject('sim')
client.setsynchronous()
sim.startSimulation()
for i in range(5):
    print(f't={sim.getSimulationTime()}')
    sleep(2)
    print('stepping...')
    client.step()
    print('stepped')
sim.stopSimulation()
print('end')