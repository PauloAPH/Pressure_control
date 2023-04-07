import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

portList = []

for onePort in ports:
    portList.append(str(onePort))
    print(str(onePort))

serialInst.baudrate = 115200
serialInst.port = '/dev/ttyACM1' #Linux
serialInst.open()

data1 = [0]*50
data2 = [0]*50
data3 = [0]*50
data4 = [0]*50
t = np.arange(0, 0.15, 0.003)
x = 0
while True:
    packet = serialInst.readline()
    if(packet.decode('utf') == "comecar teste\r\n"):    
        x = input(packet.decode('utf'))
        serialInst.write(x.encode())
    elif(packet.decode('utf') == "Entre com a pressao\r\n"):    
        x = input(packet.decode('utf'))
        serialInst.write(x.encode())
    elif(packet.decode('utf') == "Imprimir resultado?\r\n"):    
        x = input(packet.decode('utf'))
        serialInst.write(x.encode())        
    elif(packet.decode('utf') == "Imprimindo\r\n"):  
        for i in range(0,50):
            data1[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data2[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data3[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data4[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
        
        plt.plot(t, data1,t, data2,t, data3,t, data4)
        plt.legend(['Sensor1', 'Sensor2', 'Sensor3','Sensor4'])
        plt.grid()
        plt.show()