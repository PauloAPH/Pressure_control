import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
import time
import scipy.io

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

portList = []

for onePort in ports:
    portList.append(str(onePort))
    print(str(onePort))

serialInst.baudrate = 9600
#serialInst.port = '/dev/ttyACM1' #Linux
serialInst.port = 'COM9' #Windows
serialInst.open()
t_test = 0

x = 0
while True:
    #time.sleep(1)
    packet = serialInst.readline()
    if(packet.decode('utf') == "Iniciar teste?\r\n"):    
        x = input(packet.decode('utf'))
        serialInst.reset_input_buffer()
        serialInst.write(x.encode())
        x = 0
    elif(packet.decode('utf') == "Entre com a pressao\r\n"):    
        serialInst.reset_input_buffer()
        x = input(packet.decode('utf'))
        #serialInst.reset_input_buffer()
        serialInst.write(x.encode())
        x = 0
    elif(packet.decode('utf') == "Entre com o tempo\r\n"):    
        serialInst.reset_input_buffer()
        t_test = input(packet.decode('utf'))
        serialInst.reset_input_buffer()
        serialInst.write(t_test.encode())
    elif(packet.decode('utf') == "Teste em andamento\r\n"):    
        print(packet.decode('utf'))
        start_time = time.time()
    elif(packet.decode('utf') == "Imprimir resultado?\r\n"):
        print("--- %s seconds ---" % (time.time() - start_time))    
        serialInst.reset_input_buffer()
        x = input(packet.decode('utf'))
        serialInst.reset_input_buffer()
        serialInst.write(x.encode())   
        x = 0     
    elif(packet.decode('utf') == "Imprimindo\r\n"): 
        data1 = [0]*int(t_test)*100
        data2 = [0]*int(t_test)*100
        data3 = [0]*int(t_test)*100
        data4 = [0]*int(t_test)*100
        data5 = [0]*int(t_test)*100
        data6 = [0]*int(t_test)*100
        data7 = [0]*int(t_test)*100
        data8 = [0]*int(t_test)*100
        t = np.arange(0, int(t_test), 0.01) 
        for i in range(0,int(t_test)*100):
            data1[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data2[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data3[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data4[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data5[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data6[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data7[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
            data8[i] = int(serialInst.readline().decode('utf').rstrip('\n').rstrip('\r'))
        fig, ax = plt.subplots()
        ax.plot(t, data1,t, data2,t, data3,t, data4,t,data5,t,data6,t,data7,t,data8)
        ax.set_xlim(0, int(t_test))
        #ax.set_ylim(-1, 1)
        ax.set_xlabel("Tempo em segundos")
        ax.set_ylabel("Pressão em bar")
        ax.legend(['Sensor1', 'Sensor2', 'Sensor3','Sensor4','P_Ref1','P_Ref2', 'P_Ref3', 'P_Ref4'])
        ax.grid()
        plt.savefig("figuras/"+ time.strftime("%Y_%m_%d_%H%M%S") + ".png")

        scipy.io.savemat("figuras/"+ time.strftime("%Y_%m_%d_%H%M%S")+".mat", mdict={'pref1': data5, 'p1': data1, 'p2': data2, 'p3': data3, 'p4': data4, 'time': t,'pref2': data6,'pref3': data7,'pref4': data8})