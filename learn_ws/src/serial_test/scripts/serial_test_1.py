import serial
import serial.tools.list_ports as serials
import os

try:
    password = '0000'
    cmdline = ['chmod -R 777 /dev/tty*']
    for i in cmdline:
        os.system("echo %s | sudo -S %s"%(password,i))
except:
    pass
ser2=serial.Serial("/dev/ttyCH9344USB2",9600,timeout=0.5) #使用USB连接串行口
ser3=serial.Serial("/dev/ttyCH9344USB3",9600,timeout=0.5) #使用USB连接串行口
data = "im ser3\n"
ser3.write(data.encode())
read=ser2.readline()
print(read.decode())