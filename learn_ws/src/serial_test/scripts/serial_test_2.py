import serial
import serial.tools.list_ports as serials
import os
import json
try:
    password = '0000'
    cmdline = ['chmod -R 777 /dev/tty*']
    for i in cmdline:
        os.system("echo %s | sudo -S %s"%(password,i))
except:
    pass
ser2=serial.Serial("/dev/ttyACM0",115200,timeout=0.5) #使用USB连接串行口
data = 1
print(hex(data))
# ser2.write(data.encode())