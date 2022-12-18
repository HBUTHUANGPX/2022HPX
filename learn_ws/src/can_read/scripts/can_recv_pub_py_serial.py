# -*- coding: utf-8 -*-
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
ser2=serial.Serial("/dev/ttyACM0",921600,timeout=0.5) #使用USB连接串行口
a=[]
n=16
while True:
    read=ser2.read(17)
    print(read)
    # if len(read):
    #     print(read.decode("utf8"))
    #     # print(int(read.decode("utf8","ignore")))
    #     if read.decode("utf8","ignore")==0xaa:
    #         print(read)